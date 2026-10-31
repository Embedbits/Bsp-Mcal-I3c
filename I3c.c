/**
 * \author Mr.Nobody
 * \file I3c.c
 * \ingroup I3c
 * \brief Improved Inter-Integrated Circuit (I3C) MCAL module common functionality
 *
 * The module owns the I3C peripheral: the clock goes through the Rcc module, the interrupts
 * through the Nvic module, the pins through the Gpio module and the DMA transfers of the data FIFOs
 * through the Gpdma module, the registers are accessed from I3c_Reg.c only. This file holds the
 * module management (initialization with the verification by read-back, the bus timing, the
 * deinitialization, the task), the device table of the controller and the bus bring-up (reset of
 * the dynamic addresses, ENTDAA). The transfers, the CCCs and the requests of the targets are in
 * I3c_Xfer.c, the controller engine is in I3c_Ctrl.c, the target in I3c_Tgt.c.
 *
 * Scope and limits of this STM32H5 implementation (see README "Status" section):
 * - The hardware has four target device registers, the device table has \ref I3C_DEVICE_CNT_MAX entries.
 * - The kernel clock must be at least twice the push-pull SCL frequency (RM0481 54.6.2). The
 *   timing registers are computed by rounding the cycle counts up, so the achieved frequencies are
 *   equal to or lower than the requested ones (reported by I3c_Get_BusFreq()).
 * - Errata "unexpected first frame with a 0x7F address when the I3C peripheral is enabled" (bug AB#2093;
 *   ES0561 2.10.3, ES0621 2.13.3, ES0565 2.15.3; workaround sequence ES0561 2.11, ES0621 2.14,
 *   ES0565 2.16): initialization as controller applies the workaround. The pull-up of the SDA pin is
 *   enabled until the line reads high (bounded loop), then it is disabled, then the controller is
 *   initialized and enabled. The errata of the H5E4 / H5E5 / H5F4 / H5F5 lines were not available.
 * - Errata "unexpected read data bytes during a legacy I2C read" (bug AB#2091; ES0561 2.10.1, ES0621 2.13.1,
 *   ES0565 2.15.1) is avoided: the STALLT bit of I3C_TIMINGR2 is never set (the register is written with
 *   0 at the controller initialization and not touched elsewhere).
 * - Errata "SCL clock is not stalled during address ACK/NACK phase" (bug AB#2092; ES0561 2.10.2, ES0621 2.13.2,
 *   ES0565 2.15.2) is avoided: the arbitrable header is kept enabled (NOARBH = 0 in I3C_CFGR) and the
 *   STALLA bit of I3C_TIMINGR2 is never set, SCL stalling is not used.
 *
 */
/* ============================== INCLUDES ================================== */
#include "I3c.h"                            /* Module private interface       */
#include "I3c_Port.h"                       /* Module public interface        */
#include "I3c_Reg.h"                        /* I3C register access            */
#include "Rcc_Port.h"                       /* RCC Mcal layer include         */
#include "Nvic_Port.h"                      /* NVIC Mcal layer include        */
#include "Gpio_Port.h"                      /* GPIO Mcal layer include        */
/* ============================== TYPEDEFS ================================== */

/** \brief Values of the timing registers computed from the configuration */
typedef struct
{
    uint32_t     Timingr0;      /**< Value of I3C_TIMINGR0 (SCL durations)            */
    uint32_t     Timingr1;      /**< Value of I3C_TIMINGR1 (free time, time unit)      */
    i3c_FreqHz_t PushPullFreq;  /**< Achieved push-pull SCL frequency                  */
    i3c_FreqHz_t OpenDrainFreq; /**< Achieved open-drain SCL frequency                 */
}   i3c_Timing_t;

/** \brief High and low SCL time of a legacy I2C speed class */
typedef struct
{
    i3c_TimeNs_t HighTime; /**< SCL high time of the class in ns */
    i3c_TimeNs_t LowTime;  /**< SCL low time of the class in ns  */
}   i3c_I2cTime_t;

/** \brief Protocol error code of I3C_SER and the error it is reported as */
typedef struct
{
    uint32_t        Code;  /**< CODERR value                        */
    i3c_XferError_t Error; /**< Error bit reported to the callbacks */
}   i3c_CodeError_t;

/* ======================== FORWARD DECLARATIONS ============================ */

static i3c_RequestState_t I3cPriv_Check_Config    ( const i3c_Config_t * const i3cConfig );
static i3c_RequestState_t I3cPriv_Check_Clock     ( const i3c_Config_t * const i3cConfig );
static i3c_RequestState_t I3cPriv_Check_Pins      ( const i3c_Config_t * const i3cConfig );
static i3c_RequestState_t I3cPriv_Check_Pin       ( uint32_t pinValue, uint32_t unusedValue, i3c_PeriphId_t periphId );
static i3c_RequestState_t I3cPriv_Check_Dma       ( const i3c_Config_t * const i3cConfig );
static i3c_RequestState_t I3cPriv_Check_Devices   ( const i3c_Context_t * const ctx,
                                                    const i3c_Device_t * const devices,
                                                    i3c_DeviceCnt_t deviceCnt );
static i3c_RequestState_t I3cPriv_Init_Clock      ( i3c_Context_t * const ctx );
static i3c_RequestState_t I3cPriv_Init_Pins       ( const i3c_Context_t * const ctx );
static i3c_RequestState_t I3cPriv_Init_SdaLine    ( i3c_SdaPin_t sdaPin );
static i3c_RequestState_t I3cPriv_Init_Irq        ( const i3c_Context_t * const ctx );
static i3c_RequestState_t I3cPriv_Write_Regs      ( i3c_Context_t * const ctx );
static i3c_RequestState_t I3cPriv_Verify_Regs     ( const i3c_Context_t * const ctx );
static i3c_RequestState_t I3cPriv_Calc_Timing     ( i3c_FreqHz_t kernelClk,
                                                    const i3c_Config_t * const i3cConfig,
                                                    i3c_I2cSpeed_t i2cClass,
                                                    i3c_Timing_t * const timing );
static uint32_t           I3cPriv_Get_Cycles      ( i3c_FreqHz_t kernelClk, i3c_TimeNs_t timeNs );
static i3c_I2cSpeed_t     I3cPriv_Get_SlowestI2c  ( const i3c_Device_t * const devices, i3c_DeviceCnt_t deviceCnt );
static i3c_FunctionState_t I3cPriv_Get_AddressValid( const i3c_Context_t * const ctx, i3c_Address_t address );
static i3c_FunctionState_t I3cPriv_Get_AddressFree ( const i3c_Context_t * const ctx,
                                                     i3c_Address_t address,
                                                     i3c_DeviceCnt_t skipIdx );
static i3c_Address_t      I3cPriv_Get_FreeAddress ( const i3c_Context_t * const ctx, i3c_DeviceCnt_t skipIdx );
static void               I3cPriv_Release         ( i3c_Context_t * const ctx, i3c_PeriphId_t periphId );
static void               I3cPriv_Handle_Events   ( i3c_PeriphId_t periphId );
static void               I3cPriv_Isr_1           ( void );
#if defined(I3C2)
static void               I3cPriv_Isr_2           ( void );
#endif

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** \brief Version of the module */
#define I3C_MODULE_VERSION_MAJOR              ( 1u )
#define I3C_MODULE_VERSION_MINOR              ( 0u )
#define I3C_MODULE_VERSION_PATCH              ( 0u )

/** \brief Default interrupt priority of I3c_Get_DefaultConfig() */
#define I3C_DEFAULT_IRQ_PRIORITY              ( 5u )

/** \brief Count of the kernel clock sources of one instance */
#define I3C_CLK_SRC_PER_PERIPH_CNT            ( 3u )

/** \brief Hz per kHz (the time to cycles conversion is done in kHz to stay in 32 bits) */
#define I3C_HZ_PER_KHZ                        ( 1000u )

/** \brief Nanoseconds per millisecond (kHz multiplied by ns is in units of 1/1000000) */
#define I3C_NS_PER_MS                         ( 1000000u )

/** \brief Minimum SCL high time of the push-pull and open-drain phases in ns (tDIG_H) */
#define I3C_MIN_HIGH_TIME                     ( 32u )

/** \brief Minimum SCL low time of the push-pull phase in ns (tDIG_L) */
#define I3C_MIN_LOW_PP_TIME                   ( 32u )

/** \brief Minimum SCL low time of the open-drain phase in ns (tDIG_OD_L) */
#define I3C_MIN_LOW_OD_TIME                   ( 200u )

/** \brief Minimum stop and repeated start timing in ns (tCBP, tCBSr, tCASr) */
#define I3C_MIN_STOP_TIME                     ( 20u )

/** \brief SCL high and low time of the legacy I2C speed classes in ns (the period is the period of the class) */
#define I3C_SM_HIGH_TIME                      ( 5000u )
#define I3C_SM_LOW_TIME                       ( 5000u )
#define I3C_FM_HIGH_TIME                      ( 1000u )
#define I3C_FM_LOW_TIME                       ( 1500u )
#define I3C_FMP_HIGH_TIME                     ( 400u )
#define I3C_FMP_LOW_TIME                      ( 600u )

/** \brief Count of the interrupt lines of an instance (event and error line) */
#define I3C_IRQ_LINE_CNT                      ( 2u )

/** \brief Count of the bytes of the lower part of the PID in the ENTDAA payload */
#define I3C_PID_LOW_BYTE_CNT                  ( 4u )

/** \brief Maximum number of cycles of an SCL duration (the field of the register has 8 bits and holds cycles - 1) */
#define I3C_SCL_CYCLES_MAX                    ( 256u )

/** \brief Maximum value of the FREE field of I3C_TIMINGR1 (7 bits) */
#define I3C_FREE_VALUE_MAX                    ( 127u )

/** \brief Maximum value of the AVAL field of I3C_TIMINGR1 (8 bits) */
#define I3C_AVAL_VALUE_MAX                    ( 255u )

/** \brief Count of the kernel clock cycles of the bus available time added by the hardware to AVAL */
#define I3C_AVAL_OFFSET                       ( 2u )

/** \brief Ratio of the minimum kernel clock to the push-pull SCL frequency (RM0481 54.6.2) */
#define I3C_KERNEL_CLK_RATIO                  ( 2u )

/** \brief Divisor used to halve a count of cycles and to round the FREE time up */
#define I3C_HALF_DIVISOR                      ( 2u )

/** \brief Count of the entries of the I3C device registers (DEVR1 .. DEVR4) */
#define I3C_DEVR_CNT                          ( I3C_DEVICE_CNT_MAX )

/** \brief Count of the bytes of the PID stored in the high / low part and the shift of one byte */
#define I3C_PID_HIGH_BYTE_CNT                 ( 2u )

/** \brief Bounded wait of the SDA line to read high after the pull-up was enabled (errata workaround) */
#define I3C_SDA_WAIT_RAW                      ( (i3c_TimeoutCnt_t)0xFFFFu )

/** \brief Value of a protocol error code of I3C_SER that is a parity error */
#define I3C_CODERR_TE1                        ( 0x9u )
#define I3C_CODERR_TE2                        ( 0xAu )
#define I3C_CODERR_TE3                        ( 0xBu )

/** \brief Value of a protocol error code of I3C_SER that is a monitoring (bus) error */
#define I3C_CODERR_CE1                        ( 0x1u )
#define I3C_CODERR_TE6                        ( 0xEu )

/** \brief Bits of I3C_CFGR checked by the read-back of the initialization */
#define I3C_CFGR_VERIFY_MASK                  ( I3C_CFGR_EN | I3C_CFGR_CRINIT | I3C_CFGR_HKSDAEN | I3C_CFGR_TMODE | \
                                                I3C_CFGR_TXDMAEN | I3C_CFGR_RXDMAEN )

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** \brief Runtime contexts of the instances */
static i3c_Context_t i3c_CtxList[ I3C_PERIPH_CNT ];

/** \brief Kernel clock sources allowed for an instance */
static const i3c_ClkSrc_t i3c_ClkSrcList[ I3C_PERIPH_CNT ][ I3C_CLK_SRC_PER_PERIPH_CNT ] =
{
    {
        I3C_CLK_SRC_I3C1_PCLK1,
#if defined(RCC_CR_PLL3ON)
        I3C_CLK_SRC_I3C1_PLL3R,
#else
        I3C_CLK_SRC_I3C1_PLL2R,
#endif
        I3C_CLK_SRC_I3C1_HSI
    },
#if defined(I3C2)
    {
        I3C_CLK_SRC_I3C2_PCLK3,
#if defined(RCC_CR_PLL3ON)
        I3C_CLK_SRC_I3C2_PLL3R,
#else
        I3C_CLK_SRC_I3C2_PLL2R,
#endif
        I3C_CLK_SRC_I3C2_HSI
    },
#endif
};

/** \brief Interrupt lines of the instances: event line and error line */
static const nvic_PeriphIrqList_t i3c_IrqList[ I3C_PERIPH_CNT ][ I3C_IRQ_LINE_CNT ] =
{
    { NVIC_PERIPH_IRQ_I3C1_EV, NVIC_PERIPH_IRQ_I3C1_ER },
#if defined(I3C2)
    { NVIC_PERIPH_IRQ_I3C2_EV, NVIC_PERIPH_IRQ_I3C2_ER },
#endif
};

/** \brief Interrupt handlers of the instances (one handler serves both lines of an instance) */
static const nvic_IsrCallback_t i3c_IsrList[ I3C_PERIPH_CNT ] =
{
    I3cPriv_Isr_1,
#if defined(I3C2)
    I3cPriv_Isr_2,
#endif
};

/** \brief SCL high and low time of the legacy I2C speed classes (the period is the class period or longer) */
static const i3c_I2cTime_t i3c_I2cTimeList[ I3C_I2C_SPEED_CNT ] =
{
    { .HighTime = I3C_SM_HIGH_TIME , .LowTime = I3C_SM_LOW_TIME  }, /**< Standard-mode (tHIGH 4000, tLOW 4700 ns) */
    { .HighTime = I3C_FM_HIGH_TIME , .LowTime = I3C_FM_LOW_TIME  }, /**< Fast-mode (tHIGH 600, tLOW 1320 ns)      */
    { .HighTime = I3C_FMP_HIGH_TIME, .LowTime = I3C_FMP_LOW_TIME }, /**< Fast-mode plus (tHIGH 260, tLOW 500 ns)  */
};

/** \brief Protocol error codes of I3C_SER and the reported error (an unknown code is a frame error) */
static const i3c_CodeError_t i3c_CodeErrorList[] =
{
    { .Code = I3C_CODERR_TE1, .Error = (i3c_XferError_t)I3C_XFER_ERROR_PARITY },
    { .Code = I3C_CODERR_TE2, .Error = (i3c_XferError_t)I3C_XFER_ERROR_PARITY },
    { .Code = I3C_CODERR_TE3, .Error = (i3c_XferError_t)I3C_XFER_ERROR_PARITY },
    { .Code = I3C_CODERR_CE1, .Error = (i3c_XferError_t)I3C_XFER_ERROR_BUS    },
    { .Code = I3C_CODERR_TE6, .Error = (i3c_XferError_t)I3C_XFER_ERROR_BUS    },
};

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Gets the version of the module.
 *
 * \return Version of the module
 */
i3c_ModuleVersion_t I3c_Get_ModuleVersion( void )
{
    i3c_ModuleVersion_t version = { I3C_MODULE_VERSION_MAJOR, I3C_MODULE_VERSION_MINOR, I3C_MODULE_VERSION_PATCH };

    return ( version );
}


/**
 * \brief Initializes an I3C instance in the selected role and leaves the bus idle.
 *
 * The configuration is checked before any register is written. Initialization enables the kernel
 * clock, configures the pins, the timing, the FIFOs, the DMA and the interrupts, enables the
 * peripheral and verifies the result by read-back. On a failure the instance is deinitialized.
 *
 * \param i3cConfig [in]: Module configuration
 * \return I3C_REQUEST_OK when the instance is initialized, I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3c_Init( const i3c_Config_t * const i3cConfig )
{
    i3c_RequestState_t  retState = I3C_REQUEST_ERROR;
    i3c_Context_t *     ctx      = I3C_NULL_PTR;
    i3c_RequestState_t  ctxState = I3C_REQUEST_ERROR;
    i3c_FunctionState_t prepared = I3C_FUNCTION_INACTIVE;

    if( I3C_NULL_PTR != i3cConfig )
    {
        retState = I3cPriv_Check_Config( i3cConfig );
    }
    else
    {
        /* Invalid configuration pointer */
        retState = I3C_REQUEST_ERROR;
    }

    if( I3C_REQUEST_OK == retState )
    {
        ctxState = I3c_Get_Context( i3cConfig->PeriphId, &ctx );

        if( ( I3C_REQUEST_OK == ctxState ) &&
            ( I3C_NULL_PTR   != ctx      )    )
        {
            if( I3C_FUNCTION_INACTIVE == ctx->InitState )
            {
                *ctx           = (i3c_Context_t){ 0 };
                ctx->Config    = *i3cConfig;
                ctx->I2cClass  = I3C_I2C_SPEED_CNT;
                ctx->XferState = I3C_XFER_STATE_IDLE;
                ctx->ReqKind   = I3C_REQ_KIND_NONE;
                prepared       = I3C_FUNCTION_ACTIVE;
                retState       = I3cPriv_Init_Clock( ctx );
            }
            else
            {
                /* The instance is already initialized */
                retState = I3C_REQUEST_ERROR;
            }
        }
        else
        {
            retState = I3C_REQUEST_ERROR;
        }
    }
    else
    {
        /* The configuration is not valid, no register is touched */
        retState = I3C_REQUEST_ERROR;
    }

    if( I3C_REQUEST_OK == retState )
    {
        retState = I3cPriv_Init_Pins( ctx );
    }
    else
    {
        /* Nothing was initialized or the clock failed, no action required */
    }

    if( I3C_REQUEST_OK == retState )
    {
        retState = I3cPriv_Write_Regs( ctx );
    }
    else
    {
        /* The pins failed, no action required */
    }

    if( ( I3C_REQUEST_OK == retState                ) &&
        ( I3C_XFER_MODE_DMA == ctx->Config.XferMode )    )
    {
        retState = I3cDma_Init( ctx->Config.PeriphId );
    }
    else
    {
        /* DMA is not used or the registers failed, no action required */
    }

    if( ( I3C_REQUEST_OK == retState                ) &&
        ( I3C_XFER_MODE_POLL != ctx->Config.XferMode )    )
    {
        retState = I3cPriv_Init_Irq( ctx );
    }
    else
    {
        /* The events are polled or a previous step failed, no action required */
    }

    if( I3C_REQUEST_OK == retState )
    {
        ctx->InitState = I3C_FUNCTION_ACTIVE;
        retState       = I3c_Reg_Set_Enable( ctx->Config.PeriphId );
    }
    else
    {
        /* A previous step failed, no action required */
    }

    if( I3C_REQUEST_OK == retState )
    {
        retState = I3cPriv_Verify_Regs( ctx );
    }
    else
    {
        /* The peripheral was not enabled, no action required */
    }

    if( ( I3C_REQUEST_OK != retState            ) &&
        ( I3C_FUNCTION_ACTIVE == prepared       )    )
    {
        /* This call prepared the instance, the instance is released (an already initialized one is not touched) */
        I3cPriv_Release( ctx, i3cConfig->PeriphId );
    }
    else
    {
        /* The initialization succeeded or the instance was not touched, no action required */
    }

    return ( retState );
}


/**
 * \brief Deinitializes an I3C instance: the peripheral is disabled, the interrupts and the DMA
 *        channels are stopped, the peripheral is reset and its clock is disabled.
 *
 * \param periphId [in]: I3C peripheral ID
 * \return I3C_REQUEST_OK when the instance was initialized and is released, I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3c_Deinit( i3c_PeriphId_t periphId )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    i3c_Context_t *    ctx      = I3C_NULL_PTR;
    i3c_RequestState_t ctxState = I3c_Get_Context( periphId, &ctx );

    if( ( I3C_REQUEST_OK == ctxState ) &&
        ( I3C_NULL_PTR   != ctx      )    )
    {
        if( I3C_FUNCTION_ACTIVE == ctx->InitState )
        {
            I3cPriv_Release( ctx, periphId );
            retState = I3C_REQUEST_OK;
        }
        else
        {
            /* The instance is not initialized */
            retState = I3C_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid peripheral ID */
        retState = I3C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Serves the instances that use the POLL mode (events, FIFOs, requests of the targets).
 */
void I3c_Task( void )
{
    for( uint32_t idx = 0u; (uint32_t)I3C_PERIPH_CNT > idx; idx++ )
    {
        if( ( I3C_FUNCTION_ACTIVE == i3c_CtxList[ idx ].InitState          ) &&
            ( I3C_XFER_MODE_POLL  == i3c_CtxList[ idx ].Config.XferMode    )    )
        {
            I3cPriv_Handle_Events( (i3c_PeriphId_t)idx );
        }
        else
        {
            /* The instance is not initialized or not polled, no action required */
        }
    }
}


/**
 * \brief Fills the configuration with the default values: instance I3C1 as controller in the
 *        POLL mode, 4 MHz push-pull and 1 MHz open-drain, no pin and no callback.
 *
 * \param i3cConfig [out]: Configuration to fill
 * \return I3C_REQUEST_OK when the configuration was filled, I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3c_Get_DefaultConfig( i3c_Config_t * const i3cConfig )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;

    if( I3C_NULL_PTR != i3cConfig )
    {
        *i3cConfig = (i3c_Config_t){ 0 };

        i3cConfig->PeriphId         = I3C_PERIPH_1;
        i3cConfig->Role             = I3C_ROLE_CONTROLLER;
        i3cConfig->ClkSrc           = I3C_CLK_SRC_I3C1_PCLK1;
        i3cConfig->PushPullFreq     = I3C_DEFAULT_PUSH_PULL_FREQ;
        i3cConfig->OpenDrainFreq    = I3C_DEFAULT_OPEN_DRAIN_FREQ;
        i3cConfig->BusFreeTime      = I3C_DEFAULT_BUS_FREE_TIME;
        i3cConfig->BusAvailableTime = I3C_DEFAULT_BUS_AVAILABLE_TIME;
        i3cConfig->OwnAddress       = I3C_DEFAULT_OWN_ADDRESS;
        i3cConfig->SdaHighKeeper    = I3C_FUNCTION_INACTIVE;
        i3cConfig->XferMode         = I3C_XFER_MODE_POLL;
        i3cConfig->TxDmaPeriphId    = GPDMA_PERIPH_1;
        i3cConfig->TxDmaChannelId   = GPDMA_CHANNEL_0;
        i3cConfig->TxDmaPriority    = GPDMA_PRIORITY_MEDIUM;
        i3cConfig->RxDmaPeriphId    = GPDMA_PERIPH_1;
        i3cConfig->RxDmaChannelId   = GPDMA_CHANNEL_1;
        i3cConfig->RxDmaPriority    = GPDMA_PRIORITY_MEDIUM;
        i3cConfig->IrqPriority      = I3C_DEFAULT_IRQ_PRIORITY;
        i3cConfig->SclPin           = I3C_SCL_PIN_UNUSED;
        i3cConfig->SdaPin           = I3C_SDA_PIN_UNUSED;
        retState                    = I3C_REQUEST_OK;
    }
    else
    {
        /* Invalid configuration pointer */
        retState = I3C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Gets the achieved SCL frequencies of an instance initialized as controller.
 *
 * \param periphId      [in]: I3C peripheral ID
 * \param pushPullFreq  [out]: Achieved push-pull SCL frequency in Hz
 * \param openDrainFreq [out]: Achieved open-drain SCL frequency in Hz (lowered by the slowest legacy device)
 * \return I3C_REQUEST_OK when the frequencies were read, I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3c_Get_BusFreq( i3c_PeriphId_t periphId,
                                    i3c_FreqHz_t * const pushPullFreq,
                                    i3c_FreqHz_t * const openDrainFreq )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    i3c_Context_t *    ctx      = I3C_NULL_PTR;
    i3c_RequestState_t ctxState = I3c_Get_Context( periphId, &ctx );

    if( ( I3C_REQUEST_OK == ctxState     ) &&
        ( I3C_NULL_PTR   != ctx          ) &&
        ( I3C_NULL_PTR   != pushPullFreq ) &&
        ( I3C_NULL_PTR   != openDrainFreq )    )
    {
        if( ( I3C_FUNCTION_ACTIVE == ctx->InitState   ) &&
            ( I3C_ROLE_CONTROLLER == ctx->Config.Role )    )
        {
            *pushPullFreq  = ctx->PushPullFreq;
            *openDrainFreq = ctx->OpenDrainFreq;
            retState       = I3C_REQUEST_OK;
        }
        else
        {
            /* The instance is not initialized as controller */
            retState = I3C_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid parameter */
        retState = I3C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Resets the dynamic addresses of all the targets (RSTDAA broadcast CCC). The function
 *        waits for the end of the CCC (bounded loop). The addresses of the I3C devices of the
 *        device table become invalid.
 *
 * \param periphId [in]: I3C peripheral ID
 * \return I3C_REQUEST_OK when the CCC ended without an error, I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3c_Set_BusInit( i3c_PeriphId_t periphId )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    i3c_Context_t *    ctx      = I3C_NULL_PTR;
    i3c_RequestState_t ctxState = I3c_Get_Context( periphId, &ctx );
    i3c_CtrlWord_t     ctrlWord = 0u;

    if( ( I3C_REQUEST_OK == ctxState ) &&
        ( I3C_NULL_PTR   != ctx      )    )
    {
        ctrlWord = ( ( (i3c_CtrlWord_t)1u                << I3C_CR_MEND_Pos  ) |
                     ( (i3c_CtrlWord_t)I3C_MTYPE_CCC     << I3C_CR_MTYPE_Pos ) |
                     ( (i3c_CtrlWord_t)I3C_CCC_RSTDAA    << I3C_CR_CCC_Pos   ) );

        retState = I3cCtrl_Start_Frame( periphId,
                                        &ctrlWord,
                                        1u,
                                        I3C_NULL_PTR,
                                        0u,
                                        I3C_NULL_PTR,
                                        0u,
                                        I3C_FUNCTION_ACTIVE );

        if( I3C_REQUEST_OK == retState )
        {
            retState = I3cCtrl_Wait_Done( periphId );
        }
        else
        {
            /* The CCC was not started, no action required */
        }

        if( I3C_REQUEST_OK == retState )
        {
            for( i3c_DeviceCnt_t idx = 0u; ctx->DeviceCnt > idx; idx++ )
            {
                if( I3C_DEVICE_I3C == ctx->Devices[ idx ].Type )
                {
                    ctx->Devices[ idx ].AddressState = I3C_FUNCTION_INACTIVE;
                }
                else
                {
                    /* A legacy I2C device keeps its static address, no action required */
                }
            }

            retState = I3c_Set_DeviceRegs( periphId );
        }
        else
        {
            /* The CCC failed, no action required */
        }
    }
    else
    {
        /* Invalid peripheral ID */
        retState = I3C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Runs the dynamic address assignment (ENTDAA): every device that answers is stored in
 *        the device table (PID, BCR, DCR) and gets a dynamic address - the address requested by
 *        a table entry with the same PID, otherwise the next free address of the pool. The
 *        function waits for the end of the procedure (bounded loop).
 *
 * \param periphId      [in]: I3C peripheral ID
 * \param discoveredCnt [out]: Count of the devices that got an address
 * \return I3C_REQUEST_OK when all the devices were stored, I3C_REQUEST_ERROR otherwise (the count
 *         is valid when the procedure ended, a device that did not fit in the table got an
 *         address but is not stored)
 */
i3c_RequestState_t I3c_Set_DynamicAddressAssign( i3c_PeriphId_t periphId, i3c_DeviceCnt_t * const discoveredCnt )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    i3c_Context_t *    ctx      = I3C_NULL_PTR;
    i3c_RequestState_t ctxState = I3c_Get_Context( periphId, &ctx );

    if( ( I3C_REQUEST_OK == ctxState      ) &&
        ( I3C_NULL_PTR   != ctx           ) &&
        ( I3C_NULL_PTR   != discoveredCnt )    )
    {
        retState = I3cCtrl_Start_Daa( periphId );

        if( I3C_REQUEST_OK == retState )
        {
            retState = I3cCtrl_Wait_Done( periphId );
        }
        else
        {
            /* The assignment was not started, no action required */
        }

        if( I3C_REQUEST_OK == retState )
        {
            *discoveredCnt = ctx->DaaCnt;
            retState       = I3c_Set_DeviceRegs( periphId );

            if( I3C_FUNCTION_ACTIVE == ctx->DaaOverflow )
            {
                retState = I3C_REQUEST_ERROR;
            }
            else
            {
                /* All the devices fit in the table, no action required */
            }
        }
        else
        {
            /* The assignment failed, no action required */
        }
    }
    else
    {
        /* Invalid parameter */
        retState = I3C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Gets the device table of the controller.
 *
 * \param periphId  [in]: I3C peripheral ID
 * \param devices   [out]: Buffer for the entries, at least I3C_DEVICE_CNT_MAX entries
 * \param deviceCnt [out]: Count of the valid entries
 * \return I3C_REQUEST_OK when the table was read, I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3c_Get_DeviceTable( i3c_PeriphId_t periphId,
                                        i3c_Device_t * const devices,
                                        i3c_DeviceCnt_t * const deviceCnt )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    i3c_Context_t *    ctx      = I3C_NULL_PTR;
    i3c_RequestState_t ctxState = I3c_Get_Context( periphId, &ctx );

    if( ( I3C_REQUEST_OK == ctxState  ) &&
        ( I3C_NULL_PTR   != ctx       ) &&
        ( I3C_NULL_PTR   != devices   ) &&
        ( I3C_NULL_PTR   != deviceCnt )    )
    {
        if( ( I3C_FUNCTION_ACTIVE == ctx->InitState   ) &&
            ( I3C_ROLE_CONTROLLER == ctx->Config.Role )    )
        {
            for( i3c_DeviceCnt_t idx = 0u; ctx->DeviceCnt > idx; idx++ )
            {
                devices[ idx ] = ctx->Devices[ idx ];
            }

            *deviceCnt = ctx->DeviceCnt;
            retState   = I3C_REQUEST_OK;
        }
        else
        {
            /* The instance is not initialized as controller */
            retState = I3C_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid parameter */
        retState = I3C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Sets the device table of the controller: the I3C devices (with their dynamic address,
 *        PID for the assignment, in-band interrupt acceptance) and the legacy I2C devices (with
 *        their static address and speed class). The open-drain timing is recomputed for the
 *        slowest legacy device, the peripheral is disabled and enabled again to write it.
 *
 * \param periphId  [in]: I3C peripheral ID
 * \param devices   [in]: Entries of the table
 * \param deviceCnt [in]: Count of the entries (at most I3C_DEVICE_CNT_MAX)
 * \return I3C_REQUEST_OK when the table was stored, I3C_REQUEST_ERROR otherwise (nothing is changed)
 */
i3c_RequestState_t I3c_Set_DeviceTable( i3c_PeriphId_t periphId,
                                        const i3c_Device_t * const devices,
                                        i3c_DeviceCnt_t deviceCnt )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    i3c_Context_t *    ctx      = I3C_NULL_PTR;
    i3c_RequestState_t ctxState = I3c_Get_Context( periphId, &ctx );
    i3c_Timing_t       timing   = { 0u, 0u, 0u, 0u };
    i3c_I2cSpeed_t     i2cClass = I3C_I2C_SPEED_CNT;

    if( ( I3C_REQUEST_OK == ctxState ) &&
        ( I3C_NULL_PTR   != ctx      )    )
    {
        if( ( I3C_FUNCTION_ACTIVE == ctx->InitState   ) &&
            ( I3C_ROLE_CONTROLLER == ctx->Config.Role ) &&
            ( I3C_XFER_STATE_BUSY != ctx->XferState   )    )
        {
            retState = I3cPriv_Check_Devices( ctx, devices, deviceCnt );
        }
        else
        {
            /* The instance is not initialized as controller or a request is running */
            retState = I3C_REQUEST_ERROR;
        }

        if( I3C_REQUEST_OK == retState )
        {
            i2cClass = I3cPriv_Get_SlowestI2c( devices, deviceCnt );
            retState = I3cPriv_Calc_Timing( ctx->KernelClk, &ctx->Config, i2cClass, &timing );
        }
        else
        {
            /* The table is not valid, no action required */
        }

        if( ( I3C_REQUEST_OK == retState    ) &&
            ( ctx->I2cClass  != i2cClass    )    )
        {
            retState = I3c_Reg_Set_Disable( periphId );

            if( I3C_REQUEST_OK == retState )
            {
                I3c_Reg_Set_Raw( periphId, I3C_REG_TIMINGR0, timing.Timingr0 );
                retState = I3c_Reg_Set_Enable( periphId );
            }
            else
            {
                /* The peripheral did not stop, no register is written */
            }

            if( I3C_REQUEST_OK == retState )
            {
                ctx->PushPullFreq  = timing.PushPullFreq;
                ctx->OpenDrainFreq = timing.OpenDrainFreq;
                ctx->I2cClass      = i2cClass;
            }
            else
            {
                /* The timing was not written, no action required */
            }
        }
        else
        {
            /* The timing does not change or the table is not valid, no action required */
        }

        if( I3C_REQUEST_OK == retState )
        {
            ctx->DeviceCnt = deviceCnt;

            for( i3c_DeviceCnt_t idx = 0u; I3C_DEVICE_CNT_MAX > idx; idx++ )
            {
                if( deviceCnt > idx )
                {
                    ctx->Devices[ idx ] = devices[ idx ];
                }
                else
                {
                    ctx->Devices[ idx ] = (i3c_Device_t){ 0 };
                }
            }

            retState = I3c_Set_DeviceRegs( periphId );
        }
        else
        {
            /* The table was not accepted, no action required */
        }
    }
    else
    {
        /* Invalid parameter */
        retState = I3C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Gets the own dynamic address of the instance: the configured address of the controller,
 *        the address assigned by the controller to a target.
 *
 * \param periphId [in]: I3C peripheral ID
 * \param address  [out]: Own dynamic address
 * \return I3C_REQUEST_OK when the address is valid, I3C_REQUEST_ERROR otherwise (a target without
 *         an assigned address)
 */
i3c_RequestState_t I3c_Get_OwnAddress( i3c_PeriphId_t periphId, i3c_Address_t * const address )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    i3c_Context_t *    ctx      = I3C_NULL_PTR;
    i3c_RequestState_t ctxState = I3c_Get_Context( periphId, &ctx );
    uint32_t           devr0    = 0u;

    if( ( I3C_REQUEST_OK == ctxState ) &&
        ( I3C_NULL_PTR   != ctx      ) &&
        ( I3C_NULL_PTR   != address  )    )
    {
        if( I3C_FUNCTION_ACTIVE == ctx->InitState )
        {
            devr0 = I3c_Reg_Get_Val( periphId, I3C_REG_DEVR0, ( I3C_DEVR0_DAVAL | I3C_DEVR0_DA ) );

            if( 0u != ( devr0 & I3C_DEVR0_DAVAL ) )
            {
                *address = (i3c_Address_t)( ( devr0 & I3C_DEVR0_DA ) >> I3C_DEVR0_DA_Pos );
                retState = I3C_REQUEST_OK;
            }
            else
            {
                /* No dynamic address is assigned */
                retState = I3C_REQUEST_ERROR;
            }
        }
        else
        {
            /* The instance is not initialized */
            retState = I3C_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid parameter */
        retState = I3C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Sets the priority of the interrupts of an instance (event and error line).
 *
 * \param periphId [in]: I3C peripheral ID
 * \param irqPrio  [in]: Interrupt priority
 * \return I3C_REQUEST_OK when the priority was set, I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3c_Set_IrqPriority( i3c_PeriphId_t periphId, i3c_IrqPrio_t irqPrio )
{
    i3c_RequestState_t   retState  = I3C_REQUEST_ERROR;
    i3c_Context_t *      ctx       = I3C_NULL_PTR;
    i3c_RequestState_t   ctxState  = I3c_Get_Context( periphId, &ctx );
    nvic_RequestState_t  nvicState = NVIC_REQUEST_ERROR;

    if( ( I3C_REQUEST_OK == ctxState ) &&
        ( I3C_NULL_PTR   != ctx      )    )
    {
        if( I3C_FUNCTION_ACTIVE == ctx->InitState )
        {
            nvicState = Nvic_Set_PeriphIrq_Prio( i3c_IrqList[ periphId ][ 0u ], irqPrio );

            if( NVIC_REQUEST_OK == nvicState )
            {
                nvicState = Nvic_Set_PeriphIrq_Prio( i3c_IrqList[ periphId ][ 1u ], irqPrio );
            }
            else
            {
                /* The event line failed, no action required */
            }

            if( NVIC_REQUEST_OK == nvicState )
            {
                ctx->Config.IrqPriority = irqPrio;
                retState                = I3C_REQUEST_OK;
            }
            else
            {
                retState = I3C_REQUEST_ERROR;
            }
        }
        else
        {
            /* The instance is not initialized */
            retState = I3C_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid peripheral ID */
        retState = I3C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Gets the priority of the interrupts of an instance (event line).
 *
 * \param periphId [in]: I3C peripheral ID
 * \param irqPrio  [out]: Interrupt priority
 * \return I3C_REQUEST_OK when the priority was read, I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3c_Get_IrqPriority( i3c_PeriphId_t periphId, i3c_IrqPrio_t * const irqPrio )
{
    i3c_RequestState_t  retState  = I3C_REQUEST_ERROR;
    i3c_Context_t *     ctx       = I3C_NULL_PTR;
    i3c_RequestState_t  ctxState  = I3c_Get_Context( periphId, &ctx );
    nvic_RequestState_t nvicState = NVIC_REQUEST_ERROR;
    nvic_IrqPrio_t      prio      = 0u;

    if( ( I3C_REQUEST_OK == ctxState ) &&
        ( I3C_NULL_PTR   != ctx      ) &&
        ( I3C_NULL_PTR   != irqPrio  )    )
    {
        if( I3C_FUNCTION_ACTIVE == ctx->InitState )
        {
            nvicState = Nvic_Get_PeriphIrq_Prio( i3c_IrqList[ periphId ][ 0u ], &prio );

            if( NVIC_REQUEST_OK == nvicState )
            {
                *irqPrio = (i3c_IrqPrio_t)prio;
                retState = I3C_REQUEST_OK;
            }
            else
            {
                retState = I3C_REQUEST_ERROR;
            }
        }
        else
        {
            /* The instance is not initialized */
            retState = I3C_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid parameter */
        retState = I3C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Initializes the SCL GPIO pin used by the peripheral (alternate function, no pull).
 *
 * \param pinId [in]: SCL pin identification
 * \return I3C_REQUEST_OK if the request succeeded, otherwise I3C_REQUEST_ERROR
 */
i3c_RequestState_t I3c_InitSclGpio( i3c_SclPin_t pinId )
{
    i3c_RequestState_t  retValue      = I3C_REQUEST_ERROR;
    gpio_RequestState_t gpioInitState = GPIO_REQUEST_ERROR;
    gpio_Config_t       pinConfig     = { 0u };

    pinConfig.PortId         = I3C_BIT_MASK_DECODE_PORT( pinId );
    pinConfig.PinId          = I3C_BIT_MASK_DECODE_PIN( pinId );
    pinConfig.PinMode        = GPIO_PIN_MODE_ALTERNATE;
    pinConfig.PinPull        = GPIO_PIN_PULL_NONE;
    pinConfig.PinSpeed       = GPIO_PIN_SPEED_VERY_HIGH;
    pinConfig.PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
    pinConfig.PinAltFunction = I3C_BIT_MASK_DECODE_AF( pinId );

    gpioInitState = Gpio_Init( &pinConfig );

    if( GPIO_REQUEST_ERROR != gpioInitState )
    {
        retValue = I3C_REQUEST_OK;
    }
    else
    {
        retValue = I3C_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Initializes the SDA GPIO pin used by the peripheral (alternate function, no pull).
 *
 * \param pinId [in]: SDA pin identification
 * \return I3C_REQUEST_OK if the request succeeded, otherwise I3C_REQUEST_ERROR
 */
i3c_RequestState_t I3c_InitSdaGpio( i3c_SdaPin_t pinId )
{
    i3c_RequestState_t  retValue      = I3C_REQUEST_ERROR;
    gpio_RequestState_t gpioInitState = GPIO_REQUEST_ERROR;
    gpio_Config_t       pinConfig     = { 0u };

    pinConfig.PortId         = I3C_BIT_MASK_DECODE_PORT( pinId );
    pinConfig.PinId          = I3C_BIT_MASK_DECODE_PIN( pinId );
    pinConfig.PinMode        = GPIO_PIN_MODE_ALTERNATE;
    pinConfig.PinPull        = GPIO_PIN_PULL_NONE;
    pinConfig.PinSpeed       = GPIO_PIN_SPEED_VERY_HIGH;
    pinConfig.PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
    pinConfig.PinAltFunction = I3C_BIT_MASK_DECODE_AF( pinId );

    gpioInitState = Gpio_Init( &pinConfig );

    if( GPIO_REQUEST_ERROR != gpioInitState )
    {
        retValue = I3C_REQUEST_OK;
    }
    else
    {
        retValue = I3C_REQUEST_ERROR;
    }

    return ( retValue );
}


/* ===================== SERVICES FOR THE MODULE FILES ======================= */

/**
 * \brief Gets the context of an instance.
 *
 * \param periphId [in]: I3C peripheral ID
 * \param ctx      [out]: Pointer to the context
 * \return I3C_REQUEST_OK for a valid peripheral ID, I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3c_Get_Context( i3c_PeriphId_t periphId, i3c_Context_t ** const ctx )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;

    if( ( I3C_PERIPH_CNT > periphId ) &&
        ( I3C_NULL_PTR   != ctx     )    )
    {
        *ctx     = &i3c_CtxList[ periphId ];
        retState = I3C_REQUEST_OK;
    }
    else
    {
        /* Invalid parameter */
        retState = I3C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Decodes the status error register (I3C_SER) into the error bits of the module.
 *
 * \param serValue [in]: Value of I3C_SER
 * \return Bit mask of \ref i3c_XferErrorId_t items
 */
i3c_XferError_t I3c_Get_ErrorMask( uint32_t serValue )
{
    uint32_t errorMask = (uint32_t)I3C_XFER_ERROR_NONE;
    uint32_t codeErr   = ( serValue & I3C_SER_CODERR_Msk ) >> I3C_SER_CODERR_Pos;
    uint32_t protError = (uint32_t)I3C_XFER_ERROR_FRAME;

    if( 0u != ( serValue & I3C_SER_ANACK ) )
    {
        errorMask |= (uint32_t)I3C_XFER_ERROR_ADDR_NACK;
    }
    else
    {
        /* The address was acknowledged, no action required */
    }

    if( 0u != ( serValue & I3C_SER_DNACK ) )
    {
        errorMask |= (uint32_t)I3C_XFER_ERROR_DATA_NACK;
    }
    else
    {
        /* The data was acknowledged, no action required */
    }

    if( 0u != ( serValue & I3C_SER_DOVR ) )
    {
        errorMask |= ( (uint32_t)I3C_XFER_ERROR_OVERRUN | (uint32_t)I3C_XFER_ERROR_UNDERRUN );
    }
    else
    {
        /* No data FIFO overrun or underrun, no action required */
    }

    if( 0u != ( serValue & I3C_SER_COVR ) )
    {
        errorMask |= (uint32_t)I3C_XFER_ERROR_UNDERRUN;
    }
    else
    {
        /* No control FIFO underrun, no action required */
    }

    if( 0u != ( serValue & ( I3C_SER_STALL | I3C_SER_DERR ) ) )
    {
        errorMask |= (uint32_t)I3C_XFER_ERROR_BUS;
    }
    else
    {
        /* No stalled SCL or hand-off error, no action required */
    }

    if( 0u != ( serValue & I3C_SER_PERR ) )
    {
        for( uint32_t idx = 0u; ( sizeof( i3c_CodeErrorList ) / sizeof( i3c_CodeErrorList[ 0u ] ) ) > idx; idx++ )
        {
            if( codeErr == i3c_CodeErrorList[ idx ].Code )
            {
                protError = (uint32_t)i3c_CodeErrorList[ idx ].Error;
            }
            else
            {
                /* Another code, no action required */
            }
        }

        errorMask |= protError;
    }
    else
    {
        /* No protocol error, no action required */
    }

    return ( (i3c_XferError_t)errorMask );
}


/**
 * \brief Ends the running request of an instance: the state and the errors are stored and, when
 *        nobody waits for the request, the callback is called.
 *
 * \param periphId  [in]: I3C peripheral ID
 * \param xferState [in]: New request state (IDLE: success, ERROR: failure)
 * \param errorMask [in]: Errors of the request
 * \param xferCnt   [in]: Count of the transferred data bytes
 */
void I3c_Set_XferEnd( i3c_PeriphId_t periphId,
                      i3c_XferState_t xferState,
                      i3c_XferError_t errorMask,
                      i3c_DataCnt_t xferCnt )
{
    i3c_Context_t *    ctx      = I3C_NULL_PTR;
    i3c_RequestState_t ctxState = I3c_Get_Context( periphId, &ctx );
    i3c_FunctionState_t blocking = I3C_FUNCTION_INACTIVE;

    if( ( I3C_REQUEST_OK == ctxState ) &&
        ( I3C_NULL_PTR   != ctx      )    )
    {
        blocking         = ctx->ReqBlocking;
        ctx->ReqKind     = I3C_REQ_KIND_NONE;
        ctx->XferError   = errorMask;
        ctx->XferState   = xferState;
        ctx->ReqBlocking = I3C_FUNCTION_INACTIVE;

        if( I3C_FUNCTION_INACTIVE == blocking )
        {
            if( ( I3C_XFER_STATE_IDLE == xferState                  ) &&
                ( I3C_NULL_PTR != ctx->Config.XferCompleteCallback  )    )
            {
                ctx->Config.XferCompleteCallback( periphId, xferCnt );
            }
            else
            {
                /* The request failed or no completion callback is set, no action required */
            }

            if( ( I3C_XFER_STATE_ERROR == xferState           ) &&
                ( I3C_NULL_PTR != ctx->Config.ErrorCallback   )    )
            {
                ctx->Config.ErrorCallback( periphId, errorMask );
            }
            else
            {
                /* The request succeeded or no error callback is set, no action required */
            }
        }
        else
        {
            /* The caller waits for the request and reads its state, no action required */
        }
    }
    else
    {
        /* Invalid peripheral ID, no action required */
    }
}


/**
 * \brief Finds the device of the table with the given address.
 *
 * \param ctx     [in]: Context of the instance
 * \param address [in]: Dynamic address of an I3C device or static address of an I2C device
 * \param device  [out]: Pointer to the entry of the table
 * \return I3C_REQUEST_OK when the device was found, I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3c_Get_Address( i3c_Context_t * const ctx, i3c_Address_t address, i3c_Device_t ** const device )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;

    if( ( I3C_NULL_PTR != ctx    ) &&
        ( I3C_NULL_PTR != device )    )
    {
        for( i3c_DeviceCnt_t idx = 0u;
             ( ctx->DeviceCnt > idx ) &&
             ( I3C_REQUEST_OK != retState );
             idx++ )
        {
            if( ( I3C_DEVICE_I3C == ctx->Devices[ idx ].Type                ) &&
                ( I3C_FUNCTION_ACTIVE == ctx->Devices[ idx ].AddressState   ) &&
                ( address == ctx->Devices[ idx ].DynamicAddress             )    )
            {
                *device  = &ctx->Devices[ idx ];
                retState = I3C_REQUEST_OK;
            }
            else
            {
                if( ( I3C_DEVICE_I2C == ctx->Devices[ idx ].Type          ) &&
                    ( address == ctx->Devices[ idx ].StaticAddress        )    )
                {
                    *device  = &ctx->Devices[ idx ];
                    retState = I3C_REQUEST_OK;
                }
                else
                {
                    /* Another device, no action required */
                }
            }
        }
    }
    else
    {
        /* Invalid parameter */
        retState = I3C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Programs the target device registers (I3C_DEVR1 .. I3C_DEVR4) from the device table:
 *        the dynamic address, the acknowledge of the in-band interrupt and of its payload. An
 *        entry that is not an I3C device with a valid address is cleared.
 *
 * \param periphId [in]: I3C peripheral ID
 * \return I3C_REQUEST_OK when the registers were written, I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3c_Set_DeviceRegs( i3c_PeriphId_t periphId )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    i3c_Context_t *    ctx      = I3C_NULL_PTR;
    i3c_RequestState_t ctxState = I3c_Get_Context( periphId, &ctx );
    uint32_t           devrVal  = 0u;

    if( ( I3C_REQUEST_OK == ctxState ) &&
        ( I3C_NULL_PTR   != ctx      )    )
    {
        for( uint32_t idx = 0u; (uint32_t)I3C_DEVR_CNT > idx; idx++ )
        {
            devrVal = 0u;

            if( ( ctx->DeviceCnt > idx                                  ) &&
                ( I3C_DEVICE_I3C == ctx->Devices[ idx ].Type            ) &&
                ( I3C_FUNCTION_ACTIVE == ctx->Devices[ idx ].AddressState ) )
            {
                devrVal = ( (uint32_t)ctx->Devices[ idx ].DynamicAddress << I3C_DEVRX_DA_Pos );

                if( I3C_FUNCTION_ACTIVE == ctx->Devices[ idx ].IbiState )
                {
                    devrVal |= I3C_DEVRX_IBIACK;
                }
                else
                {
                    /* The in-band interrupt of the device is not accepted, no action required */
                }

                if( I3C_FUNCTION_ACTIVE == ctx->Devices[ idx ].IbiPayload )
                {
                    devrVal |= I3C_DEVRX_IBIDEN;
                }
                else
                {
                    /* The in-band interrupt has no payload, no action required */
                }
            }
            else
            {
                /* The entry is not an I3C device with an address, the register is cleared */
            }

            I3c_Reg_Set_Raw( periphId, (i3c_RegId_t)( (uint32_t)I3C_REG_DEVR1 + idx ), devrVal );
        }

        retState = I3C_REQUEST_OK;
    }
    else
    {
        /* Invalid peripheral ID */
        retState = I3C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Stores a device found by the address assignment in the device table and gets its address.
 *
 * The payload is the PID (6 bytes, most significant first), the BCR and the DCR. A table entry
 * with the same PID gets its own address if it is free, otherwise (and for a new device) the
 * address is the first free address of the pool. When the table is full the device is not
 * stored, the address is still returned (the device has to be assigned an address because the
 * procedure cannot skip it).
 *
 * \param ctx     [in]: Context of the instance
 * \param payload [in]: Eight bytes received in the assignment
 * \param address [out]: Dynamic address to be assigned to the device
 * \return I3C_REQUEST_OK when the device is stored in the table, I3C_REQUEST_ERROR when the table is full
 */
i3c_RequestState_t I3c_Set_DaaDevice( i3c_Context_t * const ctx,
                                      const i3c_Data_t * const payload,
                                      i3c_Address_t * const address )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    i3c_PidHigh_t      pidHigh  = 0u;
    i3c_PidLow_t       pidLow   = 0u;
    i3c_DeviceCnt_t    entryIdx = I3C_DEVICE_CNT_MAX;
    i3c_Address_t      newAddr  = 0u;
    i3c_Address_t      wanted   = 0u;
    i3c_FunctionState_t wantedFree = I3C_FUNCTION_INACTIVE;

    if( ( I3C_NULL_PTR != ctx     ) &&
        ( I3C_NULL_PTR != payload ) &&
        ( I3C_NULL_PTR != address )    )
    {
        for( uint32_t idx = 0u; (uint32_t)I3C_PID_HIGH_BYTE_CNT > idx; idx++ )
        {
            pidHigh = (i3c_PidHigh_t)( ( (uint32_t)pidHigh << I3C_BYTE_BIT_CNT ) | (uint32_t)payload[ idx ] );
        }

        for( uint32_t idx = 0u; (uint32_t)I3C_PID_LOW_BYTE_CNT > idx; idx++ )
        {
            pidLow = ( ( pidLow << I3C_BYTE_BIT_CNT ) |
                       (i3c_PidLow_t)payload[ (uint32_t)I3C_PID_HIGH_BYTE_CNT + idx ] );
        }

        for( i3c_DeviceCnt_t idx = 0u;
             ( ctx->DeviceCnt > idx ) &&
             ( I3C_DEVICE_CNT_MAX == entryIdx );
             idx++ )
        {
            if( ( I3C_DEVICE_I3C == ctx->Devices[ idx ].Type  ) &&
                ( pidHigh == ctx->Devices[ idx ].PidHigh      ) &&
                ( pidLow  == ctx->Devices[ idx ].PidLow       )    )
            {
                entryIdx = idx;
            }
            else
            {
                /* Another device, no action required */
            }
        }

        if( I3C_DEVICE_CNT_MAX == entryIdx )
        {
            if( I3C_DEVICE_CNT_MAX > ctx->DeviceCnt )
            {
                entryIdx = ctx->DeviceCnt;
                ctx->Devices[ entryIdx ]       = (i3c_Device_t){ 0 };
                ctx->Devices[ entryIdx ].Type  = I3C_DEVICE_I3C;
                ctx->DeviceCnt++;
            }
            else
            {
                /* The table is full, the device is not stored */
                retState = I3C_REQUEST_ERROR;
            }
        }
        else
        {
            wanted     = ctx->Devices[ entryIdx ].DynamicAddress;
            wantedFree = I3cPriv_Get_AddressFree( ctx, wanted, entryIdx );
        }

        if( ( I3C_DEVICE_CNT_MAX != entryIdx           ) &&
            ( I3C_FUNCTION_ACTIVE == wantedFree        )    )
        {
            newAddr = wanted;
        }
        else
        {
            newAddr = I3cPriv_Get_FreeAddress( ctx, entryIdx );
        }

        if( I3C_DEVICE_CNT_MAX != entryIdx )
        {
            ctx->Devices[ entryIdx ].PidHigh        = pidHigh;
            ctx->Devices[ entryIdx ].PidLow         = pidLow;
            ctx->Devices[ entryIdx ].Bcr            = payload[ I3C_PID_BYTE_CNT ];
            ctx->Devices[ entryIdx ].Dcr            = payload[ I3C_PID_BYTE_CNT + 1u ];
            ctx->Devices[ entryIdx ].DynamicAddress = newAddr;
            ctx->Devices[ entryIdx ].AddressState   = I3C_FUNCTION_ACTIVE;

            if( 0u != ( payload[ I3C_PID_BYTE_CNT ] & I3C_BCR_IBI_PAYLOAD ) )
            {
                ctx->Devices[ entryIdx ].IbiPayload = I3C_FUNCTION_ACTIVE;
            }
            else
            {
                ctx->Devices[ entryIdx ].IbiPayload = I3C_FUNCTION_INACTIVE;
            }

            retState = I3C_REQUEST_OK;
        }
        else
        {
            /* The device is not stored, the table is full */
            retState = I3C_REQUEST_ERROR;
        }

        *address = newAddr;
    }
    else
    {
        /* Invalid parameter */
        retState = I3C_REQUEST_ERROR;
    }

    return ( retState );
}


/* ============================ LOCAL FUNCTIONS ============================== */

/**
 * \brief Checks the configuration of the module before any register is written.
 *
 * \param i3cConfig [in]: Module configuration
 * \return I3C_REQUEST_OK when the configuration is valid, I3C_REQUEST_ERROR otherwise
 */
static i3c_RequestState_t I3cPriv_Check_Config( const i3c_Config_t * const i3cConfig )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;

    if( ( I3C_PERIPH_CNT   >  i3cConfig->PeriphId         ) &&
        ( I3C_ROLE_CNT     >  i3cConfig->Role             ) &&
        ( I3C_XFER_MODE_CNT > i3cConfig->XferMode         ) &&
        ( 0u               <  i3cConfig->BusAvailableTime )    )
    {
        retState = I3cPriv_Check_Clock( i3cConfig );
    }
    else
    {
        /* Invalid instance, role, transfer mode or bus time */
        retState = I3C_REQUEST_ERROR;
    }

    if( I3C_REQUEST_OK == retState )
    {
        retState = I3cPriv_Check_Pins( i3cConfig );
    }
    else
    {
        /* The clock source is not valid, no action required */
    }

    if( ( I3C_REQUEST_OK == retState                ) &&
        ( I3C_XFER_MODE_DMA == i3cConfig->XferMode  )    )
    {
        retState = I3cPriv_Check_Dma( i3cConfig );
    }
    else
    {
        /* DMA is not used or a previous check failed, no action required */
    }

    if( ( I3C_REQUEST_OK == retState                   ) &&
        ( I3C_ROLE_CONTROLLER == i3cConfig->Role       )    )
    {
        if( ( 0u                      <  i3cConfig->PushPullFreq  ) &&
            ( I3C_PUSH_PULL_FREQ_MAX  >= i3cConfig->PushPullFreq  ) &&
            ( 0u                      <  i3cConfig->OpenDrainFreq ) &&
            ( 0u                      <  i3cConfig->BusFreeTime   ) &&
            ( I3C_ADDRESS_MASK        >= i3cConfig->OwnAddress    ) &&
            ( I3C_DYNAMIC_ADDRESS_FIRST <= i3cConfig->OwnAddress  )    )
        {
            retState = I3C_REQUEST_OK;
        }
        else
        {
            /* Invalid frequency, bus free time or own address */
            retState = I3C_REQUEST_ERROR;
        }
    }
    else
    {
        /* The target role has no controller parameters, no action required */
    }

    return ( retState );
}


/**
 * \brief Checks that the kernel clock source is one of the sources of the selected instance.
 *
 * \param i3cConfig [in]: Module configuration
 * \return I3C_REQUEST_OK when the source is valid for the instance, I3C_REQUEST_ERROR otherwise
 */
static i3c_RequestState_t I3cPriv_Check_Clock( const i3c_Config_t * const i3cConfig )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;

    for( uint32_t idx = 0u; (uint32_t)I3C_CLK_SRC_PER_PERIPH_CNT > idx; idx++ )
    {
        if( i3cConfig->ClkSrc == i3c_ClkSrcList[ i3cConfig->PeriphId ][ idx ] )
        {
            retState = I3C_REQUEST_OK;
        }
        else
        {
            /* Another source, no action required */
        }
    }

    return ( retState );
}


/**
 * \brief Checks that the pins belong to the selected instance (or are unused).
 *
 * \param i3cConfig [in]: Module configuration
 * \return I3C_REQUEST_OK when the pins are valid, I3C_REQUEST_ERROR otherwise
 */
static i3c_RequestState_t I3cPriv_Check_Pins( const i3c_Config_t * const i3cConfig )
{
    i3c_RequestState_t retState = I3cPriv_Check_Pin( (uint32_t)i3cConfig->SclPin,
                                                     (uint32_t)I3C_SCL_PIN_UNUSED,
                                                     i3cConfig->PeriphId );

    if( I3C_REQUEST_OK == retState )
    {
        retState = I3cPriv_Check_Pin( (uint32_t)i3cConfig->SdaPin, (uint32_t)I3C_SDA_PIN_UNUSED, i3cConfig->PeriphId );
    }
    else
    {
        /* The SCL pin is not valid, no action required */
    }

    return ( retState );
}


/**
 * \brief Checks that a pin belongs to the selected instance or is unused.
 *
 * \param pinValue    [in]: Encoded pin
 * \param unusedValue [in]: Value of the unused pin item
 * \param periphId    [in]: I3C peripheral ID
 * \return I3C_REQUEST_OK when the pin is valid, I3C_REQUEST_ERROR otherwise
 */
static i3c_RequestState_t I3cPriv_Check_Pin( uint32_t pinValue, uint32_t unusedValue, i3c_PeriphId_t periphId )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    const uint32_t     pinInst  = I3C_BIT_MASK_DECODE_PERIPH( pinValue );

    if( unusedValue == pinValue )
    {
        retState = I3C_REQUEST_OK;
    }
    else
    {
        if( (uint32_t)periphId == pinInst )
        {
            retState = I3C_REQUEST_OK;
        }
        else
        {
            /* A pin of another instance */
            retState = I3C_REQUEST_ERROR;
        }
    }

    return ( retState );
}


/**
 * \brief Checks the DMA channels of the data FIFOs (valid ids, different channels).
 *
 * \param i3cConfig [in]: Module configuration
 * \return I3C_REQUEST_OK when the channels are valid, I3C_REQUEST_ERROR otherwise
 */
static i3c_RequestState_t I3cPriv_Check_Dma( const i3c_Config_t * const i3cConfig )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;

    if( ( GPDMA_PERIPH_CNT   > i3cConfig->TxDmaPeriphId  ) &&
        ( GPDMA_CHANNEL_CNT  > i3cConfig->TxDmaChannelId ) &&
        ( GPDMA_PRIORITY_CNT > i3cConfig->TxDmaPriority  ) &&
        ( GPDMA_PERIPH_CNT   > i3cConfig->RxDmaPeriphId  ) &&
        ( GPDMA_CHANNEL_CNT  > i3cConfig->RxDmaChannelId ) &&
        ( GPDMA_PRIORITY_CNT > i3cConfig->RxDmaPriority  )    )
    {
        if( ( i3cConfig->TxDmaPeriphId  != i3cConfig->RxDmaPeriphId  ) ||
            ( i3cConfig->TxDmaChannelId != i3cConfig->RxDmaChannelId )    )
        {
            retState = I3C_REQUEST_OK;
        }
        else
        {
            /* The same channel for both FIFOs */
            retState = I3C_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid DMA peripheral, channel or priority */
        retState = I3C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks the entries of a device table: types, addresses (valid, not reserved, not own,
 *        unique), speed classes, no I3C device without a valid dynamic address.
 *
 * \param ctx       [in]: Context of the instance
 * \param devices   [in]: Entries of the table
 * \param deviceCnt [in]: Count of the entries
 * \return I3C_REQUEST_OK when the table is valid, I3C_REQUEST_ERROR otherwise
 */
static i3c_RequestState_t I3cPriv_Check_Devices( const i3c_Context_t * const ctx,
                                                 const i3c_Device_t * const devices,
                                                 i3c_DeviceCnt_t deviceCnt )
{
    i3c_RequestState_t retState = I3C_REQUEST_OK;
    i3c_Address_t      address  = 0u;
    i3c_Address_t      other    = 0u;
    i3c_FunctionState_t addrValid = I3C_FUNCTION_INACTIVE;

    if( I3C_DEVICE_CNT_MAX < deviceCnt )
    {
        retState = I3C_REQUEST_ERROR;
    }
    else
    {
        /* The count fits in the table, no action required */
    }

    if( ( 0u < deviceCnt        ) &&
        ( I3C_NULL_PTR == devices )    )
    {
        retState = I3C_REQUEST_ERROR;
    }
    else
    {
        /* The entries are provided or none are expected, no action required */
    }

    for( i3c_DeviceCnt_t idx = 0u;
         ( deviceCnt > idx ) &&
         ( I3C_REQUEST_OK == retState );
         idx++ )
    {
        if( I3C_DEVICE_I2C == devices[ idx ].Type )
        {
            address = devices[ idx ].StaticAddress;
        }
        else
        {
            address = devices[ idx ].DynamicAddress;
        }

        addrValid = I3cPriv_Get_AddressValid( ctx, address );

        if( ( I3C_DEVICE_TYPE_CNT >  devices[ idx ].Type                         ) &&
            ( I3C_I2C_SPEED_CNT   >  devices[ idx ].I2cSpeed                     ) &&
            ( I3C_ADDRESS_MASK    >= address                                     ) &&
            ( I3C_DYNAMIC_ADDRESS_FIRST <= address                               ) &&
            ( ctx->Config.OwnAddress != address                                  ) &&
            ( I3C_FUNCTION_ACTIVE == addrValid                                   )    )
        {
            retState = I3C_REQUEST_OK;
        }
        else
        {
            retState = I3C_REQUEST_ERROR;
        }

        for( i3c_DeviceCnt_t idx2 = 0u;
             ( idx > idx2 ) &&
             ( I3C_REQUEST_OK == retState );
             idx2++ )
        {
            if( I3C_DEVICE_I2C == devices[ idx2 ].Type )
            {
                other = devices[ idx2 ].StaticAddress;
            }
            else
            {
                other = devices[ idx2 ].DynamicAddress;
            }

            if( other == address )
            {
                retState = I3C_REQUEST_ERROR;
            }
            else
            {
                /* The addresses differ, no action required */
            }
        }
    }

    return ( retState );
}


/**
 * \brief Enables the kernel clock of the instance and reads its frequency.
 *
 * \param ctx [in/out]: Context of the instance
 * \return I3C_REQUEST_OK when the clock runs and its frequency is known, I3C_REQUEST_ERROR otherwise
 */
static i3c_RequestState_t I3cPriv_Init_Clock( i3c_Context_t * const ctx )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    rcc_RequestState_t rccState = Rcc_Set_PeriphActive( (rcc_PeriphId_t)ctx->Config.ClkSrc );
    rcc_FreqHz_t       kernelClk = 0u;

    if( RCC_REQUEST_OK == rccState )
    {
        rccState = Rcc_Get_PeriphClk( (rcc_PeriphId_t)ctx->Config.ClkSrc, &kernelClk );
    }
    else
    {
        /* The clock did not start, no action required */
    }

    if( ( RCC_REQUEST_OK == rccState ) &&
        ( 0u             < kernelClk )    )
    {
        ctx->KernelClk = (i3c_FreqHz_t)kernelClk;
        retState       = I3C_REQUEST_OK;
    }
    else
    {
        /* The kernel clock frequency is not known */
        retState = I3C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Configures the SCL and SDA pins. For the controller the errata workaround of the first
 *        frame is applied to the SDA line.
 *
 * \param ctx [in]: Context of the instance
 * \return I3C_REQUEST_OK when the pins are ready, I3C_REQUEST_ERROR otherwise
 */
static i3c_RequestState_t I3cPriv_Init_Pins( const i3c_Context_t * const ctx )
{
    i3c_RequestState_t retState = I3C_REQUEST_OK;

    if( I3C_SCL_PIN_UNUSED != ctx->Config.SclPin )
    {
        retState = I3c_InitSclGpio( ctx->Config.SclPin );
    }
    else
    {
        /* The SCL pin is configured elsewhere, no action required */
    }

    if( ( I3C_REQUEST_OK == retState                 ) &&
        ( I3C_SDA_PIN_UNUSED != ctx->Config.SdaPin   )    )
    {
        retState = I3c_InitSdaGpio( ctx->Config.SdaPin );

        if( ( I3C_REQUEST_OK == retState                 ) &&
            ( I3C_ROLE_CONTROLLER == ctx->Config.Role    )    )
        {
            retState = I3cPriv_Init_SdaLine( ctx->Config.SdaPin );
        }
        else
        {
            /* The target role needs no workaround or the pin failed, no action required */
        }
    }
    else
    {
        /* The SDA pin is configured elsewhere or the SCL pin failed, no action required */
    }

    return ( retState );
}


/**
 * \brief Errata workaround of the unexpected first frame (bug AB#2093; ES0561 2.10.3, ES0621 2.13.3,
 *        ES0565 2.15.3): the pull-up of the SDA pin is enabled until the line reads high (bounded loop)
 *        and then disabled.
 *
 * \param sdaPin [in]: SDA pin
 * \return I3C_REQUEST_OK when the line read high, I3C_REQUEST_ERROR otherwise (the line is held low)
 */
static i3c_RequestState_t I3cPriv_Init_SdaLine( i3c_SdaPin_t sdaPin )
{
    i3c_RequestState_t  retState  = I3C_REQUEST_ERROR;
    gpio_RequestState_t gpioState = GPIO_REQUEST_ERROR;
    gpio_PinLevel_t     pinLevel  = GPIO_PIN_LEVEL_LOW;
    const gpio_PortId_t portId    = I3C_BIT_MASK_DECODE_PORT( (uint32_t)sdaPin );
    const gpio_PinId_t  pinId     = I3C_BIT_MASK_DECODE_PIN( (uint32_t)sdaPin );

    gpioState = Gpio_Set_PinPull( portId, pinId, GPIO_PIN_PULL_UP );

    for( i3c_TimeoutCnt_t idx = 0u;
         ( GPIO_REQUEST_OK == gpioState ) &&
         ( I3C_SDA_WAIT_RAW > idx ) &&
         ( GPIO_PIN_LEVEL_HIGH != pinLevel );
         idx++ )
    {
        gpioState = Gpio_Get_PinLevel( portId, pinId, &pinLevel );
    }

    if( ( GPIO_REQUEST_OK == gpioState        ) &&
        ( GPIO_PIN_LEVEL_HIGH == pinLevel     )    )
    {
        retState = I3C_REQUEST_OK;
    }
    else
    {
        /* The line does not read high */
        retState = I3C_REQUEST_ERROR;
    }

    gpioState = Gpio_Set_PinPull( portId, pinId, GPIO_PIN_PULL_NONE );

    if( GPIO_REQUEST_OK != gpioState )
    {
        retState = I3C_REQUEST_ERROR;
    }
    else
    {
        /* The pull-up is disabled, no action required */
    }

    return ( retState );
}


/**
 * \brief Registers the interrupt handler of the instance for both lines, sets the priority and
 *        enables the lines in the NVIC.
 *
 * \param ctx [in]: Context of the instance
 * \return I3C_REQUEST_OK when both lines are enabled, I3C_REQUEST_ERROR otherwise
 */
static i3c_RequestState_t I3cPriv_Init_Irq( const i3c_Context_t * const ctx )
{
    i3c_RequestState_t  retState  = I3C_REQUEST_ERROR;
    nvic_RequestState_t nvicState = NVIC_REQUEST_OK;
    i3c_PeriphId_t      periphId  = ctx->Config.PeriphId;

    for( uint32_t idx = 0u;
         ( I3C_IRQ_LINE_CNT > idx ) &&
         ( NVIC_REQUEST_OK == nvicState );
         idx++ )
    {
        nvicState = Nvic_Set_PeriphIrq_Handler( i3c_IrqList[ periphId ][ idx ], i3c_IsrList[ periphId ] );

        if( NVIC_REQUEST_OK == nvicState )
        {
            nvicState = Nvic_Set_PeriphIrq_Prio( i3c_IrqList[ periphId ][ idx ], ctx->Config.IrqPriority );
        }
        else
        {
            /* The handler was not registered, no action required */
        }

        if( NVIC_REQUEST_OK == nvicState )
        {
            nvicState = Nvic_Set_PeriphIrq_Active( i3c_IrqList[ periphId ][ idx ] );
        }
        else
        {
            /* The priority was not set, no action required */
        }
    }

    if( NVIC_REQUEST_OK == nvicState )
    {
        retState = I3C_REQUEST_OK;
    }
    else
    {
        retState = I3C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Writes the configuration registers of the disabled peripheral: the timing, the FIFO
 *        configuration, the interrupt enable bits and the own address (controller) or the target
 *        defaults. The peripheral is enabled by the caller.
 *
 * \param ctx [in/out]: Context of the instance
 * \return I3C_REQUEST_OK when the registers were written, I3C_REQUEST_ERROR otherwise (the timing
 *         cannot be represented by the registers)
 */
static i3c_RequestState_t I3cPriv_Write_Regs( i3c_Context_t * const ctx )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    i3c_PeriphId_t     periphId = ctx->Config.PeriphId;
    i3c_Timing_t       timing   = { 0u, 0u, 0u, 0u };
    uint32_t           cfgrVal  = 0u;
    uint32_t           ierVal   = 0u;

    retState = I3cPriv_Calc_Timing( ctx->KernelClk, &ctx->Config, I3C_I2C_SPEED_CNT, &timing );

    if( I3C_REQUEST_OK == retState )
    {
        if( I3C_ROLE_CONTROLLER == ctx->Config.Role )
        {
            cfgrVal = ( I3C_CFGR_CRINIT | I3C_CFGR_TMODE );
            ierVal  = I3C_IER_CTRL_MASK;

            if( I3C_FUNCTION_ACTIVE == ctx->Config.SdaHighKeeper )
            {
                cfgrVal |= I3C_CFGR_HKSDAEN;
            }
            else
            {
                /* The SDA high keeper is not used, no action required */
            }

            I3c_Reg_Set_Raw( periphId, I3C_REG_TIMINGR0, timing.Timingr0 );
            I3c_Reg_Set_Raw( periphId, I3C_REG_TIMINGR2, 0u );
            I3c_Reg_Set_Raw( periphId,
                             I3C_REG_DEVR0,
                             ( ( (uint32_t)ctx->Config.OwnAddress << I3C_DEVR0_DA_Pos ) | I3C_DEVR0_DAVAL ) );
            ctx->PushPullFreq  = timing.PushPullFreq;
            ctx->OpenDrainFreq = timing.OpenDrainFreq;
        }
        else
        {
            ierVal = I3C_IER_TGT_MASK;
            I3c_Reg_Set_Raw( periphId, I3C_REG_DEVR0, 0u );
#if defined(STM32H523xx) || \
    defined(STM32H533xx) || \
    defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H562xx) || \
    defined(STM32H563xx) || \
    defined(STM32H573xx)
            if( I3C_PERIPH_1 == periphId )
            {
                I3c_Reg_Set_Raw( periphId, I3C_REG_I2CCFGR, 0u );
            }
            else
            {
                /* I3C2 has no I2C target configuration register, no action required */
            }
#endif
        }

        I3c_Reg_Set_Raw( periphId, I3C_REG_TIMINGR1, timing.Timingr1 );

        if( I3C_XFER_MODE_DMA == ctx->Config.XferMode )
        {
            cfgrVal |= ( I3C_CFGR_TXDMAEN | I3C_CFGR_RXDMAEN );
        }
        else
        {
            /* The data FIFOs are served by the CPU, no action required */
        }

        if( I3C_XFER_MODE_POLL == ctx->Config.XferMode )
        {
            ierVal = 0u;
        }
        else
        {
            /* The events are served by the interrupt, no action required */
        }

        for( uint32_t idx = 0u; (uint32_t)I3C_DEVR_CNT > idx; idx++ )
        {
            I3c_Reg_Set_Raw( periphId, (i3c_RegId_t)( (uint32_t)I3C_REG_DEVR1 + idx ), 0u );
        }

        I3c_Reg_Set_Raw( periphId, I3C_REG_IER, ierVal );
        I3c_Reg_Set_EventsClear( periphId, I3C_REG_MASK_ALL );
        I3c_Reg_Set_Raw( periphId, I3C_REG_CFGR, cfgrVal );
    }
    else
    {
        /* The timing cannot be computed, no register is written */
    }

    return ( retState );
}


/**
 * \brief Verifies the initialization by read-back of the configuration registers.
 *
 * \param ctx [in]: Context of the instance
 * \return I3C_REQUEST_OK when the registers hold the configured values, I3C_REQUEST_ERROR otherwise
 */
static i3c_RequestState_t I3cPriv_Verify_Regs( const i3c_Context_t * const ctx )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    i3c_PeriphId_t     periphId = ctx->Config.PeriphId;
    i3c_Timing_t       timing   = { 0u, 0u, 0u, 0u };
    uint32_t           cfgrVal  = I3c_Reg_Get_Val( periphId, I3C_REG_CFGR, I3C_CFGR_VERIFY_MASK );
    uint32_t           timing0Val = I3c_Reg_Get_Val( periphId, I3C_REG_TIMINGR0, I3C_REG_MASK_ALL );
    uint32_t           timing1Val = I3c_Reg_Get_Val( periphId, I3C_REG_TIMINGR1, I3C_REG_MASK_ALL );
    uint32_t           expected = I3C_CFGR_EN;

    retState = I3cPriv_Calc_Timing( ctx->KernelClk, &ctx->Config, I3C_I2C_SPEED_CNT, &timing );

    if( I3C_ROLE_CONTROLLER == ctx->Config.Role )
    {
        expected |= ( I3C_CFGR_CRINIT | I3C_CFGR_TMODE );

        if( I3C_FUNCTION_ACTIVE == ctx->Config.SdaHighKeeper )
        {
            expected |= I3C_CFGR_HKSDAEN;
        }
        else
        {
            /* The SDA high keeper is not used, no action required */
        }
    }
    else
    {
        /* The target role has CRINIT cleared, no action required */
    }

    if( I3C_XFER_MODE_DMA == ctx->Config.XferMode )
    {
        expected |= ( I3C_CFGR_TXDMAEN | I3C_CFGR_RXDMAEN );
    }
    else
    {
        /* No DMA request of the FIFOs, no action required */
    }

    if( ( I3C_REQUEST_OK == retState         ) &&
        ( expected == cfgrVal                ) &&
        ( timing.Timingr1 == timing1Val      )    )
    {
        retState = I3C_REQUEST_OK;
    }
    else
    {
        /* A register does not hold the written value */
        retState = I3C_REQUEST_ERROR;
    }

    if( ( I3C_REQUEST_OK == retState              ) &&
        ( I3C_ROLE_CONTROLLER == ctx->Config.Role ) &&
        ( timing.Timingr0 != timing0Val           )    )
    {
        retState = I3C_REQUEST_ERROR;
    }
    else
    {
        /* The timing register holds the written value or the role has none, no action required */
    }

    return ( retState );
}


/**
 * \brief Computes the timing registers from the kernel clock, the requested frequencies and the
 *        slowest legacy I2C class on the bus.
 *
 * All the durations are rounded up to whole kernel clock cycles, so the achieved frequencies are
 * equal to or lower than the requested ones. The open-drain low time is at least 200 ns and, with
 * a legacy device on the bus, at least the low time of its class; the legacy high time is the
 * high time of its class.
 *
 * \param kernelClk  [in]: Kernel clock frequency in Hz
 * \param i3cConfig  [in]: Module configuration
 * \param i2cClass   [in]: Slowest legacy I2C class (I3C_I2C_SPEED_CNT for none)
 * \param timing     [out]: Register values and achieved frequencies
 * \return I3C_REQUEST_OK when the timing can be represented by the registers, I3C_REQUEST_ERROR
 *         otherwise (kernel clock too slow for the push-pull frequency or too fast for the
 *         durations)
 */
static i3c_RequestState_t I3cPriv_Calc_Timing( i3c_FreqHz_t kernelClk,
                                               const i3c_Config_t * const i3cConfig,
                                               i3c_I2cSpeed_t i2cClass,
                                               i3c_Timing_t * const timing )
{
    i3c_RequestState_t retState  = I3C_REQUEST_OK;
    uint32_t           totalPp   = 0u;
    uint32_t           totalOd   = 0u;
    uint32_t           high      = 0u;
    uint32_t           low       = 0u;
    uint32_t           lowOd     = 0u;
    uint32_t           highI2c   = 0u;
    uint32_t           minCycles = 0u;
    uint32_t           freeVal   = 0u;
    uint32_t           availVal  = 0u;
    uint32_t           cycles    = 0u;

    *timing = (i3c_Timing_t){ 0u, 0u, 0u, 0u };

    if( I3C_ROLE_CONTROLLER == i3cConfig->Role )
    {
        if( ( I3C_KERNEL_CLK_RATIO * i3cConfig->PushPullFreq ) <= kernelClk )
        {
            totalPp = ( ( kernelClk + i3cConfig->PushPullFreq - 1u ) / i3cConfig->PushPullFreq );
            high    = ( totalPp / I3C_HALF_DIVISOR );
            minCycles = I3cPriv_Get_Cycles( kernelClk, I3C_MIN_HIGH_TIME );

            if( minCycles > high )
            {
                high = minCycles;
            }
            else
            {
                /* The high time is long enough, no action required */
            }

            if( totalPp > high )
            {
                low = ( totalPp - high );
            }
            else
            {
                low = 0u;
            }

            minCycles = I3cPriv_Get_Cycles( kernelClk, I3C_MIN_LOW_PP_TIME );

            if( minCycles > low )
            {
                low = minCycles;
            }
            else
            {
                /* The low time is long enough, no action required */
            }

            totalOd = ( ( kernelClk + i3cConfig->OpenDrainFreq - 1u ) / i3cConfig->OpenDrainFreq );

            if( totalOd > high )
            {
                lowOd = ( totalOd - high );
            }
            else
            {
                lowOd = 0u;
            }

            minCycles = I3cPriv_Get_Cycles( kernelClk, I3C_MIN_LOW_OD_TIME );

            if( minCycles > lowOd )
            {
                lowOd = minCycles;
            }
            else
            {
                /* The open-drain low time is long enough, no action required */
            }

            highI2c = high;

            if( I3C_I2C_SPEED_CNT > i2cClass )
            {
                minCycles = I3cPriv_Get_Cycles( kernelClk, i3c_I2cTimeList[ i2cClass ].LowTime );

                if( minCycles > lowOd )
                {
                    lowOd = minCycles;
                }
                else
                {
                    /* The low time of the legacy class is met, no action required */
                }

                highI2c = I3cPriv_Get_Cycles( kernelClk, i3c_I2cTimeList[ i2cClass ].HighTime );
            }
            else
            {
                /* No legacy I2C device on the bus, no action required */
            }

            if( ( I3C_SCL_CYCLES_MAX < high    ) ||
                ( I3C_SCL_CYCLES_MAX < low     ) ||
                ( I3C_SCL_CYCLES_MAX < lowOd   ) ||
                ( I3C_SCL_CYCLES_MAX < highI2c )    )
            {
                retState = I3C_REQUEST_ERROR;
            }
            else
            {
                timing->Timingr0 = ( ( ( highI2c - 1u ) << I3C_TIMINGR0_SCLH_I2C_Pos ) |
                                     ( ( lowOd   - 1u ) << I3C_TIMINGR0_SCLL_OD_Pos  ) |
                                     ( ( high    - 1u ) << I3C_TIMINGR0_SCLH_I3C_Pos ) |
                                     ( ( low     - 1u ) << I3C_TIMINGR0_SCLL_PP_Pos  ) );
                timing->PushPullFreq  = ( kernelClk / ( high + low   ) );
                timing->OpenDrainFreq = ( kernelClk / ( high + lowOd ) );
            }

            cycles  = I3cPriv_Get_Cycles( kernelClk, i3cConfig->BusFreeTime );
            freeVal = ( ( cycles + 1u ) / I3C_HALF_DIVISOR );
            minCycles = I3cPriv_Get_Cycles( kernelClk, I3C_MIN_STOP_TIME );

            if( minCycles > freeVal )
            {
                freeVal = minCycles;
            }
            else
            {
                /* The free time is long enough for the stop timing, no action required */
            }

            if( I3C_FREE_VALUE_MAX < freeVal )
            {
                retState = I3C_REQUEST_ERROR;
            }
            else
            {
                /* The free time fits the field, no action required */
            }
        }
        else
        {
            /* The kernel clock is slower than twice the push-pull frequency */
            retState = I3C_REQUEST_ERROR;
        }
    }
    else
    {
        /* The target role has no SCL timing and no free time, no action required */
    }

    cycles = I3cPriv_Get_Cycles( kernelClk, i3cConfig->BusAvailableTime );

    if( I3C_AVAL_OFFSET < cycles )
    {
        availVal = ( cycles - I3C_AVAL_OFFSET );
    }
    else
    {
        availVal = 0u;
    }

    if( I3C_AVAL_VALUE_MAX < availVal )
    {
        retState = I3C_REQUEST_ERROR;
    }
    else
    {
        timing->Timingr1 = ( ( freeVal << I3C_TIMINGR1_FREE_Pos ) | ( availVal << I3C_TIMINGR1_AVAL_Pos ) );
    }

    return ( retState );
}


/**
 * \brief Converts a time to the count of kernel clock cycles, rounded up.
 *
 * \param kernelClk [in]: Kernel clock frequency in Hz
 * \param timeNs    [in]: Time in ns
 * \return Count of the kernel clock cycles (at least one)
 */
static uint32_t I3cPriv_Get_Cycles( i3c_FreqHz_t kernelClk, i3c_TimeNs_t timeNs )
{
    const uint32_t kernelKhz = ( kernelClk / I3C_HZ_PER_KHZ );
    uint32_t       cycles    = ( ( ( kernelKhz * timeNs ) + ( I3C_NS_PER_MS - 1u ) ) / I3C_NS_PER_MS );

    if( 0u == cycles )
    {
        cycles = 1u;
    }
    else
    {
        /* At least one cycle, no action required */
    }

    return ( cycles );
}


/**
 * \brief Gets the slowest legacy I2C speed class of a device table.
 *
 * \param devices   [in]: Entries of the table
 * \param deviceCnt [in]: Count of the entries
 * \return Slowest speed class, I3C_I2C_SPEED_CNT when the table has no legacy I2C device
 */
static i3c_I2cSpeed_t I3cPriv_Get_SlowestI2c( const i3c_Device_t * const devices, i3c_DeviceCnt_t deviceCnt )
{
    i3c_I2cSpeed_t slowest = I3C_I2C_SPEED_CNT;

    for( i3c_DeviceCnt_t idx = 0u; deviceCnt > idx; idx++ )
    {
        if( ( I3C_DEVICE_I2C == devices[ idx ].Type  ) &&
            ( slowest        > devices[ idx ].I2cSpeed ) )
        {
            slowest = devices[ idx ].I2cSpeed;
        }
        else
        {
            /* Not a legacy device or not slower, no action required */
        }
    }

    return ( slowest );
}


/**
 * \brief Checks that an address may be used by a device: a 7-bit address above the reserved range,
 *        not reserved (the broadcast address and the addresses that differ in one bit from it) and
 *        not the own address.
 *
 * \param ctx     [in]: Context of the instance
 * \param address [in]: Address to check
 * \return I3C_FUNCTION_ACTIVE when the address may be used
 */
static i3c_FunctionState_t I3cPriv_Get_AddressValid( const i3c_Context_t * const ctx, i3c_Address_t address )
{
    i3c_FunctionState_t retState = I3C_FUNCTION_ACTIVE;
    const uint32_t      diff     = ( (uint32_t)address ^ I3C_BROADCAST_ADDRESS );

    if( ( I3C_DYNAMIC_ADDRESS_FIRST > address        ) ||
        ( I3C_ADDRESS_MASK          < address        ) ||
        ( 0u == ( diff & ( diff - 1u ) )             ) ||
        ( ctx->Config.OwnAddress == address          )    )
    {
        retState = I3C_FUNCTION_INACTIVE;
    }
    else
    {
        /* The address is not reserved, no action required */
    }

    return ( retState );
}


/**
 * \brief Checks that an address may be handed out: valid and not used by another entry of the
 *        table (the dynamic address of an I3C entry counts also before it is assigned).
 *
 * \param ctx     [in]: Context of the instance
 * \param address [in]: Address to check
 * \param skipIdx [in]: Entry of the table that is ignored (I3C_DEVICE_CNT_MAX: none)
 * \return I3C_FUNCTION_ACTIVE when the address is free
 */
static i3c_FunctionState_t I3cPriv_Get_AddressFree( const i3c_Context_t * const ctx,
                                                    i3c_Address_t address,
                                                    i3c_DeviceCnt_t skipIdx )
{
    i3c_FunctionState_t retState = I3cPriv_Get_AddressValid( ctx, address );
    i3c_Address_t       used     = 0u;

    for( i3c_DeviceCnt_t idx = 0u;
         ( ctx->DeviceCnt > idx ) &&
         ( I3C_FUNCTION_ACTIVE == retState );
         idx++ )
    {
        if( I3C_DEVICE_I2C == ctx->Devices[ idx ].Type )
        {
            used = ctx->Devices[ idx ].StaticAddress;
        }
        else
        {
            used = ctx->Devices[ idx ].DynamicAddress;
        }

        if( ( skipIdx != idx ) &&
            ( used == address )    )
        {
            retState = I3C_FUNCTION_INACTIVE;
        }
        else
        {
            /* The entry does not use the address, no action required */
        }
    }

    return ( retState );
}


/**
 * \brief Gets the first free address of the dynamic address pool.
 *
 * \param ctx     [in]: Context of the instance
 * \param skipIdx [in]: Entry of the table that is ignored
 * \return Free address, the first address of the pool when none is free (the assignment of more
 *         devices than the table holds cannot be prevented by the bus)
 */
static i3c_Address_t I3cPriv_Get_FreeAddress( const i3c_Context_t * const ctx, i3c_DeviceCnt_t skipIdx )
{
    i3c_Address_t       retAddr = I3C_DYNAMIC_ADDRESS_FIRST;
    i3c_FunctionState_t found   = I3C_FUNCTION_INACTIVE;
    i3c_FunctionState_t free    = I3C_FUNCTION_INACTIVE;

    for( uint32_t addr = I3C_DYNAMIC_ADDRESS_FIRST;
         ( I3C_ADDRESS_MASK >= addr ) &&
         ( I3C_FUNCTION_ACTIVE != found );
         addr++ )
    {
        free = I3cPriv_Get_AddressFree( ctx, (i3c_Address_t)addr, skipIdx );

        if( I3C_FUNCTION_ACTIVE == free )
        {
            retAddr = (i3c_Address_t)addr;
            found   = I3C_FUNCTION_ACTIVE;
        }
        else
        {
            /* The address is reserved or used, no action required */
        }
    }

    return ( retAddr );
}


/**
 * \brief Releases an instance: stops the DMA, disables the peripheral and the interrupts, resets
 *        the peripheral, disables its clock and clears the context.
 *
 * \param ctx      [in/out]: Context of the instance
 * \param periphId [in]: I3C peripheral ID
 */
static void I3cPriv_Release( i3c_Context_t * const ctx, i3c_PeriphId_t periphId )
{
    const rcc_PeriphId_t rccId = (rcc_PeriphId_t)ctx->Config.ClkSrc;

    if( I3C_XFER_MODE_POLL != ctx->Config.XferMode )
    {
        (void)Nvic_Set_PeriphIrq_Inactive( i3c_IrqList[ periphId ][ 0u ] );
        (void)Nvic_Set_PeriphIrq_Inactive( i3c_IrqList[ periphId ][ 1u ] );
    }
    else
    {
        /* The interrupts were not used, no action required */
    }

    (void)I3c_Reg_Set_Disable( periphId );
    I3c_Reg_Set_Raw( periphId, I3C_REG_IER, 0u );
    I3c_Reg_Set_EventsClear( periphId, I3C_REG_MASK_ALL );

    if( I3C_XFER_MODE_DMA == ctx->Config.XferMode )
    {
        (void)I3cDma_Deinit( periphId );
    }
    else
    {
        /* DMA was not used, no action required */
    }

    (void)Rcc_Set_ResetActive( rccId );
    (void)Rcc_Set_ResetInactive( rccId );
    (void)Rcc_Set_PeriphInactive( rccId );

    *ctx = (i3c_Context_t){ 0 };
}


/**
 * \brief Serves the events of an instance according to its role.
 *
 * \param periphId [in]: I3C peripheral ID
 */
static void I3cPriv_Handle_Events( i3c_PeriphId_t periphId )
{
    if( I3C_ROLE_CONTROLLER == i3c_CtxList[ periphId ].Config.Role )
    {
        I3cCtrl_Handle_Events( periphId );
    }
    else
    {
        I3cTgt_Handle_Events( periphId );
    }
}


/** \brief Interrupt handler of I3C1 (event and error line) */
static void I3cPriv_Isr_1( void )
{
    I3cPriv_Handle_Events( I3C_PERIPH_1 );
}


#if defined(I3C2)
/** \brief Interrupt handler of I3C2 (event and error line) */
static void I3cPriv_Isr_2( void )
{
    I3cPriv_Handle_Events( I3C_PERIPH_2 );
}
#endif
