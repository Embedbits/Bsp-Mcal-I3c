/**
 * \author Mr.Nobody
 * \file Test_I3c.c
 * \ingroup I3c
 * \brief Unit tests of the Improved Inter-Integrated Circuit (I3C) module.
 *
 * I3c.c / I3c_Reg.c / I3c_Ctrl.c / I3c_Xfer.c / I3c_Tgt.c / I3c_Dma.c are compiled unchanged. The I3C1
 * registers are emulated by RegMem, the RCC / NVIC / GPIO / GPDMA modules are mocked by CMock.
 *
 * \note The emulated registers are plain RAM: the FIFOs and the flags of the hardware are modelled by the
 *       test. A read of the receive data register returns the preset value for every byte (the pop of a
 *       FIFO is not emulated), a write of the transmit data register or of the control register stores the
 *       last value only; the sequence of the written values is recorded by a RegMem model (the bytes of
 *       a transmission and the control words of a frame are written with different values). The event
 *       register is never cleared by the module (the clear register is separate), the test presets the
 *       events and checks the clear register. The flush bits of the configuration register stay set.
 *       Limits: the order of the bytes received by the address assignment is checked by
 *       I3c_Set_DaaDevice() directly, the address assignment through the FIFO uses one byte value.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "RegMem.h"                         /* Register memory emulation      */
#include "I3c_Port.h"                       /* Module under test              */
#include "I3c.h"                            /* Private interface (test only)  */
#include "I3c_Reg.h"                        /* Register access (test only)    */
#include "MockRcc_Port.h"                   /* RCC module mock                */
#include "MockNvic_Port.h"                  /* NVIC module mock               */
#include "MockGpio_Port.h"                  /* GPIO module mock               */
#include "MockGpdma_Port.h"                 /* GPDMA module mock              */
/* ============================= TYPEDEFS =================================== */

/** Recorded call of a callback with an address and a payload */
typedef struct
{
    uint32_t      CallCnt;                        /**< Count of the calls            */
    i3c_Address_t Address;                        /**< Address of the last call      */
    i3c_Data_t    Payload[ I3C_IBI_PAYLOAD_CNT_MAX ]; /**< Payload of the last call */
    i3c_DataCnt_t PayloadCnt;                     /**< Payload count of the last call */
}   utI3c_IbiRecord_t;

/* ======================= FORWARD DECLARATIONS ============================= */

static i3c_Config_t        UtI3c_Get_Config          ( i3c_Role_t role, i3c_XferMode_t xferMode );
static void                UtI3c_Expect_Clock        ( const i3c_Config_t * const config );
static void                UtI3c_Expect_Release      ( const i3c_Config_t * const config );
static void                UtI3c_Init                ( const i3c_Config_t * const config );
static void                UtI3c_Init_Ctrl           ( void );
static void                UtI3c_Init_Tgt            ( void );
static void                UtI3c_Set_Events          ( uint32_t events );
static void                UtI3c_Set_Table           ( void );
static rcc_RequestState_t  UtI3c_RccGetClkStub       ( rcc_PeriphId_t periphId,
                                                       rcc_FreqHz_t * const periphClk,
                                                       int callCnt );
static gpio_RequestState_t UtI3c_GpioInitStub        ( gpio_Config_t * gpioConfig, int callCnt );
static gpio_RequestState_t UtI3c_GpioLevelHighStub   ( gpio_PortId_t portId,
                                                       gpio_PinId_t pinId,
                                                       gpio_PinLevel_t * const pinLevel,
                                                       int callCnt );
static gpio_RequestState_t UtI3c_GpioLevelLowStub    ( gpio_PortId_t portId,
                                                       gpio_PinId_t pinId,
                                                       gpio_PinLevel_t * const pinLevel,
                                                       int callCnt );
static nvic_RequestState_t UtI3c_NvicHandlerStub     ( nvic_PeriphIrqList_t irqId,
                                                       const nvic_IsrCallback_t irqHandler,
                                                       int callCnt );
static gpdma_RequestState_t UtI3c_GpdmaInitStub      ( gpdma_ConfigStruct_t * const configStruct, int callCnt );
static void                UtI3c_XferCallback        ( i3c_PeriphId_t periphId, i3c_DataCnt_t xferCnt );
static void                UtI3c_ErrorCallback       ( i3c_PeriphId_t periphId, i3c_XferError_t errorMask );
static void                UtI3c_IbiCallback         ( i3c_PeriphId_t periphId,
                                                       i3c_Address_t address,
                                                       const i3c_Data_t * const payload,
                                                       i3c_DataCnt_t payloadCnt );
static void                UtI3c_HotJoinCallback     ( i3c_PeriphId_t periphId );
static void                UtI3c_TgtAddressCallback  ( i3c_PeriphId_t periphId, i3c_Address_t address );
static void                UtI3c_TgtReadReqCallback  ( i3c_PeriphId_t periphId );
static void                UtI3c_TgtReadCallback     ( i3c_PeriphId_t periphId, i3c_DataCnt_t xferCnt );
static void                UtI3c_TgtWriteCallback    ( i3c_PeriphId_t periphId, i3c_DataCnt_t xferCnt );
static void                UtI3c_TgtCccCallback      ( i3c_PeriphId_t periphId, i3c_CccEvent_t event );
static void                UtI3c_TgtRequestCallback  ( i3c_PeriphId_t periphId );
static void                UtI3c_HwModel_EnNotKept   ( void );
static void                UtI3c_HwModel_EnForced    ( void );
static void                UtI3c_HwModel_CfgrNotKept ( void );
static void                UtI3c_HwModel_BcrNotKept  ( void );
static void                UtI3c_HwModel_Capture     ( void );
static i3c_TargetConfig_t  UtI3c_Get_TargetConfig    ( void );
static void                UtI3c_Set_TargetConfig    ( void );
static void                UtI3c_Expect_DmaInit      ( const i3c_Config_t * const config );
static void                UtI3c_Expect_Irq          ( const i3c_Config_t * const config );
static void                UtI3c_Expect_DmaStop      ( const i3c_Config_t * const config,
                                                       i3c_FunctionState_t txStop,
                                                       i3c_FunctionState_t rxStop );
static void                UtI3c_Init_Isr            ( void );
static void                UtI3c_Init_Dma            ( void );
static void                UtI3c_Init_TgtIsr         ( void );
static void                UtI3c_Init_TgtDma         ( void );
static i3c_CtrlWord_t      UtI3c_Get_MsgWord         ( i3c_FunctionState_t lastMsg,
                                                       uint32_t msgType,
                                                       i3c_Address_t address,
                                                       i3c_Direction_t direction,
                                                       i3c_DataCnt_t dataCnt );
static i3c_CtrlWord_t      UtI3c_Get_CccWord         ( i3c_FunctionState_t lastMsg,
                                                       i3c_Ccc_t ccc,
                                                       i3c_DataCnt_t dataCnt );

/* ========================== SYMBOLIC CONSTANTS ============================= */

/** Kernel clock of the instance used by most tests (Hz) */
#define UT_I3C_KERNEL_CLK                   ( 100000000u )

/** I3C1 TIMINGR0 of 100 MHz kernel clock, 4 MHz push-pull, 1 MHz open-drain, no legacy device */
#define UT_I3C_TIMING0_DEFAULT              ( 0x0B570B0Cu )

/** I3C1 TIMINGR1 of 100 MHz kernel clock, 1300 ns bus free time, 1000 ns available time */
#define UT_I3C_TIMING1_DEFAULT              ( 0x00410062u )

/** Achieved push-pull frequency of the default timing (Hz) */
#define UT_I3C_PP_FREQ                      ( 4000000u )

/** Achieved open-drain frequency of the default timing (Hz) */
#define UT_I3C_OD_FREQ                      ( 1000000u )

/** Pattern used to check masked register accesses */
#define UT_I3C_PATTERN                      ( 0xA5A50F0Fu )

/** Mask of the low 16 bits of a register */
#define UT_I3C_LOW16_MASK                   ( 0x0000FFFFu )

/** Value written to a data register by the tests */
#define UT_I3C_DATA_BYTE                    ( 0x5Au )

/** Count of the entries of the recorded sequences */
#define UT_I3C_RECORD_CNT                   ( 32u )

/** Count of the DEVRX registers */
#define UT_I3C_DEVR_CNT                     ( 4u )

/** Largest value of a 32-bit register */
#define UT_I3C_ALL_BITS                     ( 0xFFFFFFFFu )

/** Index of the third and the fourth element of the DEVRX array */
#define UT_I3C_DEVRX_IDX_3                  ( 2u )
#define UT_I3C_DEVRX_IDX_4                  ( 3u )

/** Offset of the I2C target configuration register from the instance base address */
#define UT_I3C_I2CCFGR_OFFSET               ( 0x98u )

/** Count of the registers of the register list on the lines without the I2C target configuration register */
#define UT_I3C_REG_CNT_NO_I2CCFGR           ( 28u )

#if defined(STM32H523xx) || \
    defined(STM32H533xx) || \
    defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H562xx) || \
    defined(STM32H563xx) || \
    defined(STM32H573xx)
/** The I2C target configuration register (static address) exists on I3C1 of this line */
#define UT_I3C_HAS_I2CCFGR                  ( 1u )
#else
/** The I2C target configuration register (static address) does not exist on this line */
#define UT_I3C_HAS_I2CCFGR                  ( 0u )
#endif

#if defined(I3C2)
/** The second instance exists on this line */
#define UT_I3C_HAS_I2CCFGR_I3C2             ( UT_I3C_HAS_I2CCFGR )
#else
/** The second instance does not exist on this line */
#define UT_I3C_HAS_I2CCFGR_I3C2             ( 0u )
#endif

/** Addresses of the pool after the own address, size of the SETMRL data */
#define UT_I3C_ADDR_POOL_2                  ( I3C_DEFAULT_OWN_ADDRESS + 2u )
#define UT_I3C_ADDR_POOL_3                  ( I3C_DEFAULT_OWN_ADDRESS + 3u )
#define UT_I3C_SETMRL_CNT                   ( 3u )


/** Kernel clock below twice the push-pull frequency (Hz) */
#define UT_I3C_SLOW_CLK                     ( 6000000u )

/** Push-pull frequency the registers cannot represent at the test kernel clock (Hz) */
#define UT_I3C_TOO_SLOW_PP                  ( 100000u )

/** Low kernel clock used to test the rounding of the timing (Hz) and the achieved push-pull frequency */
#define UT_I3C_LOW_CLK                      ( 32000000u )
#define UT_I3C_LOW_CLK_PP                   ( 8000000u )

/** Addresses used by the tests */
#define UT_I3C_ADDR_A                       ( 0x30u )
#define UT_I3C_ADDR_B                       ( 0x31u )
#define UT_I3C_ADDR_I2C                     ( 0x50u )
#define UT_I3C_ADDR_UNKNOWN                 ( 0x60u )
#define UT_I3C_ADDR_STATIC                  ( 0x40u )
#define UT_I3C_ADDR_RESERVED_NEAR           ( 0x76u )

/** Interrupt priority used by the tests */
#define UT_I3C_PRIO                         ( 3u )

/** TIMINGR0 with a Fast-mode plus device (legacy high time 40 cycles) and with a Fast-mode device (150 cycles low) */
#define UT_I3C_TIMING0_FMP                  ( 0x27570B0Cu )
#define UT_I3C_TIMING0_FM                   ( 0x63950B0Cu )

/** Achieved open-drain frequency with a Fast-mode device (Hz) */
#define UT_I3C_OD_FREQ_FM                   ( 617283u )

/** PID of a device that answers with the same byte for all the bytes of the payload */
#define UT_I3C_PID_HIGH_REPEATED            ( 0x5A5Au )
#define UT_I3C_PID_LOW_REPEATED             ( 0x5A5A5A5Au )

/** PID of the device of the distinct payload */
#define UT_I3C_PID_HIGH_1                   ( 0x0102u )
#define UT_I3C_PID_LOW_1                    ( 0x03040506u )

/** Counts of the test data */
#define UT_I3C_BUF_CNT                      ( 10u )
#define UT_I3C_WRITE_CNT                    ( 3u )
#define UT_I3C_READ_CNT                     ( 4u )
#define UT_I3C_EARLY_CNT                    ( 3u )
#define UT_I3C_PAIR_CNT                     ( 2u )
#define UT_I3C_TABLE_CNT                    ( 2u )
#define UT_I3C_PID_CNT                      ( 6u )
#define UT_I3C_CODE_CNT                     ( 12u )
#define UT_I3C_GPIO_CFG_CNT                 ( 2u )
#define UT_I3C_DMA_CFG_CNT                  ( 2u )

/** A CCC code that is not offered by the module */
#define UT_I3C_CCC_UNKNOWN                  ( 0x55u )

/** In-band interrupt: count of the payload bytes and the payload register */
#define UT_I3C_IBI_CNT                      ( 2u )
#define UT_I3C_IBI_DATA                     ( 0x0000BBAAu )

/** Index of the control word of the second part of the read RSTACT in the recorded sequence */
#define UT_I3C_RSTACT_READ_IDX              ( 3u )

/** A function state that does not exist */
#define UT_I3C_BAD_STATE                    ( 2u )

/** Second data byte of the tests */
#define UT_I3C_DATA_BYTE_2                  ( 0xA5u )

/** Target configuration of the tests */
#define UT_I3C_TGT_INSTANCE_ID              ( 3u )
#define UT_I3C_INSTANCE_ID_INVALID          ( 16u )
#define UT_I3C_TGT_BCR                      ( I3C_BCR_IBI_CAPABLE | I3C_BCR_IBI_PAYLOAD )
#define UT_I3C_TGT_DCR                      ( 0x7Fu )
#define UT_I3C_TGT_MAX_WRITE                ( 0x0200u )
#define UT_I3C_TGT_MAX_READ                 ( 0x0100u )
#define UT_I3C_TGT_MAX_IBI                  ( 2u )

/** Static address register: position of the address and the valid bit */
#define UT_I3C_SA_POS                       ( 1u )
#define UT_I3C_SAVAL                        ( 1u )

#if UT_I3C_HAS_I2CCFGR
/** Result of a target configuration with a static address on this line */
#define UT_I3C_STATIC_RESULT                ( I3C_REQUEST_OK )
#else
/** Result of a target configuration with a static address on this line */
#define UT_I3C_STATIC_RESULT                ( I3C_REQUEST_ERROR )
#endif

/** Data of the transmissions and the receive buffer of the tests */
#define UT_I3C_TX_DATA_INIT \
    { 0x11u, 0x22u, 0x33u, 0x44u, 0x55u, 0x66u, 0x77u, 0x88u, 0x99u, 0xAAu }
static const i3c_Data_t utI3c_TxData[ UT_I3C_BUF_CNT ] = UT_I3C_TX_DATA_INIT;
static i3c_Data_t utI3c_RxBuf[ UT_I3C_BUF_CNT ];

/** Protocol error codes of I3C_SER */
#define UT_I3C_CODE_VALUES_INIT \
    { 0x9u, 0xAu, 0xBu, 0x1u, 0xEu, 0x0u, 0x2u, 0x3u, 0x8u, 0xCu, 0xDu, 0xFu }
static const uint32_t utI3c_CodeValues[ UT_I3C_CODE_CNT ] = UT_I3C_CODE_VALUES_INIT;

/** Errors the protocol error codes map to */
static const i3c_XferErrorId_t utI3c_CodeErrors[ UT_I3C_CODE_CNT ] =
{
    I3C_XFER_ERROR_PARITY, I3C_XFER_ERROR_PARITY, I3C_XFER_ERROR_PARITY, I3C_XFER_ERROR_BUS, I3C_XFER_ERROR_BUS,
    I3C_XFER_ERROR_FRAME, I3C_XFER_ERROR_FRAME, I3C_XFER_ERROR_FRAME, I3C_XFER_ERROR_FRAME, I3C_XFER_ERROR_FRAME,
    I3C_XFER_ERROR_FRAME, I3C_XFER_ERROR_FRAME
};

/** Answers of three devices of the address assignment (PID, BCR, DCR) */
#define UT_I3C_PAYLOAD1_INIT \
    { 0x01u, 0x02u, 0x03u, 0x04u, 0x05u, 0x06u, 0x04u, 0x08u }
#define UT_I3C_PAYLOAD2_INIT \
    { 0x11u, 0x12u, 0x13u, 0x14u, 0x15u, 0x16u, 0x00u, 0x18u }
static const i3c_Data_t utI3c_Payload1[ I3C_DAA_PAYLOAD_CNT ] = UT_I3C_PAYLOAD1_INIT;
static const i3c_Data_t utI3c_Payload2[ I3C_DAA_PAYLOAD_CNT ] = UT_I3C_PAYLOAD2_INIT;

/* ========================== LOCAL VARIABLES =============================== */

/** Kernel clock returned by UtI3c_RccGetClkStub() */
static i3c_FreqHz_t utI3c_KernelClk;

/** Count of the calls of the transfer complete callback and the last count */
static uint32_t     utI3c_XferCallCnt;
static i3c_DataCnt_t utI3c_XferLastCnt;

/** Count of the calls of the error callback and the last error mask */
static uint32_t       utI3c_ErrCallCnt;
static i3c_XferError_t utI3c_ErrLastMask;

/** Last in-band interrupt reported to the callback */
static utI3c_IbiRecord_t utI3c_Ibi;

/** Count of the calls of the hot-join callback */
static uint32_t utI3c_HjCallCnt;

/** Records of the target callbacks */
static uint32_t       utI3c_TgtAddrCallCnt;
static i3c_Address_t  utI3c_TgtAddr;
static uint32_t       utI3c_TgtReadReqCnt;
static uint32_t       utI3c_TgtReadCallCnt;
static i3c_DataCnt_t  utI3c_TgtReadCnt;
static uint32_t       utI3c_TgtWriteCallCnt;
static i3c_DataCnt_t  utI3c_TgtWriteCnt;
static uint32_t       utI3c_TgtCccCallCnt;
static i3c_CccEvent_t utI3c_TgtCccEvents[ UT_I3C_RECORD_CNT ];
static uint32_t       utI3c_TgtReqCallCnt;

/** Sequences of the values written to the transmit data register and to the control register */
static uint32_t utI3c_TxSeq[ UT_I3C_RECORD_CNT ];
static uint32_t utI3c_TxSeqCnt;
static uint32_t utI3c_CrSeq[ UT_I3C_RECORD_CNT ];
static uint32_t utI3c_CrSeqCnt;

/** Interrupt handler registered by the module (captured by UtI3c_NvicHandlerStub) */
static nvic_IsrCallback_t utI3c_Isr;

/** Last values recorded by the capture model and the priority returned by the Nvic mock */
static uint32_t       utI3c_TxLast;
static uint32_t       utI3c_CrLast;
static nvic_IrqPrio_t utI3c_PrioValue;

/** Gpio and Gpdma configurations captured by the stubs */
static gpio_Config_t         utI3c_GpioCfg[ UT_I3C_DEVR_CNT ];
static uint32_t              utI3c_GpioCfgCnt;
static gpdma_ConfigStruct_t  utI3c_DmaCfg[ UT_I3C_DEVR_CNT ];
static gpdma_TransferConfig_t utI3c_DmaXfer[ UT_I3C_DEVR_CNT ];
static uint32_t              utI3c_DmaCfgCnt;
static uint32_t              utI3c_GpioLevelCalls;

/* ============================ TEST FIXTURE ================================ */

void setUp( void )
{
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );

    utI3c_KernelClk       = UT_I3C_KERNEL_CLK;
    utI3c_XferCallCnt     = 0u;
    utI3c_XferLastCnt     = 0u;
    utI3c_ErrCallCnt      = 0u;
    utI3c_ErrLastMask     = 0u;
    utI3c_Ibi             = (utI3c_IbiRecord_t){ 0 };
    utI3c_HjCallCnt       = 0u;
    utI3c_TgtAddrCallCnt  = 0u;
    utI3c_TgtAddr         = 0u;
    utI3c_TgtReadReqCnt   = 0u;
    utI3c_TgtReadCallCnt  = 0u;
    utI3c_TgtReadCnt      = 0u;
    utI3c_TgtWriteCallCnt = 0u;
    utI3c_TgtWriteCnt     = 0u;
    utI3c_TgtCccCallCnt   = 0u;
    utI3c_TgtReqCallCnt   = 0u;
    utI3c_TxSeqCnt        = 0u;
    utI3c_CrSeqCnt        = 0u;
    utI3c_Isr             = I3C_NULL_PTR;
    utI3c_GpioCfgCnt      = 0u;
    utI3c_DmaCfgCnt       = 0u;
    utI3c_GpioLevelCalls  = 0u;
    utI3c_TxLast          = 0u;
    utI3c_CrLast          = 0u;
    utI3c_PrioValue       = UT_I3C_PRIO;
}


void tearDown( void )
{
    /* No action required */
}

/* ============================== TEST CASES ================================= */

/* ------------------------------ Reg component ------------------------------ */

/**
 * \brief Verifies that a bit read of an invalid instance returns no bit.
 *
 * \par Expected results
 * I3c_Reg_Get_Bit() with an instance ID equal to I3C_PERIPH_CNT returns zero.
 */
void Ut_I3cReg_GetBit_InvalidInstance_ReturnsZero( void )
{
    I3C1->CFGR = UT_I3C_ALL_BITS;

    TEST_ASSERT_EQUAL_UINT32( 0u, I3c_Reg_Get_Bit( I3C_PERIPH_CNT, I3C_REG_CFGR, UT_I3C_ALL_BITS ) );
}


/**
 * \brief Verifies that a bit read of an invalid register returns no bit.
 *
 * \par Expected results
 * I3c_Reg_Get_Bit() with a register ID equal to I3C_REG_CNT returns zero.
 */
void Ut_I3cReg_GetBit_InvalidRegister_ReturnsZero( void )
{
    I3C1->CFGR = UT_I3C_ALL_BITS;

    TEST_ASSERT_EQUAL_UINT32( 0u, I3c_Reg_Get_Bit( I3C_PERIPH_1, I3C_REG_CNT, UT_I3C_ALL_BITS ) );
}


/**
 * \brief Verifies that the write accessors ignore invalid identifiers.
 *
 * \par Expected results
 * The set, reset, value and raw accessors with an invalid instance or register leave the registers unchanged.
 */
void Ut_I3cReg_Set_InvalidIds_NoRegisterWritten( void )
{
    I3c_Reg_Set_Bit( I3C_PERIPH_CNT, I3C_REG_CFGR, UT_I3C_ALL_BITS );
    I3c_Reg_Reset_Bit( I3C_PERIPH_CNT, I3C_REG_CFGR, UT_I3C_ALL_BITS );
    I3c_Reg_Set_Val( I3C_PERIPH_CNT, I3C_REG_CFGR, UT_I3C_ALL_BITS, UT_I3C_ALL_BITS );
    I3c_Reg_Set_Raw( I3C_PERIPH_1, I3C_REG_CNT, UT_I3C_ALL_BITS );

    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->CFGR );
    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->CR );
}


/**
 * \brief Verifies the set and the reset of register bits.
 *
 * \par Expected results
 * Set_Bit sets the masked bits and keeps the others, Reset_Bit clears the masked bits only.
 */
void Ut_I3cReg_SetResetBit_Cfgr_ChangesMaskedBitsOnly( void )
{
    I3C1->CFGR = I3C_CFGR_HJACK;

    I3c_Reg_Set_Bit( I3C_PERIPH_1, I3C_REG_CFGR, I3C_CFGR_TMODE );
    TEST_ASSERT_EQUAL_UINT32( ( I3C_CFGR_HJACK | I3C_CFGR_TMODE ), I3C1->CFGR );

    I3c_Reg_Reset_Bit( I3C_PERIPH_1, I3C_REG_CFGR, I3C_CFGR_HJACK );
    TEST_ASSERT_EQUAL_UINT32( I3C_CFGR_TMODE, I3C1->CFGR );
}


/**
 * \brief Verifies the masked value update and read.
 *
 * \par Expected results
 * Set_Val replaces the masked bits only, Get_Val returns the masked bits.
 */
void Ut_I3cReg_SetVal_Mask_KeepsOtherBits( void )
{
    I3C1->TIMINGR0 = UT_I3C_PATTERN;

    I3c_Reg_Set_Val( I3C_PERIPH_1, I3C_REG_TIMINGR0, UT_I3C_LOW16_MASK, 0u );

    TEST_ASSERT_EQUAL_UINT32( ( UT_I3C_PATTERN & ~UT_I3C_LOW16_MASK ), I3C1->TIMINGR0 );
    TEST_ASSERT_EQUAL_UINT32( ( UT_I3C_PATTERN & ~UT_I3C_LOW16_MASK ),
                              I3c_Reg_Get_Val( I3C_PERIPH_1, I3C_REG_TIMINGR0, ~UT_I3C_LOW16_MASK ) );
}


/**
 * \brief Verifies that the raw write replaces the whole register.
 *
 * \par Expected results
 * I3c_Reg_Set_Raw() writes the value without reading the previous one.
 */
void Ut_I3cReg_SetRaw_Cr_WritesWholeValue( void )
{
    I3C1->CR = UT_I3C_ALL_BITS;

    I3c_Reg_Set_Raw( I3C_PERIPH_1, I3C_REG_CR, UT_I3C_PATTERN );

    TEST_ASSERT_EQUAL_UINT32( UT_I3C_PATTERN, I3C1->CR );
}


/**
 * \brief Verifies the address of a register used by the DMA.
 *
 * \par Expected results
 * I3c_Reg_Get_Addr() returns the address of I3C_TDR and I3C_RDR of the instance.
 */
void Ut_I3cReg_GetAddr_DataRegisters_ReturnRegisterAddress( void )
{
    TEST_ASSERT_EQUAL_UINT32( (uint32_t)&I3C1->TDR, I3c_Reg_Get_Addr( I3C_PERIPH_1, I3C_REG_TDR ) );
    TEST_ASSERT_EQUAL_UINT32( (uint32_t)&I3C1->RDR, I3c_Reg_Get_Addr( I3C_PERIPH_1, I3C_REG_RDR ) );
}


/**
 * \brief Verifies that an invalid identifier has no address.
 *
 * \par Expected results
 * I3c_Reg_Get_Addr() returns zero for an invalid instance and for an invalid register.
 */
void Ut_I3cReg_GetAddr_InvalidIds_ReturnZero( void )
{
    TEST_ASSERT_EQUAL_UINT32( 0u, I3c_Reg_Get_Addr( I3C_PERIPH_CNT, I3C_REG_TDR ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, I3c_Reg_Get_Addr( I3C_PERIPH_1, I3C_REG_CNT ) );
}


/**
 * \brief Verifies the addresses of the target device registers.
 *
 * \par Expected results
 * DEVR1 .. DEVR4 are the elements of the DEVRX array of the CMSIS type, DEVR0 is the own device register.
 */
void Ut_I3cReg_GetAddr_DeviceRegisters_MatchCmsisLayout( void )
{
    TEST_ASSERT_EQUAL_UINT32( (uint32_t)&I3C1->DEVR0, I3c_Reg_Get_Addr( I3C_PERIPH_1, I3C_REG_DEVR0 ) );
    TEST_ASSERT_EQUAL_UINT32( (uint32_t)&I3C1->DEVRX[ 0u ], I3c_Reg_Get_Addr( I3C_PERIPH_1, I3C_REG_DEVR1 ) );
    TEST_ASSERT_EQUAL_UINT32( (uint32_t)&I3C1->DEVRX[ 1u ], I3c_Reg_Get_Addr( I3C_PERIPH_1, I3C_REG_DEVR2 ) );
    TEST_ASSERT_EQUAL_UINT32( (uint32_t)&I3C1->DEVRX[ UT_I3C_DEVRX_IDX_3 ],
                              I3c_Reg_Get_Addr( I3C_PERIPH_1, I3C_REG_DEVR3 ) );
    TEST_ASSERT_EQUAL_UINT32( (uint32_t)&I3C1->DEVRX[ UT_I3C_DEVRX_IDX_4 ],
                              I3c_Reg_Get_Addr( I3C_PERIPH_1, I3C_REG_DEVR4 ) );
}


/**
 * \brief Verifies the I2C target configuration register of I3C1.
 *
 * \par Expected results
 * The register exists on the lines that have it (offset 0x98 of the instance), otherwise it is not accessible.
 */
void Ut_I3cReg_GetAddr_I2cCfgr_OffsetOnlyWhereImplemented( void )
{
#if UT_I3C_HAS_I2CCFGR
    TEST_ASSERT_EQUAL_UINT32( ( (uint32_t)I3C1 + UT_I3C_I2CCFGR_OFFSET ),
                              I3c_Reg_Get_Addr( I3C_PERIPH_1, I3C_REG_I2CCFGR ) );
#else
    TEST_ASSERT_EQUAL_UINT32( UT_I3C_REG_CNT_NO_I2CCFGR, I3C_REG_CNT );
#endif
}


/**
 * \brief Verifies that the I2C target configuration register exists on I3C1 only.
 *
 * \par Expected results
 * The register of the second instance (lines with I3C2 and the register) is not accessible.
 */
void Ut_I3cReg_Get_I2cCfgr_SecondInstance_NotAccessible( void )
{
#if UT_I3C_HAS_I2CCFGR_I3C2
    TEST_ASSERT_EQUAL_UINT32( 0u, I3c_Reg_Get_Addr( I3C_PERIPH_2, I3C_REG_I2CCFGR ) );
    I3c_Reg_Set_Raw( I3C_PERIPH_2, I3C_REG_I2CCFGR, UT_I3C_PATTERN );
    TEST_ASSERT_EQUAL_UINT32( 0u, I3c_Reg_Get_Bit( I3C_PERIPH_2, I3C_REG_I2CCFGR, UT_I3C_ALL_BITS ) );
#else
    TEST_ASSERT_EQUAL_UINT32( 0u, I3c_Reg_Get_Addr( I3C_PERIPH_CNT, I3C_REG_CNT ) );
#endif
}


/**
 * \brief Verifies the enable protocol.
 *
 * \par Expected results
 * Set_Enable sets I3C_CFGR.EN and returns OK, Get_EnableState reports the enabled state, Set_Disable clears it.
 */
void Ut_I3cReg_SetEnableDisable_RegisterFollows_ReturnsOk( void )
{
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Reg_Set_Enable( I3C_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_UINT32( I3C_CFGR_EN, I3C1->CFGR );
    TEST_ASSERT_EQUAL( I3C_FUNCTION_ACTIVE, I3c_Reg_Get_EnableState( I3C_PERIPH_1 ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Reg_Set_Disable( I3C_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->CFGR );
    TEST_ASSERT_EQUAL( I3C_FUNCTION_INACTIVE, I3c_Reg_Get_EnableState( I3C_PERIPH_1 ) );
}


/**
 * \brief Verifies that the enable fails when the bit does not stay set.
 *
 * \par Expected results
 * With a model that clears I3C_CFGR.EN after every store, Set_Enable returns the error after the bounded wait.
 */
void Ut_I3cReg_SetEnable_EnableNotKept_ReturnsError( void )
{
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtI3c_HwModel_EnNotKept ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Reg_Set_Enable( I3C_PERIPH_1 ) );
}


/**
 * \brief Verifies that the disable fails when the bit does not clear.
 *
 * \par Expected results
 * With a model that keeps I3C_CFGR.EN set, Set_Disable returns the error after the bounded wait.
 */
void Ut_I3cReg_SetDisable_DisableNotKept_ReturnsError( void )
{
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtI3c_HwModel_EnForced ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Reg_Set_Disable( I3C_PERIPH_1 ) );
}


/**
 * \brief Verifies the enable protocol with an invalid instance.
 *
 * \par Expected results
 * Set_Enable and Set_Disable return the error, Get_EnableState reports the inactive state.
 */
void Ut_I3cReg_Enable_InvalidInstance_ReturnsError( void )
{
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Reg_Set_Enable( I3C_PERIPH_CNT ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Reg_Set_Disable( I3C_PERIPH_CNT ) );
    TEST_ASSERT_EQUAL( I3C_FUNCTION_INACTIVE, I3c_Reg_Get_EnableState( I3C_PERIPH_CNT ) );
}


/**
 * \brief Verifies the FIFO flush.
 *
 * \par Expected results
 * Set_FifoFlush sets the flush bits of the configuration register and keeps the others.
 */
void Ut_I3cReg_SetFifoFlush_SetsFlushBits( void )
{
    I3C1->CFGR = I3C_CFGR_EN;

    I3c_Reg_Set_FifoFlush( I3C_PERIPH_1, ( I3C_CFGR_TXFLUSH | I3C_CFGR_RXFLUSH ) );

    TEST_ASSERT_EQUAL_UINT32( ( I3C_CFGR_EN | I3C_CFGR_TXFLUSH | I3C_CFGR_RXFLUSH ), I3C1->CFGR );
}


/**
 * \brief Verifies the event register access.
 *
 * \par Expected results
 * Get_Events returns I3C_EVR, Set_EventsClear writes the mask to the clear register without reading it.
 */
void Ut_I3cReg_Events_GetAndClear_UseEventAndClearRegisters( void )
{
    I3C1->EVR  = I3C_EVR_FCF;
    I3C1->CEVR = UT_I3C_ALL_BITS;

    TEST_ASSERT_EQUAL_UINT32( I3C_EVR_FCF, I3c_Reg_Get_Events( I3C_PERIPH_1 ) );

    I3c_Reg_Set_EventsClear( I3C_PERIPH_1, I3C_EVR_ERRF );
    TEST_ASSERT_EQUAL_UINT32( I3C_EVR_ERRF, I3C1->CEVR );
}


/**
 * \brief Verifies the data access helpers.
 *
 * \par Expected results
 * Set_ControlWord writes I3C_CR, Set_TxByte writes I3C_TDR, Get_RxByte returns the low byte of I3C_RDR.
 */
void Ut_I3cReg_DataAccess_WriteControlAndTx_ReadRx( void )
{
    I3c_Reg_Set_ControlWord( I3C_PERIPH_1, UT_I3C_PATTERN );
    I3c_Reg_Set_TxByte( I3C_PERIPH_1, UT_I3C_DATA_BYTE );
    I3C1->RDR = UT_I3C_PATTERN;

    TEST_ASSERT_EQUAL_UINT32( UT_I3C_PATTERN, I3C1->CR );
    TEST_ASSERT_EQUAL_UINT32( UT_I3C_DATA_BYTE, I3C1->TDR );
    TEST_ASSERT_EQUAL_UINT8( (uint8_t)UT_I3C_PATTERN, I3c_Reg_Get_RxByte( I3C_PERIPH_1 ) );
}

/* ---------------------------- Module management ---------------------------- */

/**
 * \brief Verifies the module version.
 *
 * \par Expected results
 * I3c_Get_ModuleVersion() returns the major version 1.
 */
void Ut_I3c_GetModuleVersion_Call_ReturnsMajorOne( void )
{
    const i3c_ModuleVersion_t version = I3c_Get_ModuleVersion();

    TEST_ASSERT_EQUAL_UINT8( 1u, version.Major );
}


/**
 * \brief Verifies that the default configuration rejects a null pointer.
 *
 * \par Expected results
 * I3c_Get_DefaultConfig( NULL ) returns I3C_REQUEST_ERROR.
 */
void Ut_I3c_GetDefaultConfig_NullPtr_ReturnsError( void )
{
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Get_DefaultConfig( I3C_NULL_PTR ) );
}


/**
 * \brief Verifies the default configuration.
 *
 * \par Expected results
 * Instance I3C1 as controller in the POLL mode with the default frequencies and addresses, no pin, no callback.
 */
void Ut_I3c_GetDefaultConfig_ValidPtr_ReturnsDefaults( void )
{
    i3c_Config_t config;

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_DefaultConfig( &config ) );

    TEST_ASSERT_EQUAL( I3C_PERIPH_1, config.PeriphId );
    TEST_ASSERT_EQUAL( I3C_ROLE_CONTROLLER, config.Role );
    TEST_ASSERT_EQUAL( I3C_CLK_SRC_I3C1_PCLK1, config.ClkSrc );
    TEST_ASSERT_EQUAL_UINT32( I3C_DEFAULT_PUSH_PULL_FREQ, config.PushPullFreq );
    TEST_ASSERT_EQUAL_UINT32( I3C_DEFAULT_OPEN_DRAIN_FREQ, config.OpenDrainFreq );
    TEST_ASSERT_EQUAL_UINT32( I3C_DEFAULT_BUS_FREE_TIME, config.BusFreeTime );
    TEST_ASSERT_EQUAL_UINT32( I3C_DEFAULT_BUS_AVAILABLE_TIME, config.BusAvailableTime );
    TEST_ASSERT_EQUAL_UINT8( I3C_DEFAULT_OWN_ADDRESS, config.OwnAddress );
    TEST_ASSERT_EQUAL( I3C_XFER_MODE_POLL, config.XferMode );
    TEST_ASSERT_EQUAL( I3C_SCL_PIN_UNUSED, config.SclPin );
    TEST_ASSERT_EQUAL( I3C_SDA_PIN_UNUSED, config.SdaPin );
    TEST_ASSERT_NULL( config.XferCompleteCallback );
    TEST_ASSERT_NULL( config.TargetCccCallback );
}


/**
 * \brief Verifies that an invalid configuration is rejected before any register or module is touched.
 *
 * \par Expected results
 * Null pointer, invalid instance, role, transfer mode, clock source, frequencies, times and addresses return
 * I3C_REQUEST_ERROR; the mocks of the other modules expect no call and the registers stay zero.
 */
void Ut_I3c_Init_InvalidConfig_ReturnsErrorWithoutAccess( void )
{
    i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_POLL );

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Init( I3C_NULL_PTR ) );

    config.PeriphId = I3C_PERIPH_CNT;
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Init( &config ) );

    config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_POLL );
    config.Role = I3C_ROLE_CNT;
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Init( &config ) );

    config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_POLL );
    config.XferMode = I3C_XFER_MODE_CNT;
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Init( &config ) );

    config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_POLL );
    config.ClkSrc = I3C_CLK_SRC_CNT;
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Init( &config ) );

    config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_POLL );
    config.BusAvailableTime = 0u;
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Init( &config ) );

    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->CFGR );
}


/**
 * \brief Verifies the checks of the controller parameters.
 *
 * \par Expected results
 * A zero or too high push-pull frequency, a zero open-drain frequency, a zero bus free time and an own address
 * that is reserved or above 7 bits return I3C_REQUEST_ERROR without access.
 */
void Ut_I3c_Init_InvalidControllerParameters_ReturnsError( void )
{
    i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_POLL );

    config.PushPullFreq = 0u;
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Init( &config ) );

    config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_POLL );
    config.PushPullFreq = I3C_PUSH_PULL_FREQ_MAX + 1u;
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Init( &config ) );

    config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_POLL );
    config.OpenDrainFreq = 0u;
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Init( &config ) );

    config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_POLL );
    config.BusFreeTime = 0u;
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Init( &config ) );

    config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_POLL );
    config.OwnAddress = I3C_DYNAMIC_ADDRESS_FIRST - 1u;
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Init( &config ) );

    config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_POLL );
    config.OwnAddress = I3C_ADDRESS_MASK + 1u;
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Init( &config ) );
}


/**
 * \brief Verifies the checks of the pins and of the DMA channels.
 *
 * \par Expected results
 * A pin of another instance, the same DMA channel for both FIFOs and an invalid DMA channel return
 * I3C_REQUEST_ERROR without access.
 */
void Ut_I3c_Init_InvalidPinsAndDma_ReturnsError( void )
{
    i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_POLL );

    config.SclPin = (i3c_SclPin_t)I3C_PIN_ENCODE( I3C_PERIPH_CNT, GPIO_PORT_B, GPIO_PIN_ID_6, GPIO_ALT_FUNC_3 );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Init( &config ) );

    config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_POLL );
    config.SdaPin = (i3c_SdaPin_t)I3C_PIN_ENCODE( I3C_PERIPH_CNT, GPIO_PORT_B, GPIO_PIN_ID_7, GPIO_ALT_FUNC_3 );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Init( &config ) );

    config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_DMA );
    config.RxDmaChannelId = config.TxDmaChannelId;
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Init( &config ) );

    config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_DMA );
    config.TxDmaChannelId = GPDMA_CHANNEL_CNT;
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Init( &config ) );

    config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_DMA );
    config.RxDmaPeriphId = GPDMA_PERIPH_CNT;
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Init( &config ) );
}


/**
 * \brief Verifies the initialization as controller in the POLL mode.
 *
 * \par Expected results
 * The clock is enabled and read through the Rcc module; I3C_CFGR holds CRINIT, TMODE and EN, the timing registers
 * hold the values of 100 MHz kernel clock, the own address is valid, no interrupt is enabled, the achieved
 * frequencies are the requested ones.
 */
void Ut_I3c_Init_ControllerPoll_ConfiguresRegisters( void )
{
    i3c_FreqHz_t pushPull = 0u;
    i3c_FreqHz_t openDrain = 0u;

    UtI3c_Init_Ctrl();

    TEST_ASSERT_EQUAL_UINT32( ( I3C_CFGR_EN | I3C_CFGR_CRINIT | I3C_CFGR_TMODE ), I3C1->CFGR );
    TEST_ASSERT_EQUAL_UINT32( UT_I3C_TIMING0_DEFAULT, I3C1->TIMINGR0 );
    TEST_ASSERT_EQUAL_UINT32( UT_I3C_TIMING1_DEFAULT, I3C1->TIMINGR1 );
    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->TIMINGR2 );
    TEST_ASSERT_EQUAL_UINT32( ( ( (uint32_t)I3C_DEFAULT_OWN_ADDRESS << I3C_DEVR0_DA_Pos ) | I3C_DEVR0_DAVAL ),
                              I3C1->DEVR0 );
    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->IER );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_BusFreq( I3C_PERIPH_1, &pushPull, &openDrain ) );
    TEST_ASSERT_EQUAL_UINT32( UT_I3C_PP_FREQ, pushPull );
    TEST_ASSERT_EQUAL_UINT32( UT_I3C_OD_FREQ, openDrain );
}


/**
 * \brief Verifies that the STALLT bit of I3C_TIMINGR2 is never set.
 *
 * \details The controller is initialized and a legacy I2C read is requested; I3C_TIMINGR2 is checked after both.
 *
 * \note    Device errata bug AB#2091 (ES0565 Rev 7 2.15.1, ES0621 Rev 2 2.13.1, ES0561 Rev 4 2.10.1 "I3C controller:
 *          unexpected read data bytes during a legacy I2C read"): with STALLT = 1 the controller does not NACK the
 *          end of a legacy I2C read and reads dummy bytes; the module avoids it by never setting STALLT.
 *
 * \par Expected results
 * I3C_TIMINGR2.STALLT is cleared (the whole register is zero) after the initialization and after the legacy read.
 */
void Ut_I3c_LegacyRead_ControllerSession_StalltNeverSet( void )
{
    UtI3c_Init_Ctrl();
    UtI3c_Set_Table();
    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->TIMINGR2 & I3C_TIMINGR2_STALLT );
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtI3c_HwModel_Capture ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_Read( I3C_PERIPH_1, UT_I3C_ADDR_I2C, utI3c_RxBuf, 1u ) );

    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->TIMINGR2 & I3C_TIMINGR2_STALLT );
    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->TIMINGR2 );
}


/**
 * \brief Verifies that the arbitrable header stays enabled and the address phase is never stalled.
 *
 * \details The controller is initialized without and with the SDA high keeper; I3C_CFGR and I3C_TIMINGR2 are checked.
 *
 * \note    Device errata bug AB#2092 (ES0565 Rev 7 2.15.2, ES0621 Rev 2 2.13.2, ES0561 Rev 4 2.10.2 "I3C controller:
 *          SCL clock is not stalled during address ACK/NACK phase following a frame start, when enabled through
 *          I3C_TIMINGR2 register"): the stall of the address phase (STALLA) is not reliable, the workaround is
 *          NOARBH = 0 (arbitrable header inserted); the module keeps NOARBH = 0 and never sets STALLA.
 *
 * \par Expected results
 * I3C_CFGR.NOARBH and I3C_TIMINGR2.STALLA are cleared in both configurations.
 */
void Ut_I3c_Init_Controller_NoarbhAndStallaCleared( void )
{
    i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_POLL );

    UtI3c_Init( &config );
    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->CFGR & I3C_CFGR_NOARBH );
    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->TIMINGR2 & I3C_TIMINGR2_STALLA );
    TEST_ASSERT_EQUAL_UINT32( I3C_CFGR_EN, I3C1->CFGR & I3C_CFGR_EN );
}


/**
 * \brief Verifies that the SDA high keeper option keeps the arbitrable header and the address phase unchanged.
 *
 * \note    Device errata bug AB#2092 (ES0565 Rev 7 2.15.2, ES0621 Rev 2 2.13.2, ES0561 Rev 4 2.10.2 "I3C controller:
 *          SCL clock is not stalled during address ACK/NACK phase following a frame start, when enabled through
 *          I3C_TIMINGR2 register"): other configuration options must not enable NOARBH or STALLA.
 *
 * \par Expected results
 * With SdaHighKeeper active I3C_CFGR.NOARBH and I3C_TIMINGR2.STALLA are still cleared.
 */
void Ut_I3c_Init_SdaHighKeeper_NoarbhAndStallaCleared( void )
{
    i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_POLL );

    config.SdaHighKeeper = I3C_FUNCTION_ACTIVE;
    UtI3c_Init( &config );

    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->CFGR & I3C_CFGR_NOARBH );
    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->TIMINGR2 & I3C_TIMINGR2_STALLA );
}


/**
 * \brief Verifies the SDA high keeper option.
 *
 * \par Expected results
 * With SdaHighKeeper active I3C_CFGR.HKSDAEN is set together with the enable.
 */
void Ut_I3c_Init_SdaHighKeeper_SetsHkSdaEn( void )
{
    i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_POLL );

    config.SdaHighKeeper = I3C_FUNCTION_ACTIVE;
    UtI3c_Init( &config );

    TEST_ASSERT_EQUAL_UINT32( I3C_CFGR_HKSDAEN, I3C1->CFGR & I3C_CFGR_HKSDAEN );
}


/**
 * \brief Verifies the initialization as target in the POLL mode.
 *
 * \par Expected results
 * I3C_CFGR has CRINIT cleared and EN set, the own device register is zero (no address), TIMINGR1 holds the
 * available time, the controller timing register is not written.
 */
void Ut_I3c_Init_TargetPoll_ConfiguresRegisters( void )
{
    UtI3c_Init_Tgt();

    TEST_ASSERT_EQUAL_UINT32( I3C_CFGR_EN, I3C1->CFGR );
    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->DEVR0 );
    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->TIMINGR0 );
    TEST_ASSERT_EQUAL_UINT32( ( UT_I3C_TIMING1_DEFAULT & I3C_TIMINGR1_AVAL_Msk ), I3C1->TIMINGR1 );
    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->IER );
}


/**
 * \brief Verifies that a second initialization of the same instance is rejected.
 *
 * \par Expected results
 * The second I3c_Init() returns the error without a call of another module and keeps the instance running.
 */
void Ut_I3c_Init_AlreadyInitialized_ReturnsErrorAndKeepsInstance( void )
{
    const i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_POLL );

    UtI3c_Init_Ctrl();

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Init( &config ) );
    TEST_ASSERT_EQUAL_UINT32( ( I3C_CFGR_EN | I3C_CFGR_CRINIT | I3C_CFGR_TMODE ), I3C1->CFGR );

    UtI3c_Expect_Release( &config );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Deinit( I3C_PERIPH_1 ) );
}


/**
 * \brief Verifies that a failing clock enable fails the initialization.
 *
 * \par Expected results
 * Rcc_Set_PeriphActive() returning the error makes I3c_Init() return the error; the clock, the reset and the
 * registers of the instance are released and no register of the peripheral is written.
 */
void Ut_I3c_Init_ClockEnableFails_ReturnsErrorAndReleases( void )
{
    const i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_POLL );

    Rcc_Set_PeriphActive_ExpectAndReturn( (rcc_PeriphId_t)config.ClkSrc, RCC_REQUEST_ERROR );
    UtI3c_Expect_Release( &config );

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Init( &config ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->TIMINGR0 );
}


/**
 * \brief Verifies that an unknown kernel clock fails the initialization.
 *
 * \par Expected results
 * A kernel clock frequency of zero returns the error and releases the instance.
 */
void Ut_I3c_Init_KernelClockZero_ReturnsErrorAndReleases( void )
{
    const i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_POLL );

    utI3c_KernelClk = 0u;
    UtI3c_Expect_Clock( &config );
    UtI3c_Expect_Release( &config );

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Init( &config ) );
}


/**
 * \brief Verifies that a kernel clock below twice the push-pull frequency is rejected.
 *
 * \par Expected results
 * A 6 MHz kernel clock with 4 MHz push-pull returns the error before a register is written.
 */
void Ut_I3c_Init_KernelClockTooSlow_ReturnsErrorBeforeRegisterWrite( void )
{
    const i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_POLL );

    utI3c_KernelClk = UT_I3C_SLOW_CLK;
    UtI3c_Expect_Clock( &config );
    UtI3c_Expect_Release( &config );

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Init( &config ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->TIMINGR0 );
}


/**
 * \brief Verifies that a timing the registers cannot represent is rejected.
 *
 * \par Expected results
 * 100 kHz push-pull at a 100 MHz kernel clock needs more than 256 cycles per phase: the error is returned.
 */
void Ut_I3c_Init_TimingNotRepresentable_ReturnsError( void )
{
    i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_POLL );

    config.PushPullFreq = UT_I3C_TOO_SLOW_PP;
    UtI3c_Expect_Clock( &config );
    UtI3c_Expect_Release( &config );

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Init( &config ) );
}


/**
 * \brief Verifies the rounding of the timing at a low kernel clock.
 *
 * \par Expected results
 * At 32 MHz and 12.5 MHz push-pull the minimum high and low time of 32 ns apply, the achieved push-pull
 * frequency is 8 MHz (not above the requested one).
 */
void Ut_I3c_Init_LowKernelClock_RoundsFrequencyDown( void )
{
    i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_POLL );
    i3c_FreqHz_t pushPull = 0u;
    i3c_FreqHz_t openDrain = 0u;

    config.PushPullFreq = I3C_PUSH_PULL_FREQ_MAX;
    utI3c_KernelClk     = UT_I3C_LOW_CLK;
    UtI3c_Init( &config );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_BusFreq( I3C_PERIPH_1, &pushPull, &openDrain ) );
    TEST_ASSERT_EQUAL_UINT32( UT_I3C_LOW_CLK_PP, pushPull );
    TEST_ASSERT_TRUE( pushPull <= I3C_PUSH_PULL_FREQ_MAX );
    TEST_ASSERT_TRUE( openDrain <= config.OpenDrainFreq );
}


/**
 * \brief Verifies that the read-back verification fails the initialization.
 *
 * \par Expected results
 * With a model that clears I3C_CFGR.CRINIT after every store the verification fails: the error is returned and
 * the instance is released.
 */
void Ut_I3c_Init_ConfigNotKept_ReturnsErrorAndReleases( void )
{
    const i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_POLL );

    UtI3c_Expect_Clock( &config );
    UtI3c_Expect_Release( &config );
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtI3c_HwModel_CfgrNotKept ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Init( &config ) );
}


/**
 * \brief Verifies that an enable that does not stay set fails the initialization.
 *
 * \par Expected results
 * With a model that clears I3C_CFGR.EN after every store I3c_Init() returns the error and releases the instance.
 */
void Ut_I3c_Init_EnableNotKept_ReturnsErrorAndReleases( void )
{
    const i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_POLL );

    UtI3c_Expect_Clock( &config );
    UtI3c_Expect_Release( &config );
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtI3c_HwModel_EnNotKept ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Init( &config ) );
}


/**
 * \brief Verifies the pin configuration and the SDA errata workaround of the controller.
 *
 * \note    Device errata bug AB#2093 (ES0565 Rev 7 2.15.3, ES0621 Rev 2 2.13.3, ES0561 Rev 4 2.10.3 "I3C controller:
 *          unexpected first frame with a 0x7F address when the I3C peripheral is enabled"): SDA read low when the
 *          peripheral is enabled starts a frame; the test proves the workaround sequence (pull-up on, wait for the
 *          high level, pull-up off, then the peripheral is initialized and enabled).
 *
 * \par Expected results
 * Both pins are initialized as alternate function without pull; the SDA pull-up is enabled, the level is read
 * until it is high and the pull-up is disabled before the peripheral is configured.
 */
void Ut_I3c_Init_ControllerWithPins_AppliesSdaWorkaround( void )
{
    i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_POLL );

    config.SclPin = I3C_SCL_PIN_I3C1_PB6;
    config.SdaPin = I3C_SDA_PIN_I3C1_PB7;
    UtI3c_Expect_Clock( &config );
    Gpio_Init_StubWithCallback( UtI3c_GpioInitStub );
    Gpio_Set_PinPull_ExpectAndReturn( GPIO_PORT_B, GPIO_PIN_ID_7, GPIO_PIN_PULL_UP, GPIO_REQUEST_OK );
    Gpio_Get_PinLevel_StubWithCallback( UtI3c_GpioLevelHighStub );
    Gpio_Set_PinPull_ExpectAndReturn( GPIO_PORT_B, GPIO_PIN_ID_7, GPIO_PIN_PULL_NONE, GPIO_REQUEST_OK );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Init( &config ) );

    TEST_ASSERT_EQUAL_UINT32( UT_I3C_GPIO_CFG_CNT, utI3c_GpioCfgCnt );
    TEST_ASSERT_EQUAL( GPIO_PORT_B, utI3c_GpioCfg[ 0u ].PortId );
    TEST_ASSERT_EQUAL( GPIO_PIN_ID_6, utI3c_GpioCfg[ 0u ].PinId );
    TEST_ASSERT_EQUAL( GPIO_PIN_MODE_ALTERNATE, utI3c_GpioCfg[ 0u ].PinMode );
    TEST_ASSERT_EQUAL( GPIO_PIN_PULL_NONE, utI3c_GpioCfg[ 0u ].PinPull );
    TEST_ASSERT_EQUAL( GPIO_ALT_FUNC_3, utI3c_GpioCfg[ 0u ].PinAltFunction );
    TEST_ASSERT_EQUAL( GPIO_PIN_ID_7, utI3c_GpioCfg[ 1u ].PinId );
    TEST_ASSERT_EQUAL( 1u, utI3c_GpioLevelCalls );
    TEST_ASSERT_EQUAL_UINT32( I3C_CFGR_EN, I3C1->CFGR & I3C_CFGR_EN );
}


/**
 * \brief Verifies that a target does not apply the SDA workaround.
 *
 * \note    Device errata bug AB#2093 (ES0565 Rev 7 2.15.3, ES0621 Rev 2 2.13.3, ES0561 Rev 4 2.10.3 "I3C controller:
 *          unexpected first frame with a 0x7F address when the I3C peripheral is enabled"): the erratum concerns
 *          the controller role only; the test proves that the workaround is not applied to a target.
 *
 * \par Expected results
 * The target role configures the pins only; the pull-up and the level of the Gpio module are not used.
 */
void Ut_I3c_Init_TargetWithPins_NoSdaWorkaround( void )
{
    i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_TARGET, I3C_XFER_MODE_POLL );

    config.SclPin = I3C_SCL_PIN_I3C1_PB6;
    config.SdaPin = I3C_SDA_PIN_I3C1_PB7;
    UtI3c_Expect_Clock( &config );
    Gpio_Init_StubWithCallback( UtI3c_GpioInitStub );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Init( &config ) );
    TEST_ASSERT_EQUAL_UINT32( UT_I3C_GPIO_CFG_CNT, utI3c_GpioCfgCnt );
}


/**
 * \brief Verifies that an SDA line that never reads high fails the initialization.
 *
 * \note    Device errata bug AB#2093 (ES0565 Rev 7 2.15.3, ES0621 Rev 2 2.13.3, ES0561 Rev 4 2.10.3 "I3C controller:
 *          unexpected first frame with a 0x7F address when the I3C peripheral is enabled"): with SDA held low the
 *          peripheral would start the unexpected frame; the test proves that the initialization fails
 *          and the pull-up is released.
 *
 * \par Expected results
 * The bounded wait ends, the pull-up is disabled again, I3c_Init() returns the error and releases the instance.
 */
void Ut_I3c_Init_SdaStaysLow_ReturnsErrorAndDisablesPullUp( void )
{
    i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_POLL );

    config.SclPin = I3C_SCL_PIN_I3C1_PB6;
    config.SdaPin = I3C_SDA_PIN_I3C1_PB7;
    UtI3c_Expect_Clock( &config );
    Gpio_Init_StubWithCallback( UtI3c_GpioInitStub );
    Gpio_Set_PinPull_ExpectAndReturn( GPIO_PORT_B, GPIO_PIN_ID_7, GPIO_PIN_PULL_UP, GPIO_REQUEST_OK );
    Gpio_Get_PinLevel_StubWithCallback( UtI3c_GpioLevelLowStub );
    Gpio_Set_PinPull_ExpectAndReturn( GPIO_PORT_B, GPIO_PIN_ID_7, GPIO_PIN_PULL_NONE, GPIO_REQUEST_OK );
    UtI3c_Expect_Release( &config );

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Init( &config ) );
    TEST_ASSERT_TRUE( 1u < utI3c_GpioLevelCalls );
}


/**
 * \brief Verifies that a failing pin initialization fails the initialization.
 *
 * \par Expected results
 * Gpio_Init() returning the error makes I3c_Init() return the error and release the instance.
 */
void Ut_I3c_Init_GpioFails_ReturnsErrorAndReleases( void )
{
    i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_POLL );

    config.SclPin = I3C_SCL_PIN_I3C1_PB6;
    UtI3c_Expect_Clock( &config );
    Gpio_Init_ExpectAnyArgsAndReturn( GPIO_REQUEST_ERROR );
    UtI3c_Expect_Release( &config );

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Init( &config ) );
}


/**
 * \brief Verifies the initialization in the ISR mode.
 *
 * \par Expected results
 * The handler is registered for the event and the error line with the configured priority, both lines are
 * enabled, the interrupts of the controller are enabled in I3C_IER (without the FIFO interrupts).
 */
void Ut_I3c_Init_ControllerIsr_RegistersInterrupts( void )
{
    i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_ISR );

    UtI3c_Expect_Clock( &config );
    Nvic_Set_PeriphIrq_Handler_StubWithCallback( UtI3c_NvicHandlerStub );
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_I3C1_EV, config.IrqPriority, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( NVIC_PERIPH_IRQ_I3C1_EV, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_I3C1_ER, config.IrqPriority, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( NVIC_PERIPH_IRQ_I3C1_ER, NVIC_REQUEST_OK );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Init( &config ) );

    TEST_ASSERT_NOT_NULL( utI3c_Isr );
    TEST_ASSERT_EQUAL_UINT32( I3C_IER_CTRL_MASK, I3C1->IER );
}


/**
 * \brief Verifies that a failing interrupt setup fails the initialization.
 *
 * \par Expected results
 * Nvic_Set_PeriphIrq_Handler() returning the error makes I3c_Init() return the error; the interrupt lines are
 * disabled again by the release.
 */
void Ut_I3c_Init_NvicFails_ReturnsErrorAndReleases( void )
{
    i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_ISR );

    UtI3c_Expect_Clock( &config );
    Nvic_Set_PeriphIrq_Handler_ExpectAnyArgsAndReturn( NVIC_REQUEST_ERROR );
    UtI3c_Expect_Release( &config );

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Init( &config ) );
}


/**
 * \brief Verifies the initialization in the DMA mode.
 *
 * \par Expected results
 * The channels of the transmit and the receive FIFO are initialized with the requests of the instance, 8-bit
 * transfers and the register addresses, the error mask without the suspension (the module suspends a channel
 * itself to stop it); I3C_CFGR enables both DMA requests.
 */
void Ut_I3c_Init_ControllerDma_InitializesChannels( void )
{
    i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_DMA );

    UtI3c_Expect_Clock( &config );
    UtI3c_Expect_DmaInit( &config );
    UtI3c_Expect_Irq( &config );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Init( &config ) );

    TEST_ASSERT_EQUAL_UINT32( UT_I3C_DMA_CFG_CNT, utI3c_DmaCfgCnt );
    TEST_ASSERT_EQUAL( GPDMA_REQ_I3C1_TX, utI3c_DmaXfer[ 0u ].RequestSource );
    TEST_ASSERT_EQUAL( GPDMA_DIR_MEMORY_TO_PERIPH, utI3c_DmaXfer[ 0u ].Direction );
    TEST_ASSERT_EQUAL_UINT32( (uint32_t)&I3C1->TDR, utI3c_DmaXfer[ 0u ].DestinationAddr );
    TEST_ASSERT_EQUAL( GPDMA_REQ_I3C1_RX, utI3c_DmaXfer[ 1u ].RequestSource );
    TEST_ASSERT_EQUAL( GPDMA_DIR_PERIPH_TO_MEMORY, utI3c_DmaXfer[ 1u ].Direction );
    TEST_ASSERT_EQUAL_UINT32( (uint32_t)&I3C1->RDR, utI3c_DmaXfer[ 1u ].SourceAddr );
    TEST_ASSERT_EQUAL( GPDMA_DATA_SIZE_8BITS, utI3c_DmaXfer[ 0u ].SourceDataSize );
    TEST_ASSERT_EQUAL( config.TxDmaChannelId, utI3c_DmaCfg[ 0u ].ChannelId );
    TEST_ASSERT_EQUAL( config.RxDmaChannelId, utI3c_DmaCfg[ 1u ].ChannelId );
    TEST_ASSERT_NOT_NULL( utI3c_DmaCfg[ 0u ].ErrorIsr );
    TEST_ASSERT_BITS_HIGH( GPDMA_ERROR_TRANSFER, utI3c_DmaCfg[ 0u ].ErrorMask );
    TEST_ASSERT_BITS_LOW( GPDMA_ERROR_SUSPENDED, utI3c_DmaCfg[ 0u ].ErrorMask );
    TEST_ASSERT_BITS_LOW( GPDMA_ERROR_SUSPENDED, utI3c_DmaCfg[ 1u ].ErrorMask );
    TEST_ASSERT_EQUAL_UINT32( ( I3C_CFGR_TXDMAEN | I3C_CFGR_RXDMAEN ),
                              I3C1->CFGR & ( I3C_CFGR_TXDMAEN | I3C_CFGR_RXDMAEN ) );
}


/**
 * \brief Verifies that a failing DMA channel fails the initialization.
 *
 * \par Expected results
 * Gpdma_Init() returning the error makes I3c_Init() return the error; the release stops and releases the
 * channels that were initialized.
 */
void Ut_I3c_Init_GpdmaFails_ReturnsErrorAndReleases( void )
{
    i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_DMA );

    UtI3c_Expect_Clock( &config );
    Gpdma_Get_DefaultConfig_ExpectAnyArgsAndReturn( GPDMA_REQUEST_OK );
    Gpdma_Init_ExpectAnyArgsAndReturn( GPDMA_REQUEST_ERROR );
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_I3C1_EV, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_I3C1_ER, NVIC_REQUEST_OK );
    Rcc_Set_ResetActive_ExpectAndReturn( (rcc_PeriphId_t)config.ClkSrc, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( (rcc_PeriphId_t)config.ClkSrc, RCC_REQUEST_OK );
    Rcc_Set_PeriphInactive_ExpectAndReturn( (rcc_PeriphId_t)config.ClkSrc, RCC_REQUEST_OK );

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Init( &config ) );
}


/**
 * \brief Verifies the deinitialization of a polled instance.
 *
 * \par Expected results
 * The peripheral is disabled, the interrupt enables are cleared, the peripheral is reset and its clock is
 * disabled through the Rcc module; the instance can be initialized again.
 */
void Ut_I3c_Deinit_Initialized_DisablesAndReleasesInstance( void )
{
    const i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_POLL );

    UtI3c_Init_Ctrl();
    I3C1->IER = UT_I3C_ALL_BITS;

    UtI3c_Expect_Release( &config );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Deinit( I3C_PERIPH_1 ) );

    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->CFGR & I3C_CFGR_EN );
    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->IER );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Deinit( I3C_PERIPH_1 ) );

    UtI3c_Init( &config );
}


/**
 * \brief Verifies the rejected deinitialization.
 *
 * \par Expected results
 * An instance that is not initialized and an invalid instance ID return the error without a call of another module.
 */
void Ut_I3c_Deinit_NotInitializedOrInvalid_ReturnsError( void )
{
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Deinit( I3C_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Deinit( I3C_PERIPH_CNT ) );
}


/**
 * \brief Verifies the deinitialization in the ISR mode.
 *
 * \par Expected results
 * The two interrupt lines are disabled in the Nvic before the peripheral is released.
 */
void Ut_I3c_Deinit_IsrMode_DisablesInterruptLines( void )
{
    i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_ISR );

    UtI3c_Expect_Clock( &config );
    UtI3c_Expect_Irq( &config );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Init( &config ) );

    UtI3c_Expect_Release( &config );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Deinit( I3C_PERIPH_1 ) );
}


/**
 * \brief Verifies the deinitialization in the DMA mode.
 *
 * \par Expected results
 * The channels of both FIFOs are stopped, their interrupts are disabled and they are released.
 */
void Ut_I3c_Deinit_DmaMode_ReleasesChannels( void )
{
    i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_DMA );

    UtI3c_Expect_Clock( &config );
    UtI3c_Expect_DmaInit( &config );
    UtI3c_Expect_Irq( &config );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Init( &config ) );

    UtI3c_Expect_Release( &config );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Deinit( I3C_PERIPH_1 ) );
}


/**
 * \brief Verifies the task of a polled instance.
 *
 * \par Expected results
 * I3c_Task() serves the events of an initialized polled instance (a hot-join request calls the callback) and
 * does nothing for a not initialized instance.
 */
void Ut_I3c_Task_PolledInstance_ServesEvents( void )
{
    i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_POLL );

    I3C1->EVR = I3C_EVR_HJF;
    I3c_Task();
    TEST_ASSERT_EQUAL_UINT32( 0u, utI3c_HjCallCnt );

    config.HotJoinCallback = UtI3c_HotJoinCallback;
    UtI3c_Init( &config );
    I3c_Task();

    TEST_ASSERT_EQUAL_UINT32( 1u, utI3c_HjCallCnt );
    TEST_ASSERT_EQUAL_UINT32( I3C_EVR_HJF, I3C1->CEVR );
}


/**
 * \brief Verifies that the task does not serve an instance that uses the interrupt.
 *
 * \par Expected results
 * In the ISR mode I3c_Task() leaves the events to the interrupt: no callback is called.
 */
void Ut_I3c_Task_IsrInstance_DoesNotServeEvents( void )
{
    i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_ISR );

    config.HotJoinCallback = UtI3c_HotJoinCallback;
    UtI3c_Expect_Clock( &config );
    UtI3c_Expect_Irq( &config );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Init( &config ) );

    I3C1->EVR = I3C_EVR_HJF;
    I3c_Task();

    TEST_ASSERT_EQUAL_UINT32( 0u, utI3c_HjCallCnt );
}


/**
 * \brief Verifies the reading of the achieved frequencies.
 *
 * \par Expected results
 * Null pointers, an invalid instance, a not initialized instance and the target role return the error.
 */
void Ut_I3c_GetBusFreq_InvalidCases_ReturnError( void )
{
    i3c_FreqHz_t pushPull = 0u;
    i3c_FreqHz_t openDrain = 0u;

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Get_BusFreq( I3C_PERIPH_1, &pushPull, &openDrain ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Get_BusFreq( I3C_PERIPH_CNT, &pushPull, &openDrain ) );

    UtI3c_Init_Tgt();

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Get_BusFreq( I3C_PERIPH_1, &pushPull, &openDrain ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Get_BusFreq( I3C_PERIPH_1, I3C_NULL_PTR, &openDrain ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Get_BusFreq( I3C_PERIPH_1, &pushPull, I3C_NULL_PTR ) );
}


/**
 * \brief Verifies the own address of the controller and of the target.
 *
 * \par Expected results
 * The controller reports its configured address, a target without an assigned address returns the error and
 * returns the address once the device register holds a valid one.
 */
void Ut_I3c_GetOwnAddress_ControllerAndTarget_ReportAddress( void )
{
    i3c_Address_t address = 0u;

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Get_OwnAddress( I3C_PERIPH_1, &address ) );

    UtI3c_Init_Tgt();
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Get_OwnAddress( I3C_PERIPH_1, &address ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Get_OwnAddress( I3C_PERIPH_1, I3C_NULL_PTR ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Get_OwnAddress( I3C_PERIPH_CNT, &address ) );

    I3C1->DEVR0 = ( ( (uint32_t)UT_I3C_ADDR_A << I3C_DEVR0_DA_Pos ) | I3C_DEVR0_DAVAL );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_OwnAddress( I3C_PERIPH_1, &address ) );
    TEST_ASSERT_EQUAL_UINT8( UT_I3C_ADDR_A, address );
}


/**
 * \brief Verifies the own address of an initialized controller.
 *
 * \par Expected results
 * I3c_Get_OwnAddress() returns the address written by the initialization.
 */
void Ut_I3c_GetOwnAddress_Controller_ReturnsConfiguredAddress( void )
{
    i3c_Address_t address = 0u;

    UtI3c_Init_Ctrl();

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_OwnAddress( I3C_PERIPH_1, &address ) );
    TEST_ASSERT_EQUAL_UINT8( I3C_DEFAULT_OWN_ADDRESS, address );
}


/**
 * \brief Verifies the interrupt priority of an instance.
 *
 * \par Expected results
 * Set sets the priority of both lines, Get reads the event line; a not initialized or invalid instance and a
 * failing Nvic return the error.
 */
void Ut_I3c_IrqPriority_SetAndGet_UseBothLines( void )
{
    i3c_IrqPrio_t irqPrio = 0u;
    i3c_Config_t  config  = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_ISR );

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_IrqPriority( I3C_PERIPH_1, UT_I3C_PRIO ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Get_IrqPriority( I3C_PERIPH_1, &irqPrio ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_IrqPriority( I3C_PERIPH_CNT, UT_I3C_PRIO ) );

    UtI3c_Expect_Clock( &config );
    UtI3c_Expect_Irq( &config );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Init( &config ) );

    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_I3C1_EV, UT_I3C_PRIO, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_I3C1_ER, UT_I3C_PRIO, NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_IrqPriority( I3C_PERIPH_1, UT_I3C_PRIO ) );

    Nvic_Get_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_I3C1_EV, I3C_NULL_PTR, NVIC_REQUEST_OK );
    Nvic_Get_PeriphIrq_Prio_IgnoreArg_irqPrio();
    Nvic_Get_PeriphIrq_Prio_ReturnThruPtr_irqPrio( &utI3c_PrioValue );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_IrqPriority( I3C_PERIPH_1, &irqPrio ) );
    TEST_ASSERT_EQUAL_UINT32( utI3c_PrioValue, irqPrio );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Get_IrqPriority( I3C_PERIPH_1, I3C_NULL_PTR ) );

    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_I3C1_EV, UT_I3C_PRIO, NVIC_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_IrqPriority( I3C_PERIPH_1, UT_I3C_PRIO ) );
}


/**
 * \brief Verifies the SCL pin initialization.
 *
 * \par Expected results
 * The pin is configured as alternate function of its AF number without pull; a failing Gpio returns the error.
 */
void Ut_I3c_InitSclGpio_ValidPin_ConfiguresAlternateFunction( void )
{
    Gpio_Init_StubWithCallback( UtI3c_GpioInitStub );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_InitSclGpio( I3C_SCL_PIN_I3C1_PB6 ) );

    TEST_ASSERT_EQUAL( GPIO_PORT_B, utI3c_GpioCfg[ 0u ].PortId );
    TEST_ASSERT_EQUAL( GPIO_PIN_ID_6, utI3c_GpioCfg[ 0u ].PinId );
    TEST_ASSERT_EQUAL( GPIO_PIN_MODE_ALTERNATE, utI3c_GpioCfg[ 0u ].PinMode );
    TEST_ASSERT_EQUAL( GPIO_PIN_PULL_NONE, utI3c_GpioCfg[ 0u ].PinPull );
    TEST_ASSERT_EQUAL( GPIO_PIN_SPEED_VERY_HIGH, utI3c_GpioCfg[ 0u ].PinSpeed );
    TEST_ASSERT_EQUAL( GPIO_PIN_OUTPUT_PUSHPULL, utI3c_GpioCfg[ 0u ].PinOutType );
    TEST_ASSERT_EQUAL( GPIO_ALT_FUNC_3, utI3c_GpioCfg[ 0u ].PinAltFunction );
}


/**
 * \brief Verifies the SDA pin initialization and its error path.
 *
 * \par Expected results
 * The SDA pin is configured like the SCL pin; Gpio_Init() returning the error returns I3C_REQUEST_ERROR.
 */
void Ut_I3c_InitSdaGpio_ValidAndFailing_ReturnsState( void )
{
    Gpio_Init_StubWithCallback( UtI3c_GpioInitStub );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_InitSdaGpio( I3C_SDA_PIN_I3C1_PB7 ) );
    TEST_ASSERT_EQUAL( GPIO_PIN_ID_7, utI3c_GpioCfg[ 0u ].PinId );
    TEST_ASSERT_EQUAL( GPIO_ALT_FUNC_3, utI3c_GpioCfg[ 0u ].PinAltFunction );

    Gpio_Init_StubWithCallback( I3C_NULL_PTR );
    Gpio_Init_ExpectAnyArgsAndReturn( GPIO_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_InitSdaGpio( I3C_SDA_PIN_I3C1_PB7 ) );
}


/* ---------------------------- Device table and bus ------------------------- */

/**
 * \brief Verifies the rejected device tables.
 *
 * \par Expected results
 * A not initialized instance, the target role, too many entries, a missing buffer, reserved, own, duplicate and
 * out of range addresses, an invalid type and an invalid speed class return I3C_REQUEST_ERROR and keep the
 * table empty.
 */
void Ut_I3c_SetDeviceTable_InvalidCases_ReturnError( void )
{
    i3c_Device_t devices[ I3C_DEVICE_CNT_MAX + 1u ] = { { 0 } };
    i3c_DeviceCnt_t deviceCnt = 0u;

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_DeviceTable( I3C_PERIPH_1, devices, 1u ) );

    UtI3c_Init_Ctrl();
    devices[ 0u ].Type = I3C_DEVICE_I3C;
    devices[ 0u ].DynamicAddress = UT_I3C_ADDR_A;

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_DeviceTable( I3C_PERIPH_1, devices, I3C_DEVICE_CNT_MAX + 1u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_DeviceTable( I3C_PERIPH_1, I3C_NULL_PTR, 1u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_DeviceTable( I3C_PERIPH_CNT, devices, 1u ) );

    devices[ 0u ].DynamicAddress = I3C_BROADCAST_ADDRESS;
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_DeviceTable( I3C_PERIPH_1, devices, 1u ) );
    devices[ 0u ].DynamicAddress = UT_I3C_ADDR_RESERVED_NEAR;
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_DeviceTable( I3C_PERIPH_1, devices, 1u ) );
    devices[ 0u ].DynamicAddress = I3C_DYNAMIC_ADDRESS_FIRST - 1u;
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_DeviceTable( I3C_PERIPH_1, devices, 1u ) );
    devices[ 0u ].DynamicAddress = I3C_DEFAULT_OWN_ADDRESS;
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_DeviceTable( I3C_PERIPH_1, devices, 1u ) );
    devices[ 0u ].DynamicAddress = I3C_ADDRESS_MASK + 1u;
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_DeviceTable( I3C_PERIPH_1, devices, 1u ) );

    devices[ 0u ].DynamicAddress = UT_I3C_ADDR_A;
    devices[ 1u ] = devices[ 0u ];
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_DeviceTable( I3C_PERIPH_1, devices, UT_I3C_TABLE_CNT ) );

    devices[ 1u ].Type = I3C_DEVICE_TYPE_CNT;
    devices[ 1u ].DynamicAddress = UT_I3C_ADDR_B;
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_DeviceTable( I3C_PERIPH_1, devices, UT_I3C_TABLE_CNT ) );

    devices[ 1u ].Type = I3C_DEVICE_I2C;
    devices[ 1u ].StaticAddress = UT_I3C_ADDR_I2C;
    devices[ 1u ].I2cSpeed = I3C_I2C_SPEED_CNT;
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_DeviceTable( I3C_PERIPH_1, devices, UT_I3C_TABLE_CNT ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_DeviceTable( I3C_PERIPH_1, devices, &deviceCnt ) );
    TEST_ASSERT_EQUAL_UINT8( 0u, deviceCnt );
}


/**
 * \brief Verifies that a target has no device table.
 *
 * \par Expected results
 * Set and Get of the device table return the error for the target role.
 */
void Ut_I3c_DeviceTable_TargetRole_ReturnsError( void )
{
    i3c_Device_t    devices[ I3C_DEVICE_CNT_MAX ] = { { 0 } };
    i3c_DeviceCnt_t deviceCnt = 0u;

    UtI3c_Init_Tgt();

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_DeviceTable( I3C_PERIPH_1, devices, 0u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Get_DeviceTable( I3C_PERIPH_1, devices, &deviceCnt ) );
}


/**
 * \brief Verifies the reading of the device table with invalid parameters.
 *
 * \par Expected results
 * A null buffer, a null count, an invalid and a not initialized instance return the error.
 */
void Ut_I3c_GetDeviceTable_InvalidParameters_ReturnError( void )
{
    i3c_Device_t    devices[ I3C_DEVICE_CNT_MAX ] = { { 0 } };
    i3c_DeviceCnt_t deviceCnt = 0u;

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Get_DeviceTable( I3C_PERIPH_1, devices, &deviceCnt ) );

    UtI3c_Init_Ctrl();

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Get_DeviceTable( I3C_PERIPH_1, I3C_NULL_PTR, &deviceCnt ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Get_DeviceTable( I3C_PERIPH_1, devices, I3C_NULL_PTR ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Get_DeviceTable( I3C_PERIPH_CNT, devices, &deviceCnt ) );
}


/**
 * \brief Verifies the device table with an I3C device.
 *
 * \par Expected results
 * The entry is stored and read back; DEVR1 holds the dynamic address and the in-band interrupt acknowledge with
 * the payload, the unused target device registers are cleared.
 */
void Ut_I3c_SetDeviceTable_I3cDevice_ProgramsDeviceRegisters( void )
{
    i3c_Device_t    devices[ I3C_DEVICE_CNT_MAX ] = { { 0 } };
    i3c_DeviceCnt_t deviceCnt = 0u;

    UtI3c_Init_Ctrl();
    I3C1->DEVRX[ 1u ] = UT_I3C_ALL_BITS;
    UtI3c_Set_Table();

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_DeviceTable( I3C_PERIPH_1, devices, &deviceCnt ) );
    TEST_ASSERT_EQUAL_UINT8( UT_I3C_TABLE_CNT, deviceCnt );
    TEST_ASSERT_EQUAL( I3C_DEVICE_I3C, devices[ 0u ].Type );
    TEST_ASSERT_EQUAL_UINT8( UT_I3C_ADDR_A, devices[ 0u ].DynamicAddress );
    TEST_ASSERT_EQUAL( I3C_DEVICE_I2C, devices[ 1u ].Type );
    TEST_ASSERT_EQUAL_UINT8( UT_I3C_ADDR_I2C, devices[ 1u ].StaticAddress );
    TEST_ASSERT_EQUAL_UINT32( ( ( (uint32_t)UT_I3C_ADDR_A << I3C_DEVRX_DA_Pos ) | I3C_DEVRX_IBIACK | I3C_DEVRX_IBIDEN ),
                              I3C1->DEVRX[ 0u ] );
    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->DEVRX[ 1u ] );
}


/**
 * \brief Verifies the open-drain timing with a Fast-mode plus device.
 *
 * \par Expected results
 * A Fm+ device sets the legacy SCL high time of its class (400 ns) in TIMINGR0, the open-drain low time of the
 * default timing is long enough and stays; the open-drain frequency is not changed.
 */
void Ut_I3c_SetDeviceTable_FmPlusDevice_SetsLegacyHighTime( void )
{
    i3c_FreqHz_t pushPull = 0u;
    i3c_FreqHz_t openDrain = 0u;

    UtI3c_Init_Ctrl();
    UtI3c_Set_Table();

    TEST_ASSERT_EQUAL_UINT32( UT_I3C_TIMING0_FMP, I3C1->TIMINGR0 );
    TEST_ASSERT_EQUAL( I3C_FUNCTION_ACTIVE, I3c_Reg_Get_EnableState( I3C_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_BusFreq( I3C_PERIPH_1, &pushPull, &openDrain ) );
    TEST_ASSERT_EQUAL_UINT32( UT_I3C_OD_FREQ, openDrain );
}


/**
 * \brief Verifies that a Fast-mode device lowers the open-drain frequency.
 *
 * \par Expected results
 * A Fm device needs a low time of 1500 ns: TIMINGR0 holds 150 cycles, the achieved open-drain frequency is
 * lower than the requested one and is reported by I3c_Get_BusFreq().
 */
void Ut_I3c_SetDeviceTable_FmDevice_LowersOpenDrainFrequency( void )
{
    i3c_Device_t devices[ 1u ] = { { 0 } };
    i3c_FreqHz_t pushPull = 0u;
    i3c_FreqHz_t openDrain = 0u;

    UtI3c_Init_Ctrl();
    devices[ 0u ].Type          = I3C_DEVICE_I2C;
    devices[ 0u ].StaticAddress = UT_I3C_ADDR_I2C;
    devices[ 0u ].I2cSpeed      = I3C_I2C_SPEED_FM;

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_DeviceTable( I3C_PERIPH_1, devices, 1u ) );

    TEST_ASSERT_EQUAL_UINT32( UT_I3C_TIMING0_FM, I3C1->TIMINGR0 );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_BusFreq( I3C_PERIPH_1, &pushPull, &openDrain ) );
    TEST_ASSERT_EQUAL_UINT32( UT_I3C_OD_FREQ_FM, openDrain );
    TEST_ASSERT_EQUAL_UINT32( UT_I3C_PP_FREQ, pushPull );
}


/**
 * \brief Verifies that a device class that the registers cannot represent is rejected.
 *
 * \par Expected results
 * A Standard-mode device at a 100 MHz kernel clock needs more than 256 cycles: the error is returned and the
 * table and the timing register stay unchanged.
 */
void Ut_I3c_SetDeviceTable_StandardModeNotRepresentable_ReturnsErrorKeepsTable( void )
{
    i3c_Device_t    devices[ 1u ] = { { 0 } };
    i3c_DeviceCnt_t deviceCnt = 0u;

    UtI3c_Init_Ctrl();
    devices[ 0u ].Type          = I3C_DEVICE_I2C;
    devices[ 0u ].StaticAddress = UT_I3C_ADDR_I2C;
    devices[ 0u ].I2cSpeed      = I3C_I2C_SPEED_SM;

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_DeviceTable( I3C_PERIPH_1, devices, 1u ) );

    TEST_ASSERT_EQUAL_UINT32( UT_I3C_TIMING0_DEFAULT, I3C1->TIMINGR0 );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_DeviceTable( I3C_PERIPH_1, devices, &deviceCnt ) );
    TEST_ASSERT_EQUAL_UINT8( 0u, deviceCnt );
}


/**
 * \brief Verifies that removing the legacy device restores the timing.
 *
 * \par Expected results
 * An empty table after a table with a Fm device writes the default timing again.
 */
void Ut_I3c_SetDeviceTable_RemoveLegacyDevice_RestoresTiming( void )
{
    i3c_Device_t devices[ 1u ] = { { 0 } };

    UtI3c_Init_Ctrl();
    devices[ 0u ].Type          = I3C_DEVICE_I2C;
    devices[ 0u ].StaticAddress = UT_I3C_ADDR_I2C;
    devices[ 0u ].I2cSpeed      = I3C_I2C_SPEED_FM;
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_DeviceTable( I3C_PERIPH_1, devices, 1u ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_DeviceTable( I3C_PERIPH_1, I3C_NULL_PTR, 0u ) );

    TEST_ASSERT_EQUAL_UINT32( UT_I3C_TIMING0_DEFAULT, I3C1->TIMINGR0 );
}


/**
 * \brief Verifies that the device table cannot change while a request runs.
 *
 * \par Expected results
 * I3c_Set_DeviceTable() returns the error while a write request is running.
 */
void Ut_I3c_SetDeviceTable_RequestRunning_ReturnsError( void )
{
    const i3c_Data_t data[ 1u ] = { UT_I3C_DATA_BYTE };

    UtI3c_Init_Ctrl();
    UtI3c_Set_Table();
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_Write( I3C_PERIPH_1, UT_I3C_ADDR_A, data, 1u ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_DeviceTable( I3C_PERIPH_1, I3C_NULL_PTR, 0u ) );
}


/**
 * \brief Verifies the reset of the dynamic addresses.
 *
 * \par Expected results
 * The RSTDAA broadcast CCC is sent as one control word with the stop bit; after its end the I3C devices of
 * the table have no valid address and DEVR1 is cleared, the legacy device keeps its static address.
 */
void Ut_I3c_SetBusInit_FrameCompletes_SendsRstdaaAndInvalidatesAddresses( void )
{
    i3c_Device_t    devices[ I3C_DEVICE_CNT_MAX ] = { { 0 } };
    i3c_DeviceCnt_t deviceCnt = 0u;

    UtI3c_Init_Ctrl();
    UtI3c_Set_Table();
    I3C1->EVR = I3C_EVR_FCF;
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtI3c_HwModel_Capture ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_BusInit( I3C_PERIPH_1 ) );

    TEST_ASSERT_EQUAL_UINT32( 1u, utI3c_CrSeqCnt );
    TEST_ASSERT_EQUAL_UINT32( UtI3c_Get_CccWord( I3C_FUNCTION_ACTIVE, I3C_CCC_RSTDAA, 0u ), utI3c_CrSeq[ 0u ] );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_DeviceTable( I3C_PERIPH_1, devices, &deviceCnt ) );
    TEST_ASSERT_EQUAL( I3C_FUNCTION_INACTIVE, devices[ 0u ].AddressState );
    TEST_ASSERT_EQUAL_UINT8( UT_I3C_ADDR_I2C, devices[ 1u ].StaticAddress );
    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->DEVRX[ 0u ] );
}


/**
 * \brief Verifies the bus init with an error of the bus.
 *
 * \par Expected results
 * An address NACK error of the frame returns I3C_REQUEST_ERROR and keeps the addresses of the table.
 */
void Ut_I3c_SetBusInit_FrameFails_ReturnsErrorKeepsTable( void )
{
    i3c_Device_t    devices[ I3C_DEVICE_CNT_MAX ] = { { 0 } };
    i3c_DeviceCnt_t deviceCnt = 0u;

    UtI3c_Init_Ctrl();
    UtI3c_Set_Table();
    I3C1->EVR = I3C_EVR_ERRF;
    I3C1->SER = I3C_SER_ANACK;

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_BusInit( I3C_PERIPH_1 ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_DeviceTable( I3C_PERIPH_1, devices, &deviceCnt ) );
    TEST_ASSERT_EQUAL( I3C_FUNCTION_ACTIVE, devices[ 0u ].AddressState );
}


/**
 * \brief Verifies that a frame that never ends is stopped by the bounded wait.
 *
 * \par Expected results
 * Without the frame completed event I3c_Set_BusInit() returns the error after the time limit, the request state
 * is the error with the aborted bit and the peripheral is enabled again.
 */
void Ut_I3c_SetBusInit_FrameNeverEnds_ReturnsErrorAfterTimeout( void )
{
    i3c_XferError_t xferError = I3C_XFER_ERROR_NONE;

    UtI3c_Init_Ctrl();

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_BusInit( I3C_PERIPH_1 ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_XferError( I3C_PERIPH_1, &xferError ) );
    TEST_ASSERT_EQUAL_UINT8( I3C_XFER_ERROR_ABORTED, xferError );
    TEST_ASSERT_EQUAL_UINT32( I3C_CFGR_EN, I3C1->CFGR & I3C_CFGR_EN );
}


/**
 * \brief Verifies the rejected bus init.
 *
 * \par Expected results
 * An invalid instance, a not initialized instance and the target role return the error.
 */
void Ut_I3c_SetBusInit_InvalidCases_ReturnError( void )
{
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_BusInit( I3C_PERIPH_CNT ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_BusInit( I3C_PERIPH_1 ) );

    UtI3c_Init_Tgt();

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_BusInit( I3C_PERIPH_1 ) );
}


/**
 * \brief Verifies the dynamic address assignment of one device.
 *
 * \par Expected results
 * The ENTDAA broadcast CCC is sent; the device that answers is stored in the table with the PID, the BCR and
 * the DCR and gets the first free address of the pool (the own address is skipped); the address is written to
 * the transmit FIFO; the count is one and DEVR1 is programmed.
 */
void Ut_I3c_SetDynamicAddressAssign_OneDevice_StoresDeviceAndWritesAddress( void )
{
    i3c_Device_t    devices[ I3C_DEVICE_CNT_MAX ] = { { 0 } };
    i3c_DeviceCnt_t deviceCnt = 0u;
    i3c_DeviceCnt_t discoveredCnt = 0u;

    UtI3c_Init_Ctrl();
    I3C1->RDR = UT_I3C_DATA_BYTE;
    I3C1->EVR = ( I3C_EVR_RXFNEF | I3C_EVR_TXFNFF | I3C_EVR_FCF );
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtI3c_HwModel_Capture ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_DynamicAddressAssign( I3C_PERIPH_1, &discoveredCnt ) );

    TEST_ASSERT_EQUAL_UINT8( 1u, discoveredCnt );
    TEST_ASSERT_EQUAL_UINT32( UtI3c_Get_CccWord( I3C_FUNCTION_ACTIVE, (i3c_Ccc_t)I3C_CCC_ENTDAA_CODE, 0u ), I3C1->CR );
    TEST_ASSERT_EQUAL_UINT32( I3C_DEFAULT_OWN_ADDRESS + 1u, I3C1->TDR );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_DeviceTable( I3C_PERIPH_1, devices, &deviceCnt ) );
    TEST_ASSERT_EQUAL_UINT8( 1u, deviceCnt );
    TEST_ASSERT_EQUAL_UINT8( I3C_DEFAULT_OWN_ADDRESS + 1u, devices[ 0u ].DynamicAddress );
    TEST_ASSERT_EQUAL_UINT16( UT_I3C_PID_HIGH_REPEATED, devices[ 0u ].PidHigh );
    TEST_ASSERT_EQUAL_UINT32( UT_I3C_PID_LOW_REPEATED, devices[ 0u ].PidLow );
    TEST_ASSERT_EQUAL_UINT8( UT_I3C_DATA_BYTE, devices[ 0u ].Bcr );
    TEST_ASSERT_EQUAL_UINT8( UT_I3C_DATA_BYTE, devices[ 0u ].Dcr );
    TEST_ASSERT_EQUAL( I3C_FUNCTION_ACTIVE, devices[ 0u ].AddressState );
    TEST_ASSERT_EQUAL_UINT32( ( (uint32_t)( I3C_DEFAULT_OWN_ADDRESS + 1u ) << I3C_DEVRX_DA_Pos ), I3C1->DEVRX[ 0u ] );
}


/**
 * \brief Verifies that a table entry with the same PID gets its requested address.
 *
 * \par Expected results
 * An entry with the PID of the answering device and the requested address 0x30 gets this address instead of the
 * first free address of the pool; no second entry is created.
 */
void Ut_I3c_SetDynamicAddressAssign_KnownPid_UsesRequestedAddress( void )
{
    i3c_Device_t    devices[ I3C_DEVICE_CNT_MAX ] = { { 0 } };
    i3c_DeviceCnt_t deviceCnt = 0u;
    i3c_DeviceCnt_t discoveredCnt = 0u;

    UtI3c_Init_Ctrl();
    devices[ 0u ].Type           = I3C_DEVICE_I3C;
    devices[ 0u ].DynamicAddress = UT_I3C_ADDR_A;
    devices[ 0u ].PidHigh        = UT_I3C_PID_HIGH_REPEATED;
    devices[ 0u ].PidLow         = UT_I3C_PID_LOW_REPEATED;
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_DeviceTable( I3C_PERIPH_1, devices, 1u ) );
    I3C1->RDR = UT_I3C_DATA_BYTE;
    I3C1->EVR = ( I3C_EVR_RXFNEF | I3C_EVR_TXFNFF | I3C_EVR_FCF );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_DynamicAddressAssign( I3C_PERIPH_1, &discoveredCnt ) );

    TEST_ASSERT_EQUAL_UINT32( UT_I3C_ADDR_A, I3C1->TDR );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_DeviceTable( I3C_PERIPH_1, devices, &deviceCnt ) );
    TEST_ASSERT_EQUAL_UINT8( 1u, deviceCnt );
    TEST_ASSERT_EQUAL( I3C_FUNCTION_ACTIVE, devices[ 0u ].AddressState );
}


/**
 * \brief Verifies the address assignment when the table is full.
 *
 * \par Expected results
 * The device still gets an address (the procedure cannot skip it) but is not stored: the function returns the
 * error and the count of the devices that got an address is not reported as stored.
 */
void Ut_I3c_SetDynamicAddressAssign_TableFull_ReturnsErrorAndDoesNotStoreDevice( void )
{
    i3c_Device_t    devices[ I3C_DEVICE_CNT_MAX ] = { { 0 } };
    i3c_DeviceCnt_t deviceCnt = 0u;
    i3c_DeviceCnt_t discoveredCnt = 0u;

    UtI3c_Init_Ctrl();

    for( uint32_t idx = 0u; I3C_DEVICE_CNT_MAX > idx; idx++ )
    {
        devices[ idx ].Type           = I3C_DEVICE_I3C;
        devices[ idx ].DynamicAddress = (i3c_Address_t)( UT_I3C_ADDR_A + idx );
        devices[ idx ].PidHigh        = (i3c_PidHigh_t)( idx + 1u );
    }

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_DeviceTable( I3C_PERIPH_1, devices, I3C_DEVICE_CNT_MAX ) );
    I3C1->RDR = UT_I3C_DATA_BYTE;
    I3C1->EVR = ( I3C_EVR_RXFNEF | I3C_EVR_TXFNFF | I3C_EVR_FCF );

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_DynamicAddressAssign( I3C_PERIPH_1, &discoveredCnt ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_DeviceTable( I3C_PERIPH_1, devices, &deviceCnt ) );
    TEST_ASSERT_EQUAL_UINT8( I3C_DEVICE_CNT_MAX, deviceCnt );
    TEST_ASSERT_EQUAL_UINT32( I3C_DEFAULT_OWN_ADDRESS + 1u, I3C1->TDR );
}


/**
 * \brief Verifies the address assignment with an error and with invalid parameters.
 *
 * \par Expected results
 * A bus error returns the error; a null count, an invalid instance, a not initialized instance and the target
 * role return the error.
 */
void Ut_I3c_SetDynamicAddressAssign_ErrorsAndInvalidCases_ReturnError( void )
{
    i3c_DeviceCnt_t discoveredCnt = 0u;

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_DynamicAddressAssign( I3C_PERIPH_1, &discoveredCnt ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_DynamicAddressAssign( I3C_PERIPH_CNT, &discoveredCnt ) );

    UtI3c_Init_Ctrl();

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_DynamicAddressAssign( I3C_PERIPH_1, I3C_NULL_PTR ) );

    I3C1->EVR = I3C_EVR_ERRF;
    I3C1->SER = I3C_SER_PERR;
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_DynamicAddressAssign( I3C_PERIPH_1, &discoveredCnt ) );
}


/**
 * \brief Verifies the decoding of the answer of a device and the address pool.
 *
 * \par Expected results
 * The PID is read most significant byte first, followed by the BCR and the DCR; the addresses of the pool are
 * handed out in order and skip the own address; a BCR with the payload bit sets the mandatory data byte.
 */
void Ut_I3c_SetDaaDevice_DistinctPayloads_StoreFieldsAndHandOutPool( void )
{
    i3c_Context_t *  ctx = I3C_NULL_PTR;
    i3c_Address_t    address = 0u;

    UtI3c_Init_Ctrl();
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_Context( I3C_PERIPH_1, &ctx ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_DaaDevice( ctx, utI3c_Payload1, &address ) );
    TEST_ASSERT_EQUAL_UINT8( I3C_DEFAULT_OWN_ADDRESS + 1u, address );
    TEST_ASSERT_EQUAL_UINT16( UT_I3C_PID_HIGH_1, ctx->Devices[ 0u ].PidHigh );
    TEST_ASSERT_EQUAL_UINT32( UT_I3C_PID_LOW_1, ctx->Devices[ 0u ].PidLow );
    TEST_ASSERT_EQUAL_UINT8( utI3c_Payload1[ I3C_PID_BYTE_CNT ], ctx->Devices[ 0u ].Bcr );
    TEST_ASSERT_EQUAL_UINT8( utI3c_Payload1[ I3C_PID_BYTE_CNT + 1u ], ctx->Devices[ 0u ].Dcr );
    TEST_ASSERT_EQUAL( I3C_FUNCTION_ACTIVE, ctx->Devices[ 0u ].IbiPayload );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_DaaDevice( ctx, utI3c_Payload2, &address ) );
    TEST_ASSERT_EQUAL_UINT8( UT_I3C_ADDR_POOL_2, address );
    TEST_ASSERT_EQUAL( I3C_FUNCTION_INACTIVE, ctx->Devices[ 1u ].IbiPayload );
    TEST_ASSERT_EQUAL_UINT8( UT_I3C_TABLE_CNT, ctx->DeviceCnt );

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_DaaDevice( I3C_NULL_PTR, utI3c_Payload1, &address ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_DaaDevice( ctx, I3C_NULL_PTR, &address ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_DaaDevice( ctx, utI3c_Payload1, I3C_NULL_PTR ) );
}


/**
 * \brief Verifies that the pool skips the reserved addresses and the addresses in use.
 *
 * \par Expected results
 * With a legacy device on 0x09 and an I3C device requesting 0x0A the next discovered device gets 0x0B; the
 * addresses that differ in one bit from the broadcast address are never handed out.
 */
void Ut_I3c_SetDaaDevice_UsedAndReservedAddresses_AreSkipped( void )
{
    i3c_Device_t     devices[ UT_I3C_TABLE_CNT ] = { { 0 } };
    i3c_Context_t *  ctx = I3C_NULL_PTR;
    i3c_Address_t    address = 0u;

    UtI3c_Init_Ctrl();
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_Context( I3C_PERIPH_1, &ctx ) );
    devices[ 0u ].Type          = I3C_DEVICE_I2C;
    devices[ 0u ].StaticAddress = I3C_DEFAULT_OWN_ADDRESS + 1u;
    devices[ 0u ].I2cSpeed      = I3C_I2C_SPEED_FM_PLUS;
    devices[ 1u ].Type           = I3C_DEVICE_I3C;
    devices[ 1u ].DynamicAddress = UT_I3C_ADDR_POOL_2;
    devices[ 1u ].PidHigh        = 1u;
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_DeviceTable( I3C_PERIPH_1, devices, UT_I3C_TABLE_CNT ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_DaaDevice( ctx, utI3c_Payload2, &address ) );

    TEST_ASSERT_EQUAL_UINT8( UT_I3C_ADDR_POOL_3, address );
}


/* ---------------------------------- Transfers ------------------------------ */

/**
 * \brief Verifies the rejected write requests.
 *
 * \par Expected results
 * A missing buffer, a zero count, an unknown device, an invalid, a not initialized instance and the target role
 * return the error; no control word is written.
 */
void Ut_I3c_SetWrite_InvalidCases_ReturnError( void )
{
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_Write( I3C_PERIPH_1, UT_I3C_ADDR_A, utI3c_TxData, 1u ) );

    UtI3c_Init_Ctrl();
    UtI3c_Set_Table();

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_Write( I3C_PERIPH_1, UT_I3C_ADDR_A, I3C_NULL_PTR, 1u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_Write( I3C_PERIPH_1, UT_I3C_ADDR_A, utI3c_TxData, 0u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_Write( I3C_PERIPH_1, UT_I3C_ADDR_UNKNOWN, utI3c_TxData, 1u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_Write( I3C_PERIPH_CNT, UT_I3C_ADDR_A, utI3c_TxData, 1u ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->CR );
}


/**
 * \brief Verifies that a target cannot start a controller request.
 *
 * \par Expected results
 * Write, read and write-read requests return the error for the target role.
 */
void Ut_I3c_Requests_TargetRole_ReturnError( void )
{
    UtI3c_Init_Tgt();

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_Write( I3C_PERIPH_1, UT_I3C_ADDR_A, utI3c_TxData, 1u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_Read( I3C_PERIPH_1, UT_I3C_ADDR_A, utI3c_RxBuf, 1u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR,
                       I3c_Set_WriteRead( I3C_PERIPH_1, UT_I3C_ADDR_A, utI3c_TxData, 1u, utI3c_RxBuf, 1u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_BroadcastCcc( I3C_PERIPH_1, I3C_CCC_RSTDAA, I3C_NULL_PTR, 0u ) );
}


/**
 * \brief Verifies a private write to an I3C device.
 *
 * \par Expected results
 * One control word with the stop bit, the private message type, the address, the write direction and the
 * count is written, the bytes are sent in order, the request is busy until the frame completed event, then
 * the completion callback reports the count and the state is idle.
 */
void Ut_I3c_SetWrite_I3cDevice_SendsControlWordAndBytes( void )
{
    i3c_XferState_t xferState = I3C_XFER_STATE_CNT;

    UtI3c_Init_Ctrl();
    UtI3c_Set_Table();
    I3C1->EVR = I3C_EVR_TXFNFF;
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtI3c_HwModel_Capture ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_Write( I3C_PERIPH_1, UT_I3C_ADDR_A, utI3c_TxData, UT_I3C_WRITE_CNT ) );

    TEST_ASSERT_EQUAL_UINT32( 1u, utI3c_CrSeqCnt );
    TEST_ASSERT_EQUAL_UINT32( UtI3c_Get_MsgWord( I3C_FUNCTION_ACTIVE,
                                                 I3C_MTYPE_PRIVATE,
                                                 UT_I3C_ADDR_A,
                                                 I3C_DIRECTION_WRITE,
                                                 UT_I3C_WRITE_CNT ),
                              utI3c_CrSeq[ 0u ] );
    TEST_ASSERT_EQUAL_UINT32( UT_I3C_WRITE_CNT, utI3c_TxSeqCnt );
    TEST_ASSERT_EQUAL_UINT32( utI3c_TxData[ 0u ], utI3c_TxSeq[ 0u ] );
    TEST_ASSERT_EQUAL_UINT32( utI3c_TxData[ UT_I3C_WRITE_CNT - 1u ], utI3c_TxSeq[ UT_I3C_WRITE_CNT - 1u ] );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_XferState( I3C_PERIPH_1, &xferState ) );
    TEST_ASSERT_EQUAL( I3C_XFER_STATE_BUSY, xferState );

    UtI3c_Set_Events( I3C_EVR_FCF );
    I3c_Task();

    TEST_ASSERT_EQUAL_UINT32( 1u, utI3c_XferCallCnt );
    TEST_ASSERT_EQUAL_UINT16( UT_I3C_WRITE_CNT, utI3c_XferLastCnt );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_XferState( I3C_PERIPH_1, &xferState ) );
    TEST_ASSERT_EQUAL( I3C_XFER_STATE_IDLE, xferState );
}


/**
 * \brief Verifies that a write longer than the FIFO is continued by the events.
 *
 * \par Expected results
 * The start fills the eight bytes of the transmit FIFO, the next task call sends the rest when the FIFO is
 * not full, the frame completed event ends the request with the full count.
 */
void Ut_I3c_SetWrite_LongerThanFifo_ContinuesOnEvents( void )
{
    UtI3c_Init_Ctrl();
    UtI3c_Set_Table();
    I3C1->EVR = I3C_EVR_TXFNFF;
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtI3c_HwModel_Capture ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_Write( I3C_PERIPH_1, UT_I3C_ADDR_A, utI3c_TxData, UT_I3C_BUF_CNT ) );
    TEST_ASSERT_EQUAL_UINT32( I3C_FIFO_DEPTH, utI3c_TxSeqCnt );

    I3c_Task();
    TEST_ASSERT_EQUAL_UINT32( UT_I3C_BUF_CNT, utI3c_TxSeqCnt );
    TEST_ASSERT_EQUAL_UINT32( utI3c_TxData[ UT_I3C_BUF_CNT - 1u ], utI3c_TxSeq[ UT_I3C_BUF_CNT - 1u ] );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI3c_XferCallCnt );

    UtI3c_Set_Events( I3C_EVR_FCF );
    I3c_Task();
    TEST_ASSERT_EQUAL_UINT32( 1u, utI3c_XferCallCnt );
    TEST_ASSERT_EQUAL_UINT16( UT_I3C_BUF_CNT, utI3c_XferLastCnt );
}


/**
 * \brief Verifies a private read of an I3C device.
 *
 * \par Expected results
 * The control word has the read direction, the received bytes are stored in the buffer, the completion
 * callback reports the count.
 */
void Ut_I3c_SetRead_I3cDevice_StoresReceivedBytes( void )
{
    UtI3c_Init_Ctrl();
    UtI3c_Set_Table();
    I3C1->RDR = UT_I3C_DATA_BYTE;
    I3C1->EVR = ( I3C_EVR_RXFNEF | I3C_EVR_FCF );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_Read( I3C_PERIPH_1, UT_I3C_ADDR_A, utI3c_RxBuf, UT_I3C_READ_CNT ) );
    I3c_Task();

    TEST_ASSERT_EQUAL_UINT32( UtI3c_Get_MsgWord( I3C_FUNCTION_ACTIVE,
                                                 I3C_MTYPE_PRIVATE,
                                                 UT_I3C_ADDR_A,
                                                 I3C_DIRECTION_READ,
                                                 UT_I3C_READ_CNT ),
                              I3C1->CR );
    TEST_ASSERT_EQUAL_UINT8( UT_I3C_DATA_BYTE, utI3c_RxBuf[ 0u ] );
    TEST_ASSERT_EQUAL_UINT8( UT_I3C_DATA_BYTE, utI3c_RxBuf[ UT_I3C_READ_CNT - 1u ] );
    TEST_ASSERT_EQUAL_UINT32( 1u, utI3c_XferCallCnt );
    TEST_ASSERT_EQUAL_UINT16( UT_I3C_READ_CNT, utI3c_XferLastCnt );
}


/**
 * \brief Verifies a read that the target ends early.
 *
 * \par Expected results
 * The count of the status register (3 of 6 bytes) is reported, the early end flag is cleared.
 */
void Ut_I3c_SetRead_TargetEndsEarly_ReportsReceivedCount( void )
{
    UtI3c_Init_Ctrl();
    UtI3c_Set_Table();
    I3C1->RDR = UT_I3C_DATA_BYTE;
    I3C1->SR  = UT_I3C_EARLY_CNT;
    I3C1->EVR = ( I3C_EVR_RXFNEF | I3C_EVR_RXTGTENDF | I3C_EVR_FCF );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_Read( I3C_PERIPH_1, UT_I3C_ADDR_A, utI3c_RxBuf, UT_I3C_BUF_CNT ) );
    I3c_Task();

    TEST_ASSERT_EQUAL_UINT32( 1u, utI3c_XferCallCnt );
    TEST_ASSERT_EQUAL_UINT16( UT_I3C_EARLY_CNT, utI3c_XferLastCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI3c_ErrCallCnt );
}


/**
 * \brief Verifies a write followed by a read of one device.
 *
 * \par Expected results
 * Two control words: the write without the stop bit and the read with the stop bit; the bytes are written,
 * the read data is stored, the completion callback reports the received count.
 */
void Ut_I3c_SetWriteRead_I3cDevice_SendsTwoControlWords( void )
{
    UtI3c_Init_Ctrl();
    UtI3c_Set_Table();
    I3C1->RDR = UT_I3C_DATA_BYTE;
    I3C1->EVR = ( I3C_EVR_TXFNFF | I3C_EVR_RXFNEF );
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtI3c_HwModel_Capture ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK,
                       I3c_Set_WriteRead( I3C_PERIPH_1,
                                          UT_I3C_ADDR_A,
                                          utI3c_TxData,
                                          UT_I3C_WRITE_CNT,
                                          utI3c_RxBuf,
                                          UT_I3C_READ_CNT ) );

    TEST_ASSERT_EQUAL_UINT32( UT_I3C_PAIR_CNT, utI3c_CrSeqCnt );
    TEST_ASSERT_EQUAL_UINT32( UtI3c_Get_MsgWord( I3C_FUNCTION_INACTIVE,
                                                 I3C_MTYPE_PRIVATE,
                                                 UT_I3C_ADDR_A,
                                                 I3C_DIRECTION_WRITE,
                                                 UT_I3C_WRITE_CNT ),
                              utI3c_CrSeq[ 0u ] );
    TEST_ASSERT_EQUAL_UINT32( UtI3c_Get_MsgWord( I3C_FUNCTION_ACTIVE,
                                                 I3C_MTYPE_PRIVATE,
                                                 UT_I3C_ADDR_A,
                                                 I3C_DIRECTION_READ,
                                                 UT_I3C_READ_CNT ),
                              utI3c_CrSeq[ 1u ] );
    TEST_ASSERT_EQUAL_UINT32( UT_I3C_WRITE_CNT, utI3c_TxSeqCnt );

    UtI3c_Set_Events( ( I3C_EVR_RXFNEF | I3C_EVR_FCF ) );
    I3c_Task();

    TEST_ASSERT_EQUAL_UINT8( UT_I3C_DATA_BYTE, utI3c_RxBuf[ UT_I3C_READ_CNT - 1u ] );
    TEST_ASSERT_EQUAL_UINT16( UT_I3C_READ_CNT, utI3c_XferLastCnt );
}


/**
 * \brief Verifies the legacy I2C messages.
 *
 * \par Expected results
 * A write to the legacy device uses the I2C message type and its static address, a read as well.
 */
void Ut_I3c_SetWriteAndRead_I2cDevice_UseLegacyMessageType( void )
{
    UtI3c_Init_Ctrl();
    UtI3c_Set_Table();
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtI3c_HwModel_Capture ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_Write( I3C_PERIPH_1, UT_I3C_ADDR_I2C, utI3c_TxData, 1u ) );
    TEST_ASSERT_EQUAL_UINT32( UtI3c_Get_MsgWord( I3C_FUNCTION_ACTIVE,
                                                 I3C_MTYPE_I2C,
                                                 UT_I3C_ADDR_I2C,
                                                 I3C_DIRECTION_WRITE,
                                                 1u ),
                              utI3c_CrSeq[ 0u ] );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_XferStop( I3C_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_Read( I3C_PERIPH_1, UT_I3C_ADDR_I2C, utI3c_RxBuf, 1u ) );
    TEST_ASSERT_EQUAL_UINT32( UtI3c_Get_MsgWord( I3C_FUNCTION_ACTIVE,
                                                 I3C_MTYPE_I2C,
                                                 UT_I3C_ADDR_I2C,
                                                 I3C_DIRECTION_READ,
                                                 1u ),
                              utI3c_CrSeq[ 1u ] );
}


/**
 * \brief Verifies the error of a frame.
 *
 * \par Expected results
 * The error callback gets the decoded errors, the state is the error, the errors can be read, the FIFOs are
 * flushed, the error flag is cleared and the completion callback is not called; a new request is accepted.
 */
void Ut_I3c_Request_BusError_ReportsErrorAndFlushesFifos( void )
{
    i3c_XferState_t xferState = I3C_XFER_STATE_CNT;
    i3c_XferError_t xferError = I3C_XFER_ERROR_NONE;

    UtI3c_Init_Ctrl();
    UtI3c_Set_Table();
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_Write( I3C_PERIPH_1, UT_I3C_ADDR_A, utI3c_TxData, 1u ) );

    UtI3c_Set_Events( I3C_EVR_ERRF );
    I3C1->SER = ( I3C_SER_ANACK | I3C_SER_DNACK );
    I3c_Task();

    TEST_ASSERT_EQUAL_UINT32( 1u, utI3c_ErrCallCnt );
    TEST_ASSERT_EQUAL_UINT8( ( I3C_XFER_ERROR_ADDR_NACK | I3C_XFER_ERROR_DATA_NACK ), utI3c_ErrLastMask );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI3c_XferCallCnt );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_XferState( I3C_PERIPH_1, &xferState ) );
    TEST_ASSERT_EQUAL( I3C_XFER_STATE_ERROR, xferState );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_XferError( I3C_PERIPH_1, &xferError ) );
    TEST_ASSERT_EQUAL_UINT8( ( I3C_XFER_ERROR_ADDR_NACK | I3C_XFER_ERROR_DATA_NACK ), xferError );
    TEST_ASSERT_EQUAL_UINT32( I3C_CFGR_FLUSH_ALL_MASK, I3C1->CFGR & I3C_CFGR_FLUSH_ALL_MASK );
    TEST_ASSERT_EQUAL_UINT32( I3C_EVR_ERRF, I3C1->CEVR );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_Write( I3C_PERIPH_1, UT_I3C_ADDR_A, utI3c_TxData, 1u ) );
}


/**
 * \brief Verifies the decoding of the status error register.
 *
 * \par Expected results
 * Every error bit and every protocol error code of I3C_SER maps to the documented error of the module.
 */
void Ut_I3c_GetErrorMask_SerBitsAndCodes_MapToErrors( void )
{
    TEST_ASSERT_EQUAL_UINT8( I3C_XFER_ERROR_NONE, I3c_Get_ErrorMask( 0u ) );
    TEST_ASSERT_EQUAL_UINT8( I3C_XFER_ERROR_ADDR_NACK, I3c_Get_ErrorMask( I3C_SER_ANACK ) );
    TEST_ASSERT_EQUAL_UINT8( I3C_XFER_ERROR_DATA_NACK, I3c_Get_ErrorMask( I3C_SER_DNACK ) );
    TEST_ASSERT_EQUAL_UINT8( ( I3C_XFER_ERROR_OVERRUN | I3C_XFER_ERROR_UNDERRUN ), I3c_Get_ErrorMask( I3C_SER_DOVR ) );
    TEST_ASSERT_EQUAL_UINT8( I3C_XFER_ERROR_UNDERRUN, I3c_Get_ErrorMask( I3C_SER_COVR ) );
    TEST_ASSERT_EQUAL_UINT8( I3C_XFER_ERROR_BUS, I3c_Get_ErrorMask( I3C_SER_STALL ) );
    TEST_ASSERT_EQUAL_UINT8( I3C_XFER_ERROR_BUS, I3c_Get_ErrorMask( I3C_SER_DERR ) );

    for( uint32_t idx = 0u; UT_I3C_CODE_CNT > idx; idx++ )
    {
        TEST_ASSERT_EQUAL_UINT8( utI3c_CodeErrors[ idx ],
                                 I3c_Get_ErrorMask( I3C_SER_PERR | utI3c_CodeValues[ idx ] ) );
    }
}


/**
 * \brief Verifies that a second request is rejected while one runs.
 *
 * \par Expected results
 * A read, a write and a CCC are rejected while the first write is busy; the first request is not disturbed.
 */
void Ut_I3c_Request_WhileBusy_ReturnsError( void )
{
    i3c_XferState_t xferState = I3C_XFER_STATE_CNT;

    UtI3c_Init_Ctrl();
    UtI3c_Set_Table();
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_Write( I3C_PERIPH_1, UT_I3C_ADDR_A, utI3c_TxData, 1u ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_Read( I3C_PERIPH_1, UT_I3C_ADDR_A, utI3c_RxBuf, 1u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_Write( I3C_PERIPH_1, UT_I3C_ADDR_A, utI3c_TxData, 1u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_BroadcastCcc( I3C_PERIPH_1, I3C_CCC_RSTDAA, I3C_NULL_PTR, 0u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_XferState( I3C_PERIPH_1, &xferState ) );
    TEST_ASSERT_EQUAL( I3C_XFER_STATE_BUSY, xferState );
}


/**
 * \brief Verifies the stop of a running request.
 *
 * \par Expected results
 * The request ends with the error state and the aborted error, the error callback is called, the FIFOs are
 * flushed and the peripheral is enabled; a stop without a request returns OK without a callback.
 */
void Ut_I3c_SetXferStop_RunningRequest_AbortsRequest( void )
{
    i3c_XferError_t xferError = I3C_XFER_ERROR_NONE;

    UtI3c_Init_Ctrl();
    UtI3c_Set_Table();
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_XferStop( I3C_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI3c_ErrCallCnt );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_Write( I3C_PERIPH_1, UT_I3C_ADDR_A, utI3c_TxData, 1u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_XferStop( I3C_PERIPH_1 ) );

    TEST_ASSERT_EQUAL_UINT32( 1u, utI3c_ErrCallCnt );
    TEST_ASSERT_EQUAL_UINT8( I3C_XFER_ERROR_ABORTED, utI3c_ErrLastMask );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_XferError( I3C_PERIPH_1, &xferError ) );
    TEST_ASSERT_EQUAL_UINT8( I3C_XFER_ERROR_ABORTED, xferError );
    TEST_ASSERT_EQUAL_UINT32( I3C_CFGR_EN, I3C1->CFGR & I3C_CFGR_EN );
    TEST_ASSERT_EQUAL_UINT32( I3C_CFGR_FLUSH_ALL_MASK, I3C1->CFGR & I3C_CFGR_FLUSH_ALL_MASK );
}


/**
 * \brief Verifies the stop and the state functions with invalid parameters.
 *
 * \par Expected results
 * A not initialized or invalid instance and null pointers return the error.
 */
void Ut_I3c_XferStateAndStop_InvalidCases_ReturnError( void )
{
    i3c_XferState_t xferState = I3C_XFER_STATE_IDLE;
    i3c_XferError_t xferError = I3C_XFER_ERROR_NONE;

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_XferStop( I3C_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Get_XferState( I3C_PERIPH_1, &xferState ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Get_XferError( I3C_PERIPH_1, &xferError ) );

    UtI3c_Init_Ctrl();

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_XferStop( I3C_PERIPH_CNT ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Get_XferState( I3C_PERIPH_1, I3C_NULL_PTR ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Get_XferError( I3C_PERIPH_1, I3C_NULL_PTR ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Get_XferState( I3C_PERIPH_CNT, &xferState ) );
}


/* ----------------------------- CCCs and events ----------------------------- */

/**
 * \brief Verifies a broadcast CCC with data.
 *
 * \par Expected results
 * ENEC is sent as one control word with the CCC code, the count of the defining bytes and the stop bit; the
 * defining byte is written to the transmit FIFO; the completion callback reports the count.
 */
void Ut_I3c_SetBroadcastCcc_Enec_SendsCodeAndDefiningByte( void )
{
    UtI3c_Init_Ctrl();
    I3C1->EVR = I3C_EVR_TXFNFF;
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtI3c_HwModel_Capture ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_BroadcastCcc( I3C_PERIPH_1, I3C_CCC_ENEC, utI3c_TxData, 1u ) );

    TEST_ASSERT_EQUAL_UINT32( 1u, utI3c_CrSeqCnt );
    TEST_ASSERT_EQUAL_UINT32( UtI3c_Get_CccWord( I3C_FUNCTION_ACTIVE, I3C_CCC_ENEC, 1u ), utI3c_CrSeq[ 0u ] );
    TEST_ASSERT_EQUAL_UINT32( 1u, utI3c_TxSeqCnt );
    TEST_ASSERT_EQUAL_UINT32( utI3c_TxData[ 0u ], utI3c_TxSeq[ 0u ] );

    UtI3c_Set_Events( I3C_EVR_FCF );
    I3c_Task();
    TEST_ASSERT_EQUAL_UINT16( 1u, utI3c_XferLastCnt );
}


/**
 * \brief Verifies a broadcast CCC without data.
 *
 * \par Expected results
 * RSTDAA and ENTAS0 are accepted without a buffer; SETMRL accepts three bytes.
 */
void Ut_I3c_SetBroadcastCcc_NoDataAndThreeBytes_AreAccepted( void )
{
    UtI3c_Init_Ctrl();

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_BroadcastCcc( I3C_PERIPH_1, I3C_CCC_RSTDAA, I3C_NULL_PTR, 0u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_XferStop( I3C_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_BroadcastCcc( I3C_PERIPH_1, I3C_CCC_ENTAS0, I3C_NULL_PTR, 0u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_XferStop( I3C_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK,
                       I3c_Set_BroadcastCcc( I3C_PERIPH_1, I3C_CCC_SETMRL, utI3c_TxData, UT_I3C_SETMRL_CNT ) );
}


/**
 * \brief Verifies the rejected broadcast CCCs.
 *
 * \par Expected results
 * A direct code, ENTDAA (own function), an unknown code, a data count outside the limits of the CCC, a missing
 * buffer, an invalid and a not initialized instance return the error.
 */
void Ut_I3c_SetBroadcastCcc_InvalidCases_ReturnError( void )
{
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_BroadcastCcc( I3C_PERIPH_1, I3C_CCC_RSTDAA, I3C_NULL_PTR, 0u ) );

    UtI3c_Init_Ctrl();

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_BroadcastCcc( I3C_PERIPH_1, I3C_CCC_ENEC_DIRECT, utI3c_TxData, 1u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR,
                       I3c_Set_BroadcastCcc( I3C_PERIPH_1, (i3c_Ccc_t)I3C_CCC_ENTDAA_CODE, I3C_NULL_PTR, 0u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR,
                       I3c_Set_BroadcastCcc( I3C_PERIPH_1, (i3c_Ccc_t)UT_I3C_CCC_UNKNOWN, I3C_NULL_PTR, 0u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_BroadcastCcc( I3C_PERIPH_1, I3C_CCC_ENEC, utI3c_TxData, 0u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_BroadcastCcc( I3C_PERIPH_1, I3C_CCC_RSTDAA, utI3c_TxData, 1u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_BroadcastCcc( I3C_PERIPH_1, I3C_CCC_ENEC, I3C_NULL_PTR, 1u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_BroadcastCcc( I3C_PERIPH_CNT, I3C_CCC_RSTDAA, I3C_NULL_PTR, 0u ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->CR );
}


/**
 * \brief Verifies a direct read CCC.
 *
 * \par Expected results
 * GETPID is sent as two control words: the CCC code without the stop bit, then the direct message with the
 * address, the read direction, six bytes and the stop bit; the six bytes are stored.
 */
void Ut_I3c_SetDirectCcc_GetPid_SendsTwoControlWordsAndReadsSixBytes( void )
{
    UtI3c_Init_Ctrl();
    UtI3c_Set_Table();
    I3C1->RDR = UT_I3C_DATA_BYTE;
    I3C1->EVR = ( I3C_EVR_RXFNEF | I3C_EVR_FCF );
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtI3c_HwModel_Capture ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK,
                       I3c_Set_DirectCcc( I3C_PERIPH_1,
                                          I3C_CCC_GETPID,
                                          UT_I3C_ADDR_A,
                                          utI3c_RxBuf,
                                          UT_I3C_PID_CNT,
                                          I3C_DIRECTION_READ ) );

    TEST_ASSERT_EQUAL_UINT32( UT_I3C_PAIR_CNT, utI3c_CrSeqCnt );
    TEST_ASSERT_EQUAL_UINT32( UtI3c_Get_CccWord( I3C_FUNCTION_INACTIVE, I3C_CCC_GETPID, 0u ), utI3c_CrSeq[ 0u ] );
    TEST_ASSERT_EQUAL_UINT32( UtI3c_Get_MsgWord( I3C_FUNCTION_ACTIVE,
                                                 I3C_MTYPE_DIRECT,
                                                 UT_I3C_ADDR_A,
                                                 I3C_DIRECTION_READ,
                                                 UT_I3C_PID_CNT ),
                              utI3c_CrSeq[ 1u ] );

    I3c_Task();
    TEST_ASSERT_EQUAL_UINT8( UT_I3C_DATA_BYTE, utI3c_RxBuf[ UT_I3C_PID_CNT - 1u ] );
    TEST_ASSERT_EQUAL_UINT16( UT_I3C_PID_CNT, utI3c_XferLastCnt );
}


/**
 * \brief Verifies a direct write CCC.
 *
 * \par Expected results
 * ENEC (direct) carries its data byte in the second part: the second control word has the write direction and
 * the count one, the first part has no defining byte.
 */
void Ut_I3c_SetDirectCcc_EnecDirect_WritesDataInSecondPart( void )
{
    UtI3c_Init_Ctrl();
    UtI3c_Set_Table();
    I3C1->EVR = I3C_EVR_TXFNFF;
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtI3c_HwModel_Capture ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK,
                       I3c_Set_DirectCcc( I3C_PERIPH_1,
                                          I3C_CCC_ENEC_DIRECT,
                                          UT_I3C_ADDR_A,
                                          (i3c_Data_t *)utI3c_TxData,
                                          1u,
                                          I3C_DIRECTION_WRITE ) );

    TEST_ASSERT_EQUAL_UINT32( UtI3c_Get_CccWord( I3C_FUNCTION_INACTIVE, I3C_CCC_ENEC_DIRECT, 0u ), utI3c_CrSeq[ 0u ] );
    TEST_ASSERT_EQUAL_UINT32( UtI3c_Get_MsgWord( I3C_FUNCTION_ACTIVE,
                                                 I3C_MTYPE_DIRECT,
                                                 UT_I3C_ADDR_A,
                                                 I3C_DIRECTION_WRITE,
                                                 1u ),
                              utI3c_CrSeq[ 1u ] );
    TEST_ASSERT_EQUAL_UINT32( utI3c_TxData[ 0u ], utI3c_TxSeq[ 0u ] );
}


/**
 * \brief Verifies the direct RSTACT CCC.
 *
 * \par Expected results
 * The first part carries the defining byte (count one); the write has no data in the second part, the read
 * reads one byte that replaces the defining byte.
 */
void Ut_I3c_SetDirectCcc_RstactDirect_UsesDefiningByte( void )
{
    i3c_Data_t data[ 1u ] = { UT_I3C_DATA_BYTE };

    UtI3c_Init_Ctrl();
    UtI3c_Set_Table();
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtI3c_HwModel_Capture ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK,
                       I3c_Set_DirectCcc( I3C_PERIPH_1,
                                          I3C_CCC_RSTACT_DIRECT,
                                          UT_I3C_ADDR_A,
                                          data,
                                          1u,
                                          I3C_DIRECTION_WRITE ) );
    TEST_ASSERT_EQUAL_UINT32( UtI3c_Get_CccWord( I3C_FUNCTION_INACTIVE, I3C_CCC_RSTACT_DIRECT, 1u ),
                              utI3c_CrSeq[ 0u ] );
    TEST_ASSERT_EQUAL_UINT32( UtI3c_Get_MsgWord( I3C_FUNCTION_ACTIVE,
                                                 I3C_MTYPE_DIRECT,
                                                 UT_I3C_ADDR_A,
                                                 I3C_DIRECTION_WRITE,
                                                 0u ),
                              utI3c_CrSeq[ 1u ] );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_XferStop( I3C_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK,
                       I3c_Set_DirectCcc( I3C_PERIPH_1,
                                          I3C_CCC_RSTACT_DIRECT,
                                          UT_I3C_ADDR_A,
                                          data,
                                          1u,
                                          I3C_DIRECTION_READ ) );
    TEST_ASSERT_EQUAL_UINT32( UtI3c_Get_MsgWord( I3C_FUNCTION_ACTIVE,
                                                 I3C_MTYPE_DIRECT,
                                                 UT_I3C_ADDR_A,
                                                 I3C_DIRECTION_READ,
                                                 1u ),
                              utI3c_CrSeq[ UT_I3C_RSTACT_READ_IDX ] );
}


/**
 * \brief Verifies that SETDASA addresses a device by its static address.
 *
 * \par Expected results
 * An I3C device without a dynamic address is found by its static address; the same address is rejected for
 * a CCC that needs a dynamic address.
 */
void Ut_I3c_SetDirectCcc_Setdasa_UsesStaticAddress( void )
{
    i3c_Device_t devices[ 1u ] = { { 0 } };
    i3c_Data_t   data[ 1u ] = { UT_I3C_DATA_BYTE };

    UtI3c_Init_Ctrl();
    devices[ 0u ].Type           = I3C_DEVICE_I3C;
    devices[ 0u ].StaticAddress  = UT_I3C_ADDR_B;
    devices[ 0u ].DynamicAddress = UT_I3C_ADDR_A;
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_DeviceTable( I3C_PERIPH_1, devices, 1u ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR,
                       I3c_Set_DirectCcc( I3C_PERIPH_1, I3C_CCC_GETBCR, UT_I3C_ADDR_A, data, 1u, I3C_DIRECTION_READ ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK,
                       I3c_Set_DirectCcc( I3C_PERIPH_1,
                                          I3C_CCC_SETDASA,
                                          UT_I3C_ADDR_B,
                                          data,
                                          1u,
                                          I3C_DIRECTION_WRITE ) );
}


/**
 * \brief Verifies the rejected direct CCCs.
 *
 * \par Expected results
 * A broadcast code, a wrong direction, a data count outside the limits, a missing buffer, an unknown device, a
 * legacy I2C device, an invalid and a not initialized instance return the error.
 */
void Ut_I3c_SetDirectCcc_InvalidCases_ReturnError( void )
{
    i3c_Data_t data[ UT_I3C_PID_CNT ] = { 0u };

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR,
                       I3c_Set_DirectCcc( I3C_PERIPH_1, I3C_CCC_GETBCR, UT_I3C_ADDR_A, data, 1u, I3C_DIRECTION_READ ) );

    UtI3c_Init_Ctrl();
    UtI3c_Set_Table();

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR,
                       I3c_Set_DirectCcc( I3C_PERIPH_1,
                                          I3C_CCC_RSTDAA,
                                          UT_I3C_ADDR_A,
                                          data,
                                          0u,
                                          I3C_DIRECTION_WRITE ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR,
                       I3c_Set_DirectCcc( I3C_PERIPH_1,
                                          I3C_CCC_GETPID,
                                          UT_I3C_ADDR_A,
                                          data,
                                          UT_I3C_PID_CNT,
                                          I3C_DIRECTION_WRITE ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR,
                       I3c_Set_DirectCcc( I3C_PERIPH_1, I3C_CCC_GETPID, UT_I3C_ADDR_A, data, 1u, I3C_DIRECTION_READ ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR,
                       I3c_Set_DirectCcc( I3C_PERIPH_1,
                                          I3C_CCC_GETBCR,
                                          UT_I3C_ADDR_A,
                                          I3C_NULL_PTR,
                                          1u,
                                          I3C_DIRECTION_READ ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR,
                       I3c_Set_DirectCcc( I3C_PERIPH_1,
                                          I3C_CCC_GETBCR,
                                          UT_I3C_ADDR_UNKNOWN,
                                          data,
                                          1u,
                                          I3C_DIRECTION_READ ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR,
                       I3c_Set_DirectCcc( I3C_PERIPH_1,
                                          I3C_CCC_GETBCR,
                                          UT_I3C_ADDR_I2C,
                                          data,
                                          1u,
                                          I3C_DIRECTION_READ ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR,
                       I3c_Set_DirectCcc( I3C_PERIPH_1, I3C_CCC_GETBCR, UT_I3C_ADDR_A, data, 1u, I3C_DIRECTION_CNT ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR,
                       I3c_Set_DirectCcc( I3C_PERIPH_CNT,
                                          I3C_CCC_GETBCR,
                                          UT_I3C_ADDR_A,
                                          data,
                                          1u,
                                          I3C_DIRECTION_READ ) );
}


/**
 * \brief Verifies the acceptance of the in-band interrupt of a device.
 *
 * \par Expected results
 * Disabling clears the acknowledge bit in DEVR1, enabling sets it again; a legacy device, an unknown device,
 * an invalid state, an invalid and a not initialized instance return the error.
 */
void Ut_I3c_SetIbiConfig_Device_UpdatesDeviceRegister( void )
{
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_IbiConfig( I3C_PERIPH_1, UT_I3C_ADDR_A, I3C_FUNCTION_ACTIVE ) );

    UtI3c_Init_Ctrl();
    UtI3c_Set_Table();

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_IbiConfig( I3C_PERIPH_1, UT_I3C_ADDR_A, I3C_FUNCTION_INACTIVE ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->DEVRX[ 0u ] & I3C_DEVRX_IBIACK );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_IbiConfig( I3C_PERIPH_1, UT_I3C_ADDR_A, I3C_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL_UINT32( I3C_DEVRX_IBIACK, I3C1->DEVRX[ 0u ] & I3C_DEVRX_IBIACK );

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_IbiConfig( I3C_PERIPH_1, UT_I3C_ADDR_I2C, I3C_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_IbiConfig( I3C_PERIPH_1, UT_I3C_ADDR_UNKNOWN, I3C_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR,
                       I3c_Set_IbiConfig( I3C_PERIPH_1, UT_I3C_ADDR_A, (i3c_FunctionState_t)UT_I3C_BAD_STATE ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_IbiConfig( I3C_PERIPH_CNT, UT_I3C_ADDR_A, I3C_FUNCTION_ACTIVE ) );
}


/**
 * \brief Verifies the acknowledge of the hot-join.
 *
 * \par Expected results
 * HotJoinActive sets I3C_CFGR.HJACK, HotJoinInactive clears it; a not initialized instance and the target role
 * return the error.
 */
void Ut_I3c_SetHotJoin_ActiveInactive_TogglesHjack( void )
{
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_HotJoinActive( I3C_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_HotJoinInactive( I3C_PERIPH_1 ) );

    UtI3c_Init_Ctrl();

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_HotJoinActive( I3C_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_UINT32( I3C_CFGR_HJACK, I3C1->CFGR & I3C_CFGR_HJACK );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_HotJoinInactive( I3C_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->CFGR & I3C_CFGR_HJACK );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_HotJoinActive( I3C_PERIPH_CNT ) );
}


/**
 * \brief Verifies the in-band interrupt of a device of the table.
 *
 * \par Expected results
 * The callback gets the address of the requester and its payload (two bytes of the payload register), the flag
 * is cleared; an interrupt of an unknown address and an interrupt without a callback are cleared silently.
 */
void Ut_I3c_Task_InBandInterrupt_CallsCallbackWithPayload( void )
{
    i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_POLL );

    UtI3c_Init( &config );
    UtI3c_Set_Table();
    I3C1->RMR   = ( ( (uint32_t)UT_I3C_ADDR_A << I3C_RMR_RADD_Pos ) | UT_I3C_IBI_CNT );
    I3C1->IBIDR = UT_I3C_IBI_DATA;
    I3C1->EVR   = I3C_EVR_IBIF;

    I3c_Task();

    TEST_ASSERT_EQUAL_UINT32( 1u, utI3c_Ibi.CallCnt );
    TEST_ASSERT_EQUAL_UINT8( UT_I3C_ADDR_A, utI3c_Ibi.Address );
    TEST_ASSERT_EQUAL_UINT16( UT_I3C_IBI_CNT, utI3c_Ibi.PayloadCnt );
    TEST_ASSERT_EQUAL_UINT8( (uint8_t)UT_I3C_IBI_DATA, utI3c_Ibi.Payload[ 0u ] );
    TEST_ASSERT_EQUAL_UINT8( (uint8_t)( UT_I3C_IBI_DATA >> I3C_BYTE_BIT_CNT ), utI3c_Ibi.Payload[ 1u ] );
    TEST_ASSERT_EQUAL_UINT32( I3C_EVR_IBIF, I3C1->CEVR );

    I3C1->RMR = ( ( (uint32_t)UT_I3C_ADDR_UNKNOWN << I3C_RMR_RADD_Pos ) | UT_I3C_IBI_CNT );
    I3c_Task();
    TEST_ASSERT_EQUAL_UINT32( 1u, utI3c_Ibi.CallCnt );
}


/**
 * \brief Verifies the limit of the payload count of an in-band interrupt.
 *
 * \par Expected results
 * A payload count above four bytes is limited to four.
 */
void Ut_I3c_Task_InBandInterruptPayloadCount_IsLimitedToFourBytes( void )
{
    UtI3c_Init_Ctrl();
    UtI3c_Set_Table();
    I3C1->RMR = ( ( (uint32_t)UT_I3C_ADDR_A << I3C_RMR_RADD_Pos ) | I3C_RMR_IBIRDCNT_Msk );
    I3C1->EVR = I3C_EVR_IBIF;

    I3c_Task();

    TEST_ASSERT_EQUAL_UINT16( I3C_IBI_PAYLOAD_CNT_MAX, utI3c_Ibi.PayloadCnt );
}


/**
 * \brief Verifies the hot-join and the controller-role request events.
 *
 * \par Expected results
 * The hot-join request calls the callback and clears its flag; the controller-role request is only cleared.
 */
void Ut_I3c_Task_HotJoinAndControllerRoleRequests_AreCleared( void )
{
    UtI3c_Init_Ctrl();

    I3C1->EVR = I3C_EVR_HJF;
    I3c_Task();
    TEST_ASSERT_EQUAL_UINT32( 1u, utI3c_HjCallCnt );
    TEST_ASSERT_EQUAL_UINT32( I3C_EVR_HJF, I3C1->CEVR );

    I3C1->EVR = I3C_EVR_CRF;
    I3c_Task();
    TEST_ASSERT_EQUAL_UINT32( 1u, utI3c_HjCallCnt );
    TEST_ASSERT_EQUAL_UINT32( I3C_EVR_CRF, I3C1->CEVR );
}


/**
 * \brief Verifies the events without any callback.
 *
 * \par Expected results
 * Without callbacks the events of the controller are cleared and the request ends without a call.
 */
void Ut_I3c_Task_NoCallbacks_EventsAreServed( void )
{
    i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_POLL );
    i3c_XferState_t xferState = I3C_XFER_STATE_CNT;

    config.XferCompleteCallback = I3C_NULL_PTR;
    config.ErrorCallback        = I3C_NULL_PTR;
    config.IbiCallback          = I3C_NULL_PTR;
    config.HotJoinCallback      = I3C_NULL_PTR;
    UtI3c_Init( &config );
    UtI3c_Set_Table();
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_Write( I3C_PERIPH_1, UT_I3C_ADDR_A, utI3c_TxData, 1u ) );

    I3C1->RMR = ( (uint32_t)UT_I3C_ADDR_A << I3C_RMR_RADD_Pos );
    I3C1->EVR = ( I3C_EVR_FCF | I3C_EVR_IBIF | I3C_EVR_HJF );
    I3c_Task();

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_XferState( I3C_PERIPH_1, &xferState ) );
    TEST_ASSERT_EQUAL( I3C_XFER_STATE_IDLE, xferState );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI3c_Ibi.CallCnt );
}


/* ------------------------------ ISR and DMA mode --------------------------- */

/**
 * \brief Verifies a write in the ISR mode.
 *
 * \par Expected results
 * The transmit FIFO interrupt is enabled while bytes are left, the interrupt handler writes them when the FIFO has
 * room and disables the interrupt, the frame completed event ends the request with the callback.
 */
void Ut_I3c_Isr_Write_FifoInterruptFollowsData( void )
{
    UtI3c_Init_Isr();
    UtI3c_Set_Table();
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtI3c_HwModel_Capture ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_Write( I3C_PERIPH_1, UT_I3C_ADDR_A, utI3c_TxData, UT_I3C_WRITE_CNT ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI3c_TxSeqCnt );
    TEST_ASSERT_EQUAL_UINT32( I3C_IER_TXFNFIE, I3C1->IER & I3C_IER_TXFNFIE );

    UtI3c_Set_Events( I3C_EVR_TXFNFF );
    utI3c_Isr();
    TEST_ASSERT_EQUAL_UINT32( UT_I3C_WRITE_CNT, utI3c_TxSeqCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->IER & I3C_IER_TXFNFIE );

    UtI3c_Set_Events( I3C_EVR_FCF );
    utI3c_Isr();
    TEST_ASSERT_EQUAL_UINT32( 1u, utI3c_XferCallCnt );
    TEST_ASSERT_EQUAL_UINT16( UT_I3C_WRITE_CNT, utI3c_XferLastCnt );
}


/**
 * \brief Verifies a read in the ISR mode.
 *
 * \par Expected results
 * The receive FIFO interrupt is enabled for the request, the interrupt handler stores the bytes and disables
 * the interrupt after the last one; the frame completed event ends the request.
 */
void Ut_I3c_Isr_Read_ReceiveInterruptFollowsData( void )
{
    UtI3c_Init_Isr();
    UtI3c_Set_Table();
    I3C1->RDR = UT_I3C_DATA_BYTE;

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_Read( I3C_PERIPH_1, UT_I3C_ADDR_A, utI3c_RxBuf, UT_I3C_READ_CNT ) );
    TEST_ASSERT_EQUAL_UINT32( I3C_IER_RXFNEIE, I3C1->IER & I3C_IER_RXFNEIE );

    I3C1->EVR = ( I3C_EVR_RXFNEF | I3C_EVR_FCF );
    utI3c_Isr();

    TEST_ASSERT_EQUAL_UINT8( UT_I3C_DATA_BYTE, utI3c_RxBuf[ UT_I3C_READ_CNT - 1u ] );
    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->IER & I3C_IER_RXFNEIE );
    TEST_ASSERT_EQUAL_UINT16( UT_I3C_READ_CNT, utI3c_XferLastCnt );
}


/**
 * \brief Verifies an error in the ISR mode.
 *
 * \par Expected results
 * The FIFO interrupts of the request are disabled and the error callback is called.
 */
void Ut_I3c_Isr_Error_DisablesFifoInterruptsAndReports( void )
{
    UtI3c_Init_Isr();
    UtI3c_Set_Table();
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK,
                       I3c_Set_WriteRead( I3C_PERIPH_1,
                                          UT_I3C_ADDR_A,
                                          utI3c_TxData,
                                          UT_I3C_WRITE_CNT,
                                          utI3c_RxBuf,
                                          UT_I3C_READ_CNT ) );
    TEST_ASSERT_EQUAL_UINT32( I3C_IER_FIFO_MASK, I3C1->IER & I3C_IER_FIFO_MASK );

    I3C1->EVR = I3C_EVR_ERRF;
    I3C1->SER = I3C_SER_DOVR;
    utI3c_Isr();

    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->IER & I3C_IER_FIFO_MASK );
    TEST_ASSERT_EQUAL_UINT32( 1u, utI3c_ErrCallCnt );
}


/**
 * \brief Verifies the events of the targets in the ISR mode.
 *
 * \par Expected results
 * The in-band interrupt and the hot-join request are served by the interrupt handler.
 */
void Ut_I3c_Isr_TargetRequests_AreServedByInterrupt( void )
{
    UtI3c_Init_Isr();
    UtI3c_Set_Table();

    I3C1->RMR = ( ( (uint32_t)UT_I3C_ADDR_A << I3C_RMR_RADD_Pos ) | UT_I3C_IBI_CNT );
    I3C1->EVR = ( I3C_EVR_IBIF | I3C_EVR_HJF );
    utI3c_Isr();

    TEST_ASSERT_EQUAL_UINT32( 1u, utI3c_Ibi.CallCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utI3c_HjCallCnt );
}


/**
 * \brief Verifies a write in the DMA mode.
 *
 * \par Expected results
 * The transmit channel gets the block size and the memory address and is activated before the control word is
 * written, the transmit FIFO is not written by the CPU; at the end both channels are stopped and the callback
 * reports the full count.
 */
void Ut_I3c_Dma_Write_StartsTransmitChannel( void )
{
    const i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_DMA );
    i3c_Data_t *       buffer = REGMEM_SRAM_PTR( i3c_Data_t, 0u );

    UtI3c_Init_Dma();
    UtI3c_Set_Table();
    I3C1->EVR = I3C_EVR_TXFNFF;

    Gpdma_Set_BlockSize_ExpectAndReturn( config.TxDmaPeriphId,
                                         config.TxDmaChannelId,
                                         UT_I3C_WRITE_CNT,
                                         GPDMA_REQUEST_OK );
    Gpdma_Set_SourceAddr_ExpectAndReturn( config.TxDmaPeriphId,
                                          config.TxDmaChannelId,
                                          REGMEM_SRAM_BASE,
                                          GPDMA_REQUEST_OK );
    Gpdma_Set_ChannelActive_ExpectAndReturn( config.TxDmaPeriphId, config.TxDmaChannelId, GPDMA_REQUEST_OK );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_Write( I3C_PERIPH_1, UT_I3C_ADDR_A, buffer, UT_I3C_WRITE_CNT ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->TDR );

    UtI3c_Expect_DmaStop( &config, I3C_FUNCTION_ACTIVE, I3C_FUNCTION_ACTIVE );
    UtI3c_Set_Events( I3C_EVR_FCF );
    utI3c_Isr();

    TEST_ASSERT_EQUAL_UINT32( 1u, utI3c_XferCallCnt );
    TEST_ASSERT_EQUAL_UINT16( UT_I3C_WRITE_CNT, utI3c_XferLastCnt );
}


/**
 * \brief Verifies a read in the DMA mode.
 *
 * \par Expected results
 * The receive channel gets the block size and the destination address and is activated; the frame completed
 * event stops the channels and reports the full count.
 */
void Ut_I3c_Dma_Read_StartsReceiveChannel( void )
{
    const i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_DMA );
    i3c_Data_t *       buffer = REGMEM_SRAM_PTR( i3c_Data_t, 0u );

    UtI3c_Init_Dma();
    UtI3c_Set_Table();

    Gpdma_Set_BlockSize_ExpectAndReturn( config.RxDmaPeriphId,
                                         config.RxDmaChannelId,
                                         UT_I3C_READ_CNT,
                                         GPDMA_REQUEST_OK );
    Gpdma_Set_DestinationAddr_ExpectAndReturn( config.RxDmaPeriphId,
                                               config.RxDmaChannelId,
                                               REGMEM_SRAM_BASE,
                                               GPDMA_REQUEST_OK );
    Gpdma_Set_ChannelActive_ExpectAndReturn( config.RxDmaPeriphId, config.RxDmaChannelId, GPDMA_REQUEST_OK );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_Read( I3C_PERIPH_1, UT_I3C_ADDR_A, buffer, UT_I3C_READ_CNT ) );

    UtI3c_Expect_DmaStop( &config, I3C_FUNCTION_ACTIVE, I3C_FUNCTION_ACTIVE );
    UtI3c_Set_Events( I3C_EVR_FCF );
    utI3c_Isr();

    TEST_ASSERT_EQUAL_UINT16( UT_I3C_READ_CNT, utI3c_XferLastCnt );
}


/**
 * \brief Verifies a write and read in the DMA mode.
 *
 * \par Expected results
 * Both channels are started, the transmit channel first.
 */
void Ut_I3c_Dma_WriteRead_StartsBothChannels( void )
{
    const i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_DMA );
    i3c_Data_t *       txBuffer = REGMEM_SRAM_PTR( i3c_Data_t, 0u );
    i3c_Data_t *       rxBuffer = REGMEM_SRAM_PTR( i3c_Data_t, UT_I3C_BUF_CNT );

    UtI3c_Init_Dma();
    UtI3c_Set_Table();

    Gpdma_Set_BlockSize_ExpectAndReturn( config.TxDmaPeriphId,
                                         config.TxDmaChannelId,
                                         UT_I3C_WRITE_CNT,
                                         GPDMA_REQUEST_OK );
    Gpdma_Set_SourceAddr_ExpectAndReturn( config.TxDmaPeriphId,
                                          config.TxDmaChannelId,
                                          REGMEM_SRAM_BASE,
                                          GPDMA_REQUEST_OK );
    Gpdma_Set_ChannelActive_ExpectAndReturn( config.TxDmaPeriphId, config.TxDmaChannelId, GPDMA_REQUEST_OK );
    Gpdma_Set_BlockSize_ExpectAndReturn( config.RxDmaPeriphId,
                                         config.RxDmaChannelId,
                                         UT_I3C_READ_CNT,
                                         GPDMA_REQUEST_OK );
    Gpdma_Set_DestinationAddr_ExpectAndReturn( config.RxDmaPeriphId,
                                               config.RxDmaChannelId,
                                               REGMEM_SRAM_BASE + UT_I3C_BUF_CNT,
                                               GPDMA_REQUEST_OK );
    Gpdma_Set_ChannelActive_ExpectAndReturn( config.RxDmaPeriphId, config.RxDmaChannelId, GPDMA_REQUEST_OK );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK,
                       I3c_Set_WriteRead( I3C_PERIPH_1,
                                          UT_I3C_ADDR_A,
                                          txBuffer,
                                          UT_I3C_WRITE_CNT,
                                          rxBuffer,
                                          UT_I3C_READ_CNT ) );
}


/**
 * \brief Verifies that a failing DMA channel rejects the request.
 *
 * \par Expected results
 * Gpdma_Set_BlockSize() returning the error makes the write return the error, both channels are stopped, no
 * control word is written and the next request is accepted.
 */
void Ut_I3c_Dma_StartFails_ReturnsErrorWithoutControlWord( void )
{
    const i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_DMA );
    i3c_Data_t *       buffer = REGMEM_SRAM_PTR( i3c_Data_t, 0u );

    UtI3c_Init_Dma();
    UtI3c_Set_Table();

    Gpdma_Set_BlockSize_ExpectAndReturn( config.TxDmaPeriphId, config.TxDmaChannelId, 1u, GPDMA_REQUEST_ERROR );
    UtI3c_Expect_DmaStop( &config, I3C_FUNCTION_ACTIVE, I3C_FUNCTION_ACTIVE );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_Write( I3C_PERIPH_1, UT_I3C_ADDR_A, buffer, 1u ) );

    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->CR );
}


/**
 * \brief Verifies a read that the target ends early in the DMA mode.
 *
 * \par Expected results
 * The early end stops the receive channel and the count of the status register is reported; the channels are
 * stopped again at the end of the frame.
 */
void Ut_I3c_Dma_TargetEndsEarly_StopsReceiveChannelAndReportsCount( void )
{
    const i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_DMA );
    i3c_Data_t *       buffer = REGMEM_SRAM_PTR( i3c_Data_t, 0u );

    UtI3c_Init_Dma();
    UtI3c_Set_Table();
    Gpdma_Set_BlockSize_ExpectAndReturn( config.RxDmaPeriphId,
                                         config.RxDmaChannelId,
                                         UT_I3C_BUF_CNT,
                                         GPDMA_REQUEST_OK );
    Gpdma_Set_DestinationAddr_ExpectAndReturn( config.RxDmaPeriphId,
                                               config.RxDmaChannelId,
                                               REGMEM_SRAM_BASE,
                                               GPDMA_REQUEST_OK );
    Gpdma_Set_ChannelActive_ExpectAndReturn( config.RxDmaPeriphId, config.RxDmaChannelId, GPDMA_REQUEST_OK );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_Read( I3C_PERIPH_1, UT_I3C_ADDR_A, buffer, UT_I3C_BUF_CNT ) );

    UtI3c_Expect_DmaStop( &config, I3C_FUNCTION_INACTIVE, I3C_FUNCTION_ACTIVE );
    UtI3c_Expect_DmaStop( &config, I3C_FUNCTION_ACTIVE, I3C_FUNCTION_ACTIVE );
    I3C1->SR = UT_I3C_EARLY_CNT;
    UtI3c_Set_Events( ( I3C_EVR_RXTGTENDF | I3C_EVR_FCF ) );
    utI3c_Isr();

    TEST_ASSERT_EQUAL_UINT16( UT_I3C_EARLY_CNT, utI3c_XferLastCnt );
}


/**
 * \brief Verifies a frame error in the DMA mode.
 *
 * \par Expected results
 * Both channels are stopped, the error callback is called.
 */
void Ut_I3c_Dma_FrameError_StopsChannelsAndReports( void )
{
    const i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_DMA );
    i3c_Data_t *       buffer = REGMEM_SRAM_PTR( i3c_Data_t, 0u );

    UtI3c_Init_Dma();
    UtI3c_Set_Table();
    Gpdma_Set_BlockSize_ExpectAndReturn( config.RxDmaPeriphId, config.RxDmaChannelId, 1u, GPDMA_REQUEST_OK );
    Gpdma_Set_DestinationAddr_ExpectAndReturn( config.RxDmaPeriphId,
                                               config.RxDmaChannelId,
                                               REGMEM_SRAM_BASE,
                                               GPDMA_REQUEST_OK );
    Gpdma_Set_ChannelActive_ExpectAndReturn( config.RxDmaPeriphId, config.RxDmaChannelId, GPDMA_REQUEST_OK );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_Read( I3C_PERIPH_1, UT_I3C_ADDR_A, buffer, 1u ) );

    UtI3c_Expect_DmaStop( &config, I3C_FUNCTION_ACTIVE, I3C_FUNCTION_ACTIVE );
    UtI3c_Set_Events( I3C_EVR_ERRF );
    utI3c_Isr();

    TEST_ASSERT_EQUAL_UINT32( 1u, utI3c_ErrCallCnt );
}


/**
 * \brief Verifies the error handlers of the DMA channels.
 *
 * \par Expected results
 * An error of the transmit channel ends the running request with an underrun, an error of the receive channel
 * with an overrun; both channels are stopped and the FIFOs are flushed. An error without a request is ignored.
 */
void Ut_I3c_Dma_ChannelError_EndsRequestWithUnderrunOrOverrun( void )
{
    const i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_DMA );
    i3c_Data_t *       buffer = REGMEM_SRAM_PTR( i3c_Data_t, 0u );

    UtI3c_Init_Dma();
    UtI3c_Set_Table();

    utI3c_DmaCfg[ 0u ].ErrorIsr( GPDMA_ERROR_TRANSFER );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI3c_ErrCallCnt );

    Gpdma_Set_BlockSize_ExpectAndReturn( config.RxDmaPeriphId, config.RxDmaChannelId, 1u, GPDMA_REQUEST_OK );
    Gpdma_Set_DestinationAddr_ExpectAndReturn( config.RxDmaPeriphId,
                                               config.RxDmaChannelId,
                                               REGMEM_SRAM_BASE,
                                               GPDMA_REQUEST_OK );
    Gpdma_Set_ChannelActive_ExpectAndReturn( config.RxDmaPeriphId, config.RxDmaChannelId, GPDMA_REQUEST_OK );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_Read( I3C_PERIPH_1, UT_I3C_ADDR_A, buffer, 1u ) );

    UtI3c_Expect_DmaStop( &config, I3C_FUNCTION_ACTIVE, I3C_FUNCTION_ACTIVE );
    utI3c_DmaCfg[ 1u ].ErrorIsr( GPDMA_ERROR_TRANSFER );
    TEST_ASSERT_EQUAL_UINT8( I3C_XFER_ERROR_OVERRUN, utI3c_ErrLastMask );

    Gpdma_Set_BlockSize_ExpectAndReturn( config.TxDmaPeriphId, config.TxDmaChannelId, 1u, GPDMA_REQUEST_OK );
    Gpdma_Set_SourceAddr_ExpectAndReturn( config.TxDmaPeriphId,
                                          config.TxDmaChannelId,
                                          REGMEM_SRAM_BASE,
                                          GPDMA_REQUEST_OK );
    Gpdma_Set_ChannelActive_ExpectAndReturn( config.TxDmaPeriphId, config.TxDmaChannelId, GPDMA_REQUEST_OK );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_Write( I3C_PERIPH_1, UT_I3C_ADDR_A, buffer, 1u ) );

    UtI3c_Expect_DmaStop( &config, I3C_FUNCTION_ACTIVE, I3C_FUNCTION_ACTIVE );
    utI3c_DmaCfg[ 0u ].ErrorIsr( GPDMA_ERROR_TRANSFER );
    TEST_ASSERT_EQUAL_UINT8( I3C_XFER_ERROR_UNDERRUN, utI3c_ErrLastMask );
    TEST_ASSERT_EQUAL_UINT32( UT_I3C_PAIR_CNT, utI3c_ErrCallCnt );
}


/**
 * \brief Verifies the DMA helper functions with invalid parameters.
 *
 * \par Expected results
 * Start, Stop and Get_Remaining return the error for an invalid direction, a zero count or address, a null
 * pointer and an instance whose channels are not initialized.
 */
void Ut_I3cDma_Functions_InvalidCases_ReturnError( void )
{
    i3c_DataCnt_t remaining = 0u;

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3cDma_Start( I3C_PERIPH_1, I3C_DMA_DIR_TX, REGMEM_SRAM_BASE, 1u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3cDma_Stop( I3C_PERIPH_1, I3C_DMA_DIR_TX ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3cDma_Get_Remaining( I3C_PERIPH_1, I3C_DMA_DIR_RX, &remaining ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3cDma_Init( I3C_PERIPH_CNT ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3cDma_Deinit( I3C_PERIPH_CNT ) );

    UtI3c_Init_Dma();

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3cDma_Start( I3C_PERIPH_1, I3C_DMA_DIR_CNT, REGMEM_SRAM_BASE, 1u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3cDma_Start( I3C_PERIPH_1, I3C_DMA_DIR_TX, REGMEM_SRAM_BASE, 0u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3cDma_Start( I3C_PERIPH_1, I3C_DMA_DIR_TX, 0u, 1u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3cDma_Stop( I3C_PERIPH_1, I3C_DMA_DIR_CNT ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3cDma_Get_Remaining( I3C_PERIPH_1, I3C_DMA_DIR_RX, I3C_NULL_PTR ) );
}


/**
 * \brief Verifies the remaining count of a DMA channel.
 *
 * \par Expected results
 * The block size register of the channel is returned as the count of the bytes not moved; a failing Gpdma and
 * failing channel control calls return the error.
 */
void Ut_I3cDma_GetRemainingAndFailingCalls_ReportGpdmaResult( void )
{
    const i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_DMA );
    gpdma_BlockSize_t  blockSize = UT_I3C_EARLY_CNT;
    i3c_DataCnt_t      remaining = 0u;

    UtI3c_Init_Dma();

    Gpdma_Get_BlockSize_ExpectAndReturn( config.RxDmaPeriphId, config.RxDmaChannelId, I3C_NULL_PTR, GPDMA_REQUEST_OK );
    Gpdma_Get_BlockSize_IgnoreArg_blockSize();
    Gpdma_Get_BlockSize_ReturnThruPtr_blockSize( &blockSize );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3cDma_Get_Remaining( I3C_PERIPH_1, I3C_DMA_DIR_RX, &remaining ) );
    TEST_ASSERT_EQUAL_UINT16( UT_I3C_EARLY_CNT, remaining );

    Gpdma_Get_BlockSize_ExpectAndReturn( config.RxDmaPeriphId,
                                         config.RxDmaChannelId,
                                         I3C_NULL_PTR,
                                         GPDMA_REQUEST_ERROR );
    Gpdma_Get_BlockSize_IgnoreArg_blockSize();
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3cDma_Get_Remaining( I3C_PERIPH_1, I3C_DMA_DIR_RX, &remaining ) );

    Gpdma_Set_ChannelInactive_ExpectAndReturn( config.TxDmaPeriphId, config.TxDmaChannelId, GPDMA_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3cDma_Stop( I3C_PERIPH_1, I3C_DMA_DIR_TX ) );

    Gpdma_Set_BlockSize_ExpectAndReturn( config.TxDmaPeriphId, config.TxDmaChannelId, 1u, GPDMA_REQUEST_OK );
    Gpdma_Set_SourceAddr_ExpectAndReturn( config.TxDmaPeriphId,
                                          config.TxDmaChannelId,
                                          REGMEM_SRAM_BASE,
                                          GPDMA_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3cDma_Start( I3C_PERIPH_1, I3C_DMA_DIR_TX, REGMEM_SRAM_BASE, 1u ) );
}


/* ---------------------------------- Target role ---------------------------- */

/**
 * \brief Verifies that the target functions are rejected for the controller role and before the initialization.
 *
 * \par Expected results
 * Configuration, in-band interrupt, hot-join and buffer functions return the error.
 */
void Ut_I3c_TargetFunctions_ControllerOrNotInitialized_ReturnError( void )
{
    const i3c_TargetConfig_t targetConfig = UtI3c_Get_TargetConfig();

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_TargetConfig( I3C_PERIPH_1, &targetConfig ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_TargetReadBuffer( I3C_PERIPH_1, utI3c_TxData, 1u ) );

    UtI3c_Init_Ctrl();

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_TargetConfig( I3C_PERIPH_1, &targetConfig ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_IbiRequest( I3C_PERIPH_1, I3C_NULL_PTR, 0u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_HotJoinRequest( I3C_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_TargetReadBuffer( I3C_PERIPH_1, utI3c_TxData, 1u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_TargetWriteBuffer( I3C_PERIPH_1, utI3c_RxBuf, 1u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_TargetConfig( I3C_PERIPH_CNT, &targetConfig ) );
}


/**
 * \brief Verifies the rejected target configurations.
 *
 * \par Expected results
 * A null pointer, an instance ID above 15, an in-band interrupt payload above four bytes, a controller capable
 * bus characteristic and an in-band interrupt without the capability bit return the error before a register is
 * written.
 */
void Ut_I3c_SetTargetConfig_InvalidConfig_ReturnsErrorWithoutRegisterWrite( void )
{
    i3c_TargetConfig_t targetConfig = UtI3c_Get_TargetConfig();

    UtI3c_Init_Tgt();

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_TargetConfig( I3C_PERIPH_1, I3C_NULL_PTR ) );

    targetConfig.InstanceId = UT_I3C_INSTANCE_ID_INVALID;
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_TargetConfig( I3C_PERIPH_1, &targetConfig ) );

    targetConfig = UtI3c_Get_TargetConfig();
    targetConfig.MaxIbiPayload = I3C_IBI_PAYLOAD_CNT_MAX + 1u;
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_TargetConfig( I3C_PERIPH_1, &targetConfig ) );

    targetConfig = UtI3c_Get_TargetConfig();
    targetConfig.Bcr = I3C_BCR_CONTROLLER_CAPABLE;
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_TargetConfig( I3C_PERIPH_1, &targetConfig ) );

    targetConfig = UtI3c_Get_TargetConfig();
    targetConfig.Bcr = 0u;
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_TargetConfig( I3C_PERIPH_1, &targetConfig ) );

    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->BCR );
    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->MAXRLR );
}


/**
 * \brief Verifies the identity registers written by a valid target configuration.
 *
 * \par Expected results
 * BCR, DCR, the maximum lengths, the instance ID of the provisioned ID and the enables of the in-band interrupt
 * and of the hot-join are written; the peripheral is enabled again. The reserved bit BCR1 (IBI request capable,
 * answered by the hardware) is not written, so the register holds the payload bit only.
 */
void Ut_I3c_SetTargetConfig_ValidConfig_WritesIdentityRegisters( void )
{
    const i3c_TargetConfig_t targetConfig = UtI3c_Get_TargetConfig();

    UtI3c_Init_Tgt();

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_TargetConfig( I3C_PERIPH_1, &targetConfig ) );

    TEST_ASSERT_EQUAL_UINT32( I3C_BCR_IBI_PAYLOAD, I3C1->BCR );
    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->BCR & I3C_BCR_IBI_CAPABLE );
    TEST_ASSERT_EQUAL_UINT32( targetConfig.Dcr, I3C1->DCR );
    TEST_ASSERT_EQUAL_UINT32( ( ( (uint32_t)targetConfig.MaxReadLen << I3C_MAXRLR_MRL_Pos ) |
                                ( (uint32_t)targetConfig.MaxIbiPayload << I3C_MAXRLR_IBIP_Pos ) ),
                              I3C1->MAXRLR );
    TEST_ASSERT_EQUAL_UINT32( targetConfig.MaxWriteLen, I3C1->MAXWLR );
    TEST_ASSERT_EQUAL_UINT32( ( (uint32_t)targetConfig.InstanceId << I3C_EPIDR_MIPIID_Pos ),
                              I3C1->EPIDR & I3C_EPIDR_MIPIID_Msk );
    TEST_ASSERT_EQUAL_UINT32( ( I3C_DEVR0_IBIEN | I3C_DEVR0_HJEN ),
                              I3C1->DEVR0 & ( I3C_DEVR0_IBIEN | I3C_DEVR0_HJEN ) );
    TEST_ASSERT_EQUAL_UINT32( I3C_CFGR_EN, I3C1->CFGR & I3C_CFGR_EN );
}


/**
 * \brief Verifies the static address of the target.
 *
 * \par Expected results
 * On I3C1 of the lines that have the register the static address and its valid bit are written, on the other
 * lines the configuration with a static address is rejected; an invalid static address is always rejected.
 */
void Ut_I3c_SetTargetConfig_StaticAddress_OnlyWhereRegisterExists( void )
{
    i3c_TargetConfig_t targetConfig = UtI3c_Get_TargetConfig();

    UtI3c_Init_Tgt();
    targetConfig.StaticAddressState = I3C_FUNCTION_ACTIVE;
    targetConfig.StaticAddress      = UT_I3C_ADDR_STATIC;

    TEST_ASSERT_EQUAL( UT_I3C_STATIC_RESULT, I3c_Set_TargetConfig( I3C_PERIPH_1, &targetConfig ) );

#if UT_I3C_HAS_I2CCFGR
    TEST_ASSERT_EQUAL_UINT32( ( ( (uint32_t)UT_I3C_ADDR_STATIC << UT_I3C_SA_POS ) | UT_I3C_SAVAL ),
                              I3c_Reg_Get_Val( I3C_PERIPH_1, I3C_REG_I2CCFGR, UT_I3C_ALL_BITS ) );
#endif

    targetConfig.StaticAddress = I3C_ADDRESS_MASK + 1u;
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_TargetConfig( I3C_PERIPH_1, &targetConfig ) );
}


/**
 * \brief Verifies that a configuration that does not stay in the registers fails.
 *
 * \par Expected results
 * With a model that clears the bus characteristics register the verification fails and the error is returned.
 */
void Ut_I3c_SetTargetConfig_RegisterNotKept_ReturnsError( void )
{
    const i3c_TargetConfig_t targetConfig = UtI3c_Get_TargetConfig();

    UtI3c_Init_Tgt();
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtI3c_HwModel_BcrNotKept ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_TargetConfig( I3C_PERIPH_1, &targetConfig ) );
}


/**
 * \brief Verifies the in-band interrupt request of the target.
 *
 * \par Expected results
 * The payload goes to the payload register, the control word has the in-band interrupt type and the count, the
 * request is busy until the end event; the target request callback is called and the state is idle.
 */
void Ut_I3c_SetIbiRequest_ValidPayload_WritesPayloadAndControlWord( void )
{
    const i3c_Data_t payload[ UT_I3C_TGT_MAX_IBI ] = { UT_I3C_DATA_BYTE, UT_I3C_DATA_BYTE_2 };
    i3c_XferState_t  xferState = I3C_XFER_STATE_CNT;

    UtI3c_Init_Tgt();
    UtI3c_Set_TargetConfig();
    I3C1->DEVR0 |= I3C_DEVR0_DAVAL;

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_IbiRequest( I3C_PERIPH_1, payload, UT_I3C_TGT_MAX_IBI ) );

    TEST_ASSERT_EQUAL_UINT32( ( (uint32_t)UT_I3C_DATA_BYTE | ( (uint32_t)UT_I3C_DATA_BYTE_2 << I3C_BYTE_BIT_CNT ) ),
                              I3C1->IBIDR );
    TEST_ASSERT_EQUAL_UINT32( ( ( I3C_MTYPE_IBI << I3C_CR_MTYPE_Pos ) | ( UT_I3C_TGT_MAX_IBI << I3C_CR_DCNT_Pos ) ),
                              I3C1->CR );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_XferState( I3C_PERIPH_1, &xferState ) );
    TEST_ASSERT_EQUAL( I3C_XFER_STATE_BUSY, xferState );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_IbiRequest( I3C_PERIPH_1, payload, 1u ) );

    UtI3c_Set_Events( I3C_EVR_IBIENDF );
    I3c_Task();

    TEST_ASSERT_EQUAL_UINT32( 1u, utI3c_TgtReqCallCnt );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_XferState( I3C_PERIPH_1, &xferState ) );
    TEST_ASSERT_EQUAL( I3C_XFER_STATE_IDLE, xferState );
}


/**
 * \brief Verifies the rejected in-band interrupt requests.
 *
 * \par Expected results
 * A target without a dynamic address, a payload longer than the configured maximum or than four bytes, a missing
 * payload buffer and a payload for a device without the payload capability return the error; a request without
 * a payload is accepted.
 */
void Ut_I3c_SetIbiRequest_InvalidCases_ReturnError( void )
{
    UtI3c_Init_Tgt();
    UtI3c_Set_TargetConfig();

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_IbiRequest( I3C_PERIPH_1, utI3c_TxData, 1u ) );

    I3C1->DEVR0 |= I3C_DEVR0_DAVAL;

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR,
                       I3c_Set_IbiRequest( I3C_PERIPH_1, utI3c_TxData, UT_I3C_TGT_MAX_IBI + 1u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_IbiRequest( I3C_PERIPH_1, I3C_NULL_PTR, 1u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_IbiRequest( I3C_PERIPH_1, I3C_NULL_PTR, 0u ) );
}


/**
 * \brief Verifies the hot-join request of the target.
 *
 * \par Expected results
 * With the hot-join enabled and no dynamic address the control word has the hot-join type; the end event calls the
 * target request callback; a target with a dynamic address is rejected.
 */
void Ut_I3c_SetHotJoinRequest_NoAddress_WritesControlWord( void )
{
    UtI3c_Init_Tgt();

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_HotJoinRequest( I3C_PERIPH_1 ) );

    UtI3c_Set_TargetConfig();
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_HotJoinRequest( I3C_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_UINT32( ( I3C_MTYPE_HOT_JOIN << I3C_CR_MTYPE_Pos ), I3C1->CR );

    UtI3c_Set_Events( I3C_EVR_HJF );
    I3c_Task();
    TEST_ASSERT_EQUAL_UINT32( 1u, utI3c_TgtReqCallCnt );

    I3C1->DEVR0 |= I3C_DEVR0_DAVAL;
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_HotJoinRequest( I3C_PERIPH_1 ) );
}


/**
 * \brief Verifies the read buffer of the target.
 *
 * \par Expected results
 * The count and the preload bit are written to I3C_TGTTDR, the first eight bytes are written to the transmit
 * FIFO, the rest follows on the task; the end of the read reports the count of the status register, a read that
 * the controller ended early flushes the transmit FIFO.
 */
void Ut_I3c_SetTargetReadBuffer_Armed_PreloadsAndReportsEnd( void )
{
    UtI3c_Init_Tgt();
    I3C1->EVR = I3C_EVR_TXFNFF;
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtI3c_HwModel_Capture ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_TargetReadBuffer( I3C_PERIPH_1, utI3c_TxData, UT_I3C_BUF_CNT ) );

    TEST_ASSERT_EQUAL_UINT32( ( I3C_TGTTDR_PRELOAD | UT_I3C_BUF_CNT ), I3C1->TGTTDR );
    TEST_ASSERT_EQUAL_UINT32( I3C_FIFO_DEPTH, utI3c_TxSeqCnt );

    I3c_Task();
    TEST_ASSERT_EQUAL_UINT32( UT_I3C_BUF_CNT, utI3c_TxSeqCnt );

    UtI3c_Set_Events( I3C_EVR_FCF );
    I3C1->SR = ( I3C_SR_DIR_Msk | UT_I3C_EARLY_CNT );
    I3c_Task();

    TEST_ASSERT_EQUAL_UINT32( 1u, utI3c_TgtReadCallCnt );
    TEST_ASSERT_EQUAL_UINT16( UT_I3C_EARLY_CNT, utI3c_TgtReadCnt );
    TEST_ASSERT_EQUAL_UINT32( I3C_CFGR_TXFLUSH, I3C1->CFGR & I3C_CFGR_TXFLUSH );
}


/**
 * \brief Verifies a read that the controller completes.
 *
 * \par Expected results
 * When the status register reports all bytes the transmit FIFO is not flushed.
 */
void Ut_I3c_SetTargetReadBuffer_ReadCompletes_DoesNotFlushFifo( void )
{
    UtI3c_Init_Tgt();
    I3C1->EVR = I3C_EVR_TXFNFF;

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_TargetReadBuffer( I3C_PERIPH_1, utI3c_TxData, UT_I3C_WRITE_CNT ) );

    I3C1->SR  = ( I3C_SR_DIR_Msk | UT_I3C_WRITE_CNT );
    UtI3c_Set_Events( I3C_EVR_FCF );
    I3c_Task();

    TEST_ASSERT_EQUAL_UINT16( UT_I3C_WRITE_CNT, utI3c_TgtReadCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->CFGR & I3C_CFGR_TXFLUSH );
}


/**
 * \brief Verifies the rejected buffers.
 *
 * \par Expected results
 * A missing buffer and a zero size return the error.
 */
void Ut_I3c_SetTargetBuffers_InvalidParameters_ReturnError( void )
{
    UtI3c_Init_Tgt();

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_TargetReadBuffer( I3C_PERIPH_1, I3C_NULL_PTR, 1u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_TargetReadBuffer( I3C_PERIPH_1, utI3c_TxData, 0u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_TargetWriteBuffer( I3C_PERIPH_1, I3C_NULL_PTR, 1u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_TargetWriteBuffer( I3C_PERIPH_1, utI3c_RxBuf, 0u ) );
}


/**
 * \brief Verifies the write buffer of the target.
 *
 * \par Expected results
 * The received bytes are stored in the buffer, the end of the write reports the count of the status register;
 * bytes beyond the size of the buffer are dropped and not counted.
 */
void Ut_I3c_SetTargetWriteBuffer_Armed_StoresReceivedBytes( void )
{
    UtI3c_Init_Tgt();
    I3C1->RDR = UT_I3C_DATA_BYTE;
    I3C1->EVR = I3C_EVR_RXFNEF;

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_TargetWriteBuffer( I3C_PERIPH_1, utI3c_RxBuf, UT_I3C_PAIR_CNT ) );
    I3c_Task();

    TEST_ASSERT_EQUAL_UINT8( UT_I3C_DATA_BYTE, utI3c_RxBuf[ 0u ] );
    TEST_ASSERT_EQUAL_UINT8( UT_I3C_DATA_BYTE, utI3c_RxBuf[ 1u ] );

    I3C1->SR = UT_I3C_EARLY_CNT;
    UtI3c_Set_Events( I3C_EVR_FCF );
    I3c_Task();

    TEST_ASSERT_EQUAL_UINT32( 1u, utI3c_TgtWriteCallCnt );
    TEST_ASSERT_EQUAL_UINT16( UT_I3C_PAIR_CNT, utI3c_TgtWriteCnt );
}


/**
 * \brief Verifies a write of the controller without a buffer.
 *
 * \par Expected results
 * The received bytes are read and dropped, the end of the write reports no stored byte.
 */
void Ut_I3c_Task_WriteWithoutBuffer_DropsBytes( void )
{
    UtI3c_Init_Tgt();
    I3C1->RDR = UT_I3C_DATA_BYTE;
    I3C1->SR  = UT_I3C_EARLY_CNT;
    I3C1->EVR = ( I3C_EVR_RXFNEF | I3C_EVR_FCF );

    I3c_Task();

    TEST_ASSERT_EQUAL_UINT32( 1u, utI3c_TgtWriteCallCnt );
    TEST_ASSERT_EQUAL_UINT16( 0u, utI3c_TgtWriteCnt );
}


/**
 * \brief Verifies the assignment of the dynamic address to the target.
 *
 * \par Expected results
 * The address callback gets the address of the own device register and the event flag is cleared.
 */
void Ut_I3c_Task_DynamicAddressAssigned_CallsAddressCallback( void )
{
    UtI3c_Init_Tgt();
    I3C1->DEVR0 = ( ( (uint32_t)UT_I3C_ADDR_A << I3C_DEVR0_DA_Pos ) | I3C_DEVR0_DAVAL );
    I3C1->EVR   = I3C_EVR_DAUPDF;

    I3c_Task();

    TEST_ASSERT_EQUAL_UINT32( 1u, utI3c_TgtAddrCallCnt );
    TEST_ASSERT_EQUAL_UINT8( UT_I3C_ADDR_A, utI3c_TgtAddr );
    TEST_ASSERT_EQUAL_UINT32( I3C_EVR_DAUPDF, I3C1->CEVR );
}


/**
 * \brief Verifies the CCC events of the target.
 *
 * \par Expected results
 * Every CCC event flag is reported to the callback as its event, in the order of the flag list; the flags are
 * cleared.
 */
void Ut_I3c_Task_CccEvents_AreReportedAndCleared( void )
{
    const uint32_t flags = ( I3C_EVR_INTUPDF | I3C_EVR_ASUPDF | I3C_EVR_MWLUPDF | I3C_EVR_MRLUPDF | I3C_EVR_RSTF |
                             I3C_EVR_GETF | I3C_EVR_STAF | I3C_EVR_DEFF | I3C_EVR_GRPF );

    UtI3c_Init_Tgt();
    I3C1->EVR = flags;

    I3c_Task();

    TEST_ASSERT_EQUAL_UINT32( I3C_CCC_EVENT_CNT, utI3c_TgtCccCallCnt );

    for( uint32_t idx = 0u; (uint32_t)I3C_CCC_EVENT_CNT > idx; idx++ )
    {
        TEST_ASSERT_EQUAL( (i3c_CccEvent_t)idx, utI3c_TgtCccEvents[ idx ] );
    }

    TEST_ASSERT_EQUAL_UINT32( flags, I3C1->CEVR );
}


/**
 * \brief Verifies the error of the target.
 *
 * \par Expected results
 * An underrun without data of a read calls the read request callback and the error callback; a protocol error
 * calls only the error callback; the FIFOs are flushed and the flag is cleared.
 */
void Ut_I3c_Task_TargetError_ReportsErrorAndReadRequest( void )
{
    UtI3c_Init_Tgt();
    I3C1->EVR = I3C_EVR_ERRF;
    I3C1->SER = I3C_SER_DOVR;

    I3c_Task();

    TEST_ASSERT_EQUAL_UINT32( 1u, utI3c_TgtReadReqCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utI3c_ErrCallCnt );
    TEST_ASSERT_EQUAL_UINT32( ( I3C_CFGR_TXFLUSH | I3C_CFGR_RXFLUSH ),
                              I3C1->CFGR & ( I3C_CFGR_TXFLUSH | I3C_CFGR_RXFLUSH ) );
    TEST_ASSERT_EQUAL_UINT32( I3C_EVR_ERRF, I3C1->CEVR );

    I3C1->SER = I3C_SER_PERR;
    I3c_Task();

    TEST_ASSERT_EQUAL_UINT32( 1u, utI3c_TgtReadReqCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_I3C_PAIR_CNT, utI3c_ErrCallCnt );
    TEST_ASSERT_EQUAL_UINT8( I3C_XFER_ERROR_FRAME, utI3c_ErrLastMask );
}


/**
 * \brief Verifies the stop of the target.
 *
 * \par Expected results
 * A running in-band interrupt request ends with the aborted error, the buffers are released (a later end of a
 * read does not call the callback with data) and the peripheral is enabled.
 */
void Ut_I3c_SetXferStop_Target_AbortsRequestAndReleasesBuffers( void )
{
    UtI3c_Init_Tgt();
    UtI3c_Set_TargetConfig();
    I3C1->DEVR0 |= I3C_DEVR0_DAVAL;
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_TargetReadBuffer( I3C_PERIPH_1, utI3c_TxData, 1u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_IbiRequest( I3C_PERIPH_1, I3C_NULL_PTR, 0u ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_XferStop( I3C_PERIPH_1 ) );

    TEST_ASSERT_EQUAL_UINT32( 1u, utI3c_ErrCallCnt );
    TEST_ASSERT_EQUAL_UINT8( I3C_XFER_ERROR_ABORTED, utI3c_ErrLastMask );
    TEST_ASSERT_EQUAL_UINT32( I3C_CFGR_EN, I3C1->CFGR & I3C_CFGR_EN );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_IbiRequest( I3C_PERIPH_1, I3C_NULL_PTR, 0u ) );
}


/**
 * \brief Verifies the transmit FIFO interrupt of the target in the ISR mode.
 *
 * \par Expected results
 * Arming a long read buffer enables the transmit FIFO interrupt, the interrupt handler writes the rest and
 * disables the interrupt.
 */
void Ut_I3c_Isr_TargetReadBuffer_FifoInterruptFollowsData( void )
{
    UtI3c_Init_TgtIsr();

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_TargetReadBuffer( I3C_PERIPH_1, utI3c_TxData, UT_I3C_BUF_CNT ) );
    TEST_ASSERT_EQUAL_UINT32( I3C_IER_TXFNFIE, I3C1->IER & I3C_IER_TXFNFIE );

    I3C1->EVR = I3C_EVR_TXFNFF;
    utI3c_Isr();
    TEST_ASSERT_EQUAL_UINT32( I3C_IER_TXFNFIE, I3C1->IER & I3C_IER_TXFNFIE );
    utI3c_Isr();

    TEST_ASSERT_EQUAL_UINT32( 0u, I3C1->IER & I3C_IER_TXFNFIE );
}


/**
 * \brief Verifies the receive FIFO interrupt of the target in the ISR mode.
 *
 * \par Expected results
 * Arming the write buffer enables the receive FIFO interrupt, the interrupt handler stores the bytes.
 */
void Ut_I3c_Isr_TargetWriteBuffer_ReceiveInterruptStoresBytes( void )
{
    UtI3c_Init_TgtIsr();
    I3C1->RDR = UT_I3C_DATA_BYTE;

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_TargetWriteBuffer( I3C_PERIPH_1, utI3c_RxBuf, UT_I3C_PAIR_CNT ) );
    TEST_ASSERT_EQUAL_UINT32( I3C_IER_RXFNEIE, I3C1->IER & I3C_IER_RXFNEIE );

    I3C1->EVR = I3C_EVR_RXFNEF;
    utI3c_Isr();

    TEST_ASSERT_EQUAL_UINT8( UT_I3C_DATA_BYTE, utI3c_RxBuf[ 0u ] );
}


/**
 * \brief Verifies the target buffers in the DMA mode.
 *
 * \par Expected results
 * The read buffer starts the transmit channel, the write buffer the receive channel; the end of the read and of
 * the write stops the channel; the FIFOs are not served by the CPU.
 */
void Ut_I3c_Dma_TargetBuffers_StartAndStopChannels( void )
{
    const i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_TARGET, I3C_XFER_MODE_DMA );
    i3c_Data_t *       buffer = REGMEM_SRAM_PTR( i3c_Data_t, 0u );

    UtI3c_Init_TgtDma();

    Gpdma_Set_BlockSize_ExpectAndReturn( config.TxDmaPeriphId,
                                         config.TxDmaChannelId,
                                         UT_I3C_WRITE_CNT,
                                         GPDMA_REQUEST_OK );
    Gpdma_Set_SourceAddr_ExpectAndReturn( config.TxDmaPeriphId,
                                          config.TxDmaChannelId,
                                          REGMEM_SRAM_BASE,
                                          GPDMA_REQUEST_OK );
    Gpdma_Set_ChannelActive_ExpectAndReturn( config.TxDmaPeriphId, config.TxDmaChannelId, GPDMA_REQUEST_OK );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_TargetReadBuffer( I3C_PERIPH_1, buffer, UT_I3C_WRITE_CNT ) );

    Gpdma_Set_BlockSize_ExpectAndReturn( config.RxDmaPeriphId,
                                         config.RxDmaChannelId,
                                         UT_I3C_PAIR_CNT,
                                         GPDMA_REQUEST_OK );
    Gpdma_Set_DestinationAddr_ExpectAndReturn( config.RxDmaPeriphId,
                                               config.RxDmaChannelId,
                                               REGMEM_SRAM_BASE,
                                               GPDMA_REQUEST_OK );
    Gpdma_Set_ChannelActive_ExpectAndReturn( config.RxDmaPeriphId, config.RxDmaChannelId, GPDMA_REQUEST_OK );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_TargetWriteBuffer( I3C_PERIPH_1, buffer, UT_I3C_PAIR_CNT ) );

    Gpdma_Set_ChannelInactive_ExpectAndReturn( config.TxDmaPeriphId, config.TxDmaChannelId, GPDMA_REQUEST_OK );
    I3C1->SR = ( I3C_SR_DIR_Msk | UT_I3C_WRITE_CNT );
    UtI3c_Set_Events( I3C_EVR_FCF );
    utI3c_Isr();
    TEST_ASSERT_EQUAL_UINT16( UT_I3C_WRITE_CNT, utI3c_TgtReadCnt );

    Gpdma_Set_ChannelInactive_ExpectAndReturn( config.RxDmaPeriphId, config.RxDmaChannelId, GPDMA_REQUEST_OK );
    I3C1->SR = UT_I3C_PAIR_CNT;
    UtI3c_Set_Events( I3C_EVR_FCF );
    utI3c_Isr();
    TEST_ASSERT_EQUAL_UINT16( UT_I3C_PAIR_CNT, utI3c_TgtWriteCnt );
}


/**
 * \brief Verifies that a failing DMA channel rejects the target buffer.
 *
 * \par Expected results
 * Gpdma_Set_BlockSize() returning the error makes both buffer functions return the error.
 */
void Ut_I3c_Dma_TargetBufferStartFails_ReturnsError( void )
{
    const i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_TARGET, I3C_XFER_MODE_DMA );
    i3c_Data_t *       buffer = REGMEM_SRAM_PTR( i3c_Data_t, 0u );

    UtI3c_Init_TgtDma();

    Gpdma_Set_BlockSize_ExpectAndReturn( config.TxDmaPeriphId, config.TxDmaChannelId, 1u, GPDMA_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_TargetReadBuffer( I3C_PERIPH_1, buffer, 1u ) );

    Gpdma_Set_BlockSize_ExpectAndReturn( config.RxDmaPeriphId, config.RxDmaChannelId, 1u, GPDMA_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_TargetWriteBuffer( I3C_PERIPH_1, buffer, 1u ) );
}


/* ============================== TEST HELPERS =============================== */

/** Gets a configuration of the tests: the default one with the role, the mode and all the callbacks set */
static i3c_Config_t UtI3c_Get_Config( i3c_Role_t role, i3c_XferMode_t xferMode )
{
    i3c_Config_t config;

    (void)I3c_Get_DefaultConfig( &config );

    config.Role                      = role;
    config.XferMode                  = xferMode;
    config.XferCompleteCallback      = UtI3c_XferCallback;
    config.ErrorCallback             = UtI3c_ErrorCallback;
    config.IbiCallback               = UtI3c_IbiCallback;
    config.HotJoinCallback           = UtI3c_HotJoinCallback;
    config.TargetAddressCallback     = UtI3c_TgtAddressCallback;
    config.TargetReadRequestCallback = UtI3c_TgtReadReqCallback;
    config.TargetReadCallback        = UtI3c_TgtReadCallback;
    config.TargetWriteCallback       = UtI3c_TgtWriteCallback;
    config.TargetCccCallback         = UtI3c_TgtCccCallback;
    config.TargetRequestCallback     = UtI3c_TgtRequestCallback;

    return ( config );
}


/** Gets the target configuration used by the tests */
static i3c_TargetConfig_t UtI3c_Get_TargetConfig( void )
{
    i3c_TargetConfig_t targetConfig = { 0 };

    targetConfig.StaticAddressState = I3C_FUNCTION_INACTIVE;
    targetConfig.InstanceId         = UT_I3C_TGT_INSTANCE_ID;
    targetConfig.Bcr                = UT_I3C_TGT_BCR;
    targetConfig.Dcr                = UT_I3C_TGT_DCR;
    targetConfig.MaxWriteLen        = UT_I3C_TGT_MAX_WRITE;
    targetConfig.MaxReadLen         = UT_I3C_TGT_MAX_READ;
    targetConfig.MaxIbiPayload      = UT_I3C_TGT_MAX_IBI;
    targetConfig.IbiState           = I3C_FUNCTION_ACTIVE;
    targetConfig.HotJoinState       = I3C_FUNCTION_ACTIVE;

    return ( targetConfig );
}


/** Configures the identity of the initialized target with the configuration of the tests */
static void UtI3c_Set_TargetConfig( void )
{
    const i3c_TargetConfig_t targetConfig = UtI3c_Get_TargetConfig();

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_TargetConfig( I3C_PERIPH_1, &targetConfig ) );
}


/** Expects the enable of the kernel clock and the read of its frequency */
static void UtI3c_Expect_Clock( const i3c_Config_t * const config )
{
    Rcc_Set_PeriphActive_ExpectAndReturn( (rcc_PeriphId_t)config->ClkSrc, RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_StubWithCallback( UtI3c_RccGetClkStub );
}


/** Expects the release of an instance: interrupt lines, DMA channels, reset and clock */
static void UtI3c_Expect_Release( const i3c_Config_t * const config )
{
    if( I3C_XFER_MODE_POLL != config->XferMode )
    {
        Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_I3C1_EV, NVIC_REQUEST_OK );
        Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_I3C1_ER, NVIC_REQUEST_OK );
    }
    else
    {
        /* The interrupts are not used, no call expected */
    }

    if( I3C_XFER_MODE_DMA == config->XferMode )
    {
        Gpdma_Set_ChannelInactive_ExpectAndReturn( config->TxDmaPeriphId, config->TxDmaChannelId, GPDMA_REQUEST_OK );
        Gpdma_Set_InterruptInactive_ExpectAndReturn( config->TxDmaPeriphId, config->TxDmaChannelId, GPDMA_REQUEST_OK );
        Gpdma_Deinit_ExpectAndReturn( config->TxDmaPeriphId, config->TxDmaChannelId, GPDMA_REQUEST_OK );
        Gpdma_Set_ChannelInactive_ExpectAndReturn( config->RxDmaPeriphId, config->RxDmaChannelId, GPDMA_REQUEST_OK );
        Gpdma_Set_InterruptInactive_ExpectAndReturn( config->RxDmaPeriphId, config->RxDmaChannelId, GPDMA_REQUEST_OK );
        Gpdma_Deinit_ExpectAndReturn( config->RxDmaPeriphId, config->RxDmaChannelId, GPDMA_REQUEST_OK );
    }
    else
    {
        /* DMA is not used, no call expected */
    }

    Rcc_Set_ResetActive_ExpectAndReturn( (rcc_PeriphId_t)config->ClkSrc, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( (rcc_PeriphId_t)config->ClkSrc, RCC_REQUEST_OK );
    Rcc_Set_PeriphInactive_ExpectAndReturn( (rcc_PeriphId_t)config->ClkSrc, RCC_REQUEST_OK );
}


/** Expects the initialization of the DMA channels of the transmit and the receive FIFO */
static void UtI3c_Expect_DmaInit( const i3c_Config_t * const config )
{
    Gpdma_Init_StubWithCallback( UtI3c_GpdmaInitStub );

    Gpdma_Get_DefaultConfig_ExpectAnyArgsAndReturn( GPDMA_REQUEST_OK );
    Gpdma_Set_InterruptActive_ExpectAndReturn( config->TxDmaPeriphId, config->TxDmaChannelId, GPDMA_REQUEST_OK );
    Gpdma_Get_DefaultConfig_ExpectAnyArgsAndReturn( GPDMA_REQUEST_OK );
    Gpdma_Set_InterruptActive_ExpectAndReturn( config->RxDmaPeriphId, config->RxDmaChannelId, GPDMA_REQUEST_OK );
}


/** Expects the registration of the interrupt handler and the enable of both interrupt lines */
static void UtI3c_Expect_Irq( const i3c_Config_t * const config )
{
    Nvic_Set_PeriphIrq_Handler_StubWithCallback( UtI3c_NvicHandlerStub );

    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_I3C1_EV, config->IrqPriority, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( NVIC_PERIPH_IRQ_I3C1_EV, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_I3C1_ER, config->IrqPriority, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( NVIC_PERIPH_IRQ_I3C1_ER, NVIC_REQUEST_OK );
}


/** Expects the stop of the DMA channels of the transmit and / or the receive FIFO */
static void UtI3c_Expect_DmaStop( const i3c_Config_t * const config,
                                  i3c_FunctionState_t txStop,
                                  i3c_FunctionState_t rxStop )
{
    if( I3C_FUNCTION_ACTIVE == txStop )
    {
        Gpdma_Set_ChannelInactive_ExpectAndReturn( config->TxDmaPeriphId, config->TxDmaChannelId, GPDMA_REQUEST_OK );
    }
    else
    {
        /* The transmit channel is not stopped, no call expected */
    }

    if( I3C_FUNCTION_ACTIVE == rxStop )
    {
        Gpdma_Set_ChannelInactive_ExpectAndReturn( config->RxDmaPeriphId, config->RxDmaChannelId, GPDMA_REQUEST_OK );
    }
    else
    {
        /* The receive channel is not stopped, no call expected */
    }
}


/** Initializes an instance with all the expectations of its mode and checks the result */
static void UtI3c_Init( const i3c_Config_t * const config )
{
    UtI3c_Expect_Clock( config );

    if( I3C_XFER_MODE_DMA == config->XferMode )
    {
        UtI3c_Expect_DmaInit( config );
    }
    else
    {
        /* DMA is not used, no call expected */
    }

    if( I3C_XFER_MODE_POLL != config->XferMode )
    {
        UtI3c_Expect_Irq( config );
    }
    else
    {
        /* The events are polled, no call expected */
    }

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Init( config ) );
}


/** Initializes I3C1 as controller in the POLL mode */
static void UtI3c_Init_Ctrl( void )
{
    const i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_POLL );

    UtI3c_Init( &config );
}


/** Initializes I3C1 as controller in the ISR mode */
static void UtI3c_Init_Isr( void )
{
    const i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_ISR );

    UtI3c_Init( &config );
}


/** Initializes I3C1 as controller in the DMA mode */
static void UtI3c_Init_Dma( void )
{
    const i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_XFER_MODE_DMA );

    UtI3c_Init( &config );
}


/** Initializes I3C1 as target in the POLL mode */
static void UtI3c_Init_Tgt( void )
{
    const i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_TARGET, I3C_XFER_MODE_POLL );

    UtI3c_Init( &config );
}


/** Initializes I3C1 as target in the ISR mode */
static void UtI3c_Init_TgtIsr( void )
{
    const i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_TARGET, I3C_XFER_MODE_ISR );

    UtI3c_Init( &config );
}


/** Initializes I3C1 as target in the DMA mode */
static void UtI3c_Init_TgtDma( void )
{
    const i3c_Config_t config = UtI3c_Get_Config( I3C_ROLE_TARGET, I3C_XFER_MODE_DMA );

    UtI3c_Init( &config );
}


/** Presets the event register */
static void UtI3c_Set_Events( uint32_t events )
{
    I3C1->EVR = events;
}


/** Sets the device table used by most tests: an I3C device (in-band interrupt with payload) and a Fm+ I2C device */
static void UtI3c_Set_Table( void )
{
    i3c_Device_t devices[ UT_I3C_TABLE_CNT ] = { { 0 } };

    devices[ 0u ].Type           = I3C_DEVICE_I3C;
    devices[ 0u ].DynamicAddress = UT_I3C_ADDR_A;
    devices[ 0u ].IbiState       = I3C_FUNCTION_ACTIVE;
    devices[ 0u ].IbiPayload     = I3C_FUNCTION_ACTIVE;
    devices[ 0u ].AddressState   = I3C_FUNCTION_ACTIVE;
    devices[ 1u ].Type           = I3C_DEVICE_I2C;
    devices[ 1u ].StaticAddress  = UT_I3C_ADDR_I2C;
    devices[ 1u ].I2cSpeed       = I3C_I2C_SPEED_FM_PLUS;

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_DeviceTable( I3C_PERIPH_1, devices, UT_I3C_TABLE_CNT ) );
}


/** Computes the control word of a private, direct or legacy message from the register field positions */
static i3c_CtrlWord_t UtI3c_Get_MsgWord( i3c_FunctionState_t lastMsg,
                                         uint32_t msgType,
                                         i3c_Address_t address,
                                         i3c_Direction_t direction,
                                         i3c_DataCnt_t dataCnt )
{
    uint32_t word = ( ( msgType << I3C_CR_MTYPE_Pos ) | ( (uint32_t)address << I3C_CR_ADD_Pos ) |
                      ( (uint32_t)dataCnt << I3C_CR_DCNT_Pos ) );

    if( I3C_FUNCTION_ACTIVE == lastMsg )
    {
        word |= I3C_CR_MEND;
    }
    else
    {
        /* A repeated start follows, no stop bit */
    }

    if( I3C_DIRECTION_READ == direction )
    {
        word |= I3C_CR_RNW;
    }
    else
    {
        /* Write message, no read bit */
    }

    return ( word );
}


/** Computes the control word of a CCC from the register field positions */
static i3c_CtrlWord_t UtI3c_Get_CccWord( i3c_FunctionState_t lastMsg, i3c_Ccc_t ccc, i3c_DataCnt_t dataCnt )
{
    uint32_t word = ( ( I3C_MTYPE_CCC << I3C_CR_MTYPE_Pos ) | ( (uint32_t)ccc << I3C_CR_CCC_Pos ) |
                      ( (uint32_t)dataCnt << I3C_CR_DCNT_Pos ) );

    if( I3C_FUNCTION_ACTIVE == lastMsg )
    {
        word |= I3C_CR_MEND;
    }
    else
    {
        /* The second part of a direct CCC follows, no stop bit */
    }

    return ( word );
}

/* ------------------------------ Stubs and callbacks ------------------------ */

static rcc_RequestState_t UtI3c_RccGetClkStub( rcc_PeriphId_t periphId, rcc_FreqHz_t * const periphClk, int callCnt )
{
    (void)periphId;
    (void)callCnt;

    *periphClk = utI3c_KernelClk;

    return ( RCC_REQUEST_OK );
}


static gpio_RequestState_t UtI3c_GpioInitStub( gpio_Config_t * gpioConfig, int callCnt )
{
    (void)callCnt;

    if( UT_I3C_DEVR_CNT > utI3c_GpioCfgCnt )
    {
        utI3c_GpioCfg[ utI3c_GpioCfgCnt ] = *gpioConfig;
        utI3c_GpioCfgCnt++;
    }
    else
    {
        /* The record is full */
    }

    return ( GPIO_REQUEST_OK );
}


static gpio_RequestState_t UtI3c_GpioLevelHighStub( gpio_PortId_t portId,
                                                    gpio_PinId_t pinId,
                                                    gpio_PinLevel_t * const pinLevel,
                                                    int callCnt )
{
    (void)portId;
    (void)pinId;
    (void)callCnt;

    utI3c_GpioLevelCalls++;
    *pinLevel = GPIO_PIN_LEVEL_HIGH;

    return ( GPIO_REQUEST_OK );
}


static gpio_RequestState_t UtI3c_GpioLevelLowStub( gpio_PortId_t portId,
                                                   gpio_PinId_t pinId,
                                                   gpio_PinLevel_t * const pinLevel,
                                                   int callCnt )
{
    (void)portId;
    (void)pinId;
    (void)callCnt;

    utI3c_GpioLevelCalls++;
    *pinLevel = GPIO_PIN_LEVEL_LOW;

    return ( GPIO_REQUEST_OK );
}


static nvic_RequestState_t UtI3c_NvicHandlerStub( nvic_PeriphIrqList_t irqId,
                                                  const nvic_IsrCallback_t irqHandler,
                                                  int callCnt )
{
    (void)irqId;
    (void)callCnt;

    utI3c_Isr = irqHandler;

    return ( NVIC_REQUEST_OK );
}


static gpdma_RequestState_t UtI3c_GpdmaInitStub( gpdma_ConfigStruct_t * const configStruct, int callCnt )
{
    (void)callCnt;

    if( UT_I3C_DEVR_CNT > utI3c_DmaCfgCnt )
    {
        utI3c_DmaXfer[ utI3c_DmaCfgCnt ] = *configStruct->TransferConfig;
        utI3c_DmaCfg[ utI3c_DmaCfgCnt ]  = *configStruct;
        utI3c_DmaCfg[ utI3c_DmaCfgCnt ].TransferConfig = &utI3c_DmaXfer[ utI3c_DmaCfgCnt ];
        utI3c_DmaCfgCnt++;
    }
    else
    {
        /* The record is full */
    }

    return ( GPDMA_REQUEST_OK );
}


static void UtI3c_XferCallback( i3c_PeriphId_t periphId, i3c_DataCnt_t xferCnt )
{
    (void)periphId;

    utI3c_XferCallCnt++;
    utI3c_XferLastCnt = xferCnt;
}


static void UtI3c_ErrorCallback( i3c_PeriphId_t periphId, i3c_XferError_t errorMask )
{
    (void)periphId;

    utI3c_ErrCallCnt++;
    utI3c_ErrLastMask = errorMask;
}


static void UtI3c_IbiCallback( i3c_PeriphId_t periphId,
                               i3c_Address_t address,
                               const i3c_Data_t * const payload,
                               i3c_DataCnt_t payloadCnt )
{
    (void)periphId;

    utI3c_Ibi.CallCnt++;
    utI3c_Ibi.Address    = address;
    utI3c_Ibi.PayloadCnt = payloadCnt;

    for( uint32_t idx = 0u; (uint32_t)I3C_IBI_PAYLOAD_CNT_MAX > idx; idx++ )
    {
        utI3c_Ibi.Payload[ idx ] = payload[ idx ];
    }
}


static void UtI3c_HotJoinCallback( i3c_PeriphId_t periphId )
{
    (void)periphId;

    utI3c_HjCallCnt++;
}


static void UtI3c_TgtAddressCallback( i3c_PeriphId_t periphId, i3c_Address_t address )
{
    (void)periphId;

    utI3c_TgtAddrCallCnt++;
    utI3c_TgtAddr = address;
}


static void UtI3c_TgtReadReqCallback( i3c_PeriphId_t periphId )
{
    (void)periphId;

    utI3c_TgtReadReqCnt++;
}


static void UtI3c_TgtReadCallback( i3c_PeriphId_t periphId, i3c_DataCnt_t xferCnt )
{
    (void)periphId;

    utI3c_TgtReadCallCnt++;
    utI3c_TgtReadCnt = xferCnt;
}


static void UtI3c_TgtWriteCallback( i3c_PeriphId_t periphId, i3c_DataCnt_t xferCnt )
{
    (void)periphId;

    utI3c_TgtWriteCallCnt++;
    utI3c_TgtWriteCnt = xferCnt;
}


static void UtI3c_TgtCccCallback( i3c_PeriphId_t periphId, i3c_CccEvent_t event )
{
    (void)periphId;

    if( UT_I3C_RECORD_CNT > utI3c_TgtCccCallCnt )
    {
        utI3c_TgtCccEvents[ utI3c_TgtCccCallCnt ] = event;
    }
    else
    {
        /* The record is full */
    }

    utI3c_TgtCccCallCnt++;
}


static void UtI3c_TgtRequestCallback( i3c_PeriphId_t periphId )
{
    (void)periphId;

    utI3c_TgtReqCallCnt++;
}

/* --------------------------------- HW models ------------------------------- */

/** HW model: the enable bit does not stay set */
static void UtI3c_HwModel_EnNotKept( void )
{
    (void)__atomic_and_fetch( &I3C1->CFGR, (uint32_t)~I3C_CFGR_EN, __ATOMIC_SEQ_CST );
}


/** HW model: the enable bit stays set (the disable does not take effect) */
static void UtI3c_HwModel_EnForced( void )
{
    (void)__atomic_or_fetch( &I3C1->CFGR, (uint32_t)I3C_CFGR_EN, __ATOMIC_SEQ_CST );
}


/** HW model: the controller initialization bit does not stay set */
static void UtI3c_HwModel_CfgrNotKept( void )
{
    (void)__atomic_and_fetch( &I3C1->CFGR, (uint32_t)~I3C_CFGR_CRINIT, __ATOMIC_SEQ_CST );
}


/** HW model: the bus characteristics register does not keep the written value */
static void UtI3c_HwModel_BcrNotKept( void )
{
    (void)__atomic_and_fetch( &I3C1->BCR, 0u, __ATOMIC_SEQ_CST );
}


/** HW model: records the sequence of the values written to the transmit data and to the control register */
static void UtI3c_HwModel_Capture( void )
{
    const uint32_t tdrValue = I3C1->TDR;
    const uint32_t crValue  = I3C1->CR;

    if( ( tdrValue != utI3c_TxLast ) &&
        ( UT_I3C_RECORD_CNT > utI3c_TxSeqCnt )    )
    {
        utI3c_TxSeq[ utI3c_TxSeqCnt ] = tdrValue;
        utI3c_TxSeqCnt++;
        utI3c_TxLast = tdrValue;
    }
    else
    {
        /* No new data byte was written */
    }

    if( ( crValue != utI3c_CrLast ) &&
        ( UT_I3C_RECORD_CNT > utI3c_CrSeqCnt )    )
    {
        utI3c_CrSeq[ utI3c_CrSeqCnt ] = crValue;
        utI3c_CrSeqCnt++;
        utI3c_CrLast = crValue;
    }
    else
    {
        /* No new control word was written */
    }
}
