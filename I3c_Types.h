/**
 * \author Mr.Nobody
 * \file I3c_Types.h
 * \defgroup I3c I3c
 * \brief Improved Inter-Integrated Circuit (I3C) module types definitions
 *
 * This file contains the types definitions used across the module and are
 * available for other modules through the Port file.
 *
 * The I3C peripheral of the STM32H5 devices is a MIPI I3C v1.1 SDR controller / target. The
 * hardware keeps four target device registers only, therefore the device table of the module
 * has \ref I3C_DEVICE_CNT_MAX entries. HDR modes are not supported by the hardware.
 *
 */

#ifndef I3C_I3C_TYPES_H
#define I3C_I3C_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================== INCLUDES ================================== */
#include "stdint.h"                         /* Module types definition        */
#include "Gpio_Types.h"                     /* GPIO types definitions         */
#include "Gpdma_Types.h"                    /* DMA types definitions          */
#include "Rcc_Types.h"                      /* RCC types definitions          */
#include "Stm32.h"                          /* CMSIS device definitions       */
/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Null pointer definition */
#define I3C_NULL_PTR                          ( ( void* ) 0u )

/** Count of the devices of the device table (hardware has four target device registers) */
#define I3C_DEVICE_CNT_MAX                    ( 4u )

/** Maximum count of the data bytes of one request (control word field is 16 bits) */
#define I3C_DATA_CNT_MAX                      ( 65535u )

/** Maximum count of the data bytes of an in-band interrupt payload (mandatory data byte included) */
#define I3C_IBI_PAYLOAD_CNT_MAX               ( 4u )

/** Maximum push-pull SCL frequency of the I3C SDR mode in Hz */
#define I3C_PUSH_PULL_FREQ_MAX                ( 12500000u )

/** Default push-pull SCL frequency of I3c_Get_DefaultConfig() in Hz */
#define I3C_DEFAULT_PUSH_PULL_FREQ            ( 4000000u )

/** Default open-drain SCL frequency of I3c_Get_DefaultConfig() in Hz */
#define I3C_DEFAULT_OPEN_DRAIN_FREQ           ( 1000000u )

/** Default bus free time (tCAS / tBUF) of I3c_Get_DefaultConfig() in ns */
#define I3C_DEFAULT_BUS_FREE_TIME             ( 1300u )

/** Default bus available time (tAVAL) of I3c_Get_DefaultConfig() in ns */
#define I3C_DEFAULT_BUS_AVAILABLE_TIME        ( 1000u )

/** Default own dynamic address of the controller of I3c_Get_DefaultConfig() */
#define I3C_DEFAULT_OWN_ADDRESS               ( 0x08u )

/** Peripheral identification bit offset in encoded pin value */
#define I3C_BIT_MASK_PERIPH_BIT_OFFSET        ( 15u )

/** Port identification bit offset in encoded pin value */
#define I3C_BIT_MASK_PORT_BIT_OFFSET          ( 10u )

/** Pin identification bit offset in encoded pin value */
#define I3C_BIT_MASK_PIN_BIT_OFFSET           ( 5u )

/** Alternative function identification bit offset in encoded pin value */
#define I3C_BIT_MASK_AF_BIT_OFFSET            ( 0u )

/** Bus characteristics (BCR) bit: the device is limited in the maximum data speed (BCR0) */
#define I3C_BCR_SPEED_LIMIT                   ( 0x01u )

/** Bus characteristics (BCR) bit: the device is able to request an in-band interrupt (BCR1); the bit of the register
 *  is reserved, the hardware answers 1 on the bus, the configuration needs it to allow the interrupt request */
#define I3C_BCR_IBI_CAPABLE                   ( 0x02u )

/** Bus characteristics (BCR) bit: a mandatory data byte follows the in-band interrupt (BCR2) */
#define I3C_BCR_IBI_PAYLOAD                   ( 0x04u )

/** Bus characteristics (BCR) bit: the device is controller capable (BCR6) */
#define I3C_BCR_CONTROLLER_CAPABLE            ( 0x40u )

/* ========================== EXPORTED MACROS =============================== */

/** Encode pin configuration (instance, port, pin, alternate function) into a single value */
#define I3C_PIN_ENCODE( PERIPH_ID, PORT_ID, PIN_ID, AF_ID ) \
    ( ( (uint32_t)(PERIPH_ID) << I3C_BIT_MASK_PERIPH_BIT_OFFSET ) | \
      ( (uint32_t)(PORT_ID)   << I3C_BIT_MASK_PORT_BIT_OFFSET   ) | \
      ( (uint32_t)(PIN_ID)    << I3C_BIT_MASK_PIN_BIT_OFFSET    ) | \
      ( (uint32_t)(AF_ID)     << I3C_BIT_MASK_AF_BIT_OFFSET     )   )

/** Extract the instance ID from an encoded pin value */
#define I3C_BIT_MASK_DECODE_PERIPH( CODED_VAL ) \
    ( ( (CODED_VAL) >> I3C_BIT_MASK_PERIPH_BIT_OFFSET ) & 0x0Fu )

/** Extract the port ID from an encoded pin value */
#define I3C_BIT_MASK_DECODE_PORT( CODED_VAL ) \
    ( ( (CODED_VAL) >> I3C_BIT_MASK_PORT_BIT_OFFSET ) & 0x1Fu )

/** Extract the pin ID from an encoded pin value */
#define I3C_BIT_MASK_DECODE_PIN( CODED_VAL ) \
    ( ( (CODED_VAL) >> I3C_BIT_MASK_PIN_BIT_OFFSET ) & 0x1Fu )

/** Extract the alternative function ID from an encoded pin value */
#define I3C_BIT_MASK_DECODE_AF( CODED_VAL ) \
    ( ( (CODED_VAL) >> I3C_BIT_MASK_AF_BIT_OFFSET ) & 0x1Fu )

/* ============================== TYPEDEFS ================================== */

/** \brief Type signaling major version of SW module */
typedef uint8_t i3c_MajorVersion_t;

/** \brief Type signaling minor version of SW module */
typedef uint8_t i3c_MinorVersion_t;

/** \brief Type signaling patch version of SW module */
typedef uint8_t i3c_PatchVersion_t;

/** \brief Type signaling actual version of SW module */
typedef struct
{
    i3c_MajorVersion_t Major; /**< Major version */
    i3c_MinorVersion_t Minor; /**< Minor version */
    i3c_PatchVersion_t Patch; /**< Patch version */
}   i3c_ModuleVersion_t;


/** Function status enumeration */
typedef enum
{
    I3C_FUNCTION_INACTIVE = 0u, /**< Function status is inactive */
    I3C_FUNCTION_ACTIVE         /**< Function status is active   */
}   i3c_FunctionState_t;


/** Flag states enumeration */
typedef enum
{
    I3C_FLAG_INACTIVE = 0u, /**< Inactive flag state */
    I3C_FLAG_ACTIVE         /**< Active flag state   */
}   i3c_FlagState_t;


/** Enumeration used to signal request processing state */
typedef enum
{
    I3C_REQUEST_ERROR = 0u, /**< Processing request failed  */
    I3C_REQUEST_OK          /**< Processing request succeed */
}   i3c_RequestState_t;


/** \brief I3C peripheral (instance) identification */
typedef enum
{
    I3C_PERIPH_1 = 0u, /**< I3C1 */
#if defined(I3C2)
    I3C_PERIPH_2     , /**< I3C2 (not available on STM32H562 / H563 / H573) */
#endif
    I3C_PERIPH_CNT     /**< Count of the I3C instances of the device line */
}   i3c_PeriphId_t;

/* Pin enumerations i3c_SclPin_t and i3c_SdaPin_t (generated) use I3C_PIN_ENCODE and I3C_PERIPH_CNT */
#include "I3c_Pins.h"                       /* I3C pin enumerations           */


/** \brief Role of the I3C peripheral on the bus */
typedef enum
{
    I3C_ROLE_CONTROLLER = 0u, /**< Primary controller of the bus                        */
    I3C_ROLE_TARGET         , /**< Target (a device on the bus of another controller)   */
    I3C_ROLE_CNT              /**< Count of the roles                                   */
}   i3c_Role_t;


/** \brief Mode of the data transfers (the same request and callbacks in all modes) */
typedef enum
{
    I3C_XFER_MODE_POLL = 0u, /**< Polling, the events are served by I3c_Task()      */
    I3C_XFER_MODE_ISR      , /**< Interrupt, the data FIFOs are served by the CPU   */
    I3C_XFER_MODE_DMA      , /**< Interrupt, the data FIFOs are served by the Gpdma */
    I3C_XFER_MODE_CNT        /**< Count of the transfer modes                       */
}   i3c_XferMode_t;


/** \brief Kernel clock source of the I3C instances (value of \ref rcc_PeriphId_t) */
typedef enum
{
    I3C_CLK_SRC_I3C1_PCLK1 = RCC_PERIPH_I3C1_PCLK1, /**< I3C1 kernel clock is the APB1 bus clock  */
#if defined(RCC_CR_PLL3ON)
    I3C_CLK_SRC_I3C1_PLL3R = RCC_PERIPH_I3C1_PLL3R, /**< I3C1 kernel clock is the PLL3 output R   */
#else
    I3C_CLK_SRC_I3C1_PLL2R = RCC_PERIPH_I3C1_PLL2R, /**< I3C1 kernel clock is the PLL2 output R   */
#endif
    I3C_CLK_SRC_I3C1_HSI   = RCC_PERIPH_I3C1_HSI,   /**< I3C1 kernel clock is the HSI oscillator  */
#if defined(I3C2)
    I3C_CLK_SRC_I3C2_PCLK3 = RCC_PERIPH_I3C2_PCLK3, /**< I3C2 kernel clock is the APB3 bus clock  */
#if defined(RCC_CR_PLL3ON)
    I3C_CLK_SRC_I3C2_PLL3R = RCC_PERIPH_I3C2_PLL3R, /**< I3C2 kernel clock is the PLL3 output R   */
#else
    I3C_CLK_SRC_I3C2_PLL2R = RCC_PERIPH_I3C2_PLL2R, /**< I3C2 kernel clock is the PLL2 output R   */
#endif
    I3C_CLK_SRC_I3C2_HSI   = RCC_PERIPH_I3C2_HSI,   /**< I3C2 kernel clock is the HSI oscillator  */
#endif
    I3C_CLK_SRC_CNT                                 /**< Count of the kernel clock sources        */
}   i3c_ClkSrc_t;


/** \brief Type of a device of the device table */
typedef enum
{
    I3C_DEVICE_I3C = 0u, /**< I3C device with a dynamic address               */
    I3C_DEVICE_I2C     , /**< Legacy I2C device with a static 7-bit address   */
    I3C_DEVICE_TYPE_CNT  /**< Count of the device types                       */
}   i3c_DeviceType_t;


/** \brief Speed class of a legacy I2C device (limits the open-drain phase of the bus) */
typedef enum
{
    I3C_I2C_SPEED_SM = 0u, /**< Standard-mode, up to 100 kHz   */
    I3C_I2C_SPEED_FM     , /**< Fast-mode, up to 400 kHz       */
    I3C_I2C_SPEED_FM_PLUS, /**< Fast-mode plus, up to 1 MHz    */
    I3C_I2C_SPEED_CNT      /**< Count of the I2C speed classes */
}   i3c_I2cSpeed_t;


/** \brief Direction of a message */
typedef enum
{
    I3C_DIRECTION_WRITE = 0u, /**< Write (controller to target) */
    I3C_DIRECTION_READ      , /**< Read (target to controller)  */
    I3C_DIRECTION_CNT         /**< Count of the directions      */
}   i3c_Direction_t;


/** \brief State of the request of an instance */
typedef enum
{
    I3C_XFER_STATE_IDLE = 0u, /**< No request is running, the last one succeeded      */
    I3C_XFER_STATE_BUSY     , /**< A request is running                               */
    I3C_XFER_STATE_ERROR    , /**< The last request failed, see I3c_Get_XferError()   */
    I3C_XFER_STATE_CNT        /**< Count of the request states                        */
}   i3c_XferState_t;


/** \brief Errors of a request (bit mask, the items are combined) */
typedef enum
{
    I3C_XFER_ERROR_NONE      = 0x00u, /**< No error                                        */
    I3C_XFER_ERROR_ADDR_NACK = 0x01u, /**< Address not acknowledged                        */
    I3C_XFER_ERROR_DATA_NACK = 0x02u, /**< Data not acknowledged                           */
    I3C_XFER_ERROR_PARITY    = 0x04u, /**< Parity / CRC error of the SDR frame             */
    I3C_XFER_ERROR_OVERRUN   = 0x08u, /**< FIFO overrun (receive FIFO or status FIFO)      */
    I3C_XFER_ERROR_UNDERRUN  = 0x10u, /**< FIFO underrun (transmit FIFO or control FIFO)   */
    I3C_XFER_ERROR_FRAME     = 0x20u, /**< Frame (protocol) error, illegal CCC             */
    I3C_XFER_ERROR_BUS       = 0x40u, /**< Bus error (SCL stalled, controller-role error)  */
    I3C_XFER_ERROR_ABORTED   = 0x80u  /**< Request stopped by I3c_Set_XferStop()           */
}   i3c_XferErrorId_t;

/** Bit mask of \ref i3c_XferErrorId_t items */
typedef uint8_t i3c_XferError_t;


/** \brief Events a target reports about a received CCC (the callback of a CCC) */
typedef enum
{
    I3C_CCC_EVENT_EVENTS_UPDATE = 0u, /**< ENEC / DISEC: the enabled events changed (IBI, hot-join, role request) */
    I3C_CCC_EVENT_ACTIVITY_STATE    , /**< ENTASx: the activity state changed                                      */
    I3C_CCC_EVENT_MAX_WRITE_UPDATE  , /**< SETMWL: the maximum write length changed                                */
    I3C_CCC_EVENT_MAX_READ_UPDATE   , /**< SETMRL: the maximum read length changed                                 */
    I3C_CCC_EVENT_RESET             , /**< RSTACT: reset pattern detected                                          */
    I3C_CCC_EVENT_GET               , /**< Direct GET CCC (GETPID, GETBCR, GETDCR, ...) answered by the hardware   */
    I3C_CCC_EVENT_GET_STATUS        , /**< Direct GETSTATUS answered by the hardware                               */
    I3C_CCC_EVENT_DEFINE_TARGETS    , /**< DEFTGTS received                                                        */
    I3C_CCC_EVENT_DEFINE_GROUP      , /**< DEFGRPA received                                                        */
    I3C_CCC_EVENT_CNT                 /**< Count of the CCC events                                                 */
}   i3c_CccEvent_t;


/** \brief CCC codes of the module (8-bit code on the bus, a code with the bit 7 set is a direct CCC) */
typedef enum
{
    I3C_CCC_ENEC           = 0x00u, /**< Broadcast: enable events                  */
    I3C_CCC_DISEC          = 0x01u, /**< Broadcast: disable events                 */
    I3C_CCC_ENTAS0         = 0x02u, /**< Broadcast: enter activity state 0         */
    I3C_CCC_ENTAS1         = 0x03u, /**< Broadcast: enter activity state 1         */
    I3C_CCC_ENTAS2         = 0x04u, /**< Broadcast: enter activity state 2         */
    I3C_CCC_ENTAS3         = 0x05u, /**< Broadcast: enter activity state 3         */
    I3C_CCC_RSTDAA         = 0x06u, /**< Broadcast: reset dynamic address assignment */
    I3C_CCC_SETMWL         = 0x09u, /**< Broadcast: set maximum write length       */
    I3C_CCC_SETMRL         = 0x0Au, /**< Broadcast: set maximum read length        */
    I3C_CCC_SETAASA        = 0x29u, /**< Broadcast: set all addresses to static    */
    I3C_CCC_RSTACT         = 0x2Au, /**< Broadcast: target reset action            */
    I3C_CCC_ENEC_DIRECT    = 0x80u, /**< Direct: enable events                     */
    I3C_CCC_DISEC_DIRECT   = 0x81u, /**< Direct: disable events                    */
    I3C_CCC_ENTAS0_DIRECT  = 0x82u, /**< Direct: enter activity state 0            */
    I3C_CCC_ENTAS1_DIRECT  = 0x83u, /**< Direct: enter activity state 1            */
    I3C_CCC_ENTAS2_DIRECT  = 0x84u, /**< Direct: enter activity state 2            */
    I3C_CCC_ENTAS3_DIRECT  = 0x85u, /**< Direct: enter activity state 3            */
    I3C_CCC_RSTDAA_DIRECT  = 0x86u, /**< Direct: reset dynamic address assignment  */
    I3C_CCC_SETDASA        = 0x87u, /**< Direct: set dynamic address from static   */
    I3C_CCC_SETNEWDA       = 0x88u, /**< Direct: set new dynamic address           */
    I3C_CCC_SETMWL_DIRECT  = 0x89u, /**< Direct: set maximum write length          */
    I3C_CCC_SETMRL_DIRECT  = 0x8Au, /**< Direct: set maximum read length           */
    I3C_CCC_GETMWL         = 0x8Bu, /**< Direct: get maximum write length          */
    I3C_CCC_GETMRL         = 0x8Cu, /**< Direct: get maximum read length           */
    I3C_CCC_GETPID         = 0x8Du, /**< Direct: get provisioned ID                */
    I3C_CCC_GETBCR         = 0x8Eu, /**< Direct: get bus characteristics           */
    I3C_CCC_GETDCR         = 0x8Fu, /**< Direct: get device characteristics        */
    I3C_CCC_GETSTATUS      = 0x90u, /**< Direct: get status                        */
    I3C_CCC_GETMXDS        = 0x94u, /**< Direct: get maximum data speed            */
    I3C_CCC_GETCAPS        = 0x95u, /**< Direct: get capabilities                  */
    I3C_CCC_RSTACT_DIRECT  = 0x9Au  /**< Direct: target reset action               */
}   i3c_Ccc_t;


/** I3C 7-bit address (static or dynamic) */
typedef uint8_t i3c_Address_t;

/** Type representing transferred data (a byte) */
typedef uint8_t i3c_Data_t;

/** Type representing count of transferred data bytes */
typedef uint16_t i3c_DataCnt_t;

/** Type representing count of the devices */
typedef uint8_t i3c_DeviceCnt_t;

/** Frequency values type represented in Hz */
typedef uint32_t i3c_FreqHz_t;

/** Time values type represented in ns */
typedef uint32_t i3c_TimeNs_t;

/** Interrupt priority type definition */
typedef uint32_t i3c_IrqPrio_t;

/** Bus characteristics register value (BCR) of a device (raw register value) */
typedef uint8_t i3c_Bcr_t;

/** Device characteristics register value (DCR) of a device (raw register value) */
typedef uint8_t i3c_Dcr_t;

/** Upper 16 bits (47:32) of the 48-bit provisioned ID of a device */
typedef uint16_t i3c_PidHigh_t;

/** Lower 32 bits (31:0) of the 48-bit provisioned ID of a device */
typedef uint32_t i3c_PidLow_t;

/** 4-bit MIPI instance ID of the target (bits 15:12 of the provisioned ID) */
typedef uint8_t i3c_InstanceId_t;

/** Maximum read / write length of a target in bytes */
typedef uint16_t i3c_MaxLen_t;

/** Type of the DMA peripheral identification of the data FIFOs */
typedef gpdma_PeriphId_t i3c_DmaPeriphId_t;

/** Type of the DMA channel identification of the data FIFOs */
typedef gpdma_ChannelId_t i3c_DmaChannelId_t;

/** Type of the DMA channel priority of the data FIFOs */
typedef gpdma_Priority_t i3c_DmaPriority_t;


/** \brief Entry of the device table of the controller */
typedef struct
{
    i3c_DeviceType_t    Type;           /**< I3C device or legacy I2C device                              */
    i3c_Address_t       StaticAddress;  /**< Static address (I2C device, I3C device with SETDASA)         */
    i3c_Address_t       DynamicAddress; /**< Dynamic address (I3C device), assigned or requested          */
    i3c_I2cSpeed_t      I2cSpeed;       /**< Speed class (I2C device)                                     */
    i3c_PidHigh_t       PidHigh;        /**< Provisioned ID bits 47:32 (I3C device, read in the discovery) */
    i3c_PidLow_t        PidLow;         /**< Provisioned ID bits 31:0 (I3C device, read in the discovery) */
    i3c_Bcr_t           Bcr;            /**< Bus characteristics (I3C device, read in the discovery)      */
    i3c_Dcr_t           Dcr;            /**< Device characteristics (I3C device, read in the discovery)   */
    i3c_FunctionState_t IbiState;       /**< In-band interrupts of the device are accepted                */
    i3c_FunctionState_t IbiPayload;     /**< The in-band interrupt of the device has a mandatory data byte */
    i3c_FunctionState_t AddressState;   /**< The address of the device is valid (assigned)                */
}   i3c_Device_t;


/** \brief Configuration of the target role (identity, address and limits of the target) */
typedef struct
{
    i3c_Address_t       StaticAddress;    /**< Static 7-bit address (valid if StaticAddressState is active)   */
    i3c_FunctionState_t StaticAddressState; /**< Static address is used (I3C1 of STM32H523 .. H573 only)     */
    i3c_InstanceId_t    InstanceId;       /**< 4-bit instance ID, the rest of the provisioned ID is fixed     */
    i3c_Bcr_t           Bcr;              /**< Bus characteristics (see I3C_BCR_* constants)                  */
    i3c_Dcr_t           Dcr;              /**< Device characteristics                                         */
    i3c_MaxLen_t        MaxWriteLen;      /**< Maximum write length (controller to target) in bytes           */
    i3c_MaxLen_t        MaxReadLen;       /**< Maximum read length (target to controller) in bytes            */
    i3c_DataCnt_t       MaxIbiPayload;    /**< Maximum size of the in-band interrupt payload (0 - 4 bytes)    */
    i3c_FunctionState_t IbiState;         /**< The target is allowed to request an in-band interrupt          */
    i3c_FunctionState_t HotJoinState;     /**< The target is allowed to request a hot-join                    */
}   i3c_TargetConfig_t;


/** Callback: request ended successfully, \p xferCnt data bytes were transferred (received for a read) */
typedef void ( i3c_XferCallback_t )( i3c_PeriphId_t periphId, i3c_DataCnt_t xferCnt );

/** Callback: request failed, \p errorMask is the bit mask of \ref i3c_XferErrorId_t */
typedef void ( i3c_ErrorCallback_t )( i3c_PeriphId_t periphId, i3c_XferError_t errorMask );

/** Callback (controller): in-band interrupt of the device with the dynamic \p address and its payload */
typedef void ( i3c_IbiCallback_t )( i3c_PeriphId_t periphId, i3c_Address_t address,
                                    const i3c_Data_t * const payload, i3c_DataCnt_t payloadCnt );

/** Callback (controller): a target requests the hot-join */
typedef void ( i3c_HotJoinCallback_t )( i3c_PeriphId_t periphId );

/** Callback (target): the controller assigned the dynamic address */
typedef void ( i3c_TargetAddressCallback_t )( i3c_PeriphId_t periphId, i3c_Address_t address );

/** Callback (target): a read of the controller found no data (the hardware has no event for the start of
 *  a read), the application arms the next buffer by I3c_Set_TargetReadBuffer() */
typedef void ( i3c_TargetReadRequestCallback_t )( i3c_PeriphId_t periphId );

/** Callback (target): the read of the controller ended, \p xferCnt bytes were sent */
typedef void ( i3c_TargetReadCallback_t )( i3c_PeriphId_t periphId, i3c_DataCnt_t xferCnt );

/** Callback (target): the write of the controller ended, \p xferCnt bytes were stored in the write buffer */
typedef void ( i3c_TargetWriteCallback_t )( i3c_PeriphId_t periphId, i3c_DataCnt_t xferCnt );

/** Callback (target): a CCC was received (\p event tells which kind) */
typedef void ( i3c_TargetCccCallback_t )( i3c_PeriphId_t periphId, i3c_CccEvent_t event );

/** Callback (target): the in-band interrupt / hot-join / role request of the target ended */
typedef void ( i3c_TargetRequestCallback_t )( i3c_PeriphId_t periphId );


/** \brief I3C module configuration structure (unused callbacks shall be set to I3C_NULL_PTR) */
typedef struct
{
    i3c_PeriphId_t                    PeriphId;            /**< I3C peripheral ID                                    */
    i3c_Role_t                        Role;                /**< Role of the peripheral (controller / target)         */
    i3c_ClkSrc_t                      ClkSrc;              /**< Kernel clock source (of the selected instance)       */
    i3c_FreqHz_t                      PushPullFreq;        /**< Push-pull SCL frequency, up to 12.5 MHz (controller) */
    i3c_FreqHz_t                      OpenDrainFreq;       /**< Open-drain SCL frequency (controller)                */
    i3c_TimeNs_t                      BusFreeTime;         /**< Bus free time tCAS / tBUF (controller)               */
    i3c_TimeNs_t                      BusAvailableTime;    /**< Bus available time tAVAL (target, idle time is 200x) */
    i3c_Address_t                     OwnAddress;          /**< Own dynamic address (controller)                     */
    i3c_FunctionState_t               SdaHighKeeper;       /**< SDA high keeper (controller)                         */
    i3c_XferMode_t                    XferMode;            /**< Transfer mode (POLL / ISR / DMA)                     */
    i3c_DmaPeriphId_t                 TxDmaPeriphId;       /**< DMA peripheral of the transmit FIFO (DMA mode)       */
    i3c_DmaChannelId_t                TxDmaChannelId;      /**< DMA channel of the transmit FIFO (DMA mode)          */
    i3c_DmaPriority_t                 TxDmaPriority;       /**< DMA channel priority of the transmit FIFO (DMA mode) */
    i3c_DmaPeriphId_t                 RxDmaPeriphId;       /**< DMA peripheral of the receive FIFO (DMA mode)        */
    i3c_DmaChannelId_t                RxDmaChannelId;      /**< DMA channel of the receive FIFO (DMA mode)           */
    i3c_DmaPriority_t                 RxDmaPriority;       /**< DMA channel priority of the receive FIFO (DMA mode)  */
    i3c_IrqPrio_t                     IrqPriority;         /**< Interrupt priority (ISR and DMA mode)                */
    i3c_SclPin_t                      SclPin;              /**< SCL pin                                              */
    i3c_SdaPin_t                      SdaPin;              /**< SDA pin                                              */
    i3c_XferCallback_t               *XferCompleteCallback;      /**< Request ended (controller)                      */
    i3c_ErrorCallback_t              *ErrorCallback;             /**< Request failed                                  */
    i3c_IbiCallback_t                *IbiCallback;               /**< In-band interrupt received (controller)         */
    i3c_HotJoinCallback_t            *HotJoinCallback;           /**< Hot-join request received (controller)          */
    i3c_TargetAddressCallback_t      *TargetAddressCallback;     /**< Dynamic address assigned (target)               */
    i3c_TargetReadRequestCallback_t  *TargetReadRequestCallback; /**< Read found no data (target)                     */
    i3c_TargetReadCallback_t         *TargetReadCallback;        /**< Read of the controller ended (target)           */
    i3c_TargetWriteCallback_t        *TargetWriteCallback;       /**< Write of the controller ended (target)          */
    i3c_TargetCccCallback_t          *TargetCccCallback;         /**< CCC received (target)                           */
    i3c_TargetRequestCallback_t      *TargetRequestCallback;     /**< IBI / hot-join / role request ended (target)    */
}   i3c_Config_t;


/* ========================== EXPORTED VARIABLES ============================ */

/* ========================= EXPORTED FUNCTIONS ============================= */

#ifdef __cplusplus
}
#endif

#endif /* I3C_I3C_TYPES_H */
