/**
 * \author Mr.Nobody
 * \file I3c.h
 * \ingroup I3c
 * \brief Improved Inter-Integrated Circuit (I3C) MCAL module common functionality header file
 *
 * This file contains the common functionality used internally by the module: the runtime
 * context of an instance and the services shared by the module files (module management,
 * controller engine, target engine, DMA handling).
 */

#ifndef I3C_I3C_H
#define I3C_I3C_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "I3c_Types.h"                      /* Module types definition        */
/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Count of the control words of a frame (the C-FIFO has two words) */
#define I3C_CTRL_WORD_CNT_MAX                 ( 2u )

/** Count of the bytes of the payload of one ENTDAA step: 48-bit PID, BCR and DCR */
#define I3C_DAA_PAYLOAD_CNT                   ( 8u )

/* ============================= TYPEDEFS =================================== */

/** \brief Type representing iteration count of busy-wait loops (register read-back) */
typedef uint32_t i3c_TimeoutCnt_t;

/** \brief Raw value of a control word (I3C_CR) */
typedef uint32_t i3c_CtrlWord_t;

/** \brief Raw value of an event / status register */
typedef uint32_t i3c_EventMask_t;

/** \brief Count of the control words of a frame / the index of a control word */
typedef uint8_t i3c_CtrlWordCnt_t;


/** \brief Kind of the running controller request */
typedef enum
{
    I3C_REQ_KIND_NONE = 0u, /**< No request is running                                           */
    I3C_REQ_KIND_FRAME    , /**< Frame of control words and data (private, I2C, CCC)             */
    I3C_REQ_KIND_ENTDAA   , /**< Dynamic address assignment (ENTDAA broadcast CCC)               */
    I3C_REQ_KIND_CNT        /**< Count of the request kinds                                      */
}   i3c_ReqKind_t;


/** \brief Direction of a DMA channel of the module */
typedef enum
{
    I3C_DMA_DIR_TX = 0u, /**< Transmit FIFO (memory to peripheral) */
    I3C_DMA_DIR_RX     , /**< Receive FIFO (peripheral to memory)  */
    I3C_DMA_DIR_CNT      /**< Count of the DMA directions          */
}   i3c_DmaDir_t;


/** \brief Runtime context of the I3C peripheral (one per instance) */
typedef struct
{
    i3c_Config_t                Config;         /**< Copy of the module configuration                      */
    i3c_FunctionState_t         InitState;      /**< The instance is initialized                           */
    i3c_FreqHz_t                KernelClk;      /**< Kernel clock frequency                                */
    i3c_FreqHz_t                PushPullFreq;   /**< Achieved push-pull SCL frequency                      */
    i3c_FreqHz_t                OpenDrainFreq;  /**< Achieved open-drain SCL frequency                     */
    i3c_Device_t                Devices[ I3C_DEVICE_CNT_MAX ]; /**< Device table (controller)              */
    i3c_DeviceCnt_t             DeviceCnt;      /**< Count of the valid entries of the device table        */
    i3c_FunctionState_t         HotJoinState;   /**< Hot-join requests are acknowledged (controller)       */
    i3c_I2cSpeed_t              I2cClass;       /**< Slowest legacy I2C class of the table (CNT: no I2C)    */
    volatile i3c_XferState_t    XferState;      /**< State of the request                                  */
    volatile i3c_XferError_t    XferError;      /**< Errors of the last request                            */
    volatile i3c_ReqKind_t      ReqKind;        /**< Kind of the running controller request                */
    i3c_FunctionState_t         ReqBlocking;    /**< The running request is waited for (no callbacks)      */
    i3c_CtrlWord_t              CtrlWord[ I3C_CTRL_WORD_CNT_MAX ]; /**< Control words of the frame         */
    i3c_CtrlWordCnt_t           CtrlCnt;        /**< Count of the control words of the frame               */
    i3c_CtrlWordCnt_t           CtrlIdx;        /**< Index of the next control word to be written          */
    const i3c_Data_t * volatile TxData;         /**< Data of the running request to be transmitted         */
    volatile i3c_DataCnt_t      TxCnt;          /**< Count of the bytes to be transmitted                  */
    volatile i3c_DataCnt_t      TxIdx;          /**< Index of the next byte to be transmitted              */
    i3c_Data_t * volatile       RxData;         /**< Buffer of the running request for the received data   */
    volatile i3c_DataCnt_t      RxCnt;          /**< Count of the bytes to be received                     */
    volatile i3c_DataCnt_t      RxIdx;          /**< Index of the next received byte                       */
    i3c_Data_t                  DaaPayload[ I3C_DAA_PAYLOAD_CNT ]; /**< PID, BCR and DCR of the assignment */
    volatile i3c_DataCnt_t      DaaIdx;         /**< Count of the payload bytes received                   */
    volatile i3c_DeviceCnt_t    DaaCnt;         /**< Count of the devices assigned by the running ENTDAA   */
    volatile i3c_Address_t      DaaAddress;     /**< Address to be written for the received payload        */
    volatile i3c_FunctionState_t DaaAddressPending; /**< The address of the received payload is to be written */
    i3c_FunctionState_t         DaaOverflow;    /**< A device did not fit in the device table              */
    const i3c_Data_t * volatile TgtReadData;    /**< Buffer provided by I3c_Set_TargetReadBuffer()         */
    volatile i3c_DataCnt_t      TgtReadCnt;     /**< Size of the target read buffer                        */
    volatile i3c_DataCnt_t      TgtReadIdx;     /**< Count of the bytes of the read buffer sent            */
    i3c_Data_t * volatile       TgtWriteData;   /**< Buffer provided by I3c_Set_TargetWriteBuffer()        */
    volatile i3c_DataCnt_t      TgtWriteCnt;    /**< Size of the target write buffer                       */
    volatile i3c_DataCnt_t      TgtWriteIdx;    /**< Count of the bytes stored in the write buffer         */
    volatile i3c_FunctionState_t TgtReadAsked;  /**< The read request callback was called for this read    */
    i3c_TargetConfig_t          TgtConfig;      /**< Copy of the target configuration                      */
    i3c_FunctionState_t         TxDmaInit;      /**< DMA channel of the transmit FIFO is initialized       */
    i3c_FunctionState_t         RxDmaInit;      /**< DMA channel of the receive FIFO is initialized        */
}   i3c_Context_t;

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Busy-wait iteration budget used for register read-back (common for all I3c module files) */
#define I3C_TIMEOUT_RAW                       ( (i3c_TimeoutCnt_t)0x84FCBu )

/** Busy-wait iteration budget of a blocking request (bus bring-up, dynamic address assignment) */
#define I3C_BLOCKING_TIMEOUT_RAW              ( (i3c_TimeoutCnt_t)0x1FFFFFFu )

/** Count of the bytes of the provisioned ID in the ENTDAA payload */
#define I3C_PID_BYTE_CNT                      ( 6u )

/** Depth of the data FIFOs in bytes */
#define I3C_FIFO_DEPTH                        ( 8u )

/** Lowest dynamic address handed out by the address assignment (addresses below are reserved) */
#define I3C_DYNAMIC_ADDRESS_FIRST             ( 0x08u )

/** Mask of a 7-bit address */
#define I3C_ADDRESS_MASK                      ( 0x7Fu )

/** Broadcast address of the bus (reserved byte 0x7E) */
#define I3C_BROADCAST_ADDRESS                 ( 0x7Eu )

/** Bit mask of one byte */
#define I3C_BYTE_MASK                         ( 0xFFu )

/** Count of the bits of a byte (shift of the next byte of a word) */
#define I3C_BYTE_BIT_CNT                      ( 8u )

/** Bit of the CCC code that marks a direct CCC */
#define I3C_CCC_DIRECT_BIT                    ( 0x80u )

/** Message type of a control word: private SDR message */
#define I3C_MTYPE_PRIVATE                     ( 0x2u )

/** Message type of a control word: second part of a direct CCC */
#define I3C_MTYPE_DIRECT                      ( 0x3u )

/** Message type of a control word: legacy I2C message */
#define I3C_MTYPE_I2C                         ( 0x4u )

/** Message type of a control word: broadcast CCC or first part of a direct CCC */
#define I3C_MTYPE_CCC                         ( 0x6u )

/** Message type of a control word of a target: hot-join request */
#define I3C_MTYPE_HOT_JOIN                    ( 0x8u )

/** Message type of a control word of a target: in-band interrupt request */
#define I3C_MTYPE_IBI                         ( 0xAu )

/** Interrupt enable bits of the data FIFOs (served by the interrupt only while data is to be moved) */
#define I3C_IER_FIFO_MASK                     ( I3C_IER_TXFNFIE | I3C_IER_RXFNEIE )

/** Interrupts of the controller role enabled in the ISR and DMA mode (the data FIFOs are enabled per request) */
#define I3C_IER_CTRL_MASK \
    ( I3C_IER_FCIE | I3C_IER_ERRIE | I3C_IER_IBIIE | I3C_IER_HJIE | I3C_IER_RXTGTENDIE )

/** Interrupts of the target role enabled in the ISR and DMA mode (the data FIFOs are enabled per buffer) */
#define I3C_IER_TGT_MASK \
    ( I3C_IER_FCIE     | I3C_IER_ERRIE    | I3C_IER_DAUPDIE | I3C_IER_IBIENDIE | \
      I3C_IER_CRUPDIE  | I3C_IER_HJIE     | I3C_IER_GETIE   | I3C_IER_STAIE    | \
      I3C_IER_MWLUPDIE | I3C_IER_MRLUPDIE | I3C_IER_RSTIE   | I3C_IER_ASUPDIE  | \
      I3C_IER_INTUPDIE | I3C_IER_DEFIE    | I3C_IER_GRPIE )

/** Flush bits of all the FIFOs of the controller */
#define I3C_CFGR_FLUSH_ALL_MASK \
    ( I3C_CFGR_CFLUSH | I3C_CFGR_SFLUSH | I3C_CFGR_TXFLUSH | I3C_CFGR_RXFLUSH )

/** CCC code of the ENTDAA broadcast CCC (not offered in \ref i3c_Ccc_t, it has its own function) */
#define I3C_CCC_ENTDAA_CODE                   ( 0x07u )

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

/* Implemented in I3c.c - common services used by the controller and target engines */
i3c_RequestState_t I3c_Get_Context          ( i3c_PeriphId_t periphId, i3c_Context_t ** const ctx );
i3c_XferError_t    I3c_Get_ErrorMask        ( uint32_t serValue );
void               I3c_Set_XferEnd          ( i3c_PeriphId_t periphId,
                                              i3c_XferState_t xferState,
                                              i3c_XferError_t errorMask,
                                              i3c_DataCnt_t xferCnt );
i3c_RequestState_t I3c_Get_Address          ( i3c_Context_t * const ctx,
                                              i3c_Address_t address,
                                              i3c_Device_t ** const device );
i3c_RequestState_t I3c_Set_DeviceRegs       ( i3c_PeriphId_t periphId );
i3c_RequestState_t I3c_Set_DaaDevice        ( i3c_Context_t * const ctx,
                                              const i3c_Data_t * const payload,
                                              i3c_Address_t * const address );

/* Implemented in I3c_Ctrl.c - controller engine (frames, dynamic address assignment, events) */
i3c_RequestState_t I3cCtrl_Start_Frame      ( i3c_PeriphId_t periphId,
                                              const i3c_CtrlWord_t * const words,
                                              i3c_CtrlWordCnt_t wordCnt,
                                              const i3c_Data_t * const txData,
                                              i3c_DataCnt_t txCnt,
                                              i3c_Data_t * const rxData,
                                              i3c_DataCnt_t rxCnt,
                                              i3c_FunctionState_t blocking );
i3c_RequestState_t I3cCtrl_Start_Daa        ( i3c_PeriphId_t periphId );
i3c_RequestState_t I3cCtrl_Wait_Done        ( i3c_PeriphId_t periphId );
i3c_RequestState_t I3cCtrl_Set_Stop         ( i3c_PeriphId_t periphId );
void               I3cCtrl_Handle_Events    ( i3c_PeriphId_t periphId );

/* Implemented in I3c_Xfer.c - messages, CCCs and target requests (IBI, hot-join) of the controller */
i3c_RequestState_t I3cCtrl_Start_Message    ( i3c_PeriphId_t periphId,
                                              const i3c_Device_t * const device,
                                              const i3c_Data_t * const txData,
                                              i3c_DataCnt_t txCnt,
                                              i3c_Data_t * const rxData,
                                              i3c_DataCnt_t rxCnt );
i3c_RequestState_t I3cCtrl_Start_Ccc        ( i3c_PeriphId_t periphId,
                                              i3c_Ccc_t ccc,
                                              i3c_Address_t address,
                                              i3c_Data_t * const data,
                                              i3c_DataCnt_t dataCnt,
                                              i3c_Direction_t direction );
void               I3cCtrl_Handle_Requests  ( i3c_PeriphId_t periphId, i3c_EventMask_t events );

/* Implemented in I3c_Tgt.c - target engine (the public target functions are defined there as well) */
void               I3cTgt_Handle_Events     ( i3c_PeriphId_t periphId );
i3c_RequestState_t I3cTgt_Set_Stop          ( i3c_PeriphId_t periphId );

/* Implemented in I3c_Dma.c - DMA of the data FIFOs */
i3c_RequestState_t I3cDma_Init              ( i3c_PeriphId_t periphId );
i3c_RequestState_t I3cDma_Deinit            ( i3c_PeriphId_t periphId );
i3c_RequestState_t I3cDma_Start             ( i3c_PeriphId_t periphId,
                                              i3c_DmaDir_t dmaDir,
                                              uint32_t dataAddr,
                                              i3c_DataCnt_t dataCnt );
i3c_RequestState_t I3cDma_Stop              ( i3c_PeriphId_t periphId, i3c_DmaDir_t dmaDir );
i3c_RequestState_t I3cDma_Get_Remaining     ( i3c_PeriphId_t periphId,
                                              i3c_DmaDir_t dmaDir,
                                              i3c_DataCnt_t * const remaining );

#ifdef __cplusplus
}
#endif

#endif /* I3C_I3C_H */
