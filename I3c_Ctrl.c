/**
 * \author Mr.Nobody
 * \file I3c_Ctrl.c
 * \ingroup I3c
 * \brief I3c module controller engine
 *
 * The engine of the controller role: it starts a frame (up to two control words and the data of
 * the transmit and the receive FIFO), starts the dynamic address assignment (ENTDAA) and serves
 * the events of the peripheral in the three transfer modes. The same event handler serves the
 * POLL mode (called by I3c_Task()) and the ISR / DMA mode (called by the interrupt). In the DMA
 * mode the data FIFOs are served by the Gpdma, the control words are always written by the CPU
 * (a frame has two control words at most, the C-FIFO has two words) and the status FIFO is not
 * used (SMODE = 0), the status register is read after the frame.
 *
 * Device behaviour the engine respects (RM0481 54.7, 54.9.3, 54.10):
 * - The control words are written before the data (TMODE = 1, the peripheral preloads the C-FIFO
 *   and the TX-FIFO before it starts the frame).
 * - ENTDAA: for every target the RX-FIFO receives eight bytes (PID, BCR, DCR), then the TX-FIFO
 *   asks for the 7-bit dynamic address; the frame ends when no target answers any more.
 * - A private read ended by the target early sets RXTGTENDF, the number of received bytes is
 *   I3C_SR.XDCNT, the flag must be cleared by the software.
 * - On a transfer error (ERRF) the hardware flushes the C-FIFO and the TX-FIFO, the module
 *   flushes the RX-FIFO and stops the DMA channels.
 *
 */
/* ============================== INCLUDES ================================== */
#include "I3c.h"                            /* Module private interface       */
#include "I3c_Port.h"                       /* Module public interface        */
#include "I3c_Reg.h"                        /* I3C register access            */
/* ============================== TYPEDEFS ================================== */

/* ======================== FORWARD DECLARATIONS ============================ */

static void I3cPriv_Set_FifoIrq    ( const i3c_Context_t * const ctx,
                                     i3c_PeriphId_t periphId,
                                     i3c_EventMask_t irqMask,
                                     i3c_FunctionState_t irqState );
static void I3cPriv_Fill_Tx        ( i3c_Context_t * const ctx, i3c_PeriphId_t periphId );
static void I3cPriv_Drain_Rx       ( i3c_Context_t * const ctx, i3c_PeriphId_t periphId );
static void I3cPriv_Serve_Frame    ( i3c_Context_t * const ctx, i3c_PeriphId_t periphId, i3c_EventMask_t events );
static void I3cPriv_Serve_Daa      ( i3c_Context_t * const ctx, i3c_PeriphId_t periphId, i3c_EventMask_t events );
static void I3cPriv_Drain_Daa      ( i3c_Context_t * const ctx, i3c_PeriphId_t periphId );
static void I3cPriv_Handle_Error   ( i3c_Context_t * const ctx, i3c_PeriphId_t periphId );
static void I3cPriv_Handle_Complete( i3c_Context_t * const ctx, i3c_PeriphId_t periphId );
static void I3cPriv_Handle_EarlyEnd( i3c_Context_t * const ctx, i3c_PeriphId_t periphId );
static void I3cPriv_Stop_Dma       ( const i3c_Context_t * const ctx, i3c_PeriphId_t periphId );
static i3c_RequestState_t I3cPriv_Check_Data ( const i3c_Data_t * const txData,
                                               i3c_DataCnt_t txCnt,
                                               const i3c_Data_t * const rxData,
                                               i3c_DataCnt_t rxCnt );

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** \brief Bits of the event register cleared at the start of a request */
#define I3C_EVENTS_REQUEST_MASK               ( I3C_EVR_FCF | I3C_EVR_ERRF | I3C_EVR_RXTGTENDF )

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Starts a frame of one or two control words with the data of the transmit and the
 *        receive FIFO.
 *
 * The transmit data belongs to the first message, the receive data to the last one. The
 * function returns at once, the end of the frame is reported by I3c_Set_XferEnd().
 *
 * \param periphId [in]: I3C peripheral ID
 * \param words    [in]: Control words of the frame
 * \param wordCnt  [in]: Count of the control words (1 or 2)
 * \param txData   [in]: Data to be transmitted (NULL if txCnt is zero)
 * \param txCnt    [in]: Count of the bytes to be transmitted
 * \param rxData   [in]: Buffer for the received data (NULL if rxCnt is zero)
 * \param rxCnt    [in]: Count of the bytes to be received
 * \param blocking [in]: The caller waits for the end, no callback is called
 * \return I3C_REQUEST_OK when the frame is started, I3C_REQUEST_ERROR otherwise (no state change)
 */
i3c_RequestState_t I3cCtrl_Start_Frame( i3c_PeriphId_t periphId,
                                        const i3c_CtrlWord_t * const words,
                                        i3c_CtrlWordCnt_t wordCnt,
                                        const i3c_Data_t * const txData,
                                        i3c_DataCnt_t txCnt,
                                        i3c_Data_t * const rxData,
                                        i3c_DataCnt_t rxCnt,
                                        i3c_FunctionState_t blocking )
{
    i3c_RequestState_t retState  = I3C_REQUEST_ERROR;
    i3c_Context_t *    ctx       = I3C_NULL_PTR;
    i3c_RequestState_t ctxState  = I3c_Get_Context( periphId, &ctx );
    i3c_RequestState_t dataState = I3cPriv_Check_Data( txData, txCnt, rxData, rxCnt );

    if( ( I3C_REQUEST_OK        == ctxState  ) &&
        ( I3C_REQUEST_OK        == dataState ) &&
        ( I3C_NULL_PTR          != ctx       ) &&
        ( I3C_NULL_PTR          != words     ) &&
        ( 0u                    <  wordCnt   ) &&
        ( I3C_CTRL_WORD_CNT_MAX >= wordCnt   )    )
    {
        if( ( I3C_FUNCTION_ACTIVE   == ctx->InitState        ) &&
            ( I3C_ROLE_CONTROLLER   == ctx->Config.Role      ) &&
            ( I3C_XFER_STATE_BUSY   != ctx->XferState        ) &&
            ( I3C_REQ_KIND_NONE     == ctx->ReqKind          )    )
        {
            ctx->CtrlWord[ 0u ]   = words[ 0u ];
            ctx->CtrlWord[ 1u ]   = words[ wordCnt - 1u ];
            ctx->CtrlCnt          = wordCnt;
            ctx->CtrlIdx          = 0u;
            ctx->TxData           = txData;
            ctx->TxCnt            = txCnt;
            ctx->TxIdx            = 0u;
            ctx->RxData           = rxData;
            ctx->RxCnt            = rxCnt;
            ctx->RxIdx            = 0u;
            ctx->ReqBlocking      = blocking;
            ctx->XferError        = (i3c_XferError_t)I3C_XFER_ERROR_NONE;
            ctx->DaaAddressPending = I3C_FUNCTION_INACTIVE;

            I3c_Reg_Set_EventsClear( periphId, I3C_EVENTS_REQUEST_MASK );
            retState = I3C_REQUEST_OK;

            if( I3C_XFER_MODE_DMA == ctx->Config.XferMode )
            {
                if( 0u < txCnt )
                {
                    retState = I3cDma_Start( periphId, I3C_DMA_DIR_TX, (uint32_t)txData, txCnt );
                }
                else
                {
                    /* No data to be transmitted, no action required */
                }

                if( ( I3C_REQUEST_OK == retState ) &&
                    ( 0u < rxCnt )    )
                {
                    retState = I3cDma_Start( periphId, I3C_DMA_DIR_RX, (uint32_t)rxData, rxCnt );
                }
                else
                {
                    /* No data to be received or the transmit channel failed, no action required */
                }
            }
            else
            {
                /* The data FIFOs are served by the CPU, no action required */
            }

            if( I3C_REQUEST_OK == retState )
            {
                ctx->ReqKind   = I3C_REQ_KIND_FRAME;
                ctx->XferState = I3C_XFER_STATE_BUSY;

                if( ( I3C_XFER_MODE_ISR == ctx->Config.XferMode ) &&
                    ( 0u < txCnt )    )
                {
                    I3cPriv_Set_FifoIrq( ctx, periphId, I3C_IER_TXFNFIE, I3C_FUNCTION_ACTIVE );
                }
                else
                {
                    /* No data to be transmitted by the interrupt, no action required */
                }

                if( ( I3C_XFER_MODE_ISR == ctx->Config.XferMode ) &&
                    ( 0u < rxCnt )    )
                {
                    I3cPriv_Set_FifoIrq( ctx, periphId, I3C_IER_RXFNEIE, I3C_FUNCTION_ACTIVE );
                }
                else
                {
                    /* No data to be received by the interrupt, no action required */
                }

                for( i3c_CtrlWordCnt_t idx = 0u; wordCnt > idx; idx++ )
                {
                    I3c_Reg_Set_ControlWord( periphId, words[ idx ] );
                }

                if( I3C_XFER_MODE_DMA != ctx->Config.XferMode )
                {
                    I3cPriv_Fill_Tx( ctx, periphId );
                }
                else
                {
                    /* The transmit FIFO is served by the DMA, no action required */
                }
            }
            else
            {
                I3cPriv_Stop_Dma( ctx, periphId );
                retState = I3C_REQUEST_ERROR;
            }
        }
        else
        {
            /* The module is not initialized as controller or a request is running */
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
 * \brief Starts the dynamic address assignment (ENTDAA broadcast CCC).
 *
 * The devices that answer are stored in the device table by the event handler. The request is
 * always waited for by the caller (see I3cCtrl_Wait_Done()), no callback is called.
 *
 * \param periphId [in]: I3C peripheral ID
 * \return I3C_REQUEST_OK when the assignment is started, I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3cCtrl_Start_Daa( i3c_PeriphId_t periphId )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    i3c_Context_t *    ctx      = I3C_NULL_PTR;
    i3c_RequestState_t ctxState = I3c_Get_Context( periphId, &ctx );

    if( ( I3C_REQUEST_OK == ctxState ) &&
        ( I3C_NULL_PTR   != ctx      )    )
    {
        if( ( I3C_FUNCTION_ACTIVE   == ctx->InitState        ) &&
            ( I3C_ROLE_CONTROLLER   == ctx->Config.Role      ) &&
            ( I3C_XFER_STATE_BUSY   != ctx->XferState        ) &&
            ( I3C_REQ_KIND_NONE     == ctx->ReqKind          )    )
        {
            ctx->CtrlWord[ 0u ]    = ( ( (i3c_CtrlWord_t)1u              << I3C_CR_MEND_Pos  ) |
                                       ( (i3c_CtrlWord_t)I3C_MTYPE_CCC   << I3C_CR_MTYPE_Pos ) |
                                       ( (i3c_CtrlWord_t)I3C_CCC_ENTDAA_CODE << I3C_CR_CCC_Pos ) );
            ctx->CtrlCnt           = 1u;
            ctx->CtrlIdx           = 0u;
            ctx->TxData            = I3C_NULL_PTR;
            ctx->TxCnt             = 0u;
            ctx->TxIdx             = 0u;
            ctx->RxData            = I3C_NULL_PTR;
            ctx->RxCnt             = 0u;
            ctx->RxIdx             = 0u;
            ctx->DaaIdx            = 0u;
            ctx->DaaCnt            = 0u;
            ctx->DaaAddress        = 0u;
            ctx->DaaAddressPending = I3C_FUNCTION_INACTIVE;
            ctx->DaaOverflow       = I3C_FUNCTION_INACTIVE;
            ctx->ReqBlocking       = I3C_FUNCTION_ACTIVE;
            ctx->XferError         = (i3c_XferError_t)I3C_XFER_ERROR_NONE;
            ctx->ReqKind           = I3C_REQ_KIND_ENTDAA;
            ctx->XferState         = I3C_XFER_STATE_BUSY;

            I3c_Reg_Set_EventsClear( periphId, I3C_EVENTS_REQUEST_MASK );

            if( I3C_XFER_MODE_ISR == ctx->Config.XferMode )
            {
                I3cPriv_Set_FifoIrq( ctx, periphId, I3C_IER_RXFNEIE, I3C_FUNCTION_ACTIVE );
            }
            else
            {
                /* The events are polled, no action required */
            }

            I3c_Reg_Set_ControlWord( periphId, ctx->CtrlWord[ 0u ] );
            retState = I3C_REQUEST_OK;
        }
        else
        {
            /* The module is not initialized as controller or a request is running */
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
 * \brief Waits (bounded loop) until the running request ends. In the POLL mode the events are
 *        served by the loop itself, in the ISR / DMA mode by the interrupt.
 *
 * A request that does not end in the time limit is stopped.
 *
 * \param periphId [in]: I3C peripheral ID
 * \return I3C_REQUEST_OK when the request ended without an error, I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3cCtrl_Wait_Done( i3c_PeriphId_t periphId )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    i3c_Context_t *    ctx      = I3C_NULL_PTR;
    i3c_RequestState_t ctxState = I3c_Get_Context( periphId, &ctx );

    if( ( I3C_REQUEST_OK == ctxState ) &&
        ( I3C_NULL_PTR   != ctx      )    )
    {
        for( i3c_TimeoutCnt_t idx = 0u;
             ( I3C_BLOCKING_TIMEOUT_RAW > idx ) &&
             ( I3C_XFER_STATE_BUSY == ctx->XferState );
             idx++ )
        {
            if( I3C_XFER_MODE_POLL == ctx->Config.XferMode )
            {
                I3cCtrl_Handle_Events( periphId );
            }
            else
            {
                /* The events are served by the interrupt, no action required */
            }
        }

        if( I3C_XFER_STATE_IDLE == ctx->XferState )
        {
            retState = I3C_REQUEST_OK;
        }
        else
        {
            if( I3C_XFER_STATE_BUSY == ctx->XferState )
            {
                (void)I3cCtrl_Set_Stop( periphId );
            }
            else
            {
                /* The request failed, the error is stored, no action required */
            }

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
 * \brief Stops the running request: the peripheral is disabled, the FIFOs are flushed, the DMA
 *        channels are stopped and the peripheral is enabled again (the configuration is kept).
 *
 * \param periphId [in]: I3C peripheral ID
 * \return I3C_REQUEST_OK when the peripheral is enabled again, I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3cCtrl_Set_Stop( i3c_PeriphId_t periphId )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    i3c_Context_t *    ctx      = I3C_NULL_PTR;
    i3c_RequestState_t ctxState = I3c_Get_Context( periphId, &ctx );

    if( ( I3C_REQUEST_OK == ctxState ) &&
        ( I3C_NULL_PTR   != ctx      )    )
    {
        if( ( I3C_FUNCTION_ACTIVE == ctx->InitState   ) &&
            ( I3C_ROLE_CONTROLLER == ctx->Config.Role )    )
        {
            I3cPriv_Stop_Dma( ctx, periphId );
            I3cPriv_Set_FifoIrq( ctx, periphId, I3C_IER_FIFO_MASK, I3C_FUNCTION_INACTIVE );
            retState = I3c_Reg_Set_Disable( periphId );
            I3c_Reg_Set_FifoFlush( periphId, I3C_CFGR_FLUSH_ALL_MASK );
            I3c_Reg_Set_EventsClear( periphId, I3C_REG_MASK_ALL );

            if( I3C_REQUEST_OK == retState )
            {
                retState = I3c_Reg_Set_Enable( periphId );
            }
            else
            {
                /* The peripheral did not stop, no action required */
            }

            if( I3C_XFER_STATE_BUSY == ctx->XferState )
            {
                I3c_Set_XferEnd( periphId, I3C_XFER_STATE_ERROR, (i3c_XferError_t)I3C_XFER_ERROR_ABORTED, 0u );
            }
            else
            {
                /* No request is running, no action required */
            }
        }
        else
        {
            /* The module is not initialized as controller */
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
 * \brief Serves the events of an instance initialized as controller: the data FIFOs, the end of
 *        a frame, the errors and the requests of the targets (IBI, hot-join). Called by I3c_Task()
 *        in the POLL mode and by the interrupt in the ISR and DMA mode.
 *
 * \param periphId [in]: I3C peripheral ID
 */
void I3cCtrl_Handle_Events( i3c_PeriphId_t periphId )
{
    i3c_Context_t *    ctx      = I3C_NULL_PTR;
    i3c_RequestState_t ctxState = I3c_Get_Context( periphId, &ctx );
    i3c_EventMask_t    events   = 0u;

    if( ( I3C_REQUEST_OK == ctxState ) &&
        ( I3C_NULL_PTR   != ctx      )    )
    {
        if( ( I3C_FUNCTION_ACTIVE == ctx->InitState   ) &&
            ( I3C_ROLE_CONTROLLER == ctx->Config.Role )    )
        {
            events = I3c_Reg_Get_Events( periphId );

            if( 0u != ( events & I3C_EVR_ERRF ) )
            {
                I3cPriv_Handle_Error( ctx, periphId );
            }
            else
            {
                if( I3C_REQ_KIND_ENTDAA == ctx->ReqKind )
                {
                    I3cPriv_Serve_Daa( ctx, periphId, events );
                }
                else
                {
                    if( I3C_REQ_KIND_FRAME == ctx->ReqKind )
                    {
                        I3cPriv_Serve_Frame( ctx, periphId, events );
                    }
                    else
                    {
                        /* No request is running, no action required */
                    }
                }
            }

            I3cCtrl_Handle_Requests( periphId, events );
        }
        else
        {
            /* The module is not initialized as controller, no action required */
        }
    }
    else
    {
        /* Invalid peripheral ID, no action required */
    }
}


/* ============================ LOCAL FUNCTIONS ============================== */

/**
 * \brief Enables or disables interrupt enable bits of the data FIFOs.
 *
 * \param ctx      [in]: Context of the instance
 * \param periphId [in]: I3C peripheral ID
 * \param irqMask  [in]: Mask of the interrupt enable bits (TXFNFIE, RXFNEIE)
 * \param irqState [in]: Interrupts are enabled / disabled
 */
static void I3cPriv_Set_FifoIrq( const i3c_Context_t * const ctx,
                                 i3c_PeriphId_t periphId,
                                 i3c_EventMask_t irqMask,
                                 i3c_FunctionState_t irqState )
{
    if( I3C_XFER_MODE_ISR == ctx->Config.XferMode )
    {
        if( I3C_FUNCTION_ACTIVE == irqState )
        {
            I3c_Reg_Set_Bit( periphId, I3C_REG_IER, irqMask );
        }
        else
        {
            I3c_Reg_Reset_Bit( periphId, I3C_REG_IER, irqMask );
        }
    }
    else
    {
        /* The FIFOs are not served by the interrupt, no action required */
    }
}


/**
 * \brief Writes the data of the request into the transmit FIFO while it has room (bounded by the
 *        depth of the FIFO per call), the interrupt of the FIFO is disabled when all data is sent.
 *
 * \param ctx      [in]: Context of the instance
 * \param periphId [in]: I3C peripheral ID
 */
static void I3cPriv_Fill_Tx( i3c_Context_t * const ctx, i3c_PeriphId_t periphId )
{
    i3c_EventMask_t events = I3c_Reg_Get_Events( periphId );

    for( uint32_t idx = 0u;
         ( I3C_FIFO_DEPTH > idx ) &&
         ( ctx->TxCnt > ctx->TxIdx ) &&
         ( 0u != ( events & I3C_EVR_TXFNFF ) );
         idx++ )
    {
        I3c_Reg_Set_TxByte( periphId, ctx->TxData[ ctx->TxIdx ] );
        ctx->TxIdx++;
        events = I3c_Reg_Get_Events( periphId );
    }

    if( ctx->TxCnt <= ctx->TxIdx )
    {
        I3cPriv_Set_FifoIrq( ctx, periphId, I3C_IER_TXFNFIE, I3C_FUNCTION_INACTIVE );
    }
    else
    {
        /* Data is left to be transmitted, no action required */
    }
}


/**
 * \brief Reads the received data from the receive FIFO while it holds data (bounded by the depth
 *        of the FIFO per call), the interrupt of the FIFO is disabled when all data is received.
 *
 * \param ctx      [in]: Context of the instance
 * \param periphId [in]: I3C peripheral ID
 */
static void I3cPriv_Drain_Rx( i3c_Context_t * const ctx, i3c_PeriphId_t periphId )
{
    i3c_EventMask_t events = I3c_Reg_Get_Events( periphId );

    for( uint32_t idx = 0u;
         ( I3C_FIFO_DEPTH > idx ) &&
         ( ctx->RxCnt > ctx->RxIdx ) &&
         ( 0u != ( events & I3C_EVR_RXFNEF ) );
         idx++ )
    {
        ctx->RxData[ ctx->RxIdx ] = (i3c_Data_t)I3c_Reg_Get_RxByte( periphId );
        ctx->RxIdx++;
        events = I3c_Reg_Get_Events( periphId );
    }

    if( ctx->RxCnt <= ctx->RxIdx )
    {
        I3cPriv_Set_FifoIrq( ctx, periphId, I3C_IER_RXFNEIE, I3C_FUNCTION_INACTIVE );
    }
    else
    {
        /* Data is left to be received, no action required */
    }
}


/**
 * \brief Serves the events of a running frame.
 *
 * \param ctx      [in]: Context of the instance
 * \param periphId [in]: I3C peripheral ID
 * \param events   [in]: Event register value
 */
static void I3cPriv_Serve_Frame( i3c_Context_t * const ctx, i3c_PeriphId_t periphId, i3c_EventMask_t events )
{
    if( 0u != ( events & I3C_EVR_RXTGTENDF ) )
    {
        I3cPriv_Handle_EarlyEnd( ctx, periphId );
    }
    else
    {
        /* No early end of a read, no action required (the count of the read is limited before the FIFO is drained) */
    }

    if( I3C_XFER_MODE_DMA != ctx->Config.XferMode )
    {
        if( 0u != ( events & I3C_EVR_TXFNFF ) )
        {
            I3cPriv_Fill_Tx( ctx, periphId );
        }
        else
        {
            /* The transmit FIFO is full, no action required */
        }

        if( 0u != ( events & I3C_EVR_RXFNEF ) )
        {
            I3cPriv_Drain_Rx( ctx, periphId );
        }
        else
        {
            /* The receive FIFO is empty, no action required */
        }
    }
    else
    {
        /* The data FIFOs are served by the DMA, no action required */
    }

    if( 0u != ( events & I3C_EVR_FCF ) )
    {
        I3cPriv_Handle_Complete( ctx, periphId );
    }
    else
    {
        /* The frame is not completed, no action required */
    }
}


/**
 * \brief Serves the events of a running dynamic address assignment.
 *
 * The eight bytes of a device (PID, BCR, DCR) are received, the device is stored in the device
 * table and its 7-bit dynamic address is written when the transmit FIFO asks for it.
 *
 * \param ctx      [in]: Context of the instance
 * \param periphId [in]: I3C peripheral ID
 * \param events   [in]: Event register value
 */
static void I3cPriv_Serve_Daa( i3c_Context_t * const ctx, i3c_PeriphId_t periphId, i3c_EventMask_t events )
{
    if( 0u != ( events & I3C_EVR_RXFNEF ) )
    {
        I3cPriv_Drain_Daa( ctx, periphId );
    }
    else
    {
        /* No payload byte received, no action required */
    }

    if( ( 0u != ( events & I3C_EVR_TXFNFF )             ) &&
        ( I3C_FUNCTION_ACTIVE == ctx->DaaAddressPending )    )
    {
        I3c_Reg_Set_TxByte( periphId, (uint8_t)ctx->DaaAddress );
        ctx->DaaAddressPending = I3C_FUNCTION_INACTIVE;
        ctx->DaaIdx            = 0u;
        ctx->DaaCnt++;
        I3cPriv_Set_FifoIrq( ctx, periphId, I3C_IER_TXFNFIE, I3C_FUNCTION_INACTIVE );
        I3cPriv_Set_FifoIrq( ctx, periphId, I3C_IER_RXFNEIE, I3C_FUNCTION_ACTIVE );
    }
    else
    {
        /* No address is requested, no action required */
    }

    if( 0u != ( events & I3C_EVR_FCF ) )
    {
        I3c_Reg_Set_EventsClear( periphId, I3C_EVR_FCF );
        I3c_Set_XferEnd( periphId,
                         I3C_XFER_STATE_IDLE,
                         (i3c_XferError_t)I3C_XFER_ERROR_NONE,
                         (i3c_DataCnt_t)ctx->DaaCnt );
    }
    else
    {
        /* The assignment is not completed, no action required */
    }
}


/**
 * \brief Reads the payload bytes of a device of the dynamic address assignment. When the payload
 *        is complete, the device is stored and its address is prepared for the transmit FIFO.
 *
 * \param ctx      [in]: Context of the instance
 * \param periphId [in]: I3C peripheral ID
 */
static void I3cPriv_Drain_Daa( i3c_Context_t * const ctx, i3c_PeriphId_t periphId )
{
    i3c_EventMask_t    events   = I3c_Reg_Get_Events( periphId );
    i3c_Address_t      address  = 0u;
    i3c_RequestState_t devState = I3C_REQUEST_ERROR;

    for( uint32_t idx = 0u;
         ( I3C_FIFO_DEPTH > idx ) &&
         ( I3C_DAA_PAYLOAD_CNT > ctx->DaaIdx ) &&
         ( 0u != ( events & I3C_EVR_RXFNEF ) );
         idx++ )
    {
        ctx->DaaPayload[ ctx->DaaIdx ] = (i3c_Data_t)I3c_Reg_Get_RxByte( periphId );
        ctx->DaaIdx++;
        events = I3c_Reg_Get_Events( periphId );
    }

    if( ( I3C_DAA_PAYLOAD_CNT <= ctx->DaaIdx                ) &&
        ( I3C_FUNCTION_INACTIVE == ctx->DaaAddressPending   )    )
    {
        devState = I3c_Set_DaaDevice( ctx, ctx->DaaPayload, &address );

        if( I3C_REQUEST_OK != devState )
        {
            ctx->DaaOverflow = I3C_FUNCTION_ACTIVE;
        }
        else
        {
            /* The device is stored in the device table, no action required */
        }

        ctx->DaaAddress        = address;
        ctx->DaaAddressPending = I3C_FUNCTION_ACTIVE;
        I3cPriv_Set_FifoIrq( ctx, periphId, I3C_IER_RXFNEIE, I3C_FUNCTION_INACTIVE );
        I3cPriv_Set_FifoIrq( ctx, periphId, I3C_IER_TXFNFIE, I3C_FUNCTION_ACTIVE );
    }
    else
    {
        /* The payload is not complete, no action required */
    }
}


/**
 * \brief Handles a transfer error: the error bits are decoded, the FIFOs are flushed, the DMA
 *        channels are stopped and the request ends with an error.
 *
 * \param ctx      [in]: Context of the instance
 * \param periphId [in]: I3C peripheral ID
 */
static void I3cPriv_Handle_Error( i3c_Context_t * const ctx, i3c_PeriphId_t periphId )
{
    const uint32_t        serValue  = I3c_Reg_Get_Val( periphId, I3C_REG_SER, I3C_REG_MASK_ALL );
    const i3c_XferError_t errorMask = I3c_Get_ErrorMask( serValue );

    I3cPriv_Stop_Dma( ctx, periphId );
    I3cPriv_Set_FifoIrq( ctx, periphId, I3C_IER_FIFO_MASK, I3C_FUNCTION_INACTIVE );
    I3c_Reg_Set_FifoFlush( periphId, I3C_CFGR_FLUSH_ALL_MASK );
    I3c_Reg_Set_EventsClear( periphId, I3C_EVR_ERRF );
    I3c_Set_XferEnd( periphId, I3C_XFER_STATE_ERROR, errorMask, 0u );
}


/**
 * \brief Handles the end of a frame: the receive FIFO is drained, the DMA transfer counts are
 *        completed and the request ends.
 *
 * \param ctx      [in]: Context of the instance
 * \param periphId [in]: I3C peripheral ID
 */
static void I3cPriv_Handle_Complete( i3c_Context_t * const ctx, i3c_PeriphId_t periphId )
{
    i3c_DataCnt_t xferCnt = 0u;

    if( I3C_XFER_MODE_DMA == ctx->Config.XferMode )
    {
        ctx->TxIdx = ctx->TxCnt;
        ctx->RxIdx = ctx->RxCnt;
        I3cPriv_Stop_Dma( ctx, periphId );
    }
    else
    {
        I3cPriv_Drain_Rx( ctx, periphId );
    }

    I3c_Reg_Set_EventsClear( periphId, I3C_EVR_FCF );

    if( 0u < ctx->RxCnt )
    {
        xferCnt = ctx->RxIdx;
    }
    else
    {
        xferCnt = ctx->TxIdx;
    }

    I3cPriv_Set_FifoIrq( ctx, periphId, I3C_IER_FIFO_MASK, I3C_FUNCTION_INACTIVE );
    I3c_Set_XferEnd( periphId, I3C_XFER_STATE_IDLE, (i3c_XferError_t)I3C_XFER_ERROR_NONE, xferCnt );
}


/**
 * \brief Handles a read ended early by the target: the number of received bytes is taken from the
 *        status register and the event flag is cleared (the bus waits for it).
 *
 * \param ctx      [in]: Context of the instance
 * \param periphId [in]: I3C peripheral ID
 */
static void I3cPriv_Handle_EarlyEnd( i3c_Context_t * const ctx, i3c_PeriphId_t periphId )
{
    const i3c_DataCnt_t rxCnt = (i3c_DataCnt_t)( I3c_Reg_Get_Val( periphId, I3C_REG_SR, I3C_SR_XDCNT_Msk ) >>
                                                 I3C_SR_XDCNT_Pos );

    I3c_Reg_Set_EventsClear( periphId, I3C_EVR_RXTGTENDF );

    if( ctx->RxCnt > rxCnt )
    {
        ctx->RxCnt = rxCnt;
    }
    else
    {
        /* The target sent all the data requested, no action required */
    }

    if( I3C_XFER_MODE_DMA == ctx->Config.XferMode )
    {
        (void)I3cDma_Stop( periphId, I3C_DMA_DIR_RX );
        ctx->RxIdx = ctx->RxCnt;
    }
    else
    {
        /* The receive FIFO is drained by the next events, no action required */
    }
}


/**
 * \brief Stops the DMA channels of the instance (DMA mode only).
 *
 * \param ctx      [in]: Context of the instance
 * \param periphId [in]: I3C peripheral ID
 */
static void I3cPriv_Stop_Dma( const i3c_Context_t * const ctx, i3c_PeriphId_t periphId )
{
    if( I3C_XFER_MODE_DMA == ctx->Config.XferMode )
    {
        (void)I3cDma_Stop( periphId, I3C_DMA_DIR_TX );
        (void)I3cDma_Stop( periphId, I3C_DMA_DIR_RX );
    }
    else
    {
        /* No DMA channel is used, no action required */
    }
}


/**
 * \brief Checks that a buffer is provided for every non-empty data direction of a frame.
 *
 * \param txData [in]: Data to be transmitted
 * \param txCnt  [in]: Count of the bytes to be transmitted
 * \param rxData [in]: Buffer for the received data
 * \param rxCnt  [in]: Count of the bytes to be received
 * \return I3C_REQUEST_OK when the buffers match the counts, I3C_REQUEST_ERROR otherwise
 */
static i3c_RequestState_t I3cPriv_Check_Data( const i3c_Data_t * const txData,
                                              i3c_DataCnt_t txCnt,
                                              const i3c_Data_t * const rxData,
                                              i3c_DataCnt_t rxCnt )
{
    i3c_RequestState_t retState = I3C_REQUEST_OK;

    if( ( 0u < txCnt          ) &&
        ( I3C_NULL_PTR == txData )    )
    {
        retState = I3C_REQUEST_ERROR;
    }
    else
    {
        /* The transmit buffer matches its count, no action required */
    }

    if( ( 0u < rxCnt          ) &&
        ( I3C_NULL_PTR == rxData )    )
    {
        retState = I3C_REQUEST_ERROR;
    }
    else
    {
        /* The receive buffer matches its count, no action required */
    }

    return ( retState );
}
