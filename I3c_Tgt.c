/**
 * \author Mr.Nobody
 * \file I3c_Tgt.c
 * \ingroup I3c
 * \brief I3c module target role
 *
 * The target role of the peripheral: the configuration of the identity (BCR, DCR, instance ID of the
 * provisioned ID, maximum read and write length, static address), the requests of the target
 * (in-band interrupt, hot-join), the buffers of the private read and write and the event handler
 * (dynamic address assignment, CCCs, end of a read or write, errors). The handler is called by
 * I3c_Task() in the POLL mode and by the interrupt in the ISR and DMA mode.
 *
 * Device behaviour the target respects (RM0481 54.7, 54.9, 54.11):
 * - The identity registers (BCR, DCR, MAXRLR, MAXWLR, EPIDR, DEVR0 enables, I3C_I2CCFGR) are
 *   written with the peripheral disabled, the peripheral is enabled again afterwards.
 * - The 48-bit provisioned ID is fixed by the device, only the 4-bit instance ID is written
 *   (RM0481 54.15.30).
 * - Only the bits BCR0, BCR2 and BCR6 of I3C_BCR are writable (RM0481 54.15.25), the hardware returns
 *   BCR1 (IBI request capable) as 1 on the bus by itself, so the bit is not written and not verified.
 * - A read of the controller needs the data in the TX-FIFO in advance (preload): the buffer is armed
 *   by I3c_Set_TargetReadBuffer() before the controller reads, the count is written to I3C_TGTTDR.
 *   The hardware has no event for the start of a read, therefore the read request callback is called
 *   when a read finds no data (TX-FIFO underrun) so that the application arms the next buffer.
 * - When the controller ends a read early, the number of bytes sent is I3C_SR.XDCNT and the TX-FIFO
 *   is flushed by the software (RM0481 54.7.3).
 * - The static address (I3C_I2CCFGR) exists on I3C1 of the STM32H523 .. H573 lines only.
 *
 */
/* ============================== INCLUDES ================================== */
#include "I3c.h"                            /* Module private interface       */
#include "I3c_Port.h"                       /* Module public interface        */
#include "I3c_Reg.h"                        /* I3C register access            */
/* ============================== TYPEDEFS ================================== */

/** \brief Event flag of a CCC received by the target and the event it is reported as */
typedef struct
{
    uint32_t       EventMask; /**< Flag of the event register */
    i3c_CccEvent_t CccEvent;  /**< Reported event             */
}   i3c_CccFlag_t;

/* ======================== FORWARD DECLARATIONS ============================ */

static i3c_RequestState_t I3cPriv_Get_Target       ( i3c_PeriphId_t periphId, i3c_Context_t ** const ctx );
static i3c_RequestState_t I3cPriv_Check_Config     ( i3c_PeriphId_t periphId,
                                                     const i3c_TargetConfig_t * const targetConfig );
static i3c_RequestState_t I3cPriv_Write_Config     ( i3c_PeriphId_t periphId,
                                                     const i3c_TargetConfig_t * const targetConfig );
static i3c_RequestState_t I3cPriv_Verify_Config    ( i3c_PeriphId_t periphId,
                                                     const i3c_TargetConfig_t * const targetConfig );
static i3c_FunctionState_t I3cPriv_Get_StaticSupport ( i3c_PeriphId_t periphId );
static void               I3cPriv_Set_FifoIrq      ( const i3c_Context_t * const ctx,
                                                     i3c_PeriphId_t periphId,
                                                     i3c_EventMask_t irqMask,
                                                     i3c_FunctionState_t irqState );
static void               I3cPriv_Fill_Read        ( i3c_Context_t * const ctx, i3c_PeriphId_t periphId );
static void               I3cPriv_Drain_Write      ( i3c_Context_t * const ctx, i3c_PeriphId_t periphId );
static void               I3cPriv_Handle_Error     ( i3c_Context_t * const ctx, i3c_PeriphId_t periphId );
static void               I3cPriv_Handle_Complete  ( i3c_Context_t * const ctx, i3c_PeriphId_t periphId );
static void               I3cPriv_Handle_Address   ( const i3c_Context_t * const ctx, i3c_PeriphId_t periphId );
static void               I3cPriv_Handle_Ccc       ( const i3c_Context_t * const ctx,
                                                     i3c_PeriphId_t periphId,
                                                     i3c_EventMask_t events );
static void               I3cPriv_Handle_Request   ( i3c_Context_t * const ctx,
                                                     i3c_PeriphId_t periphId,
                                                     i3c_EventMask_t events );
static void               I3cPriv_Release_Buffers  ( i3c_Context_t * const ctx, i3c_PeriphId_t periphId );

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** \brief Writable bits of I3C_BCR: BCR0 (speed limit), BCR2 (IBI payload) and BCR6 (controller capable) */
#define I3C_TGT_BCR_REG_MASK                  ( I3C_BCR_SPEED_LIMIT | I3C_BCR_IBI_PAYLOAD | I3C_BCR_CONTROLLER_CAPABLE )

/** \brief Bits of I3C_I2CCFGR: static address valid (bit 0) and the position of the 7-bit static address */
#define I3C_I2CCFGR_SAVAL                     ( 0x01u )
#define I3C_I2CCFGR_SA_POS                    ( 1u )

/** \brief Bits of the event register that end a request of the target (in-band interrupt, role, hot-join) */
#define I3C_EVENTS_TGT_REQUEST_MASK           ( I3C_EVR_IBIENDF | I3C_EVR_CRUPDF | I3C_EVR_HJF )

/** \brief Bits of the event register about a CCC received by the target */
#define I3C_EVENTS_TGT_CCC_MASK \
    ( I3C_EVR_INTUPDF | I3C_EVR_ASUPDF | I3C_EVR_MWLUPDF | I3C_EVR_MRLUPDF | I3C_EVR_RSTF | \
      I3C_EVR_GETF    | I3C_EVR_STAF   | I3C_EVR_DEFF    | I3C_EVR_GRPF )

/** \brief Bits of the status register: the message was a read of the controller */
#define I3C_SR_DIR_READ                       ( 1u )

/** \brief Maximum value of the 4-bit instance ID of the provisioned ID */
#define I3C_INSTANCE_ID_MAX                   ( 15u )

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** \brief CCC event flags of the target and the events they are reported as */
static const i3c_CccFlag_t i3c_CccFlagList[] =
{
    { .EventMask = I3C_EVR_INTUPDF, .CccEvent = I3C_CCC_EVENT_EVENTS_UPDATE   },
    { .EventMask = I3C_EVR_ASUPDF,  .CccEvent = I3C_CCC_EVENT_ACTIVITY_STATE  },
    { .EventMask = I3C_EVR_MWLUPDF, .CccEvent = I3C_CCC_EVENT_MAX_WRITE_UPDATE },
    { .EventMask = I3C_EVR_MRLUPDF, .CccEvent = I3C_CCC_EVENT_MAX_READ_UPDATE },
    { .EventMask = I3C_EVR_RSTF,    .CccEvent = I3C_CCC_EVENT_RESET           },
    { .EventMask = I3C_EVR_GETF,    .CccEvent = I3C_CCC_EVENT_GET             },
    { .EventMask = I3C_EVR_STAF,    .CccEvent = I3C_CCC_EVENT_GET_STATUS      },
    { .EventMask = I3C_EVR_DEFF,    .CccEvent = I3C_CCC_EVENT_DEFINE_TARGETS  },
    { .EventMask = I3C_EVR_GRPF,    .CccEvent = I3C_CCC_EVENT_DEFINE_GROUP    },
};

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Configures the identity of the target: the static address, the instance ID of the
 *        provisioned ID, the bus and device characteristics, the maximum read and write length
 *        and the events the target may request. The peripheral is disabled while the registers
 *        are written and enabled again, the result is verified by read-back.
 *
 * \param periphId     [in]: I3C peripheral ID
 * \param targetConfig [in]: Target configuration
 * \return I3C_REQUEST_OK when the identity is configured, I3C_REQUEST_ERROR otherwise (an invalid
 *         configuration is rejected before any register is written)
 */
i3c_RequestState_t I3c_Set_TargetConfig( i3c_PeriphId_t periphId, const i3c_TargetConfig_t * const targetConfig )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    i3c_Context_t *    ctx      = I3C_NULL_PTR;

    if( I3C_NULL_PTR != targetConfig )
    {
        retState = I3cPriv_Get_Target( periphId, &ctx );
    }
    else
    {
        /* Invalid configuration pointer */
        retState = I3C_REQUEST_ERROR;
    }

    if( I3C_REQUEST_OK == retState )
    {
        retState = I3cPriv_Check_Config( periphId, targetConfig );
    }
    else
    {
        /* The instance is not an idle target, no action required */
    }

    if( I3C_REQUEST_OK == retState )
    {
        retState = I3cPriv_Write_Config( periphId, targetConfig );
    }
    else
    {
        /* The configuration is not valid, no register is written */
    }

    if( I3C_REQUEST_OK == retState )
    {
        retState = I3cPriv_Verify_Config( periphId, targetConfig );
    }
    else
    {
        /* The registers were not written, no action required */
    }

    if( I3C_REQUEST_OK == retState )
    {
        ctx->TgtConfig = *targetConfig;
    }
    else
    {
        /* The configuration was not applied, no action required */
    }

    return ( retState );
}


/**
 * \brief Requests an in-band interrupt of the target. The controller reads the payload and
 *        serves the target. The end is reported by the target request callback.
 *
 * The target needs a dynamic address, the in-band interrupt must be allowed by the configuration
 * and the payload follows the bus characteristics: one up to the configured maximum number of
 * bytes (first byte is the mandatory data byte) when BCR2 is set, none otherwise.
 *
 * \param periphId   [in]: I3C peripheral ID
 * \param payload    [in]: Payload of the in-band interrupt (NULL when it has none)
 * \param payloadCnt [in]: Count of the payload bytes
 * \return I3C_REQUEST_OK when the request is started, I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3c_Set_IbiRequest( i3c_PeriphId_t periphId,
                                       const i3c_Data_t * const payload,
                                       i3c_DataCnt_t payloadCnt )
{
    i3c_RequestState_t retState   = I3C_REQUEST_ERROR;
    i3c_Context_t *    ctx        = I3C_NULL_PTR;
    uint32_t           ibiVal     = 0u;
    uint32_t           devr0      = 0u;
    i3c_DataCnt_t      payloadMax = 0u;
    i3c_RequestState_t payloadState = I3C_REQUEST_OK;
    i3c_CtrlWord_t     ctrlWord   = 0u;

    if( ( 0u < payloadCnt           ) &&
        ( I3C_NULL_PTR == payload   )    )
    {
        payloadState = I3C_REQUEST_ERROR;
    }
    else
    {
        /* The payload buffer matches its count, no action required */
    }

    retState = I3cPriv_Get_Target( periphId, &ctx );

    if( I3C_REQUEST_OK == retState )
    {
        devr0 = I3c_Reg_Get_Val( periphId, I3C_REG_DEVR0, ( I3C_DEVR0_DAVAL | I3C_DEVR0_IBIEN ) );

        if( 0u != ( ctx->TgtConfig.Bcr & I3C_BCR_IBI_PAYLOAD ) )
        {
            payloadMax = ctx->TgtConfig.MaxIbiPayload;
        }
        else
        {
            payloadMax = 0u;
        }

        if( ( ( I3C_DEVR0_DAVAL | I3C_DEVR0_IBIEN ) == devr0 ) &&
            ( payloadMax >= payloadCnt                       ) &&
            ( I3C_IBI_PAYLOAD_CNT_MAX >= payloadCnt          ) &&
            ( I3C_REQUEST_OK == payloadState                 )    )
        {
            retState = I3C_REQUEST_OK;
        }
        else
        {
            /* No address, the request is not allowed or the payload does not fit */
            retState = I3C_REQUEST_ERROR;
        }
    }
    else
    {
        /* The instance is not an idle target, no action required */
    }

    if( I3C_REQUEST_OK == retState )
    {
        for( uint32_t idx = 0u; (uint32_t)payloadCnt > idx; idx++ )
        {
            ibiVal |= ( (uint32_t)payload[ idx ] << ( idx * I3C_BYTE_BIT_CNT ) );
        }

        I3c_Reg_Set_Raw( periphId, I3C_REG_IBIDR, ibiVal );
        I3c_Reg_Set_EventsClear( periphId, I3C_EVENTS_TGT_REQUEST_MASK );
        ctx->XferError = (i3c_XferError_t)I3C_XFER_ERROR_NONE;
        ctx->XferState = I3C_XFER_STATE_BUSY;
        ctrlWord = ( ( (i3c_CtrlWord_t)I3C_MTYPE_IBI << I3C_CR_MTYPE_Pos ) |
                     ( (i3c_CtrlWord_t)payloadCnt << I3C_CR_DCNT_Pos ) );
        I3c_Reg_Set_ControlWord( periphId, ctrlWord );
    }
    else
    {
        /* The request was not started, no action required */
    }

    return ( retState );
}


/**
 * \brief Requests a hot-join of the target (it takes part in the next dynamic address assignment).
 *        The end is reported by the target request callback.
 *
 * \param periphId [in]: I3C peripheral ID
 * \return I3C_REQUEST_OK when the request is started, I3C_REQUEST_ERROR otherwise (the hot-join
 *         is not allowed by the configuration or the target has a dynamic address already)
 */
i3c_RequestState_t I3c_Set_HotJoinRequest( i3c_PeriphId_t periphId )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    i3c_Context_t *    ctx      = I3C_NULL_PTR;
    uint32_t           devr0    = 0u;

    retState = I3cPriv_Get_Target( periphId, &ctx );

    if( I3C_REQUEST_OK == retState )
    {
        devr0 = I3c_Reg_Get_Val( periphId, I3C_REG_DEVR0, ( I3C_DEVR0_DAVAL | I3C_DEVR0_HJEN ) );

        if( I3C_DEVR0_HJEN == devr0 )
        {
            I3c_Reg_Set_EventsClear( periphId, I3C_EVENTS_TGT_REQUEST_MASK );
            ctx->XferError = (i3c_XferError_t)I3C_XFER_ERROR_NONE;
            ctx->XferState = I3C_XFER_STATE_BUSY;
            I3c_Reg_Set_ControlWord( periphId, ( (i3c_CtrlWord_t)I3C_MTYPE_HOT_JOIN << I3C_CR_MTYPE_Pos ) );
        }
        else
        {
            /* The hot-join is not allowed or the target has an address already */
            retState = I3C_REQUEST_ERROR;
        }
    }
    else
    {
        /* The instance is not an idle target, no action required */
    }

    return ( retState );
}


/**
 * \brief Arms the data of the next read of the controller: the count is written to I3C_TGTTDR
 *        (preload), the transmit FIFO is filled and the rest is served by the event handler
 *        (POLL, ISR) or by the DMA. The end of the read is reported by the target read callback.
 *
 * \param periphId [in]: I3C peripheral ID
 * \param data     [in]: Data to be sent to the controller, it must stay valid until the read ends
 * \param dataCnt  [in]: Count of the bytes (at least one)
 * \return I3C_REQUEST_OK when the buffer is armed, I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3c_Set_TargetReadBuffer( i3c_PeriphId_t periphId,
                                             const i3c_Data_t * const data,
                                             i3c_DataCnt_t dataCnt )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    i3c_Context_t *    ctx      = I3C_NULL_PTR;
    uint32_t           tgttdr   = 0u;

    if( ( I3C_NULL_PTR != data    ) &&
        ( 0u           <  dataCnt )    )
    {
        retState = I3cPriv_Get_Target( periphId, &ctx );
    }
    else
    {
        /* Invalid parameter */
        retState = I3C_REQUEST_ERROR;
    }

    if( I3C_REQUEST_OK == retState )
    {
        ctx->TgtReadData = data;
        ctx->TgtReadCnt  = dataCnt;
        ctx->TgtReadIdx  = 0u;
        tgttdr = ( I3C_TGTTDR_PRELOAD | ( (uint32_t)dataCnt << I3C_TGTTDR_TGTTDCNT_Pos ) );
        I3c_Reg_Set_Raw( periphId, I3C_REG_TGTTDR, tgttdr );

        if( I3C_XFER_MODE_DMA == ctx->Config.XferMode )
        {
            retState = I3cDma_Start( periphId, I3C_DMA_DIR_TX, (uint32_t)data, dataCnt );
        }
        else
        {
            I3cPriv_Fill_Read( ctx, periphId );
        }

        if( I3C_REQUEST_OK != retState )
        {
            ctx->TgtReadData = I3C_NULL_PTR;
            ctx->TgtReadCnt  = 0u;
        }
        else
        {
            /* The buffer is armed, no action required */
        }
    }
    else
    {
        /* The instance is not an idle target or the parameter is not valid, no action required */
    }

    return ( retState );
}


/**
 * \brief Arms the buffer for the data of the next write of the controller. The bytes beyond the
 *        size of the buffer are dropped. The end of the write is reported by the target write
 *        callback with the count of the bytes stored.
 *
 * \param periphId [in]: I3C peripheral ID
 * \param data     [out]: Buffer for the received data, it must stay valid until the write ends
 * \param dataCnt  [in]: Size of the buffer in bytes (at least one)
 * \return I3C_REQUEST_OK when the buffer is armed, I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3c_Set_TargetWriteBuffer( i3c_PeriphId_t periphId, i3c_Data_t * const data, i3c_DataCnt_t dataCnt )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    i3c_Context_t *    ctx      = I3C_NULL_PTR;

    if( ( I3C_NULL_PTR != data    ) &&
        ( 0u           <  dataCnt )    )
    {
        retState = I3cPriv_Get_Target( periphId, &ctx );
    }
    else
    {
        /* Invalid parameter */
        retState = I3C_REQUEST_ERROR;
    }

    if( I3C_REQUEST_OK == retState )
    {
        ctx->TgtWriteData = data;
        ctx->TgtWriteCnt  = dataCnt;
        ctx->TgtWriteIdx  = 0u;

        if( I3C_XFER_MODE_DMA == ctx->Config.XferMode )
        {
            retState = I3cDma_Start( periphId, I3C_DMA_DIR_RX, (uint32_t)data, dataCnt );
        }
        else
        {
            I3cPriv_Set_FifoIrq( ctx, periphId, I3C_IER_RXFNEIE, I3C_FUNCTION_ACTIVE );
        }

        if( I3C_REQUEST_OK != retState )
        {
            ctx->TgtWriteData = I3C_NULL_PTR;
            ctx->TgtWriteCnt  = 0u;
        }
        else
        {
            /* The buffer is armed, no action required */
        }
    }
    else
    {
        /* The instance is not an idle target or the parameter is not valid, no action required */
    }

    return ( retState );
}


/**
 * \brief Stops the running request of the target: the DMA channels are stopped, the FIFOs are
 *        flushed, the buffers are released and the running request (in-band interrupt, hot-join)
 *        ends with the aborted error. The peripheral is disabled and enabled again.
 *
 * \param periphId [in]: I3C peripheral ID
 * \return I3C_REQUEST_OK when the peripheral is enabled again, I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3cTgt_Set_Stop( i3c_PeriphId_t periphId )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    i3c_Context_t *    ctx      = I3C_NULL_PTR;
    i3c_RequestState_t ctxState = I3c_Get_Context( periphId, &ctx );

    if( ( I3C_REQUEST_OK == ctxState ) &&
        ( I3C_NULL_PTR   != ctx      )    )
    {
        if( ( I3C_FUNCTION_ACTIVE == ctx->InitState ) &&
            ( I3C_ROLE_TARGET     == ctx->Config.Role )    )
        {
            I3cPriv_Release_Buffers( ctx, periphId );
            retState = I3c_Reg_Set_Disable( periphId );
            I3c_Reg_Set_FifoFlush( periphId, ( I3C_CFGR_TXFLUSH | I3C_CFGR_RXFLUSH ) );
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
            /* The module is not initialized as target */
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
 * \brief Serves the events of an instance initialized as target: the dynamic address assignment,
 *        the CCCs, the data FIFOs, the end of a private read or write, the end of a request of
 *        the target and the errors. Called by I3c_Task() in the POLL mode and by the interrupt in
 *        the ISR and DMA mode.
 *
 * \param periphId [in]: I3C peripheral ID
 */
void I3cTgt_Handle_Events( i3c_PeriphId_t periphId )
{
    i3c_Context_t *    ctx      = I3C_NULL_PTR;
    i3c_RequestState_t ctxState = I3c_Get_Context( periphId, &ctx );
    i3c_EventMask_t    events   = 0u;

    if( ( I3C_REQUEST_OK == ctxState ) &&
        ( I3C_NULL_PTR   != ctx      )    )
    {
        if( ( I3C_FUNCTION_ACTIVE == ctx->InitState   ) &&
            ( I3C_ROLE_TARGET     == ctx->Config.Role )    )
        {
            events = I3c_Reg_Get_Events( periphId );

            if( 0u != ( events & I3C_EVR_ERRF ) )
            {
                I3cPriv_Handle_Error( ctx, periphId );
            }
            else
            {
                if( I3C_XFER_MODE_DMA != ctx->Config.XferMode )
                {
                    if( 0u != ( events & I3C_EVR_TXFNFF ) )
                    {
                        I3cPriv_Fill_Read( ctx, periphId );
                    }
                    else
                    {
                        /* The transmit FIFO is full, no action required */
                    }

                    if( 0u != ( events & I3C_EVR_RXFNEF ) )
                    {
                        I3cPriv_Drain_Write( ctx, periphId );
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

                if( 0u != ( events & I3C_EVR_DAUPDF ) )
                {
                    I3cPriv_Handle_Address( ctx, periphId );
                }
                else
                {
                    /* No dynamic address assigned, no action required */
                }

                if( 0u != ( events & I3C_EVENTS_TGT_CCC_MASK ) )
                {
                    I3cPriv_Handle_Ccc( ctx, periphId, events );
                }
                else
                {
                    /* No CCC received, no action required */
                }

                if( 0u != ( events & I3C_EVENTS_TGT_REQUEST_MASK ) )
                {
                    I3cPriv_Handle_Request( ctx, periphId, events );
                }
                else
                {
                    /* No request of the target ended, no action required */
                }

                if( 0u != ( events & I3C_EVR_FCF ) )
                {
                    I3cPriv_Handle_Complete( ctx, periphId );
                }
                else
                {
                    /* No private transfer ended, no action required */
                }
            }
        }
        else
        {
            /* The module is not initialized as target, no action required */
        }
    }
    else
    {
        /* Invalid peripheral ID, no action required */
    }
}


/* ============================ LOCAL FUNCTIONS ============================== */

/**
 * \brief Gets the context of an instance that is initialized as target and has no running request.
 *
 * \param periphId [in]: I3C peripheral ID
 * \param ctx      [out]: Context of the instance
 * \return I3C_REQUEST_OK when the instance is an idle target, I3C_REQUEST_ERROR otherwise
 */
static i3c_RequestState_t I3cPriv_Get_Target( i3c_PeriphId_t periphId, i3c_Context_t ** const ctx )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    i3c_RequestState_t ctxState = I3c_Get_Context( periphId, ctx );

    if( I3C_REQUEST_OK == ctxState )
    {
        if( ( I3C_FUNCTION_ACTIVE == (*ctx)->InitState   ) &&
            ( I3C_ROLE_TARGET     == (*ctx)->Config.Role ) &&
            ( I3C_XFER_STATE_BUSY != (*ctx)->XferState   )    )
        {
            retState = I3C_REQUEST_OK;
        }
        else
        {
            /* The instance is not initialized as target or a request is running */
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
 * \brief Checks a target configuration before any register is written.
 *
 * \param periphId     [in]: I3C peripheral ID
 * \param targetConfig [in]: Target configuration
 * \return I3C_REQUEST_OK when the configuration is valid, I3C_REQUEST_ERROR otherwise
 */
static i3c_RequestState_t I3cPriv_Check_Config( i3c_PeriphId_t periphId,
                                                const i3c_TargetConfig_t * const targetConfig )
{
    i3c_RequestState_t  retState   = I3C_REQUEST_OK;
    i3c_FunctionState_t staticOk   = I3cPriv_Get_StaticSupport( periphId );

    if( ( I3C_INSTANCE_ID_MAX < targetConfig->InstanceId           ) ||
        ( I3C_IBI_PAYLOAD_CNT_MAX < targetConfig->MaxIbiPayload    ) ||
        ( 0u != ( targetConfig->Bcr & I3C_BCR_CONTROLLER_CAPABLE ) )    )
    {
        retState = I3C_REQUEST_ERROR;
    }
    else
    {
        /* The instance ID, the payload size and the bus characteristics are valid, no action required */
    }

    if( I3C_FUNCTION_ACTIVE == targetConfig->StaticAddressState )
    {
        if( ( I3C_FUNCTION_ACTIVE == staticOk                       ) &&
            ( I3C_ADDRESS_MASK >= targetConfig->StaticAddress       ) &&
            ( I3C_DYNAMIC_ADDRESS_FIRST <= targetConfig->StaticAddress ) )
        {
            /* The instance has the register and the address is valid, no action required */
        }
        else
        {
            retState = I3C_REQUEST_ERROR;
        }
    }
    else
    {
        /* No static address is used, no action required */
    }

    if( ( I3C_FUNCTION_ACTIVE == targetConfig->IbiState             ) &&
        ( 0u == ( targetConfig->Bcr & I3C_BCR_IBI_CAPABLE )         )    )
    {
        retState = I3C_REQUEST_ERROR;
    }
    else
    {
        /* The in-band interrupt is not requested or the device is able to request it, no action required */
    }

    return ( retState );
}


/**
 * \brief Writes the identity registers with the peripheral disabled and enables it again.
 *
 * \param periphId     [in]: I3C peripheral ID
 * \param targetConfig [in]: Target configuration
 * \return I3C_REQUEST_OK when the peripheral is enabled again, I3C_REQUEST_ERROR otherwise
 */
static i3c_RequestState_t I3cPriv_Write_Config( i3c_PeriphId_t periphId,
                                                const i3c_TargetConfig_t * const targetConfig )
{
    i3c_RequestState_t retState = I3c_Reg_Set_Disable( periphId );
    uint32_t           devr0    = 0u;
    uint32_t           maxrlr   = 0u;

    if( I3C_REQUEST_OK == retState )
    {
        if( I3C_FUNCTION_ACTIVE == targetConfig->IbiState )
        {
            devr0 |= I3C_DEVR0_IBIEN;
        }
        else
        {
            /* The in-band interrupt is not allowed, no action required */
        }

        if( I3C_FUNCTION_ACTIVE == targetConfig->HotJoinState )
        {
            devr0 |= I3C_DEVR0_HJEN;
        }
        else
        {
            /* The hot-join is not allowed, no action required */
        }

        I3c_Reg_Set_Val( periphId, I3C_REG_DEVR0, ( I3C_DEVR0_IBIEN | I3C_DEVR0_HJEN ), devr0 );
        I3c_Reg_Set_Raw( periphId, I3C_REG_BCR, ( (uint32_t)targetConfig->Bcr & I3C_TGT_BCR_REG_MASK ) );
        I3c_Reg_Set_Raw( periphId, I3C_REG_DCR, (uint32_t)targetConfig->Dcr );
        maxrlr = ( ( (uint32_t)targetConfig->MaxReadLen << I3C_MAXRLR_MRL_Pos ) |
                   ( (uint32_t)targetConfig->MaxIbiPayload << I3C_MAXRLR_IBIP_Pos ) );
        I3c_Reg_Set_Raw( periphId, I3C_REG_MAXRLR, maxrlr );
        I3c_Reg_Set_Raw( periphId, I3C_REG_MAXWLR, ( (uint32_t)targetConfig->MaxWriteLen << I3C_MAXWLR_MWL_Pos ) );
        I3c_Reg_Set_Val( periphId,
                         I3C_REG_EPIDR,
                         I3C_EPIDR_MIPIID_Msk,
                         ( (uint32_t)targetConfig->InstanceId << I3C_EPIDR_MIPIID_Pos ) );
#if defined(STM32H523xx) || \
    defined(STM32H533xx) || \
    defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H562xx) || \
    defined(STM32H563xx) || \
    defined(STM32H573xx)
        if( I3C_PERIPH_1 == periphId )
        {
            if( I3C_FUNCTION_ACTIVE == targetConfig->StaticAddressState )
            {
                const uint32_t i2cCfgr = ( ( (uint32_t)targetConfig->StaticAddress << I3C_I2CCFGR_SA_POS ) |
                                           I3C_I2CCFGR_SAVAL );
                I3c_Reg_Set_Raw( periphId, I3C_REG_I2CCFGR, i2cCfgr );
            }
            else
            {
                I3c_Reg_Set_Raw( periphId, I3C_REG_I2CCFGR, 0u );
            }
        }
        else
        {
            /* I3C2 has no I2C target configuration register, no action required */
        }
#endif
        retState = I3c_Reg_Set_Enable( periphId );
    }
    else
    {
        /* The peripheral did not stop, no register is written */
    }

    return ( retState );
}


/**
 * \brief Verifies the identity registers by read-back.
 *
 * \param periphId     [in]: I3C peripheral ID
 * \param targetConfig [in]: Target configuration
 * \return I3C_REQUEST_OK when the registers hold the configured values, I3C_REQUEST_ERROR otherwise
 */
static i3c_RequestState_t I3cPriv_Verify_Config( i3c_PeriphId_t periphId,
                                                 const i3c_TargetConfig_t * const targetConfig )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    const uint32_t     bcrVal   = I3c_Reg_Get_Val( periphId, I3C_REG_BCR, I3C_BCR_BCR_Msk );
    const uint32_t     dcrVal   = I3c_Reg_Get_Val( periphId, I3C_REG_DCR, I3C_DCR_DCR_Msk );
    const uint32_t     mrlVal   = I3c_Reg_Get_Val( periphId, I3C_REG_MAXRLR, I3C_MAXRLR_MRL_Msk );
    const uint32_t     mwlVal   = I3c_Reg_Get_Val( periphId, I3C_REG_MAXWLR, I3C_MAXWLR_MWL_Msk );

    if( ( ( (uint32_t)targetConfig->Bcr & I3C_TGT_BCR_REG_MASK ) == bcrVal            ) &&
        ( (uint32_t)targetConfig->Dcr == dcrVal                                       ) &&
        ( ( (uint32_t)targetConfig->MaxReadLen << I3C_MAXRLR_MRL_Pos ) == mrlVal      ) &&
        ( ( (uint32_t)targetConfig->MaxWriteLen << I3C_MAXWLR_MWL_Pos ) == mwlVal     )    )
    {
        retState = I3C_REQUEST_OK;
    }
    else
    {
        /* A register does not hold the written value */
        retState = I3C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Tells whether the instance has the I2C target configuration register (static address).
 *
 * \param periphId [in]: I3C peripheral ID
 * \return I3C_FUNCTION_ACTIVE when the static address is supported
 */
static i3c_FunctionState_t I3cPriv_Get_StaticSupport( i3c_PeriphId_t periphId )
{
    i3c_FunctionState_t retState = I3C_FUNCTION_INACTIVE;

#if defined(STM32H523xx) || \
    defined(STM32H533xx) || \
    defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H562xx) || \
    defined(STM32H563xx) || \
    defined(STM32H573xx)
    if( I3C_PERIPH_1 == periphId )
    {
        retState = I3C_FUNCTION_ACTIVE;
    }
    else
    {
        /* I3C2 has no I2C target configuration register, no action required */
    }
#else
    (void)periphId;
    retState = I3C_FUNCTION_INACTIVE;
#endif

    return ( retState );
}


/**
 * \brief Enables or disables interrupt enable bits of the data FIFOs (ISR mode only).
 *
 * \param ctx      [in]: Context of the instance
 * \param periphId [in]: I3C peripheral ID
 * \param irqMask  [in]: Mask of the interrupt enable bits
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
 * \brief Writes the data of the armed read buffer into the transmit FIFO while it has room (bounded
 *        by the depth of the FIFO per call). The interrupt of the FIFO is enabled while data is left.
 *
 * \param ctx      [in]: Context of the instance
 * \param periphId [in]: I3C peripheral ID
 */
static void I3cPriv_Fill_Read( i3c_Context_t * const ctx, i3c_PeriphId_t periphId )
{
    i3c_EventMask_t events = I3c_Reg_Get_Events( periphId );

    if( I3C_NULL_PTR != ctx->TgtReadData )
    {
        for( uint32_t idx = 0u;
             ( I3C_FIFO_DEPTH > idx ) &&
             ( ctx->TgtReadCnt > ctx->TgtReadIdx ) &&
             ( 0u != ( events & I3C_EVR_TXFNFF ) );
             idx++ )
        {
            I3c_Reg_Set_TxByte( periphId, ctx->TgtReadData[ ctx->TgtReadIdx ] );
            ctx->TgtReadIdx++;
            events = I3c_Reg_Get_Events( periphId );
        }

        if( ctx->TgtReadCnt > ctx->TgtReadIdx )
        {
            I3cPriv_Set_FifoIrq( ctx, periphId, I3C_IER_TXFNFIE, I3C_FUNCTION_ACTIVE );
        }
        else
        {
            I3cPriv_Set_FifoIrq( ctx, periphId, I3C_IER_TXFNFIE, I3C_FUNCTION_INACTIVE );
        }
    }
    else
    {
        /* No read buffer is armed, no action required */
    }
}


/**
 * \brief Reads the received data from the receive FIFO while it holds data (bounded by the depth
 *        of the FIFO per call). The bytes beyond the size of the write buffer (or without a buffer)
 *        are read and dropped, so that the FIFO does not overrun.
 *
 * \param ctx      [in]: Context of the instance
 * \param periphId [in]: I3C peripheral ID
 */
static void I3cPriv_Drain_Write( i3c_Context_t * const ctx, i3c_PeriphId_t periphId )
{
    i3c_EventMask_t events = I3c_Reg_Get_Events( periphId );
    i3c_Data_t      rxByte = 0u;

    for( uint32_t idx = 0u;
         ( I3C_FIFO_DEPTH > idx ) &&
         ( 0u != ( events & I3C_EVR_RXFNEF ) );
         idx++ )
    {
        rxByte = (i3c_Data_t)I3c_Reg_Get_RxByte( periphId );

        if( ( I3C_NULL_PTR != ctx->TgtWriteData         ) &&
            ( ctx->TgtWriteCnt > ctx->TgtWriteIdx       )    )
        {
            ctx->TgtWriteData[ ctx->TgtWriteIdx ] = rxByte;
            ctx->TgtWriteIdx++;
        }
        else
        {
            /* No room in the buffer, the byte is dropped */
        }

        events = I3c_Reg_Get_Events( periphId );
    }
}


/**
 * \brief Handles a transfer error: the read request callback is called when a read found no data,
 *        the DMA channels are stopped, the FIFOs are flushed and the request ends with the error.
 *
 * \param ctx      [in]: Context of the instance
 * \param periphId [in]: I3C peripheral ID
 */
static void I3cPriv_Handle_Error( i3c_Context_t * const ctx, i3c_PeriphId_t periphId )
{
    const uint32_t        serValue  = I3c_Reg_Get_Val( periphId, I3C_REG_SER, I3C_REG_MASK_ALL );
    const i3c_XferError_t errorMask = I3c_Get_ErrorMask( serValue );
    i3c_FunctionState_t   readEmpty = I3C_FUNCTION_INACTIVE;

    if( ( 0u != ( serValue & I3C_SER_DOVR )                ) &&
        ( ctx->TgtReadCnt <= ctx->TgtReadIdx               )    )
    {
        readEmpty = I3C_FUNCTION_ACTIVE;
    }
    else
    {
        /* No read without data, no action required */
    }

    I3cPriv_Release_Buffers( ctx, periphId );
    I3c_Reg_Set_FifoFlush( periphId, ( I3C_CFGR_TXFLUSH | I3C_CFGR_RXFLUSH ) );
    I3c_Reg_Set_EventsClear( periphId, I3C_EVR_ERRF );
    I3c_Set_XferEnd( periphId, I3C_XFER_STATE_ERROR, errorMask, 0u );

    if( ( I3C_FUNCTION_ACTIVE == readEmpty                      ) &&
        ( I3C_NULL_PTR != ctx->Config.TargetReadRequestCallback )    )
    {
        ctx->Config.TargetReadRequestCallback( periphId );
    }
    else
    {
        /* The error is not an underrun of a read or no callback is set, no action required */
    }
}


/**
 * \brief Handles the end of a private read or write (frame completed): the number of bytes comes
 *        from the status register, a read that the controller ended early flushes the transmit FIFO.
 *
 * \param ctx      [in]: Context of the instance
 * \param periphId [in]: I3C peripheral ID
 */
static void I3cPriv_Handle_Complete( i3c_Context_t * const ctx, i3c_PeriphId_t periphId )
{
    const uint32_t srVal    = I3c_Reg_Get_Val( periphId, I3C_REG_SR, ( I3C_SR_XDCNT_Msk | I3C_SR_DIR_Msk ) );
    i3c_DataCnt_t  xferCnt  = (i3c_DataCnt_t)( ( srVal & I3C_SR_XDCNT_Msk ) >> I3C_SR_XDCNT_Pos );
    uint32_t       dirRead  = ( ( srVal & I3C_SR_DIR_Msk ) >> I3C_SR_DIR_Pos );

    I3c_Reg_Set_EventsClear( periphId, I3C_EVR_FCF );

    if( I3C_SR_DIR_READ == dirRead )
    {
        if( ctx->TgtReadCnt > xferCnt )
        {
            I3c_Reg_Set_FifoFlush( periphId, I3C_CFGR_TXFLUSH );
        }
        else
        {
            /* The controller read all the data, no action required */
        }

        (void)I3cDma_Stop( periphId, I3C_DMA_DIR_TX );
        ctx->TgtReadData = I3C_NULL_PTR;
        ctx->TgtReadCnt  = 0u;
        ctx->TgtReadIdx  = 0u;
        I3cPriv_Set_FifoIrq( ctx, periphId, I3C_IER_TXFNFIE, I3C_FUNCTION_INACTIVE );

        if( I3C_NULL_PTR != ctx->Config.TargetReadCallback )
        {
            ctx->Config.TargetReadCallback( periphId, xferCnt );
        }
        else
        {
            /* No read callback is set, no action required */
        }
    }
    else
    {
        if( I3C_XFER_MODE_DMA != ctx->Config.XferMode )
        {
            I3cPriv_Drain_Write( ctx, periphId );
        }
        else
        {
            (void)I3cDma_Stop( periphId, I3C_DMA_DIR_RX );
        }

        if( xferCnt > ctx->TgtWriteCnt )
        {
            xferCnt = ctx->TgtWriteCnt;
        }
        else
        {
            /* All the bytes were stored, no action required */
        }

        ctx->TgtWriteData = I3C_NULL_PTR;
        ctx->TgtWriteCnt  = 0u;
        ctx->TgtWriteIdx  = 0u;

        if( I3C_NULL_PTR != ctx->Config.TargetWriteCallback )
        {
            ctx->Config.TargetWriteCallback( periphId, xferCnt );
        }
        else
        {
            /* No write callback is set, no action required */
        }
    }
}


/**
 * \brief Handles the assignment of the dynamic address: the address is read from I3C_DEVR0.
 *
 * \param ctx      [in]: Context of the instance
 * \param periphId [in]: I3C peripheral ID
 */
static void I3cPriv_Handle_Address( const i3c_Context_t * const ctx, i3c_PeriphId_t periphId )
{
    const uint32_t devr0 = I3c_Reg_Get_Val( periphId, I3C_REG_DEVR0, I3C_DEVR0_DA );

    I3c_Reg_Set_EventsClear( periphId, I3C_EVR_DAUPDF );

    if( I3C_NULL_PTR != ctx->Config.TargetAddressCallback )
    {
        ctx->Config.TargetAddressCallback( periphId, (i3c_Address_t)( devr0 >> I3C_DEVR0_DA_Pos ) );
    }
    else
    {
        /* No address callback is set, no action required */
    }
}


/**
 * \brief Handles the CCCs the hardware processed: every event is cleared and reported.
 *
 * \param ctx      [in]: Context of the instance
 * \param periphId [in]: I3C peripheral ID
 * \param events   [in]: Event register value
 */
static void I3cPriv_Handle_Ccc( const i3c_Context_t * const ctx, i3c_PeriphId_t periphId, i3c_EventMask_t events )
{
    I3c_Reg_Set_EventsClear( periphId, ( events & I3C_EVENTS_TGT_CCC_MASK ) );

    for( uint32_t idx = 0u; ( sizeof( i3c_CccFlagList ) / sizeof( i3c_CccFlagList[ 0u ] ) ) > idx; idx++ )
    {
        if( ( 0u != ( events & i3c_CccFlagList[ idx ].EventMask )  ) &&
            ( I3C_NULL_PTR != ctx->Config.TargetCccCallback        )    )
        {
            ctx->Config.TargetCccCallback( periphId, i3c_CccFlagList[ idx ].CccEvent );
        }
        else
        {
            /* The CCC did not happen or no callback is set, no action required */
        }
    }
}


/**
 * \brief Handles the end of a request of the target (in-band interrupt, controller-role request,
 *        hot-join): the request ends and the callback is called.
 *
 * \param ctx      [in]: Context of the instance
 * \param periphId [in]: I3C peripheral ID
 * \param events   [in]: Event register value
 */
static void I3cPriv_Handle_Request( i3c_Context_t * const ctx, i3c_PeriphId_t periphId, i3c_EventMask_t events )
{
    I3c_Reg_Set_EventsClear( periphId, ( events & I3C_EVENTS_TGT_REQUEST_MASK ) );
    ctx->XferState = I3C_XFER_STATE_IDLE;

    if( I3C_NULL_PTR != ctx->Config.TargetRequestCallback )
    {
        ctx->Config.TargetRequestCallback( periphId );
    }
    else
    {
        /* No request callback is set, no action required */
    }
}


/**
 * \brief Releases the buffers of the private read and write and stops the DMA channels.
 *
 * \param ctx      [in]: Context of the instance
 * \param periphId [in]: I3C peripheral ID
 */
static void I3cPriv_Release_Buffers( i3c_Context_t * const ctx, i3c_PeriphId_t periphId )
{
    if( I3C_XFER_MODE_DMA == ctx->Config.XferMode )
    {
        (void)I3cDma_Stop( periphId, I3C_DMA_DIR_TX );
        (void)I3cDma_Stop( periphId, I3C_DMA_DIR_RX );
    }
    else
    {
        I3cPriv_Set_FifoIrq( ctx, periphId, I3C_IER_FIFO_MASK, I3C_FUNCTION_INACTIVE );
    }

    ctx->TgtReadData  = I3C_NULL_PTR;
    ctx->TgtReadCnt   = 0u;
    ctx->TgtReadIdx   = 0u;
    ctx->TgtWriteData = I3C_NULL_PTR;
    ctx->TgtWriteCnt  = 0u;
    ctx->TgtWriteIdx  = 0u;
}
