/**
 * \author Mr.Nobody
 * \file I3c_Dma.c
 * \ingroup I3c
 * \brief I3c module DMA handling of the data FIFOs
 *
 * The transmit FIFO (I3C_TDR) and the receive FIFO (I3C_RDR) are served by one Gpdma channel
 * each in the DMA mode. A channel is initialized once (I3cDma_Init) with the request of the
 * instance, 8-bit data and a static peripheral address; a transfer only sets the block size and
 * the memory address and activates the channel. The end of a frame is signalled by the I3C
 * peripheral itself (frame completed event), the DMA transfer complete interrupt is not used.
 * The control FIFO and the status FIFO are not served by the DMA (see I3c_Ctrl.c).
 *
 * A read that the target ends early cannot be followed by the DMA channel (the peripheral flow
 * control of the Gpdma is not used), the module stops the channel and takes the number of
 * received bytes from the status register.
 *
 */
/* ============================== INCLUDES ================================== */
#include "I3c.h"                            /* Module private interface       */
#include "I3c_Reg.h"                        /* I3C register access            */
#include "Gpdma_Port.h"                     /* GPDMA Port module definitions  */
/* ============================== TYPEDEFS ================================== */

/** \brief DMA channel identification of a direction of an instance */
typedef struct
{
    gpdma_PeriphId_t  DmaPeriphId;  /**< Gpdma peripheral */
    gpdma_ChannelId_t DmaChannelId; /**< Gpdma channel    */
    gpdma_Priority_t  DmaPriority;  /**< Channel priority */
}   i3c_DmaChannel_t;

/* ======================== FORWARD DECLARATIONS ============================ */

static i3c_RequestState_t I3cPriv_Get_DmaChannel ( const i3c_Context_t * const ctx,
                                                   i3c_DmaDir_t dmaDir,
                                                   i3c_DmaChannel_t * const dmaChannel );
static i3c_RequestState_t I3cPriv_Init_Channel   ( i3c_Context_t * const ctx,
                                                   i3c_PeriphId_t periphId,
                                                   i3c_DmaDir_t dmaDir );
static void               I3cPriv_Set_DmaError   ( i3c_PeriphId_t periphId, i3c_DmaDir_t dmaDir );

static void               I3cPriv_Dma_Error_1_Tx ( gpdma_ErrorMaskId_t errorMask );
static void               I3cPriv_Dma_Error_1_Rx ( gpdma_ErrorMaskId_t errorMask );
#if defined(I3C2)
static void               I3cPriv_Dma_Error_2_Tx ( gpdma_ErrorMaskId_t errorMask );
static void               I3cPriv_Dma_Error_2_Rx ( gpdma_ErrorMaskId_t errorMask );
#endif

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** \brief Gpdma error bit mask that ends the running request. The suspension (GPDMA_ERROR_SUSPENDED) is left out: the
 *         module suspends the channel itself to stop a request (\ref I3cDma_Stop) and the stop polls the suspension
 *         flag, which the Gpdma interrupt handler would clear first and report as an error. */
#define I3C_DMA_ERROR_MASK                    ( GPDMA_ERROR_TRANSFER      | \
                                                GPDMA_ERROR_CONFIG_UPDATE | \
                                                GPDMA_ERROR_CONFIG_ERROR  | \
                                                GPDMA_ERROR_TRIG_OVERRUN    )

/** \brief Burst length of the Gpdma transfers (one byte per request of the I3C peripheral) */
#define I3C_DMA_BURST_LEN                     ( 1u )

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** \brief Gpdma transfer lists (linked-list items of the channels), one per instance and direction */
static volatile gpdma_XferList_t i3c_DmaXferList[ I3C_PERIPH_CNT ][ I3C_DMA_DIR_CNT ];

/** \brief Gpdma requests of the instances: transmit and receive FIFO */
static const gpdma_PeriphReqId_t i3c_DmaRequestList[ I3C_PERIPH_CNT ][ I3C_DMA_DIR_CNT ] =
{
    { GPDMA_REQ_I3C1_TX, GPDMA_REQ_I3C1_RX }, /**< I3C1 */
#if defined(I3C2)
    { GPDMA_REQ_I3C2_TX, GPDMA_REQ_I3C2_RX }, /**< I3C2 */
#endif
};

/** \brief Error handlers of the Gpdma channels, they differ per instance and direction */
static gpdma_IsrErrCallback * const i3c_DmaErrorIsrList[ I3C_PERIPH_CNT ][ I3C_DMA_DIR_CNT ] =
{
    { I3cPriv_Dma_Error_1_Tx, I3cPriv_Dma_Error_1_Rx }, /**< I3C1 */
#if defined(I3C2)
    { I3cPriv_Dma_Error_2_Tx, I3cPriv_Dma_Error_2_Rx }, /**< I3C2 */
#endif
};

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Initializes the Gpdma channels of the transmit and the receive FIFO of an instance.
 *
 * \param periphId [in]: I3C peripheral ID
 * \return I3C_REQUEST_OK when both channels are initialized, I3C_REQUEST_ERROR otherwise (the
 *         channels initialized so far stay initialized, I3cDma_Deinit() releases them)
 */
i3c_RequestState_t I3cDma_Init( i3c_PeriphId_t periphId )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    i3c_Context_t *    ctx      = I3C_NULL_PTR;
    i3c_RequestState_t ctxState = I3c_Get_Context( periphId, &ctx );

    if( ( I3C_REQUEST_OK == ctxState ) &&
        ( I3C_NULL_PTR   != ctx      )    )
    {
        retState = I3cPriv_Init_Channel( ctx, periphId, I3C_DMA_DIR_TX );

        if( I3C_REQUEST_OK == retState )
        {
            retState = I3cPriv_Init_Channel( ctx, periphId, I3C_DMA_DIR_RX );
        }
        else
        {
            /* The transmit channel failed, no action required */
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
 * \brief Releases the Gpdma channels of an instance (a channel that was not initialized is skipped).
 *
 * \param periphId [in]: I3C peripheral ID
 * \return I3C_REQUEST_OK when all initialized channels were released, I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3cDma_Deinit( i3c_PeriphId_t periphId )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    i3c_Context_t *    ctx      = I3C_NULL_PTR;
    i3c_RequestState_t ctxState = I3c_Get_Context( periphId, &ctx );
    i3c_DmaChannel_t   dmaChan  = { GPDMA_PERIPH_1, GPDMA_CHANNEL_0, GPDMA_PRIORITY_LOW };
    gpdma_RequestState_t dmaState = GPDMA_REQUEST_OK;

    if( ( I3C_REQUEST_OK == ctxState ) &&
        ( I3C_NULL_PTR   != ctx      )    )
    {
        retState = I3C_REQUEST_OK;

        for( uint32_t dirIdx = 0u; (uint32_t)I3C_DMA_DIR_CNT > dirIdx; dirIdx++ )
        {
            i3c_FunctionState_t initState = ctx->TxDmaInit;

            if( (uint32_t)I3C_DMA_DIR_RX == dirIdx )
            {
                initState = ctx->RxDmaInit;
            }
            else
            {
                /* The transmit channel state is used, no action required */
            }

            if( I3C_FUNCTION_ACTIVE == initState )
            {
                ctxState = I3cPriv_Get_DmaChannel( ctx, (i3c_DmaDir_t)dirIdx, &dmaChan );
                dmaState = Gpdma_Set_ChannelInactive( dmaChan.DmaPeriphId, dmaChan.DmaChannelId );

                if( GPDMA_REQUEST_OK == dmaState )
                {
                    dmaState = Gpdma_Set_InterruptInactive( dmaChan.DmaPeriphId, dmaChan.DmaChannelId );
                }
                else
                {
                    /* The channel did not stop, no action required */
                }

                if( GPDMA_REQUEST_OK == dmaState )
                {
                    dmaState = Gpdma_Deinit( dmaChan.DmaPeriphId, dmaChan.DmaChannelId );
                }
                else
                {
                    /* The interrupts did not stop, no action required */
                }

                if( GPDMA_REQUEST_OK != dmaState )
                {
                    retState = I3C_REQUEST_ERROR;
                }
                else
                {
                    /* The channel is released, no action required */
                }

                if( (uint32_t)I3C_DMA_DIR_RX == dirIdx )
                {
                    ctx->RxDmaInit = I3C_FUNCTION_INACTIVE;
                }
                else
                {
                    ctx->TxDmaInit = I3C_FUNCTION_INACTIVE;
                }
            }
            else
            {
                /* The channel was not initialized, no action required */
            }
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
 * \brief Starts a DMA transfer of a data FIFO: the block size and the memory address are set and
 *        the channel is activated.
 *
 * \param periphId [in]: I3C peripheral ID
 * \param dmaDir   [in]: Transmit or receive FIFO
 * \param dataAddr [in]: Address of the memory buffer (source of the transmit, destination of the
 *                       receive transfer)
 * \param dataCnt  [in]: Count of the bytes to be moved (non-zero)
 * \return I3C_REQUEST_OK when the channel runs, I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3cDma_Start( i3c_PeriphId_t periphId,
                                 i3c_DmaDir_t dmaDir,
                                 uint32_t dataAddr,
                                 i3c_DataCnt_t dataCnt )
{
    i3c_RequestState_t   retState = I3C_REQUEST_ERROR;
    i3c_Context_t *      ctx      = I3C_NULL_PTR;
    i3c_RequestState_t   ctxState = I3c_Get_Context( periphId, &ctx );
    i3c_DmaChannel_t     dmaChan  = { GPDMA_PERIPH_1, GPDMA_CHANNEL_0, GPDMA_PRIORITY_LOW };
    gpdma_RequestState_t dmaState = GPDMA_REQUEST_ERROR;

    if( ( I3C_REQUEST_OK == ctxState        ) &&
        ( I3C_NULL_PTR   != ctx             ) &&
        ( I3C_DMA_DIR_CNT > dmaDir          ) &&
        ( 0u              < dataCnt         ) &&
        ( 0u              != dataAddr       )    )
    {
        ctxState = I3cPriv_Get_DmaChannel( ctx, dmaDir, &dmaChan );

        if( I3C_REQUEST_OK == ctxState )
        {
            dmaState = Gpdma_Set_BlockSize( dmaChan.DmaPeriphId, dmaChan.DmaChannelId, (gpdma_BlockSize_t)dataCnt );

            if( ( GPDMA_REQUEST_OK == dmaState ) &&
                ( I3C_DMA_DIR_TX == dmaDir )    )
            {
                dmaState = Gpdma_Set_SourceAddr( dmaChan.DmaPeriphId, dmaChan.DmaChannelId, (gpdma_SrcAddr_t)dataAddr );
            }
            else
            {
                /* The transmit source address is not set, no action required */
            }

            if( ( GPDMA_REQUEST_OK == dmaState ) &&
                ( I3C_DMA_DIR_RX == dmaDir )    )
            {
                dmaState = Gpdma_Set_DestinationAddr( dmaChan.DmaPeriphId,
                                                      dmaChan.DmaChannelId,
                                                      (gpdma_DstAddr_t)dataAddr );
            }
            else
            {
                /* The receive destination address is not set, no action required */
            }

            if( GPDMA_REQUEST_OK == dmaState )
            {
                dmaState = Gpdma_Set_ChannelActive( dmaChan.DmaPeriphId, dmaChan.DmaChannelId );
            }
            else
            {
                /* The channel configuration failed, no action required */
            }

            if( GPDMA_REQUEST_OK == dmaState )
            {
                retState = I3C_REQUEST_OK;
            }
            else
            {
                retState = I3C_REQUEST_ERROR;
            }
        }
        else
        {
            /* The channel is not initialized */
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
 * \brief Stops the DMA channel of a data FIFO.
 *
 * \param periphId [in]: I3C peripheral ID
 * \param dmaDir   [in]: Transmit or receive FIFO
 * \return I3C_REQUEST_OK when the channel is stopped, I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3cDma_Stop( i3c_PeriphId_t periphId, i3c_DmaDir_t dmaDir )
{
    i3c_RequestState_t   retState = I3C_REQUEST_ERROR;
    i3c_Context_t *      ctx      = I3C_NULL_PTR;
    i3c_RequestState_t   ctxState = I3c_Get_Context( periphId, &ctx );
    i3c_DmaChannel_t     dmaChan  = { GPDMA_PERIPH_1, GPDMA_CHANNEL_0, GPDMA_PRIORITY_LOW };
    gpdma_RequestState_t dmaState = GPDMA_REQUEST_ERROR;

    if( ( I3C_REQUEST_OK == ctxState ) &&
        ( I3C_NULL_PTR   != ctx      ) &&
        ( I3C_DMA_DIR_CNT > dmaDir   )    )
    {
        ctxState = I3cPriv_Get_DmaChannel( ctx, dmaDir, &dmaChan );

        if( I3C_REQUEST_OK == ctxState )
        {
            dmaState = Gpdma_Set_ChannelInactive( dmaChan.DmaPeriphId, dmaChan.DmaChannelId );

            if( GPDMA_REQUEST_OK == dmaState )
            {
                retState = I3C_REQUEST_OK;
            }
            else
            {
                retState = I3C_REQUEST_ERROR;
            }
        }
        else
        {
            /* The channel is not initialized */
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
 * \brief Gets the count of the bytes that the DMA channel has not moved yet.
 *
 * \param periphId  [in]: I3C peripheral ID
 * \param dmaDir    [in]: Transmit or receive FIFO
 * \param remaining [out]: Count of the remaining bytes
 * \return I3C_REQUEST_OK when the count was read, I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3cDma_Get_Remaining( i3c_PeriphId_t periphId, i3c_DmaDir_t dmaDir, i3c_DataCnt_t * const remaining )
{
    i3c_RequestState_t   retState = I3C_REQUEST_ERROR;
    i3c_Context_t *      ctx      = I3C_NULL_PTR;
    i3c_RequestState_t   ctxState = I3c_Get_Context( periphId, &ctx );
    i3c_DmaChannel_t     dmaChan  = { GPDMA_PERIPH_1, GPDMA_CHANNEL_0, GPDMA_PRIORITY_LOW };
    gpdma_BlockSize_t    blockSize = 0u;
    gpdma_RequestState_t dmaState = GPDMA_REQUEST_ERROR;

    if( ( I3C_REQUEST_OK == ctxState   ) &&
        ( I3C_NULL_PTR   != ctx        ) &&
        ( I3C_DMA_DIR_CNT > dmaDir     ) &&
        ( I3C_NULL_PTR   != remaining  )    )
    {
        ctxState = I3cPriv_Get_DmaChannel( ctx, dmaDir, &dmaChan );

        if( I3C_REQUEST_OK == ctxState )
        {
            dmaState = Gpdma_Get_BlockSize( dmaChan.DmaPeriphId, dmaChan.DmaChannelId, &blockSize );

            if( GPDMA_REQUEST_OK == dmaState )
            {
                *remaining = (i3c_DataCnt_t)blockSize;
                retState   = I3C_REQUEST_OK;
            }
            else
            {
                retState = I3C_REQUEST_ERROR;
            }
        }
        else
        {
            /* The channel is not initialized */
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


/* ============================ LOCAL FUNCTIONS ============================== */

/**
 * \brief Gets the Gpdma channel of a direction of an initialized channel.
 *
 * \param ctx        [in]: Context of the instance
 * \param dmaDir     [in]: Transmit or receive FIFO
 * \param dmaChannel [out]: Gpdma peripheral, channel and priority
 * \return I3C_REQUEST_OK when the channel is initialized, I3C_REQUEST_ERROR otherwise
 */
static i3c_RequestState_t I3cPriv_Get_DmaChannel( const i3c_Context_t * const ctx,
                                                  i3c_DmaDir_t dmaDir,
                                                  i3c_DmaChannel_t * const dmaChannel )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;

    if( ( I3C_DMA_DIR_TX == dmaDir                  ) &&
        ( I3C_FUNCTION_ACTIVE == ctx->TxDmaInit     )    )
    {
        dmaChannel->DmaPeriphId  = ctx->Config.TxDmaPeriphId;
        dmaChannel->DmaChannelId = ctx->Config.TxDmaChannelId;
        dmaChannel->DmaPriority  = ctx->Config.TxDmaPriority;
        retState = I3C_REQUEST_OK;
    }
    else
    {
        if( ( I3C_DMA_DIR_RX == dmaDir                  ) &&
            ( I3C_FUNCTION_ACTIVE == ctx->RxDmaInit     )    )
        {
            dmaChannel->DmaPeriphId  = ctx->Config.RxDmaPeriphId;
            dmaChannel->DmaChannelId = ctx->Config.RxDmaChannelId;
            dmaChannel->DmaPriority  = ctx->Config.RxDmaPriority;
            retState = I3C_REQUEST_OK;
        }
        else
        {
            /* Invalid direction or the channel is not initialized */
            retState = I3C_REQUEST_ERROR;
        }
    }

    return ( retState );
}


/**
 * \brief Initializes the Gpdma channel of one data FIFO.
 *
 * \param ctx      [in]: Context of the instance
 * \param periphId [in]: I3C peripheral ID
 * \param dmaDir   [in]: Transmit or receive FIFO
 * \return I3C_REQUEST_OK when the channel is initialized and its interrupts are enabled
 */
static i3c_RequestState_t I3cPriv_Init_Channel( i3c_Context_t * const ctx,
                                                i3c_PeriphId_t periphId,
                                                i3c_DmaDir_t dmaDir )
{
    i3c_RequestState_t     retState   = I3C_REQUEST_ERROR;
    gpdma_ConfigStruct_t   dmaConfig  = { 0 };
    gpdma_TransferConfig_t xferConfig = { 0 };
    gpdma_RequestState_t   dmaState   = GPDMA_REQUEST_ERROR;
    i3c_DmaChannel_t       dmaChan    = { GPDMA_PERIPH_1, GPDMA_CHANNEL_0, GPDMA_PRIORITY_LOW };

    if( I3C_DMA_DIR_TX == dmaDir )
    {
        dmaChan.DmaPeriphId  = ctx->Config.TxDmaPeriphId;
        dmaChan.DmaChannelId = ctx->Config.TxDmaChannelId;
        dmaChan.DmaPriority  = ctx->Config.TxDmaPriority;

        xferConfig.Direction           = GPDMA_DIR_MEMORY_TO_PERIPH;
        xferConfig.DestinationAddr     = (gpdma_DstAddr_t)I3c_Reg_Get_Addr( periphId, I3C_REG_TDR );
        xferConfig.DestinationAddrMode = GPDMA_ADDR_STATIC;
        xferConfig.SourceAddrMode      = GPDMA_ADDR_INCREMENT;
    }
    else
    {
        dmaChan.DmaPeriphId  = ctx->Config.RxDmaPeriphId;
        dmaChan.DmaChannelId = ctx->Config.RxDmaChannelId;
        dmaChan.DmaPriority  = ctx->Config.RxDmaPriority;

        xferConfig.Direction           = GPDMA_DIR_PERIPH_TO_MEMORY;
        xferConfig.SourceAddr          = (gpdma_SrcAddr_t)I3c_Reg_Get_Addr( periphId, I3C_REG_RDR );
        xferConfig.SourceAddrMode      = GPDMA_ADDR_STATIC;
        xferConfig.DestinationAddrMode = GPDMA_ADDR_INCREMENT;
    }

    xferConfig.EventMode              = GPDMA_TRANSFER_EVENT_BLOCK;
    xferConfig.TriggerType            = GPDMA_TRG_NOT_USED;
    xferConfig.TriggerMode            = GPDMA_TRIGGER_BLOCK;
    xferConfig.RequestSource          = i3c_DmaRequestList[ periphId ][ dmaDir ];
    xferConfig.RequestMode            = GPDMA_PERIPH_REQ_SINGLE;
    xferConfig.BlockSize              = 0u; /* Set per transfer by I3cDma_Start */
    xferConfig.BlockRepetitionCount   = 0u;
    xferConfig.SourceDataSize         = GPDMA_DATA_SIZE_8BITS;
    xferConfig.SourceBurstLength      = I3C_DMA_BURST_LEN;
    xferConfig.SourcePortId           = GPDMA_PORT_DEFAULT;
    xferConfig.SourceDataOp           = GPDMA_SRC_DATA_PRESERVE;
    xferConfig.SourceBlockOffset2D    = 0u;
    xferConfig.SourceRepBlockOffset2D = 0u;
    xferConfig.DestinationDataSize    = GPDMA_DATA_SIZE_8BITS;
    xferConfig.DestinationBurstLength = I3C_DMA_BURST_LEN;
    xferConfig.DestinationPortId      = GPDMA_PORT_DEFAULT;
    xferConfig.DestinationDataOp      = GPDMA_DEST_DATA_PRESERVE;
    xferConfig.DestinationBlockOffset2D    = 0u;
    xferConfig.DestinationRepBlockOffset2D = 0u;

    dmaState = Gpdma_Get_DefaultConfig( &dmaConfig );

    dmaConfig.PeriphId            = dmaChan.DmaPeriphId;
    dmaConfig.ChannelId           = dmaChan.DmaChannelId;
    dmaConfig.ChannelPrio         = dmaChan.DmaPriority;
    dmaConfig.TransferExecMode    = GPDMA_XFER_EXEC_BLOCKING;
    dmaConfig.TransferConfig      = &xferConfig;
    dmaConfig.TransfersCount      = 1u;
    dmaConfig.XferListAccessMode  = GPDMA_TRANSFER_LIST_ACCESS_SINGLE;
    dmaConfig.XferList            = &i3c_DmaXferList[ periphId ][ dmaDir ];
    dmaConfig.TransferLockState   = GPDMA_TRANSFER_LIST_LOCKED;
    dmaConfig.TransferCompleteIsr = I3C_NULL_PTR;
    dmaConfig.HalfTransferIsr     = I3C_NULL_PTR;
    dmaConfig.ErrorIsr            = i3c_DmaErrorIsrList[ periphId ][ dmaDir ];
    dmaConfig.ErrorMask           = I3C_DMA_ERROR_MASK;

    if( GPDMA_REQUEST_OK == dmaState )
    {
        dmaState = Gpdma_Init( &dmaConfig );
    }
    else
    {
        /* The default configuration is not available, no action required */
    }

    if( GPDMA_REQUEST_OK == dmaState )
    {
        if( I3C_DMA_DIR_TX == dmaDir )
        {
            ctx->TxDmaInit = I3C_FUNCTION_ACTIVE;
        }
        else
        {
            ctx->RxDmaInit = I3C_FUNCTION_ACTIVE;
        }

        dmaState = Gpdma_Set_InterruptActive( dmaChan.DmaPeriphId, dmaChan.DmaChannelId );
    }
    else
    {
        /* The channel initialization failed, no action required */
    }

    if( GPDMA_REQUEST_OK == dmaState )
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
 * \brief Ends the running request because of an error of a Gpdma channel: the channels are
 *        stopped, the FIFOs are flushed and the end is reported with an overrun (receive FIFO) or
 *        an underrun (transmit FIFO).
 *
 * \param periphId [in]: I3C peripheral ID
 * \param dmaDir   [in]: Direction of the failed channel
 */
static void I3cPriv_Set_DmaError( i3c_PeriphId_t periphId, i3c_DmaDir_t dmaDir )
{
    i3c_Context_t *    ctx      = I3C_NULL_PTR;
    i3c_RequestState_t ctxState = I3c_Get_Context( periphId, &ctx );
    i3c_XferError_t    errorMask = (i3c_XferError_t)I3C_XFER_ERROR_UNDERRUN;

    if( I3C_DMA_DIR_RX == dmaDir )
    {
        errorMask = (i3c_XferError_t)I3C_XFER_ERROR_OVERRUN;
    }
    else
    {
        /* The transmit channel failed, no action required */
    }

    if( ( I3C_REQUEST_OK == ctxState ) &&
        ( I3C_NULL_PTR   != ctx      )    )
    {
        if( I3C_XFER_STATE_BUSY == ctx->XferState )
        {
            (void)I3cDma_Stop( periphId, I3C_DMA_DIR_TX );
            (void)I3cDma_Stop( periphId, I3C_DMA_DIR_RX );
            I3c_Reg_Set_FifoFlush( periphId, I3C_CFGR_FLUSH_ALL_MASK );
            I3c_Set_XferEnd( periphId, I3C_XFER_STATE_ERROR, errorMask, 0u );
        }
        else
        {
            /* No request is running, no action required */
        }
    }
    else
    {
        /* Invalid peripheral ID, no action required */
    }
}


/** \brief Error handler of the transmit channel of I3C1 */
static void I3cPriv_Dma_Error_1_Tx( gpdma_ErrorMaskId_t errorMask )
{
    (void)errorMask;
    I3cPriv_Set_DmaError( I3C_PERIPH_1, I3C_DMA_DIR_TX );
}


/** \brief Error handler of the receive channel of I3C1 */
static void I3cPriv_Dma_Error_1_Rx( gpdma_ErrorMaskId_t errorMask )
{
    (void)errorMask;
    I3cPriv_Set_DmaError( I3C_PERIPH_1, I3C_DMA_DIR_RX );
}


#if defined(I3C2)
/** \brief Error handler of the transmit channel of I3C2 */
static void I3cPriv_Dma_Error_2_Tx( gpdma_ErrorMaskId_t errorMask )
{
    (void)errorMask;
    I3cPriv_Set_DmaError( I3C_PERIPH_2, I3C_DMA_DIR_TX );
}


/** \brief Error handler of the receive channel of I3C2 */
static void I3cPriv_Dma_Error_2_Rx( gpdma_ErrorMaskId_t errorMask )
{
    (void)errorMask;
    I3cPriv_Set_DmaError( I3C_PERIPH_2, I3C_DMA_DIR_RX );
}
#endif
