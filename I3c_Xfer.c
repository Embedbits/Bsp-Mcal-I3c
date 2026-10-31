/**
 * \author Mr.Nobody
 * \file I3c_Xfer.c
 * \ingroup I3c
 * \brief I3c module requests of the controller: private and legacy I2C messages, CCCs, events
 *
 * The public request functions of the controller role. A request checks its parameters, finds the
 * device in the device table (a request to a device that is not in the table is rejected), builds
 * the control words and starts the frame of the controller engine (I3c_Ctrl.c); it returns at
 * once and the end is reported by the callbacks. The events started by the targets (in-band
 * interrupt, hot-join, controller-role request) are served by I3cCtrl_Handle_Requests().
 *
 * Properties of the hardware the requests respect (RM0481 54.9, 54.15.1):
 * - A private or legacy I2C message has a non-null data count (DCNT).
 * - A direct CCC is two parts: the first part carries the CCC code (and the defining byte of RSTACT),
 *   the second part the address, the direction and the data. A CCC to be sent to a device uses the
 *   dynamic address of the device (SETDASA: its static address).
 * - A write followed by a read of the same device is one frame, the two messages are separated by
 *   a repeated start (the first control word has MEND = 0).
 * - The data of a broadcast CCC are the defining byte or the data bytes of the CCC.
 *
 * Limits of this first version (see README "Status" section): the CCCs with a defining byte other
 * than RSTACT (GETSTATUS format 2, GETCAPS format 2, ...) are not offered, the table is not updated
 * by a SETDASA (the application stores the assigned address with I3c_Set_DeviceTable()).
 *
 */
/* ============================== INCLUDES ================================== */
#include "I3c.h"                            /* Module private interface       */
#include "I3c_Port.h"                       /* Module public interface        */
#include "I3c_Reg.h"                        /* I3C register access            */
/* ============================== TYPEDEFS ================================== */

/** \brief Properties of a CCC offered by the module */
typedef struct
{
    i3c_Ccc_t Ccc;      /**< CCC code                                                  */
    uint8_t   MinCnt;   /**< Minimum count of the data bytes                           */
    uint8_t   MaxCnt;   /**< Maximum count of the data bytes                           */
    uint8_t   DirMask;  /**< Allowed directions (bit mask of I3C_CCC_DIR_ items)       */
}   i3c_CccInfo_t;

/* ======================== FORWARD DECLARATIONS ============================ */

static i3c_RequestState_t I3cPriv_Get_Controller ( i3c_PeriphId_t periphId, i3c_Context_t ** const ctx );
static i3c_RequestState_t I3cPriv_Get_Device     ( i3c_Context_t * const ctx,
                                                   i3c_Address_t address,
                                                   i3c_Device_t ** const device );
static i3c_RequestState_t I3cPriv_Get_CccInfo    ( i3c_Ccc_t ccc, const i3c_CccInfo_t ** const cccInfo );
static i3c_RequestState_t I3cPriv_Check_Data     ( const i3c_Data_t * const data, i3c_DataCnt_t dataCnt );
static i3c_CtrlWord_t     I3cPriv_Get_MsgWord    ( i3c_FunctionState_t lastMsg,
                                                   uint32_t msgType,
                                                   i3c_Address_t address,
                                                   i3c_Direction_t direction,
                                                   i3c_DataCnt_t dataCnt );
static i3c_CtrlWord_t     I3cPriv_Get_CccWord    ( i3c_FunctionState_t lastMsg, i3c_Ccc_t ccc, i3c_DataCnt_t dataCnt );

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** \brief Allowed directions of a CCC: write (controller to target) and read (target to controller) */
#define I3C_CCC_DIR_WRITE                     ( 0x01u )
#define I3C_CCC_DIR_READ                      ( 0x02u )
#define I3C_CCC_DIR_BOTH                      ( 0x03u )

/** \brief Count of the data bytes of the CCCs */
#define I3C_CCC_LEN_NONE                      ( 0u )
#define I3C_CCC_LEN_BYTE                      ( 1u )
#define I3C_CCC_LEN_WORD                      ( 2u )
#define I3C_CCC_LEN_TRIPLE                    ( 3u )
#define I3C_CCC_LEN_QUAD                      ( 4u )
#define I3C_CCC_LEN_MXDS                      ( 5u )
#define I3C_CCC_LEN_PID                       ( 6u )

/** \brief Count of the defining bytes in the first part of the RSTACT CCC */
#define I3C_CCC_RSTACT_DEFINING_CNT           ( 1u )

/** \brief Bits of the IBI payload register (one byte per data byte of the payload) */
#define I3C_IBI_BYTE_MASK                     ( 0xFFu )

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** \brief CCCs offered by the module with the limits of their data */
static const i3c_CccInfo_t i3c_CccInfoList[] =
{
    { I3C_CCC_ENEC,          I3C_CCC_LEN_BYTE, I3C_CCC_LEN_BYTE,   I3C_CCC_DIR_WRITE },
    { I3C_CCC_DISEC,         I3C_CCC_LEN_BYTE, I3C_CCC_LEN_BYTE,   I3C_CCC_DIR_WRITE },
    { I3C_CCC_ENTAS0,        I3C_CCC_LEN_NONE, I3C_CCC_LEN_NONE,   I3C_CCC_DIR_WRITE },
    { I3C_CCC_ENTAS1,        I3C_CCC_LEN_NONE, I3C_CCC_LEN_NONE,   I3C_CCC_DIR_WRITE },
    { I3C_CCC_ENTAS2,        I3C_CCC_LEN_NONE, I3C_CCC_LEN_NONE,   I3C_CCC_DIR_WRITE },
    { I3C_CCC_ENTAS3,        I3C_CCC_LEN_NONE, I3C_CCC_LEN_NONE,   I3C_CCC_DIR_WRITE },
    { I3C_CCC_RSTDAA,        I3C_CCC_LEN_NONE, I3C_CCC_LEN_NONE,   I3C_CCC_DIR_WRITE },
    { I3C_CCC_SETMWL,        I3C_CCC_LEN_WORD, I3C_CCC_LEN_WORD,   I3C_CCC_DIR_WRITE },
    { I3C_CCC_SETMRL,        I3C_CCC_LEN_WORD, I3C_CCC_LEN_TRIPLE, I3C_CCC_DIR_WRITE },
    { I3C_CCC_SETAASA,       I3C_CCC_LEN_NONE, I3C_CCC_LEN_NONE,   I3C_CCC_DIR_WRITE },
    { I3C_CCC_RSTACT,        I3C_CCC_LEN_BYTE, I3C_CCC_LEN_BYTE,   I3C_CCC_DIR_WRITE },
    { I3C_CCC_ENEC_DIRECT,   I3C_CCC_LEN_BYTE, I3C_CCC_LEN_BYTE,   I3C_CCC_DIR_WRITE },
    { I3C_CCC_DISEC_DIRECT,  I3C_CCC_LEN_BYTE, I3C_CCC_LEN_BYTE,   I3C_CCC_DIR_WRITE },
    { I3C_CCC_ENTAS0_DIRECT, I3C_CCC_LEN_NONE, I3C_CCC_LEN_NONE,   I3C_CCC_DIR_WRITE },
    { I3C_CCC_ENTAS1_DIRECT, I3C_CCC_LEN_NONE, I3C_CCC_LEN_NONE,   I3C_CCC_DIR_WRITE },
    { I3C_CCC_ENTAS2_DIRECT, I3C_CCC_LEN_NONE, I3C_CCC_LEN_NONE,   I3C_CCC_DIR_WRITE },
    { I3C_CCC_ENTAS3_DIRECT, I3C_CCC_LEN_NONE, I3C_CCC_LEN_NONE,   I3C_CCC_DIR_WRITE },
    { I3C_CCC_RSTDAA_DIRECT, I3C_CCC_LEN_NONE, I3C_CCC_LEN_NONE,   I3C_CCC_DIR_WRITE },
    { I3C_CCC_SETDASA,       I3C_CCC_LEN_BYTE, I3C_CCC_LEN_BYTE,   I3C_CCC_DIR_WRITE },
    { I3C_CCC_SETNEWDA,      I3C_CCC_LEN_BYTE, I3C_CCC_LEN_BYTE,   I3C_CCC_DIR_WRITE },
    { I3C_CCC_SETMWL_DIRECT, I3C_CCC_LEN_WORD, I3C_CCC_LEN_WORD,   I3C_CCC_DIR_WRITE },
    { I3C_CCC_SETMRL_DIRECT, I3C_CCC_LEN_WORD, I3C_CCC_LEN_TRIPLE, I3C_CCC_DIR_WRITE },
    { I3C_CCC_GETMWL,        I3C_CCC_LEN_WORD, I3C_CCC_LEN_WORD,   I3C_CCC_DIR_READ  },
    { I3C_CCC_GETMRL,        I3C_CCC_LEN_WORD, I3C_CCC_LEN_TRIPLE, I3C_CCC_DIR_READ  },
    { I3C_CCC_GETPID,        I3C_CCC_LEN_PID,  I3C_CCC_LEN_PID,    I3C_CCC_DIR_READ  },
    { I3C_CCC_GETBCR,        I3C_CCC_LEN_BYTE, I3C_CCC_LEN_BYTE,   I3C_CCC_DIR_READ  },
    { I3C_CCC_GETDCR,        I3C_CCC_LEN_BYTE, I3C_CCC_LEN_BYTE,   I3C_CCC_DIR_READ  },
    { I3C_CCC_GETSTATUS,     I3C_CCC_LEN_WORD, I3C_CCC_LEN_WORD,   I3C_CCC_DIR_READ  },
    { I3C_CCC_GETMXDS,       I3C_CCC_LEN_WORD, I3C_CCC_LEN_MXDS,   I3C_CCC_DIR_READ  },
    { I3C_CCC_GETCAPS,       I3C_CCC_LEN_BYTE, I3C_CCC_LEN_QUAD,   I3C_CCC_DIR_READ  },
    { I3C_CCC_RSTACT_DIRECT, I3C_CCC_LEN_BYTE, I3C_CCC_LEN_BYTE,   I3C_CCC_DIR_BOTH  },
};

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Writes data to a device (private write to an I3C device, legacy write to an I2C device).
 *
 * \param periphId [in]: I3C peripheral ID
 * \param address  [in]: Dynamic address (I3C device) or static address (I2C device) of a device of the table
 * \param data     [in]: Data to be written
 * \param dataCnt  [in]: Count of the bytes to be written (at least one)
 * \return I3C_REQUEST_OK when the request is started (the callbacks report the end), I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3c_Set_Write( i3c_PeriphId_t periphId,
                                  i3c_Address_t address,
                                  const i3c_Data_t * const data,
                                  i3c_DataCnt_t dataCnt )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    i3c_Context_t *    ctx      = I3C_NULL_PTR;
    i3c_Device_t *     device   = I3C_NULL_PTR;

    if( ( I3C_NULL_PTR != data ) &&
        ( 0u           <  dataCnt )    )
    {
        retState = I3cPriv_Get_Controller( periphId, &ctx );

        if( I3C_REQUEST_OK == retState )
        {
            retState = I3cPriv_Get_Device( ctx, address, &device );
        }
        else
        {
            /* The instance is not an idle controller, no action required */
        }

        if( I3C_REQUEST_OK == retState )
        {
            retState = I3cCtrl_Start_Message( periphId, device, data, dataCnt, I3C_NULL_PTR, 0u );
        }
        else
        {
            /* The device is not in the table or the instance is not ready, no action required */
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
 * \brief Reads data from a device (private read of an I3C device, legacy read of an I2C device).
 *
 * \param periphId [in]: I3C peripheral ID
 * \param address  [in]: Dynamic address (I3C device) or static address (I2C device) of a device of the table
 * \param data     [out]: Buffer for the data, it must stay valid until the request ends
 * \param dataCnt  [in]: Count of the bytes to be read (at least one)
 * \return I3C_REQUEST_OK when the request is started (the callbacks report the end), I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3c_Set_Read( i3c_PeriphId_t periphId,
                                 i3c_Address_t address,
                                 i3c_Data_t * const data,
                                 i3c_DataCnt_t dataCnt )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    i3c_Context_t *    ctx      = I3C_NULL_PTR;
    i3c_Device_t *     device   = I3C_NULL_PTR;

    if( ( I3C_NULL_PTR != data ) &&
        ( 0u           <  dataCnt )    )
    {
        retState = I3cPriv_Get_Controller( periphId, &ctx );

        if( I3C_REQUEST_OK == retState )
        {
            retState = I3cPriv_Get_Device( ctx, address, &device );
        }
        else
        {
            /* The instance is not an idle controller, no action required */
        }

        if( I3C_REQUEST_OK == retState )
        {
            retState = I3cCtrl_Start_Message( periphId, device, I3C_NULL_PTR, 0u, data, dataCnt );
        }
        else
        {
            /* The device is not in the table or the instance is not ready, no action required */
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
 * \brief Writes data to a device and reads data from it in one frame (repeated start between the
 *        two messages).
 *
 * \param periphId  [in]: I3C peripheral ID
 * \param address   [in]: Dynamic address (I3C device) or static address (I2C device) of a device of the table
 * \param writeData [in]: Data to be written
 * \param writeCnt  [in]: Count of the bytes to be written (at least one)
 * \param readData  [out]: Buffer for the data read, it must stay valid until the request ends
 * \param readCnt   [in]: Count of the bytes to be read (at least one)
 * \return I3C_REQUEST_OK when the request is started (the callbacks report the end), I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3c_Set_WriteRead( i3c_PeriphId_t periphId,
                                      i3c_Address_t address,
                                      const i3c_Data_t * const writeData,
                                      i3c_DataCnt_t writeCnt,
                                      i3c_Data_t * const readData,
                                      i3c_DataCnt_t readCnt )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    i3c_Context_t *    ctx      = I3C_NULL_PTR;
    i3c_Device_t *     device   = I3C_NULL_PTR;

    if( ( I3C_NULL_PTR != writeData ) &&
        ( 0u           <  writeCnt  ) &&
        ( I3C_NULL_PTR != readData  ) &&
        ( 0u           <  readCnt   )    )
    {
        retState = I3cPriv_Get_Controller( periphId, &ctx );

        if( I3C_REQUEST_OK == retState )
        {
            retState = I3cPriv_Get_Device( ctx, address, &device );
        }
        else
        {
            /* The instance is not an idle controller, no action required */
        }

        if( I3C_REQUEST_OK == retState )
        {
            retState = I3cCtrl_Start_Message( periphId, device, writeData, writeCnt, readData, readCnt );
        }
        else
        {
            /* The device is not in the table or the instance is not ready, no action required */
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
 * \brief Stops the running request of an instance (the controller: the peripheral is disabled,
 *        the FIFOs are flushed and the peripheral is enabled again; the target: the buffers are
 *        released). The end is reported by the error callback with the aborted error.
 *
 * \param periphId [in]: I3C peripheral ID
 * \return I3C_REQUEST_OK when the request was stopped, I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3c_Set_XferStop( i3c_PeriphId_t periphId )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    i3c_Context_t *    ctx      = I3C_NULL_PTR;
    i3c_RequestState_t ctxState = I3c_Get_Context( periphId, &ctx );

    if( ( I3C_REQUEST_OK == ctxState ) &&
        ( I3C_NULL_PTR   != ctx      )    )
    {
        if( I3C_FUNCTION_ACTIVE == ctx->InitState )
        {
            if( I3C_ROLE_CONTROLLER == ctx->Config.Role )
            {
                retState = I3cCtrl_Set_Stop( periphId );
            }
            else
            {
                retState = I3cTgt_Set_Stop( periphId );
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
 * \brief Gets the state of the request of an instance.
 *
 * \param periphId  [in]: I3C peripheral ID
 * \param xferState [out]: Request state (IDLE, BUSY, ERROR)
 * \return I3C_REQUEST_OK when the state was read, I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3c_Get_XferState( i3c_PeriphId_t periphId, i3c_XferState_t * const xferState )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    i3c_Context_t *    ctx      = I3C_NULL_PTR;
    i3c_RequestState_t ctxState = I3c_Get_Context( periphId, &ctx );

    if( ( I3C_REQUEST_OK == ctxState  ) &&
        ( I3C_NULL_PTR   != ctx       ) &&
        ( I3C_NULL_PTR   != xferState )    )
    {
        if( I3C_FUNCTION_ACTIVE == ctx->InitState )
        {
            *xferState = ctx->XferState;
            retState   = I3C_REQUEST_OK;
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
 * \brief Gets the errors of the last request of an instance.
 *
 * \param periphId  [in]: I3C peripheral ID
 * \param xferError [out]: Bit mask of the errors of \ref i3c_XferErrorId_t
 * \return I3C_REQUEST_OK when the errors were read, I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3c_Get_XferError( i3c_PeriphId_t periphId, i3c_XferError_t * const xferError )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    i3c_Context_t *    ctx      = I3C_NULL_PTR;
    i3c_RequestState_t ctxState = I3c_Get_Context( periphId, &ctx );

    if( ( I3C_REQUEST_OK == ctxState  ) &&
        ( I3C_NULL_PTR   != ctx       ) &&
        ( I3C_NULL_PTR   != xferError )    )
    {
        if( I3C_FUNCTION_ACTIVE == ctx->InitState )
        {
            *xferError = ctx->XferError;
            retState   = I3C_REQUEST_OK;
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
 * \brief Sends a broadcast CCC to all the targets.
 *
 * \param periphId [in]: I3C peripheral ID
 * \param ccc      [in]: Broadcast CCC (bit 7 of the code is clear)
 * \param data     [in]: Defining byte or data bytes of the CCC (NULL if the CCC has none)
 * \param dataCnt  [in]: Count of the bytes of the CCC
 * \return I3C_REQUEST_OK when the request is started (the callbacks report the end), I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3c_Set_BroadcastCcc( i3c_PeriphId_t periphId,
                                         i3c_Ccc_t ccc,
                                         const i3c_Data_t * const data,
                                         i3c_DataCnt_t dataCnt )
{
    i3c_RequestState_t     retState  = I3C_REQUEST_ERROR;
    i3c_Context_t *        ctx       = I3C_NULL_PTR;
    const i3c_CccInfo_t *  cccInfo   = I3C_NULL_PTR;
    i3c_RequestState_t     infoState = I3cPriv_Get_CccInfo( ccc, &cccInfo );
    i3c_RequestState_t     dataState = I3cPriv_Check_Data( data, dataCnt );

    if( ( I3C_REQUEST_OK == infoState                   ) &&
        ( I3C_REQUEST_OK == dataState                   ) &&
        ( 0u == ( (uint32_t)ccc & I3C_CCC_DIRECT_BIT )  )    )
    {
        retState = I3cPriv_Get_Controller( periphId, &ctx );
    }
    else
    {
        /* The CCC is not a broadcast CCC of the module */
        retState = I3C_REQUEST_ERROR;
    }

    if( I3C_REQUEST_OK == retState )
    {
        if( ( cccInfo->MinCnt <= dataCnt ) &&
            ( cccInfo->MaxCnt >= dataCnt )    )
        {
            retState = I3cCtrl_Start_Ccc( periphId, ccc, 0u, (i3c_Data_t *)data, dataCnt, I3C_DIRECTION_WRITE );
        }
        else
        {
            /* The data does not fit the CCC */
            retState = I3C_REQUEST_ERROR;
        }
    }
    else
    {
        /* The instance is not an idle controller, no action required */
    }

    return ( retState );
}


/**
 * \brief Sends a direct CCC to a device of the device table (read or write of the CCC).
 *
 * The device is addressed by its dynamic address (SETDASA: by its static address). The first
 * data byte of RSTACT (direct) is its defining byte, the byte read of the read RSTACT replaces it.
 *
 * \param periphId  [in]: I3C peripheral ID
 * \param ccc       [in]: Direct CCC (bit 7 of the code is set)
 * \param address   [in]: Address of the device
 * \param data      [in/out]: Data of the CCC (written or read)
 * \param dataCnt   [in]: Count of the bytes of the CCC
 * \param direction [in]: Write or read
 * \return I3C_REQUEST_OK when the request is started (the callbacks report the end), I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3c_Set_DirectCcc( i3c_PeriphId_t periphId,
                                      i3c_Ccc_t ccc,
                                      i3c_Address_t address,
                                      i3c_Data_t * const data,
                                      i3c_DataCnt_t dataCnt,
                                      i3c_Direction_t direction )
{
    i3c_RequestState_t    retState  = I3C_REQUEST_ERROR;
    i3c_Context_t *       ctx       = I3C_NULL_PTR;
    i3c_Device_t *        device    = I3C_NULL_PTR;
    const i3c_CccInfo_t * cccInfo   = I3C_NULL_PTR;
    i3c_RequestState_t    infoState = I3cPriv_Get_CccInfo( ccc, &cccInfo );
    uint8_t               dirBit    = I3C_CCC_DIR_WRITE;

    if( I3C_DIRECTION_READ == direction )
    {
        dirBit = I3C_CCC_DIR_READ;
    }
    else
    {
        /* The write direction is selected, no action required */
    }

    if( ( I3C_REQUEST_OK == infoState                        ) &&
        ( I3C_DIRECTION_CNT > direction                      ) &&
        ( 0u != ( (uint32_t)ccc & I3C_CCC_DIRECT_BIT )       ) &&
        ( I3C_NULL_PTR != data                               ) &&
        ( cccInfo->MinCnt <= dataCnt                         ) &&
        ( cccInfo->MaxCnt >= dataCnt                         ) &&
        ( 0u != ( cccInfo->DirMask & dirBit )                )    )
    {
        retState = I3cPriv_Get_Controller( periphId, &ctx );
    }
    else
    {
        /* The CCC, its data or its direction is not valid */
        retState = I3C_REQUEST_ERROR;
    }

    if( I3C_REQUEST_OK == retState )
    {
        retState = I3cPriv_Get_Device( ctx, address, &device );
    }
    else
    {
        /* The instance is not an idle controller or the parameters are not valid, no action required */
    }

    if( ( I3C_REQUEST_OK == retState        ) &&
        ( I3C_DEVICE_I3C == device->Type    )    )
    {
        retState = I3cCtrl_Start_Ccc( periphId, ccc, address, data, dataCnt, direction );
    }
    else
    {
        /* The device is not in the table, it is a legacy I2C device or a previous check failed */
        retState = I3C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Enables or disables the acceptance of the in-band interrupts of an I3C device of the
 *        device table (the acknowledge of its in-band interrupt request).
 *
 * \param periphId [in]: I3C peripheral ID
 * \param address  [in]: Dynamic address of the device
 * \param ibiState [in]: In-band interrupts of the device are accepted
 * \return I3C_REQUEST_OK when the state was set, I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3c_Set_IbiConfig( i3c_PeriphId_t periphId, i3c_Address_t address, i3c_FunctionState_t ibiState )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    i3c_Context_t *    ctx      = I3C_NULL_PTR;
    i3c_Device_t *     device   = I3C_NULL_PTR;

    if( ( I3C_FUNCTION_ACTIVE == ibiState   ) ||
        ( I3C_FUNCTION_INACTIVE == ibiState )    )
    {
        retState = I3cPriv_Get_Controller( periphId, &ctx );
    }
    else
    {
        /* Invalid state */
        retState = I3C_REQUEST_ERROR;
    }

    if( I3C_REQUEST_OK == retState )
    {
        retState = I3cPriv_Get_Device( ctx, address, &device );
    }
    else
    {
        /* The instance is not an idle controller, no action required */
    }

    if( ( I3C_REQUEST_OK == retState        ) &&
        ( I3C_DEVICE_I3C == device->Type    )    )
    {
        device->IbiState = ibiState;
        retState         = I3c_Set_DeviceRegs( periphId );
    }
    else
    {
        /* The device is not in the table or it is a legacy I2C device */
        retState = I3C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Accepts the hot-join requests of the targets (the hot-join callback reports them).
 *
 * \param periphId [in]: I3C peripheral ID
 * \return I3C_REQUEST_OK when the hot-join is accepted, I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3c_Set_HotJoinActive( i3c_PeriphId_t periphId )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    i3c_Context_t *    ctx      = I3C_NULL_PTR;

    retState = I3cPriv_Get_Controller( periphId, &ctx );

    if( I3C_REQUEST_OK == retState )
    {
        I3c_Reg_Set_Bit( periphId, I3C_REG_CFGR, I3C_CFGR_HJACK );
        ctx->HotJoinState = I3C_FUNCTION_ACTIVE;
    }
    else
    {
        /* The instance is not an idle controller, no action required */
    }

    return ( retState );
}


/**
 * \brief Rejects the hot-join requests of the targets.
 *
 * \param periphId [in]: I3C peripheral ID
 * \return I3C_REQUEST_OK when the hot-join is rejected, I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3c_Set_HotJoinInactive( i3c_PeriphId_t periphId )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    i3c_Context_t *    ctx      = I3C_NULL_PTR;

    retState = I3cPriv_Get_Controller( periphId, &ctx );

    if( I3C_REQUEST_OK == retState )
    {
        I3c_Reg_Reset_Bit( periphId, I3C_REG_CFGR, I3C_CFGR_HJACK );
        ctx->HotJoinState = I3C_FUNCTION_INACTIVE;
    }
    else
    {
        /* The instance is not an idle controller, no action required */
    }

    return ( retState );
}


/* ===================== SERVICES FOR THE CONTROLLER ENGINE ================== */

/**
 * \brief Starts a private (I3C device) or legacy (I2C device) message of one or two directions.
 *
 * \param periphId [in]: I3C peripheral ID
 * \param device   [in]: Device of the device table
 * \param txData   [in]: Data to be written (NULL for a read)
 * \param txCnt    [in]: Count of the bytes to be written
 * \param rxData   [in]: Buffer for the data read (NULL for a write)
 * \param rxCnt    [in]: Count of the bytes to be read
 * \return I3C_REQUEST_OK when the frame is started, I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3cCtrl_Start_Message( i3c_PeriphId_t periphId,
                                          const i3c_Device_t * const device,
                                          const i3c_Data_t * const txData,
                                          i3c_DataCnt_t txCnt,
                                          i3c_Data_t * const rxData,
                                          i3c_DataCnt_t rxCnt )
{
    i3c_RequestState_t retState  = I3C_REQUEST_ERROR;
    i3c_CtrlWord_t     words[ I3C_CTRL_WORD_CNT_MAX ] = { 0u, 0u };
    i3c_CtrlWordCnt_t  wordCnt   = 0u;
    uint32_t           msgType   = I3C_MTYPE_PRIVATE;
    i3c_Address_t      address   = 0u;

    if( I3C_NULL_PTR != device )
    {
        if( I3C_DEVICE_I2C == device->Type )
        {
            msgType = I3C_MTYPE_I2C;
            address = device->StaticAddress;
        }
        else
        {
            msgType = I3C_MTYPE_PRIVATE;
            address = device->DynamicAddress;
        }

        if( ( 0u < txCnt ) &&
            ( 0u < rxCnt )    )
        {
            words[ 0u ] = I3cPriv_Get_MsgWord( I3C_FUNCTION_INACTIVE, msgType, address, I3C_DIRECTION_WRITE, txCnt );
            words[ 1u ] = I3cPriv_Get_MsgWord( I3C_FUNCTION_ACTIVE, msgType, address, I3C_DIRECTION_READ, rxCnt );
            wordCnt     = I3C_CTRL_WORD_CNT_MAX;
        }
        else
        {
            if( 0u < txCnt )
            {
                words[ 0u ] = I3cPriv_Get_MsgWord( I3C_FUNCTION_ACTIVE, msgType, address, I3C_DIRECTION_WRITE, txCnt );
            }
            else
            {
                words[ 0u ] = I3cPriv_Get_MsgWord( I3C_FUNCTION_ACTIVE, msgType, address, I3C_DIRECTION_READ, rxCnt );
            }

            wordCnt = 1u;
        }

        retState = I3cCtrl_Start_Frame( periphId, words, wordCnt, txData, txCnt, rxData, rxCnt, I3C_FUNCTION_INACTIVE );
    }
    else
    {
        /* No device */
        retState = I3C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Starts a broadcast or a direct CCC.
 *
 * \param periphId  [in]: I3C peripheral ID
 * \param ccc       [in]: CCC code
 * \param address   [in]: Address of the device (direct CCC)
 * \param data      [in/out]: Data of the CCC
 * \param dataCnt   [in]: Count of the bytes of the CCC
 * \param direction [in]: Write or read (direct CCC)
 * \return I3C_REQUEST_OK when the frame is started, I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3cCtrl_Start_Ccc( i3c_PeriphId_t periphId,
                                      i3c_Ccc_t ccc,
                                      i3c_Address_t address,
                                      i3c_Data_t * const data,
                                      i3c_DataCnt_t dataCnt,
                                      i3c_Direction_t direction )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    i3c_CtrlWord_t     words[ I3C_CTRL_WORD_CNT_MAX ] = { 0u, 0u };
    i3c_CtrlWordCnt_t  wordCnt  = 1u;
    i3c_DataCnt_t      firstCnt = 0u;
    i3c_DataCnt_t      txCnt    = 0u;
    i3c_DataCnt_t      rxCnt    = 0u;
    i3c_DataCnt_t      msgCnt   = dataCnt;

    if( 0u == ( (uint32_t)ccc & I3C_CCC_DIRECT_BIT ) )
    {
        /* Broadcast CCC: one control word, the data bytes are written */
        words[ 0u ] = I3cPriv_Get_CccWord( I3C_FUNCTION_ACTIVE, ccc, dataCnt );
        txCnt       = dataCnt;
    }
    else
    {
        /* Direct CCC: the first part holds the CCC code, the second part the address and the data */
        if( I3C_CCC_RSTACT_DIRECT == ccc )
        {
            firstCnt = I3C_CCC_RSTACT_DEFINING_CNT;
            txCnt    = I3C_CCC_RSTACT_DEFINING_CNT;
            msgCnt   = 0u;

            if( I3C_DIRECTION_READ == direction )
            {
                msgCnt = I3C_CCC_RSTACT_DEFINING_CNT;
                rxCnt  = I3C_CCC_RSTACT_DEFINING_CNT;
            }
            else
            {
                /* The write RSTACT has no data in the second part, no action required */
            }
        }
        else
        {
            if( I3C_DIRECTION_READ == direction )
            {
                rxCnt = dataCnt;
            }
            else
            {
                txCnt = dataCnt;
            }
        }

        words[ 0u ] = I3cPriv_Get_CccWord( I3C_FUNCTION_INACTIVE, ccc, firstCnt );
        words[ 1u ] = I3cPriv_Get_MsgWord( I3C_FUNCTION_ACTIVE, I3C_MTYPE_DIRECT, address, direction, msgCnt );
        wordCnt     = I3C_CTRL_WORD_CNT_MAX;
    }

    retState = I3cCtrl_Start_Frame( periphId, words, wordCnt, data, txCnt, data, rxCnt, I3C_FUNCTION_INACTIVE );

    return ( retState );
}


/**
 * \brief Serves the requests started by the targets: the in-band interrupt (the address of the
 *        device and its payload are reported), the hot-join request and the controller-role
 *        request (not acknowledged by the module, the flag is only cleared).
 *
 * \param periphId [in]: I3C peripheral ID
 * \param events   [in]: Event register value
 */
void I3cCtrl_Handle_Requests( i3c_PeriphId_t periphId, i3c_EventMask_t events )
{
    i3c_Context_t *    ctx        = I3C_NULL_PTR;
    i3c_RequestState_t ctxState   = I3c_Get_Context( periphId, &ctx );
    i3c_Data_t         payload[ I3C_IBI_PAYLOAD_CNT_MAX ] = { 0u, 0u, 0u, 0u };
    i3c_Device_t *     device     = I3C_NULL_PTR;
    uint32_t           rmrVal     = 0u;
    uint32_t           ibiVal     = 0u;
    i3c_Address_t      address    = 0u;
    i3c_DataCnt_t      payloadCnt = 0u;
    i3c_RequestState_t devState   = I3C_REQUEST_ERROR;

    if( ( I3C_REQUEST_OK == ctxState ) &&
        ( I3C_NULL_PTR   != ctx      )    )
    {
        if( 0u != ( events & I3C_EVR_IBIF ) )
        {
            rmrVal     = I3c_Reg_Get_Val( periphId, I3C_REG_RMR, I3C_REG_MASK_ALL );
            ibiVal     = I3c_Reg_Get_Val( periphId, I3C_REG_IBIDR, I3C_REG_MASK_ALL );
            address    = (i3c_Address_t)( ( rmrVal & I3C_RMR_RADD_Msk ) >> I3C_RMR_RADD_Pos );
            payloadCnt = (i3c_DataCnt_t)( ( rmrVal & I3C_RMR_IBIRDCNT_Msk ) >> I3C_RMR_IBIRDCNT_Pos );

            if( I3C_IBI_PAYLOAD_CNT_MAX < payloadCnt )
            {
                payloadCnt = I3C_IBI_PAYLOAD_CNT_MAX;
            }
            else
            {
                /* The payload fits the buffer, no action required */
            }

            for( uint32_t idx = 0u; (uint32_t)I3C_IBI_PAYLOAD_CNT_MAX > idx; idx++ )
            {
                payload[ idx ] = (i3c_Data_t)( ( ibiVal >> ( idx * I3C_BYTE_BIT_CNT ) ) & I3C_IBI_BYTE_MASK );
            }

            I3c_Reg_Set_EventsClear( periphId, I3C_EVR_IBIF );
            devState = I3c_Get_Address( ctx, address, &device );

            if( ( I3C_REQUEST_OK == devState                     ) &&
                ( I3C_NULL_PTR != ctx->Config.IbiCallback        )    )
            {
                ctx->Config.IbiCallback( periphId, address, payload, payloadCnt );
            }
            else
            {
                /* The device is not in the table or no callback is set, no action required */
            }
        }
        else
        {
            /* No in-band interrupt, no action required */
        }

        if( 0u != ( events & I3C_EVR_HJF ) )
        {
            I3c_Reg_Set_EventsClear( periphId, I3C_EVR_HJF );

            if( I3C_NULL_PTR != ctx->Config.HotJoinCallback )
            {
                ctx->Config.HotJoinCallback( periphId );
            }
            else
            {
                /* No hot-join callback is set, no action required */
            }
        }
        else
        {
            /* No hot-join request, no action required */
        }

        if( 0u != ( events & I3C_EVR_CRF ) )
        {
            I3c_Reg_Set_EventsClear( periphId, I3C_EVR_CRF );
        }
        else
        {
            /* No controller-role request, no action required */
        }
    }
    else
    {
        /* Invalid peripheral ID, no action required */
    }
}


/* ============================ LOCAL FUNCTIONS ============================== */

/**
 * \brief Gets the context of an instance that is initialized as controller and has no running
 *        request.
 *
 * \param periphId [in]: I3C peripheral ID
 * \param ctx      [out]: Context of the instance
 * \return I3C_REQUEST_OK when the instance is an idle controller, I3C_REQUEST_ERROR otherwise
 */
static i3c_RequestState_t I3cPriv_Get_Controller( i3c_PeriphId_t periphId, i3c_Context_t ** const ctx )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;
    i3c_RequestState_t ctxState = I3c_Get_Context( periphId, ctx );

    if( I3C_REQUEST_OK == ctxState )
    {
        if( ( I3C_FUNCTION_ACTIVE == (*ctx)->InitState   ) &&
            ( I3C_ROLE_CONTROLLER == (*ctx)->Config.Role ) &&
            ( I3C_XFER_STATE_BUSY != (*ctx)->XferState   )    )
        {
            retState = I3C_REQUEST_OK;
        }
        else
        {
            /* The instance is not initialized as controller or a request is running */
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
 * \brief Finds the device of the device table that a request addresses.
 *
 * \param ctx     [in]: Context of the instance
 * \param address [in]: Dynamic address (I3C device) or static address (I2C device)
 * \param device  [out]: Entry of the device table
 * \return I3C_REQUEST_OK when the device is in the table, I3C_REQUEST_ERROR otherwise
 */
static i3c_RequestState_t I3cPriv_Get_Device( i3c_Context_t * const ctx,
                                              i3c_Address_t address,
                                              i3c_Device_t ** const device )
{
    i3c_RequestState_t retState = I3c_Get_Address( ctx, address, device );
    i3c_Device_t *     found    = I3C_NULL_PTR;

    if( ( I3C_REQUEST_OK != retState ) &&
        ( I3C_NULL_PTR != ctx )    )
    {
        /* SETDASA addresses the device by its static address: an I3C device without a dynamic address */
        for( i3c_DeviceCnt_t idx = 0u;
             ( ctx->DeviceCnt > idx ) &&
             ( I3C_NULL_PTR == found );
             idx++ )
        {
            if( ( I3C_DEVICE_I3C == ctx->Devices[ idx ].Type              ) &&
                ( I3C_FUNCTION_INACTIVE == ctx->Devices[ idx ].AddressState ) &&
                ( address == ctx->Devices[ idx ].StaticAddress            )    )
            {
                found = &ctx->Devices[ idx ];
            }
            else
            {
                /* Another device, no action required */
            }
        }

        if( I3C_NULL_PTR != found )
        {
            *device  = found;
            retState = I3C_REQUEST_OK;
        }
        else
        {
            /* The device is not in the table */
            retState = I3C_REQUEST_ERROR;
        }
    }
    else
    {
        /* The device was found by its address or the context is not valid, no action required */
    }

    return ( retState );
}


/**
 * \brief Finds the properties of a CCC offered by the module.
 *
 * \param ccc     [in]: CCC code
 * \param cccInfo [out]: Properties of the CCC
 * \return I3C_REQUEST_OK when the CCC is offered, I3C_REQUEST_ERROR otherwise
 */
static i3c_RequestState_t I3cPriv_Get_CccInfo( i3c_Ccc_t ccc, const i3c_CccInfo_t ** const cccInfo )
{
    i3c_RequestState_t retState = I3C_REQUEST_ERROR;

    for( uint32_t idx = 0u;
         ( ( sizeof( i3c_CccInfoList ) / sizeof( i3c_CccInfoList[ 0u ] ) ) > idx ) &&
         ( I3C_REQUEST_OK != retState );
         idx++ )
    {
        if( ccc == i3c_CccInfoList[ idx ].Ccc )
        {
            *cccInfo = &i3c_CccInfoList[ idx ];
            retState = I3C_REQUEST_OK;
        }
        else
        {
            /* Another CCC, no action required */
        }
    }

    return ( retState );
}


/**
 * \brief Builds the control word of a private, direct or legacy I2C message.
 *
 * \param lastMsg   [in]: The message is the last of the frame (ends with a stop)
 * \param msgType   [in]: Message type (private, direct, legacy I2C)
 * \param address   [in]: 7-bit address
 * \param direction [in]: Write or read
 * \param dataCnt   [in]: Count of the data bytes
 * \return Control word
 */
static i3c_CtrlWord_t I3cPriv_Get_MsgWord( i3c_FunctionState_t lastMsg,
                                           uint32_t msgType,
                                           i3c_Address_t address,
                                           i3c_Direction_t direction,
                                           i3c_DataCnt_t dataCnt )
{
    i3c_CtrlWord_t word = ( ( msgType << I3C_CR_MTYPE_Pos ) |
                            ( (i3c_CtrlWord_t)address << I3C_CR_ADD_Pos ) |
                            ( (i3c_CtrlWord_t)dataCnt << I3C_CR_DCNT_Pos ) );

    if( I3C_FUNCTION_ACTIVE == lastMsg )
    {
        word |= I3C_CR_MEND;
    }
    else
    {
        /* A repeated start follows the message, no action required */
    }

    if( I3C_DIRECTION_READ == direction )
    {
        word |= I3C_CR_RNW;
    }
    else
    {
        /* Write message, no action required */
    }

    return ( word );
}


/**
 * \brief Builds the control word of a broadcast CCC or of the first part of a direct CCC.
 *
 * \param lastMsg [in]: The message is the last of the frame (broadcast CCC)
 * \param ccc     [in]: CCC code
 * \param dataCnt [in]: Count of the bytes of the CCC (defining bytes)
 * \return Control word
 */
static i3c_CtrlWord_t I3cPriv_Get_CccWord( i3c_FunctionState_t lastMsg, i3c_Ccc_t ccc, i3c_DataCnt_t dataCnt )
{
    i3c_CtrlWord_t word = ( ( (i3c_CtrlWord_t)I3C_MTYPE_CCC << I3C_CR_MTYPE_Pos ) |
                            ( (i3c_CtrlWord_t)ccc << I3C_CR_CCC_Pos ) |
                            ( (i3c_CtrlWord_t)dataCnt << I3C_CR_DCNT_Pos ) );

    if( I3C_FUNCTION_ACTIVE == lastMsg )
    {
        word |= I3C_CR_MEND;
    }
    else
    {
        /* The second part of a direct CCC follows, no action required */
    }

    return ( word );
}


/**
 * \brief Checks that a buffer is provided for a non-empty data of a CCC.
 *
 * \param data    [in]: Data of the CCC
 * \param dataCnt [in]: Count of the bytes of the CCC
 * \return I3C_REQUEST_OK when the buffer matches the count, I3C_REQUEST_ERROR otherwise
 */
static i3c_RequestState_t I3cPriv_Check_Data( const i3c_Data_t * const data, i3c_DataCnt_t dataCnt )
{
    i3c_RequestState_t retState = I3C_REQUEST_OK;

    if( ( 0u < dataCnt          ) &&
        ( I3C_NULL_PTR == data  )    )
    {
        retState = I3C_REQUEST_ERROR;
    }
    else
    {
        /* The buffer matches its count, no action required */
    }

    return ( retState );
}
