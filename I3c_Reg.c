/**
 * \author Mr.Nobody
 * \file I3c_Reg.c
 * \ingroup I3c
 * \brief I3c module Reg component functionality.
 *
 * Register access of the module: generic accessors (bounds checked instance and register ID) and
 * the register protocol of the I3C peripheral. The registers are written only here, the masks
 * come from the CMSIS device header. The address of a register is the base address of the
 * instance plus the offset of the register in the CMSIS type I3C_TypeDef.
 *
 * Properties of the registers, which the protocol respects (RM0481 / RM0492 / RM0517, I3C chapter):
 * - I3C_CR, I3C_TDR and I3C_CEVR are write only (a control word / a data byte is pushed into the
 *   FIFO, a flag is cleared by writing 1), they are never written by a read-modify-write.
 * - The flush bits of I3C_CFGR are write only and self-clearing.
 * - I3C_TIMINGR0, I3C_TIMINGR1, I3C_I2CCFGR, I3C_EPIDR and the CRINIT / HKSDAEN bits can be
 *   written only while I3C_CFGR.EN is 0.
 * - The I3C_I2CCFGR register (offset 0x98) is not described by the CMSIS type, it exists on I3C1
 *   of the STM32H523 .. H573 lines only.
 *
 */
/* ============================== INCLUDES ================================== */
#include "I3c_Reg.h"                        /* Self include                   */
#include "I3c_Types.h"                      /* Module types definitions       */
#include "I3c.h"                            /* Module private definitions     */
#include <stddef.h>                         /* offsetof                       */
/* ============================== TYPEDEFS ================================== */

/** \brief Structure type used to store the offset of an I3C register from the instance base address.
 * This array is used to reduce size of configuration array. With this style
 * the registers are referenced through enumeration.
 */
typedef struct
{
    i3c_RegId_t RegId;     /**< Peripheral register ID                      */
    uint32_t    RegOffset; /**< Offset of the register from the base address */
}   i3c_RegList_t;

/* ======================== FORWARD DECLARATIONS ============================ */

static volatile uint32_t * I3cPriv_Get_RegPtr ( i3c_PeriphId_t instId, i3c_RegId_t regId );

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** \brief Register value without any bit set (masked register value of an inactive bit field) */
#define I3C_REG_BITS_CLEAR                    ( 0u )

/** \brief Offset of the DEVRX array (target device registers) from the instance base address */
#define I3C_REG_DEVRX_OFFSET                  ( offsetof( I3C_TypeDef, DEVRX ) )

/** \brief Size of an element of the DEVRX array of the target device registers (4 bytes per register) */
#define I3C_DEVRX_SIZE            ( 4u )

/** \brief Index of the third and the fourth element of the DEVRX array */
#define I3C_DEVRX_IDX_3                   ( 2u )
#define I3C_DEVRX_IDX_4                   ( 3u )

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** \brief Base addresses of the I3C instances */
static I3C_TypeDef * const i3c_RegBaseList[] =
{
    I3C1,    /**< I3C1 */
#if defined(I3C2)
    I3C2,    /**< I3C2 */
#endif
};

_Static_assert( ( sizeof( i3c_RegBaseList ) / sizeof( i3c_RegBaseList[ 0u ] ) ) == I3C_PERIPH_CNT,
                "I3c_Reg: i3c_RegBaseList has incorrect size." );

/** \brief I3C registers configuration structure
 *
 * This structure is used to store the offsets of the I3C registers.
 * It is used to reduce code size while referring to the I3C registers.
 */
static const i3c_RegList_t i3c_RegList[] =
{
    { .RegId = I3C_REG_CR      , .RegOffset = offsetof( I3C_TypeDef, CR       ) }, /**< Control word          */
    { .RegId = I3C_REG_CFGR    , .RegOffset = offsetof( I3C_TypeDef, CFGR     ) }, /**< Configuration         */
    { .RegId = I3C_REG_RDR     , .RegOffset = offsetof( I3C_TypeDef, RDR      ) }, /**< Receive data          */
    { .RegId = I3C_REG_TDR     , .RegOffset = offsetof( I3C_TypeDef, TDR      ) }, /**< Transmit data         */
    { .RegId = I3C_REG_IBIDR   , .RegOffset = offsetof( I3C_TypeDef, IBIDR    ) }, /**< IBI payload           */
    { .RegId = I3C_REG_TGTTDR  , .RegOffset = offsetof( I3C_TypeDef, TGTTDR   ) }, /**< Target transmit       */
    { .RegId = I3C_REG_SR      , .RegOffset = offsetof( I3C_TypeDef, SR       ) }, /**< Status                */
    { .RegId = I3C_REG_SER     , .RegOffset = offsetof( I3C_TypeDef, SER      ) }, /**< Status error          */
    { .RegId = I3C_REG_RMR     , .RegOffset = offsetof( I3C_TypeDef, RMR      ) }, /**< Received message      */
    { .RegId = I3C_REG_EVR     , .RegOffset = offsetof( I3C_TypeDef, EVR      ) }, /**< Event                 */
    { .RegId = I3C_REG_IER     , .RegOffset = offsetof( I3C_TypeDef, IER      ) }, /**< Interrupt enable      */
    { .RegId = I3C_REG_CEVR    , .RegOffset = offsetof( I3C_TypeDef, CEVR     ) }, /**< Clear event           */
    { .RegId = I3C_REG_DEVR0   , .RegOffset = offsetof( I3C_TypeDef, DEVR0    ) }, /**< Own device            */
    { .RegId = I3C_REG_DEVR1   , .RegOffset = I3C_REG_DEVRX_OFFSET }, /**< Device 1              */
    { .RegId = I3C_REG_DEVR2   , .RegOffset = I3C_REG_DEVRX_OFFSET + ( 1u * I3C_DEVRX_SIZE ) },
    { .RegId = I3C_REG_DEVR3   , .RegOffset = I3C_REG_DEVRX_OFFSET + ( I3C_DEVRX_IDX_3 * I3C_DEVRX_SIZE ) },
    { .RegId = I3C_REG_DEVR4   , .RegOffset = I3C_REG_DEVRX_OFFSET + ( I3C_DEVRX_IDX_4 * I3C_DEVRX_SIZE ) },
    { .RegId = I3C_REG_MAXRLR  , .RegOffset = offsetof( I3C_TypeDef, MAXRLR   ) }, /**< Maximum read length   */
    { .RegId = I3C_REG_MAXWLR  , .RegOffset = offsetof( I3C_TypeDef, MAXWLR   ) }, /**< Maximum write length  */
#if defined(STM32H523xx) || \
    defined(STM32H533xx) || \
    defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H562xx) || \
    defined(STM32H563xx) || \
    defined(STM32H573xx)
    { .RegId = I3C_REG_I2CCFGR , .RegOffset = I3C_REG_I2CCFGR_OFFSET            }, /**< I2C target config     */
#endif
    { .RegId = I3C_REG_TIMINGR0, .RegOffset = offsetof( I3C_TypeDef, TIMINGR0 ) }, /**< Timing 0              */
    { .RegId = I3C_REG_TIMINGR1, .RegOffset = offsetof( I3C_TypeDef, TIMINGR1 ) }, /**< Timing 1              */
    { .RegId = I3C_REG_TIMINGR2, .RegOffset = offsetof( I3C_TypeDef, TIMINGR2 ) }, /**< Timing 2              */
    { .RegId = I3C_REG_BCR     , .RegOffset = offsetof( I3C_TypeDef, BCR      ) }, /**< Bus characteristics   */
    { .RegId = I3C_REG_DCR     , .RegOffset = offsetof( I3C_TypeDef, DCR      ) }, /**< Device characteristics */
    { .RegId = I3C_REG_GETCAPR , .RegOffset = offsetof( I3C_TypeDef, GETCAPR  ) }, /**< Get capability        */
    { .RegId = I3C_REG_CRCAPR  , .RegOffset = offsetof( I3C_TypeDef, CRCAPR   ) }, /**< Controller capability */
    { .RegId = I3C_REG_GETMXDSR, .RegOffset = offsetof( I3C_TypeDef, GETMXDSR ) }, /**< Get max data speed    */
    { .RegId = I3C_REG_EPIDR   , .RegOffset = offsetof( I3C_TypeDef, EPIDR    ) }, /**< Extended provisioned ID */
};

_Static_assert( ( sizeof( i3c_RegList ) / sizeof( i3c_RegList_t ) ) == I3C_REG_CNT,
                "I3c_Reg: i3c_RegList has incorrect size." );

/* The clear register and the interrupt enable register use the bit positions of the event register */
_Static_assert( ( I3C_EVR_FCF_Msk       == I3C_CEVR_CFCF_Msk       ) &&
                ( I3C_EVR_ERRF_Msk      == I3C_CEVR_CERRF_Msk      ) &&
                ( I3C_EVR_IBIF_Msk      == I3C_CEVR_CIBIF_Msk      ) &&
                ( I3C_EVR_HJF_Msk       == I3C_CEVR_CHJF_Msk       ) &&
                ( I3C_EVR_RXTGTENDF_Msk == I3C_CEVR_CRXTGTENDF_Msk ) &&
                ( I3C_EVR_DAUPDF_Msk    == I3C_CEVR_CDAUPDF_Msk    ) &&
                ( I3C_EVR_FCF_Msk       == I3C_IER_FCIE_Msk        ) &&
                ( I3C_EVR_ERRF_Msk      == I3C_IER_ERRIE_Msk       ) &&
                ( I3C_EVR_TXFNFF_Msk    == I3C_IER_TXFNFIE_Msk     ) &&
                ( I3C_EVR_RXFNEF_Msk    == I3C_IER_RXFNEIE_Msk     ),
                "I3c_Reg: event, clear and interrupt enable registers do not share the bit positions." );

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Sets the register bit with given mask.
 *
 * \param instId  [in]: I3C instance ID
 * \param regId   [in]: I3C register ID
 * \param bitMask [in]: Mask to apply to the register
 */
void I3c_Reg_Set_Bit( i3c_PeriphId_t instId, i3c_RegId_t regId, uint32_t bitMask )
{
    volatile uint32_t * const regPtr = I3cPriv_Get_RegPtr( instId, regId );

    if( I3C_NULL_PTR != regPtr )
    {
        *regPtr |= bitMask;
    }
    else
    {
        /* Invalid instance or register ID, no action required */
    }
}


/**
 * \brief Resets the register bit with given mask.
 *
 * \param instId  [in]: I3C instance ID
 * \param regId   [in]: I3C register ID
 * \param bitMask [in]: Mask to apply to the register
 */
void I3c_Reg_Reset_Bit( i3c_PeriphId_t instId, i3c_RegId_t regId, uint32_t bitMask )
{
    volatile uint32_t * const regPtr = I3cPriv_Get_RegPtr( instId, regId );

    if( I3C_NULL_PTR != regPtr )
    {
        *regPtr &= ~bitMask;
    }
    else
    {
        /* Invalid instance or register ID, no action required */
    }
}


/**
 * \brief Gets the register bit value with given mask.
 *
 * \param instId  [in]: I3C instance ID
 * \param regId   [in]: I3C register ID
 * \param bitMask [in]: Mask to apply to the register
 * \return Register bit value with applied mask, no bit set for an invalid instance or register ID
 */
uint32_t I3c_Reg_Get_Bit( i3c_PeriphId_t instId, i3c_RegId_t regId, uint32_t bitMask )
{
    const volatile uint32_t * const regPtr   = I3cPriv_Get_RegPtr( instId, regId );
    uint32_t                        retValue = I3C_REG_BITS_CLEAR;

    if( I3C_NULL_PTR != regPtr )
    {
        retValue = ( *regPtr & bitMask );
    }
    else
    {
        /* Invalid instance or register ID, no bit is set */
        retValue = I3C_REG_BITS_CLEAR;
    }

    return ( retValue );
}


/**
 * \brief Updates the register value with given mask and value.
 *
 * \param instId   [in]: I3C instance ID
 * \param regId    [in]: I3C register ID
 * \param regMask  [in]: Mask to apply to the register
 * \param regValue [in]: Value to set in the register
 */
void I3c_Reg_Set_Val( i3c_PeriphId_t instId, i3c_RegId_t regId, uint32_t regMask, uint32_t regValue )
{
    volatile uint32_t * const regPtr = I3cPriv_Get_RegPtr( instId, regId );

    if( I3C_NULL_PTR != regPtr )
    {
        *regPtr = ( ( *regPtr & ~regMask ) | ( regValue & regMask ) );
    }
    else
    {
        /* Invalid instance or register ID, no action required */
    }
}


/**
 * \brief Gets the register value with given mask.
 *
 * \param instId  [in]: I3C instance ID
 * \param regId   [in]: I3C register ID
 * \param regMask [in]: Mask to apply to the register
 * \return Register value with applied mask, no bit set for an invalid instance or register ID
 */
uint32_t I3c_Reg_Get_Val( i3c_PeriphId_t instId, i3c_RegId_t regId, uint32_t regMask )
{
    return ( I3c_Reg_Get_Bit( instId, regId, regMask ) );
}


/**
 * \brief Writes the whole register (no read-modify-write), used for the write only registers.
 *
 * \param instId   [in]: I3C instance ID
 * \param regId    [in]: I3C register ID
 * \param regValue [in]: Value to write into the register
 */
void I3c_Reg_Set_Raw( i3c_PeriphId_t instId, i3c_RegId_t regId, uint32_t regValue )
{
    volatile uint32_t * const regPtr = I3cPriv_Get_RegPtr( instId, regId );

    if( I3C_NULL_PTR != regPtr )
    {
        *regPtr = regValue;
    }
    else
    {
        /* Invalid instance or register ID, no action required */
    }
}


/**
 * \brief Returns the address of a register, used as the Gpdma source or destination address.
 *
 * \param instId [in]: I3C instance ID
 * \param regId  [in]: I3C register ID
 * \return Address of the register, zero for an invalid instance or register ID
 */
uint32_t I3c_Reg_Get_Addr( i3c_PeriphId_t instId, i3c_RegId_t regId )
{
    const volatile uint32_t * const regPtr   = I3cPriv_Get_RegPtr( instId, regId );
    uint32_t                        retValue = 0u;

    if( I3C_NULL_PTR != regPtr )
    {
        retValue = (uint32_t)regPtr;
    }
    else
    {
        retValue = 0u;
    }

    return ( retValue );
}


/**
 * \brief Enables the peripheral (I3C_CFGR.EN) and waits until the bit reads back as set.
 *
 * \param instId [in]: I3C instance ID
 * \return I3C_REQUEST_OK when the peripheral is enabled, I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3c_Reg_Set_Enable( i3c_PeriphId_t instId )
{
    i3c_RequestState_t  retState = I3C_REQUEST_ERROR;
    i3c_FunctionState_t enState  = I3C_FUNCTION_INACTIVE;

    if( I3C_PERIPH_CNT > instId )
    {
        I3c_Reg_Set_Bit( instId, I3C_REG_CFGR, I3C_CFGR_EN );
        enState = I3c_Reg_Get_EnableState( instId );

        for( i3c_TimeoutCnt_t idx = 0u;
             ( I3C_TIMEOUT_RAW > idx ) &&
             ( I3C_FUNCTION_ACTIVE != enState );
             idx++ )
        {
            enState = I3c_Reg_Get_EnableState( instId );
        }

        if( I3C_FUNCTION_ACTIVE == enState )
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
        retState = I3C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Disables the peripheral (I3C_CFGR.EN) and waits until the bit reads back as cleared.
 *
 * \param instId [in]: I3C instance ID
 * \return I3C_REQUEST_OK when the peripheral is disabled, I3C_REQUEST_ERROR otherwise
 */
i3c_RequestState_t I3c_Reg_Set_Disable( i3c_PeriphId_t instId )
{
    i3c_RequestState_t  retState = I3C_REQUEST_ERROR;
    i3c_FunctionState_t enState  = I3C_FUNCTION_ACTIVE;

    if( I3C_PERIPH_CNT > instId )
    {
        I3c_Reg_Reset_Bit( instId, I3C_REG_CFGR, I3C_CFGR_EN );
        enState = I3c_Reg_Get_EnableState( instId );

        for( i3c_TimeoutCnt_t idx = 0u;
             ( I3C_TIMEOUT_RAW > idx ) &&
             ( I3C_FUNCTION_INACTIVE != enState );
             idx++ )
        {
            enState = I3c_Reg_Get_EnableState( instId );
        }

        if( I3C_FUNCTION_INACTIVE == enState )
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
        retState = I3C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Gets the state of the peripheral enable bit (I3C_CFGR.EN).
 *
 * \param instId [in]: I3C instance ID
 * \return I3C_FUNCTION_ACTIVE when the peripheral is enabled, I3C_FUNCTION_INACTIVE otherwise
 */
i3c_FunctionState_t I3c_Reg_Get_EnableState( i3c_PeriphId_t instId )
{
    i3c_FunctionState_t retState = I3C_FUNCTION_INACTIVE;
    const uint32_t      enBit    = I3c_Reg_Get_Bit( instId, I3C_REG_CFGR, I3C_CFGR_EN );

    if( I3C_REG_BITS_CLEAR != enBit )
    {
        retState = I3C_FUNCTION_ACTIVE;
    }
    else
    {
        retState = I3C_FUNCTION_INACTIVE;
    }

    return ( retState );
}


/**
 * \brief Flushes FIFOs (write only flush bits of I3C_CFGR).
 *
 * \param instId    [in]: I3C instance ID
 * \param flushMask [in]: Mask of the flush bits (CFLUSH, SFLUSH, TXFLUSH, RXFLUSH)
 */
void I3c_Reg_Set_FifoFlush( i3c_PeriphId_t instId, uint32_t flushMask )
{
    I3c_Reg_Set_Bit( instId, I3C_REG_CFGR, flushMask );
}


/**
 * \brief Gets the event register (I3C_EVR).
 *
 * \param instId [in]: I3C instance ID
 * \return Value of the event register, zero for an invalid instance ID
 */
uint32_t I3c_Reg_Get_Events( i3c_PeriphId_t instId )
{
    return ( I3c_Reg_Get_Bit( instId, I3C_REG_EVR, I3C_REG_MASK_ALL ) );
}


/**
 * \brief Clears event flags (write one to the matching bits of I3C_CEVR).
 *
 * \param instId    [in]: I3C instance ID
 * \param eventMask [in]: Mask of the event flags to clear (bit positions of I3C_EVR)
 */
void I3c_Reg_Set_EventsClear( i3c_PeriphId_t instId, uint32_t eventMask )
{
    I3c_Reg_Set_Raw( instId, I3C_REG_CEVR, eventMask );
}


/**
 * \brief Pushes a control word into the C-FIFO (I3C_CR).
 *
 * \param instId   [in]: I3C instance ID
 * \param ctrlWord [in]: Control word
 */
void I3c_Reg_Set_ControlWord( i3c_PeriphId_t instId, uint32_t ctrlWord )
{
    I3c_Reg_Set_Raw( instId, I3C_REG_CR, ctrlWord );
}


/**
 * \brief Pushes a data byte into the TX-FIFO (I3C_TDR).
 *
 * \param instId [in]: I3C instance ID
 * \param txByte [in]: Data byte
 */
void I3c_Reg_Set_TxByte( i3c_PeriphId_t instId, uint8_t txByte )
{
    I3c_Reg_Set_Raw( instId, I3C_REG_TDR, (uint32_t)txByte );
}


/**
 * \brief Pops a data byte from the RX-FIFO (I3C_RDR).
 *
 * \param instId [in]: I3C instance ID
 * \return Received data byte, zero for an invalid instance ID
 */
uint8_t I3c_Reg_Get_RxByte( i3c_PeriphId_t instId )
{
    return ( (uint8_t)( I3c_Reg_Get_Bit( instId, I3C_REG_RDR, I3C_RDR_RDB0_Msk ) ) );
}


/**
 * \brief Gets the pointer to a register of an instance.
 *
 * \param instId [in]: I3C instance ID
 * \param regId  [in]: I3C register ID
 * \return Pointer to the register, NULL for an invalid instance ID or register ID (the I2C target
 *         configuration register exists on I3C1 only)
 */
static volatile uint32_t * I3cPriv_Get_RegPtr( i3c_PeriphId_t instId, i3c_RegId_t regId )
{
    volatile uint32_t * retPtr = I3C_NULL_PTR;
    uint32_t            baseAddr = 0u;

    if( ( I3C_PERIPH_CNT > instId ) &&
        ( I3C_REG_CNT    > regId  )    )
    {
        baseAddr = (uint32_t)i3c_RegBaseList[ instId ];
        retPtr   = (volatile uint32_t *)( baseAddr + i3c_RegList[ regId ].RegOffset );
#if defined(STM32H523xx) || \
    defined(STM32H533xx) || \
    defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H562xx) || \
    defined(STM32H563xx) || \
    defined(STM32H573xx)
        if( ( I3C_REG_I2CCFGR == regId ) &&
            ( I3C_PERIPH_1 != instId )    )
        {
            retPtr = I3C_NULL_PTR;
        }
        else
        {
            /* The register exists, no action required */
        }
#endif
    }
    else
    {
        /* Invalid instance or register ID */
        retPtr = I3C_NULL_PTR;
    }

    return ( retPtr );
}
