/**
 * \author Mr.Nobody
 * \file I3c_Reg.h
 * \ingroup I3c
 * \brief I3c module Reg component header file.
 *
 * This component contains functionality for register operations. It is private
 * to the module (the register access is not shared with other modules).
 *
 */

#ifndef I3C_I3C_REG_H
#define I3C_I3C_REG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "I3c_Types.h"                      /* Module types definition        */
/* ============================= TYPEDEFS =================================== */

/** \brief List of I3C registers used by the module (the same list for every instance) */
typedef enum
{
    I3C_REG_CR       = 0u, /**< Message control register (write only, C-FIFO)               */
    I3C_REG_CFGR         , /**< Configuration register                                      */
    I3C_REG_RDR          , /**< Receive data byte register (RX-FIFO)                        */
    I3C_REG_TDR          , /**< Transmit data byte register (TX-FIFO)                       */
    I3C_REG_IBIDR        , /**< IBI payload data register                                   */
    I3C_REG_TGTTDR       , /**< Target transmit configuration register                      */
    I3C_REG_SR           , /**< Status register                                             */
    I3C_REG_SER          , /**< Status error register                                       */
    I3C_REG_RMR          , /**< Received message register                                   */
    I3C_REG_EVR          , /**< Event register                                              */
    I3C_REG_IER          , /**< Interrupt enable register                                   */
    I3C_REG_CEVR         , /**< Clear event register (write only)                           */
    I3C_REG_DEVR0        , /**< Own device characteristics register                         */
    I3C_REG_DEVR1        , /**< Device 1 characteristics register                           */
    I3C_REG_DEVR2        , /**< Device 2 characteristics register                           */
    I3C_REG_DEVR3        , /**< Device 3 characteristics register                           */
    I3C_REG_DEVR4        , /**< Device 4 characteristics register                           */
    I3C_REG_MAXRLR       , /**< Maximum read length register                                */
    I3C_REG_MAXWLR       , /**< Maximum write length register                               */
#if defined(STM32H523xx) || \
    defined(STM32H533xx) || \
    defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H562xx) || \
    defined(STM32H563xx) || \
    defined(STM32H573xx)
    I3C_REG_I2CCFGR      , /**< I2C target configuration register (I3C1 of the RM0481 lines) */
#endif
    I3C_REG_TIMINGR0     , /**< Timing register 0 (SCL high and low durations)              */
    I3C_REG_TIMINGR1     , /**< Timing register 1 (free, available and idle time)           */
    I3C_REG_TIMINGR2     , /**< Timing register 2 (SCL stalling)                            */
    I3C_REG_BCR          , /**< Bus characteristics register (target)                       */
    I3C_REG_DCR          , /**< Device characteristics register (target)                    */
    I3C_REG_GETCAPR      , /**< Get capability register (target)                            */
    I3C_REG_CRCAPR       , /**< Controller-role capability register (target)                */
    I3C_REG_GETMXDSR     , /**< Get maximum data speed register (target)                    */
    I3C_REG_EPIDR        , /**< Extended provisioned ID register (target)                   */
    I3C_REG_CNT            /**< Count of available I3C registers                            */
}   i3c_RegId_t;

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Mask of the whole 32-bit register */
#define I3C_REG_MASK_ALL                      ( 0xFFFFFFFFu )

/** Offset of the I2C target configuration register from the instance base address */
#define I3C_REG_I2CCFGR_OFFSET                ( 0x98u )

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

/* ---------------------------- Generic accessors --------------------------- */

void                I3c_Reg_Set_Bit        ( i3c_PeriphId_t instId, i3c_RegId_t regId, uint32_t bitMask );
void                I3c_Reg_Reset_Bit      ( i3c_PeriphId_t instId, i3c_RegId_t regId, uint32_t bitMask );
uint32_t            I3c_Reg_Get_Bit        ( i3c_PeriphId_t instId, i3c_RegId_t regId, uint32_t bitMask );

void                I3c_Reg_Set_Val        ( i3c_PeriphId_t instId,
                                             i3c_RegId_t regId,
                                             uint32_t regMask,
                                             uint32_t regValue );
uint32_t            I3c_Reg_Get_Val        ( i3c_PeriphId_t instId, i3c_RegId_t regId, uint32_t regMask );

void                I3c_Reg_Set_Raw        ( i3c_PeriphId_t instId, i3c_RegId_t regId, uint32_t regValue );
uint32_t            I3c_Reg_Get_Addr       ( i3c_PeriphId_t instId, i3c_RegId_t regId );

/* ----------------------------- Register protocol -------------------------- */

i3c_RequestState_t  I3c_Reg_Set_Enable     ( i3c_PeriphId_t instId );
i3c_RequestState_t  I3c_Reg_Set_Disable    ( i3c_PeriphId_t instId );
i3c_FunctionState_t I3c_Reg_Get_EnableState( i3c_PeriphId_t instId );

void                I3c_Reg_Set_FifoFlush  ( i3c_PeriphId_t instId, uint32_t flushMask );
uint32_t            I3c_Reg_Get_Events     ( i3c_PeriphId_t instId );
void                I3c_Reg_Set_EventsClear( i3c_PeriphId_t instId, uint32_t eventMask );
void                I3c_Reg_Set_ControlWord( i3c_PeriphId_t instId, uint32_t ctrlWord );
void                I3c_Reg_Set_TxByte     ( i3c_PeriphId_t instId, uint8_t txByte );
uint8_t             I3c_Reg_Get_RxByte     ( i3c_PeriphId_t instId );

#ifdef __cplusplus
}
#endif

#endif /* I3C_I3C_REG_H */
