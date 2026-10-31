/**
 * \author Mr.Nobody
 * \file I3c_Port.h
 * \ingroup I3c
 * \brief Improved Inter-Integrated Circuit (I3C) MCAL module public functionality
 *
 * This file contains all available public functionality, no other files shall
 * be used outside of the module.
 *
 */

#ifndef I3C_I3C_PORT_H
#define I3C_I3C_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================== INCLUDES ================================== */
#include "I3c_Types.h"                      /* Module types definition        */
/* ============================== TYPEDEFS ================================== */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/* ========================== EXPORTED MACROS =============================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* ========================= EXPORTED FUNCTIONS ============================= */

/*------------------------------ Module management ---------------------------*/

i3c_ModuleVersion_t I3c_Get_ModuleVersion        ( void );

i3c_RequestState_t  I3c_Init                     ( const i3c_Config_t * const i3cConfig );
i3c_RequestState_t  I3c_Deinit                   ( i3c_PeriphId_t periphId );
void                I3c_Task                     ( void );

i3c_RequestState_t  I3c_Get_DefaultConfig        ( i3c_Config_t * const i3cConfig );

i3c_RequestState_t  I3c_Get_BusFreq              ( i3c_PeriphId_t periphId,
                                                   i3c_FreqHz_t * const pushPullFreq,
                                                   i3c_FreqHz_t * const openDrainFreq );

/*------------------------------ Bus (controller) ----------------------------*/

i3c_RequestState_t  I3c_Set_BusInit              ( i3c_PeriphId_t periphId );
i3c_RequestState_t  I3c_Set_DynamicAddressAssign ( i3c_PeriphId_t periphId, i3c_DeviceCnt_t * const discoveredCnt );

i3c_RequestState_t  I3c_Get_DeviceTable          ( i3c_PeriphId_t periphId,
                                                   i3c_Device_t * const devices,
                                                   i3c_DeviceCnt_t * const deviceCnt );
i3c_RequestState_t  I3c_Set_DeviceTable          ( i3c_PeriphId_t periphId,
                                                   const i3c_Device_t * const devices,
                                                   i3c_DeviceCnt_t deviceCnt );

/*------------------------------ Transfers (controller) ----------------------*/

i3c_RequestState_t  I3c_Set_Write                ( i3c_PeriphId_t periphId,
                                                   i3c_Address_t address,
                                                   const i3c_Data_t * const data,
                                                   i3c_DataCnt_t dataCnt );
i3c_RequestState_t  I3c_Set_Read                 ( i3c_PeriphId_t periphId,
                                                   i3c_Address_t address,
                                                   i3c_Data_t * const data,
                                                   i3c_DataCnt_t dataCnt );
i3c_RequestState_t  I3c_Set_WriteRead            ( i3c_PeriphId_t periphId,
                                                   i3c_Address_t address,
                                                   const i3c_Data_t * const writeData,
                                                   i3c_DataCnt_t writeCnt,
                                                   i3c_Data_t * const readData,
                                                   i3c_DataCnt_t readCnt );

i3c_RequestState_t  I3c_Set_XferStop             ( i3c_PeriphId_t periphId );
i3c_RequestState_t  I3c_Get_XferState            ( i3c_PeriphId_t periphId, i3c_XferState_t * const xferState );
i3c_RequestState_t  I3c_Get_XferError            ( i3c_PeriphId_t periphId, i3c_XferError_t * const xferError );

/*------------------------------ Commands (controller) -----------------------*/

i3c_RequestState_t  I3c_Set_BroadcastCcc         ( i3c_PeriphId_t periphId,
                                                   i3c_Ccc_t ccc,
                                                   const i3c_Data_t * const data,
                                                   i3c_DataCnt_t dataCnt );
i3c_RequestState_t  I3c_Set_DirectCcc            ( i3c_PeriphId_t periphId,
                                                   i3c_Ccc_t ccc,
                                                   i3c_Address_t address,
                                                   i3c_Data_t * const data,
                                                   i3c_DataCnt_t dataCnt,
                                                   i3c_Direction_t direction );

/*------------------------------ Events (controller) -------------------------*/

i3c_RequestState_t  I3c_Set_IbiConfig            ( i3c_PeriphId_t periphId,
                                                   i3c_Address_t address,
                                                   i3c_FunctionState_t ibiState );
i3c_RequestState_t  I3c_Set_HotJoinActive        ( i3c_PeriphId_t periphId );
i3c_RequestState_t  I3c_Set_HotJoinInactive      ( i3c_PeriphId_t periphId );

/*------------------------------ Target --------------------------------------*/

i3c_RequestState_t  I3c_Set_TargetConfig         ( i3c_PeriphId_t periphId,
                                                   const i3c_TargetConfig_t * const targetConfig );
i3c_RequestState_t  I3c_Set_IbiRequest           ( i3c_PeriphId_t periphId,
                                                   const i3c_Data_t * const payload,
                                                   i3c_DataCnt_t payloadCnt );
i3c_RequestState_t  I3c_Set_HotJoinRequest       ( i3c_PeriphId_t periphId );
i3c_RequestState_t  I3c_Set_TargetReadBuffer     ( i3c_PeriphId_t periphId,
                                                   const i3c_Data_t * const data,
                                                   i3c_DataCnt_t dataCnt );
i3c_RequestState_t  I3c_Set_TargetWriteBuffer    ( i3c_PeriphId_t periphId,
                                                   i3c_Data_t * const data,
                                                   i3c_DataCnt_t dataCnt );
i3c_RequestState_t  I3c_Get_OwnAddress           ( i3c_PeriphId_t periphId, i3c_Address_t * const address );

/*------------------------------ Interrupts ----------------------------------*/

i3c_RequestState_t  I3c_Set_IrqPriority          ( i3c_PeriphId_t periphId, i3c_IrqPrio_t irqPrio );
i3c_RequestState_t  I3c_Get_IrqPriority          ( i3c_PeriphId_t periphId, i3c_IrqPrio_t * const irqPrio );

/*------------------------------ GPIO configuration functionality ------------*/

i3c_RequestState_t  I3c_InitSclGpio              ( i3c_SclPin_t pinId );
i3c_RequestState_t  I3c_InitSdaGpio              ( i3c_SdaPin_t pinId );

#ifdef __cplusplus
}
#endif

#endif /* I3C_I3C_PORT_H */
