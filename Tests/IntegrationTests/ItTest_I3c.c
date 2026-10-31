/**
 * \author Mr.Nobody
 * \file ItTest_I3c.c
 * \ingroup I3c
 * \brief Integration tests of the Improved Inter-Integrated Circuit (I3C) module on target.
 *
 * The I3c module runs on the MCU together with the real Rcc, Nvic, Gpio and Gpdma modules and hardware.
 * The tests verify behaviour that unit tests with emulated registers cannot verify: the peripheral
 * enables, the timing registers are accepted, the legacy device lowers the open-drain frequency and the
 * controller and the target of two instances talk to each other.
 *
 * Tests without wiring (run on every board): initialization and deinitialization of both roles, the
 * bus frequencies with the kernel clock HSI, the device table and the rejected requests.
 *
 * Tests that need the fixture I3C_PAIR (IT_FEATURE_I3C_PAIR): the controller I3C1 (PB6 SCL, PB7 SDA)
 * wired to the target I3C2 (PB3 SCL, PB4 SDA) with the pull-up resistors of an I3C bus, available on the
 * lines STM32H503, H523 and H533. The tests run the bus bring-up, the dynamic address assignment, the private
 * transfers, the CCCs and the in-band interrupt between the two instances. The fixture is checked by
 * ItI3c_Require_Pair(): a board without the fixture ignores the test.
 *
 * \note Waiting is bounded by the number of loop iterations (no reference timer), the target instance is
 *       served by I3c_Task() in the POLL mode. The transfers are short (at most the eight bytes of a FIFO).
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "I3c_Port.h"                       /* Module under test              */
#include "Gpdma_Port.h"                     /* State of the Gpdma channels    */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

static i3c_Config_t     ItI3c_Get_Config        ( i3c_Role_t role, i3c_PeriphId_t periphId );
static void             ItI3c_Require_Pair      ( void );
static void             ItI3c_Setup_Pair        ( void );
static void             ItI3c_Release_Pair      ( void );
static i3c_XferState_t  ItI3c_Wait_Request      ( void );
static void             ItI3c_Wait_Counter      ( const volatile uint32_t * const counter );
static void             ItI3c_TargetAddressCb   ( i3c_PeriphId_t periphId, i3c_Address_t address );
static void             ItI3c_TargetWriteCb     ( i3c_PeriphId_t periphId, i3c_DataCnt_t xferCnt );
static void             ItI3c_TargetReadCb      ( i3c_PeriphId_t periphId, i3c_DataCnt_t xferCnt );
static void             ItI3c_TargetCccCb       ( i3c_PeriphId_t periphId, i3c_CccEvent_t event );
static void             ItI3c_ErrorCb           ( i3c_PeriphId_t periphId, i3c_XferError_t errorMask );
static void             ItI3c_IbiCb             ( i3c_PeriphId_t periphId,
                                                  i3c_Address_t address,
                                                  const i3c_Data_t * const payload,
                                                  i3c_DataCnt_t payloadCnt );

/* ========================= SYMBOLIC CONSTANTS ============================= */

#if defined(I3C2)
/** Instance used as target by the tests of the fixture */
#define IT_I3C_TGT_PERIPH                   ( I3C_PERIPH_2 )
#else
/** Instance used as target by the tests of the fixture (the tests are ignored on this line) */
#define IT_I3C_TGT_PERIPH                   ( I3C_PERIPH_1 )
#endif

/** Count of the bits of a byte and the mask of a byte */
#define IT_I3C_BYTE_BIT_CNT                 ( 8u )
#define IT_I3C_BYTE_MASK                    ( 0xFFu )

/** Maximal number of wait loop iterations */
#define IT_I3C_WAIT_LOOPS                   ( 5000000u )

/** Push-pull and open-drain frequency requested by the tests (Hz) */
#define IT_I3C_PP_FREQ                      ( 4000000u )
#define IT_I3C_OD_FREQ                      ( 1000000u )

/** Static address of a legacy I2C device of the device table */
#define IT_I3C_ADDR_I2C                     ( 0x50u )

/** Instance ID of the target (4 bits of the provisioned ID) and the device characteristics of the target */
#define IT_I3C_TGT_INSTANCE_ID              ( 5u )
#define IT_I3C_TGT_DCR                      ( 0x01u )

/** Maximum read and write length and the in-band interrupt payload size of the target */
#define IT_I3C_TGT_MAX_LEN                  ( 8u )
#define IT_I3C_TGT_MAX_IBI                  ( 1u )

/** Count of the data bytes of the transfers */
#define IT_I3C_DATA_CNT                     ( 4u )

/** Count of the data bytes of the DMA write to a missing device: more than the transmit FIFO holds, so the transmit
 *  channel still runs when the address is not acknowledged */
#define IT_I3C_DMA_WRITE_CNT                ( 64u )

/** Count of the bytes of the provisioned ID and the position of its instance ID */
#define IT_I3C_PID_CNT                      ( 6u )
#define IT_I3C_PID_NEXT_TO_LAST             ( 4u )
#define IT_I3C_PID_INSTANCE_POS             ( 12u )
#define IT_I3C_PID_INSTANCE_MASK            ( 0x0Fu )

/** Mandatory data byte of the in-band interrupt */
#define IT_I3C_IBI_MDB                      ( 0xA7u )

/** Event mask byte of the broadcast DISEC */
#define IT_I3C_DISEC_MASK                   ( 0x01u )

#if defined(STM32H503xx) || \
    defined(STM32H523xx) || \
    defined(STM32H533xx)
/** The fixture I3C_PAIR can be wired on this line (I3C1 and I3C2 with the pins below) */
#define IT_I3C_PAIR_LINE                    ( 1u )

/** Pins of the controller (I3C1) and of the target (I3C2) of the fixture */
#define IT_I3C_CTRL_SCL_PIN                 ( I3C_SCL_PIN_I3C1_PB6 )
#define IT_I3C_CTRL_SDA_PIN                 ( I3C_SDA_PIN_I3C1_PB7 )
#define IT_I3C_TGT_SCL_PIN                  ( I3C_SCL_PIN_I3C2_PB3 )
#define IT_I3C_TGT_SDA_PIN                  ( I3C_SDA_PIN_I3C2_PB4 )
#else
/** The fixture I3C_PAIR cannot be wired on this line */
#define IT_I3C_PAIR_LINE                    ( 0u )

/** Pins of the controller (I3C1) and of the target of the fixture (not used on this line) */
#define IT_I3C_CTRL_SCL_PIN                 ( I3C_SCL_PIN_UNUSED )
#define IT_I3C_CTRL_SDA_PIN                 ( I3C_SDA_PIN_UNUSED )
#define IT_I3C_TGT_SCL_PIN                  ( I3C_SCL_PIN_UNUSED )
#define IT_I3C_TGT_SDA_PIN                  ( I3C_SDA_PIN_UNUSED )
#endif

/* ========================== LOCAL VARIABLES =============================== */

/** Data written by the controller to the target and read from the target */
static const i3c_Data_t itI3c_Data[ IT_I3C_DATA_CNT ] = { 0x11u, 0x22u, 0x33u, 0x44u };

/** Buffers of the receiving side */
static i3c_Data_t itI3c_RxBuffer[ IT_I3C_DATA_CNT ];
static i3c_Data_t itI3c_PidBuffer[ IT_I3C_PID_CNT ];

/** Records of the callbacks of the target and of the controller */
static volatile uint32_t     itI3c_TgtAddrCnt;
static volatile i3c_Address_t itI3c_TgtAddr;
static volatile uint32_t     itI3c_TgtWriteCnt;
static volatile i3c_DataCnt_t itI3c_TgtWriteLast;
static volatile uint32_t     itI3c_TgtReadCnt;
static volatile uint32_t     itI3c_TgtCccCnt;
static volatile i3c_CccEvent_t itI3c_TgtCccEvent;
static volatile uint32_t     itI3c_IbiCnt;
static volatile i3c_Address_t itI3c_IbiAddr;
static volatile i3c_Data_t   itI3c_IbiMdb;
static volatile uint32_t     itI3c_ErrorCnt;
static volatile i3c_XferError_t itI3c_ErrorMask;

/** Data of the DMA write to the missing legacy device */
static const i3c_Data_t itI3c_DmaData[ IT_I3C_DMA_WRITE_CNT ] = { 0u };

/* ============================= TEST SETUP ================================= */

void setUp( void )
{
    /* No action required */
}


void tearDown( void )
{
    /* Peripheral state is cleared by system reset after the test case */
}

/* ============================== TEST CASES ================================= */

/**
 * \brief Verifies that I3c_Init() enables the controller on real hardware.
 *
 * \par Expected results
 * I3c_Init() as controller with the kernel clock HSI returns I3C_REQUEST_OK, the achieved push-pull and
 * open-drain frequencies are not above the requested ones, the own address is the configured one and
 * I3c_Deinit() returns I3C_REQUEST_OK.
 */
void It_I3c_Init_ControllerHsi_FrequenciesNotAboveRequested( void )
{
    const i3c_Config_t config = ItI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_PERIPH_1 );
    i3c_FreqHz_t       pushPull  = 0u;
    i3c_FreqHz_t       openDrain = 0u;
    i3c_Address_t      address   = 0u;

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Init( &config ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_BusFreq( I3C_PERIPH_1, &pushPull, &openDrain ) );
    TEST_ASSERT_TRUE( 0u < pushPull );
    TEST_ASSERT_TRUE( IT_I3C_PP_FREQ >= pushPull );
    TEST_ASSERT_TRUE( 0u < openDrain );
    TEST_ASSERT_TRUE( IT_I3C_OD_FREQ >= openDrain );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_OwnAddress( I3C_PERIPH_1, &address ) );
    TEST_ASSERT_EQUAL_UINT8( config.OwnAddress, address );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Deinit( I3C_PERIPH_1 ) );
}


/**
 * \brief Verifies that the instance can be initialized again after the deinitialization.
 *
 * \par Expected results
 * Two sequences of I3c_Init() and I3c_Deinit() return I3C_REQUEST_OK, the second initialization of a
 * running instance returns I3C_REQUEST_ERROR.
 */
void It_I3c_InitDeinit_Repeated_InstanceReusable( void )
{
    const i3c_Config_t config = ItI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_PERIPH_1 );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Init( &config ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Init( &config ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Deinit( I3C_PERIPH_1 ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Init( &config ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Deinit( I3C_PERIPH_1 ) );
}


/**
 * \brief Verifies the initialization as target and the identity registers on real hardware.
 *
 * \par Expected results
 * The target is initialized, the identity (instance ID, characteristics, maximum lengths) is accepted by the
 * registers, an in-band interrupt request without a dynamic address is rejected.
 */
void It_I3c_InitTarget_IdentityAccepted_IbiWithoutAddressRejected( void )
{
    const i3c_Config_t config = ItI3c_Get_Config( I3C_ROLE_TARGET, I3C_PERIPH_1 );
    i3c_TargetConfig_t targetConfig = { 0 };

    targetConfig.InstanceId    = IT_I3C_TGT_INSTANCE_ID;
    targetConfig.Bcr           = ( I3C_BCR_IBI_CAPABLE | I3C_BCR_IBI_PAYLOAD );
    targetConfig.Dcr           = IT_I3C_TGT_DCR;
    targetConfig.MaxReadLen    = IT_I3C_TGT_MAX_LEN;
    targetConfig.MaxWriteLen   = IT_I3C_TGT_MAX_LEN;
    targetConfig.MaxIbiPayload = IT_I3C_TGT_MAX_IBI;
    targetConfig.IbiState      = I3C_FUNCTION_ACTIVE;

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Init( &config ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_TargetConfig( I3C_PERIPH_1, &targetConfig ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_IbiRequest( I3C_PERIPH_1, itI3c_Data, 1u ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Deinit( I3C_PERIPH_1 ) );
}


/**
 * \brief Verifies that a legacy I2C device lowers the open-drain frequency on real hardware.
 *
 * \par Expected results
 * The device table with a Fast-mode I2C device is accepted and the open-drain frequency is lower than the one
 * of the empty table; a Standard-mode device is accepted too (the kernel clock HSI after the reset divider is
 * 32 MHz, so the low time of 5000 ns is 160 cycles of the register range) and lowers the frequency further; a
 * device with an invalid speed class is rejected.
 */
void It_I3c_SetDeviceTable_LegacyDevices_AdaptOpenDrainTiming( void )
{
    const i3c_Config_t config = ItI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_PERIPH_1 );
    i3c_Device_t       device = { 0 };
    i3c_FreqHz_t       pushPull  = 0u;
    i3c_FreqHz_t       openDrainBefore = 0u;
    i3c_FreqHz_t       openDrainAfter  = 0u;
    i3c_FreqHz_t       openDrainStandard = 0u;

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Init( &config ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_BusFreq( I3C_PERIPH_1, &pushPull, &openDrainBefore ) );

    device.Type          = I3C_DEVICE_I2C;
    device.StaticAddress = IT_I3C_ADDR_I2C;
    device.I2cSpeed      = I3C_I2C_SPEED_FM;
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_DeviceTable( I3C_PERIPH_1, &device, 1u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_BusFreq( I3C_PERIPH_1, &pushPull, &openDrainAfter ) );
    TEST_ASSERT_TRUE( openDrainAfter < openDrainBefore );

    device.I2cSpeed = I3C_I2C_SPEED_SM;
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_DeviceTable( I3C_PERIPH_1, &device, 1u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_BusFreq( I3C_PERIPH_1, &pushPull, &openDrainStandard ) );
    TEST_ASSERT_TRUE( openDrainStandard < openDrainAfter );

    device.I2cSpeed = I3C_I2C_SPEED_CNT;
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_DeviceTable( I3C_PERIPH_1, &device, 1u ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Deinit( I3C_PERIPH_1 ) );
}


/**
 * \brief Verifies the rejected requests on real hardware.
 *
 * \par Expected results
 * A write to a device that is not in the device table and a CCC to an unknown device return I3C_REQUEST_ERROR,
 * the state of the instance stays idle.
 */
void It_I3c_Requests_UnknownDevice_RejectedStateIdle( void )
{
    const i3c_Config_t config = ItI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_PERIPH_1 );
    i3c_XferState_t    xferState = I3C_XFER_STATE_BUSY;
    i3c_Data_t         data[ 1u ] = { 0u };

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Init( &config ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR, I3c_Set_Write( I3C_PERIPH_1, IT_I3C_ADDR_I2C, itI3c_Data, 1u ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_ERROR,
                       I3c_Set_DirectCcc( I3C_PERIPH_1,
                                          I3C_CCC_GETBCR,
                                          IT_I3C_ADDR_I2C,
                                          data,
                                          1u,
                                          I3C_DIRECTION_READ ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_XferState( I3C_PERIPH_1, &xferState ) );
    TEST_ASSERT_EQUAL( I3C_XFER_STATE_IDLE, xferState );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Deinit( I3C_PERIPH_1 ) );
}


/**
 * \brief Verifies that a DMA write on a bus without any device ends with one protocol error and stops the transmit
 *        channel.
 *
 * \details The controller sends the arbitration header 0x7E first (I3C_CFGR.NOARBH = 0), no device acknowledges it:
 *          protocol error CE2 (RM0481 I3C_SER.CODERR), reported as I3C_XFER_ERROR_FRAME. The transmit Gpdma channel
 *          still feeds the FIFO at that moment; the error handler stops it (suspension, wait for the suspension flag,
 *          reset) and the suspension is no error of the request. The controller drives the pins of the fixture
 *          I3C_PAIR (pull-ups of the I2C loop on the boards).
 *
 * \par Expected results
 * - The request ends with I3C_XFER_STATE_ERROR, one error callback with I3C_XFER_ERROR_FRAME only.
 * - The transmit Gpdma channel is inactive.
 */
void It_I3c_DmaWrite_EmptyBus_OneErrorStopsChannel( void )
{
#if ( 0u != IT_I3C_PAIR_LINE )
    i3c_Config_t          config       = ItI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_PERIPH_1 );
    i3c_Device_t          device       = { 0 };
    gpdma_FunctionState_t channelState = GPDMA_FUNCTION_ACTIVE;

    config.SclPin        = IT_I3C_CTRL_SCL_PIN;
    config.SdaPin        = IT_I3C_CTRL_SDA_PIN;
    config.XferMode      = I3C_XFER_MODE_DMA;
    config.ErrorCallback = ItI3c_ErrorCb;
    itI3c_ErrorCnt       = 0u;
    itI3c_ErrorMask      = I3C_XFER_ERROR_NONE;
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Init( &config ) );

    device.Type          = I3C_DEVICE_I2C;
    device.StaticAddress = IT_I3C_ADDR_I2C;
    device.I2cSpeed      = I3C_I2C_SPEED_FM;
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_DeviceTable( I3C_PERIPH_1, &device, 1u ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK,
                       I3c_Set_Write( I3C_PERIPH_1,
                                      IT_I3C_ADDR_I2C,
                                      itI3c_DmaData,
                                      IT_I3C_DMA_WRITE_CNT ) );
    TEST_ASSERT_EQUAL( I3C_XFER_STATE_ERROR, ItI3c_Wait_Request() );
    TEST_ASSERT_EQUAL_UINT32( 1u, itI3c_ErrorCnt );
    TEST_ASSERT_EQUAL_HEX8( I3C_XFER_ERROR_FRAME, itI3c_ErrorMask );

    TEST_ASSERT_EQUAL( GPDMA_REQUEST_OK,
                       Gpdma_Get_ChannelState( config.TxDmaPeriphId,
                                               config.TxDmaChannelId,
                                               &channelState ) );
    TEST_ASSERT_EQUAL( GPDMA_FUNCTION_INACTIVE, channelState );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Deinit( I3C_PERIPH_1 ) );
#else
    TEST_IGNORE_MESSAGE( "MCU line without the I3C1 pins of the fixture" );
#endif
}


/**
 * \brief Verifies the bus bring-up between the controller and the target of the fixture.
 *
 * \par Expected results
 * I3c_Set_BusInit() (RSTDAA) of the controller completes with I3C_REQUEST_OK.
 *
 * Fixture: I3C_PAIR.
 */
void It_I3c_SetBusInit_Pair_Completes( void )
{
    ItI3c_Require_Pair();
    ItI3c_Setup_Pair();

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_BusInit( I3C_PERIPH_1 ) );

    ItI3c_Release_Pair();
}


/**
 * \brief Verifies the dynamic address assignment between the controller and the target of the fixture.
 *
 * \par Expected results
 * ENTDAA discovers one device, its provisioned ID carries the instance ID of the target, the address callback
 * of the target reports the address of the device table and I3c_Get_OwnAddress() of the target returns it.
 *
 * Fixture: I3C_PAIR.
 */
void It_I3c_SetDynamicAddressAssign_Pair_AssignsAddressToTarget( void )
{
    i3c_Device_t    devices[ I3C_DEVICE_CNT_MAX ] = { { 0 } };
    i3c_DeviceCnt_t deviceCnt = 0u;
    i3c_DeviceCnt_t discoveredCnt = 0u;
    i3c_Address_t   ownAddress = 0u;

    ItI3c_Require_Pair();
    ItI3c_Setup_Pair();

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_BusInit( I3C_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_DynamicAddressAssign( I3C_PERIPH_1, &discoveredCnt ) );
    TEST_ASSERT_EQUAL_UINT8( 1u, discoveredCnt );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_DeviceTable( I3C_PERIPH_1, devices, &deviceCnt ) );
    TEST_ASSERT_EQUAL_UINT8( 1u, deviceCnt );
    TEST_ASSERT_EQUAL_UINT32( IT_I3C_TGT_INSTANCE_ID,
                              ( devices[ 0u ].PidLow >> IT_I3C_PID_INSTANCE_POS ) & IT_I3C_PID_INSTANCE_MASK );

    ItI3c_Wait_Counter( &itI3c_TgtAddrCnt );
    TEST_ASSERT_EQUAL_UINT8( devices[ 0u ].DynamicAddress, itI3c_TgtAddr );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_OwnAddress( IT_I3C_TGT_PERIPH, &ownAddress ) );
    TEST_ASSERT_EQUAL_UINT8( devices[ 0u ].DynamicAddress, ownAddress );

    ItI3c_Release_Pair();
}


/**
 * \brief Verifies a private write between the controller and the target of the fixture.
 *
 * \par Expected results
 * The data written by the controller arrives in the write buffer of the target and the target write
 * callback reports the count.
 *
 * Fixture: I3C_PAIR.
 */
void It_I3c_PrivateWrite_Pair_TargetReceivesData( void )
{
    i3c_Device_t    devices[ I3C_DEVICE_CNT_MAX ] = { { 0 } };
    i3c_DeviceCnt_t deviceCnt = 0u;
    i3c_DeviceCnt_t discoveredCnt = 0u;

    ItI3c_Require_Pair();
    ItI3c_Setup_Pair();
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_DynamicAddressAssign( I3C_PERIPH_1, &discoveredCnt ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_DeviceTable( I3C_PERIPH_1, devices, &deviceCnt ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK,
                       I3c_Set_TargetWriteBuffer( IT_I3C_TGT_PERIPH, itI3c_RxBuffer, IT_I3C_DATA_CNT ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK,
                       I3c_Set_Write( I3C_PERIPH_1, devices[ 0u ].DynamicAddress, itI3c_Data, IT_I3C_DATA_CNT ) );
    TEST_ASSERT_EQUAL( I3C_XFER_STATE_IDLE, ItI3c_Wait_Request() );

    ItI3c_Wait_Counter( &itI3c_TgtWriteCnt );
    TEST_ASSERT_EQUAL_UINT16( IT_I3C_DATA_CNT, itI3c_TgtWriteLast );
    TEST_ASSERT_EQUAL_UINT8_ARRAY( itI3c_Data, itI3c_RxBuffer, IT_I3C_DATA_CNT );

    ItI3c_Release_Pair();
}


/**
 * \brief Verifies a private read between the controller and the target of the fixture.
 *
 * \par Expected results
 * The controller reads the data armed in the read buffer of the target.
 *
 * Fixture: I3C_PAIR.
 */
void It_I3c_PrivateRead_Pair_ControllerReceivesTargetData( void )
{
    i3c_Device_t    devices[ I3C_DEVICE_CNT_MAX ] = { { 0 } };
    i3c_DeviceCnt_t deviceCnt = 0u;
    i3c_DeviceCnt_t discoveredCnt = 0u;

    ItI3c_Require_Pair();
    ItI3c_Setup_Pair();
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_DynamicAddressAssign( I3C_PERIPH_1, &discoveredCnt ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_DeviceTable( I3C_PERIPH_1, devices, &deviceCnt ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_TargetReadBuffer( IT_I3C_TGT_PERIPH, itI3c_Data, IT_I3C_DATA_CNT ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK,
                       I3c_Set_Read( I3C_PERIPH_1, devices[ 0u ].DynamicAddress, itI3c_RxBuffer, IT_I3C_DATA_CNT ) );
    TEST_ASSERT_EQUAL( I3C_XFER_STATE_IDLE, ItI3c_Wait_Request() );

    TEST_ASSERT_EQUAL_UINT8_ARRAY( itI3c_Data, itI3c_RxBuffer, IT_I3C_DATA_CNT );
    ItI3c_Wait_Counter( &itI3c_TgtReadCnt );

    ItI3c_Release_Pair();
}


/**
 * \brief Verifies a direct CCC between the controller and the target of the fixture.
 *
 * \par Expected results
 * GETPID returns the six bytes of the provisioned ID, the byte of the instance ID equals the configured one.
 *
 * Fixture: I3C_PAIR.
 */
void It_I3c_DirectCcc_Pair_GetPidReturnsProvisionedId( void )
{
    i3c_Device_t    devices[ I3C_DEVICE_CNT_MAX ] = { { 0 } };
    i3c_DeviceCnt_t deviceCnt = 0u;
    i3c_DeviceCnt_t discoveredCnt = 0u;

    ItI3c_Require_Pair();
    ItI3c_Setup_Pair();
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_DynamicAddressAssign( I3C_PERIPH_1, &discoveredCnt ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_DeviceTable( I3C_PERIPH_1, devices, &deviceCnt ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK,
                       I3c_Set_DirectCcc( I3C_PERIPH_1,
                                          I3C_CCC_GETPID,
                                          devices[ 0u ].DynamicAddress,
                                          itI3c_PidBuffer,
                                          IT_I3C_PID_CNT,
                                          I3C_DIRECTION_READ ) );
    TEST_ASSERT_EQUAL( I3C_XFER_STATE_IDLE, ItI3c_Wait_Request() );

    TEST_ASSERT_EQUAL_UINT8( ( devices[ 0u ].PidLow >> IT_I3C_BYTE_BIT_CNT ) & IT_I3C_BYTE_MASK,
                             itI3c_PidBuffer[ IT_I3C_PID_NEXT_TO_LAST ] );
    TEST_ASSERT_EQUAL_UINT8( ( devices[ 0u ].PidLow & IT_I3C_BYTE_MASK ), itI3c_PidBuffer[ IT_I3C_PID_CNT - 1u ] );

    ItI3c_Release_Pair();
}


/**
 * \brief Verifies a broadcast CCC between the controller and the target of the fixture.
 *
 * \par Expected results
 * The target reports the DISEC broadcast as the events update.
 *
 * Fixture: I3C_PAIR.
 */
void It_I3c_BroadcastCcc_Pair_TargetReportsEventsUpdate( void )
{
    i3c_Data_t mask[ 1u ] = { IT_I3C_DISEC_MASK };

    ItI3c_Require_Pair();
    ItI3c_Setup_Pair();

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_BroadcastCcc( I3C_PERIPH_1, I3C_CCC_DISEC, mask, 1u ) );
    TEST_ASSERT_EQUAL( I3C_XFER_STATE_IDLE, ItI3c_Wait_Request() );

    ItI3c_Wait_Counter( &itI3c_TgtCccCnt );
    TEST_ASSERT_EQUAL( I3C_CCC_EVENT_EVENTS_UPDATE, itI3c_TgtCccEvent );

    ItI3c_Release_Pair();
}


/**
 * \brief Verifies the in-band interrupt between the target and the controller of the fixture.
 *
 * \par Expected results
 * With the acceptance enabled for the device the in-band interrupt of the target reaches the controller
 * callback with the address of the device and the mandatory data byte.
 *
 * Fixture: I3C_PAIR.
 */
void It_I3c_InBandInterrupt_Pair_ControllerReceivesMandatoryDataByte( void )
{
    i3c_Device_t    devices[ I3C_DEVICE_CNT_MAX ] = { { 0 } };
    i3c_DeviceCnt_t deviceCnt = 0u;
    i3c_DeviceCnt_t discoveredCnt = 0u;
    i3c_Data_t      mdb[ 1u ] = { IT_I3C_IBI_MDB };

    ItI3c_Require_Pair();
    ItI3c_Setup_Pair();
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_DynamicAddressAssign( I3C_PERIPH_1, &discoveredCnt ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Get_DeviceTable( I3C_PERIPH_1, devices, &deviceCnt ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK,
                       I3c_Set_IbiConfig( I3C_PERIPH_1, devices[ 0u ].DynamicAddress, I3C_FUNCTION_ACTIVE ) );

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_IbiRequest( IT_I3C_TGT_PERIPH, mdb, 1u ) );
    ItI3c_Wait_Counter( &itI3c_IbiCnt );

    TEST_ASSERT_EQUAL_UINT8( devices[ 0u ].DynamicAddress, itI3c_IbiAddr );
    TEST_ASSERT_EQUAL_UINT8( IT_I3C_IBI_MDB, itI3c_IbiMdb );

    ItI3c_Release_Pair();
}

/* ============================ TEST HELPERS ================================= */

/** Gets the configuration of an instance: kernel clock HSI, no pin, the POLL mode, the callbacks of the target */
static i3c_Config_t ItI3c_Get_Config( i3c_Role_t role, i3c_PeriphId_t periphId )
{
    i3c_Config_t config;

    (void)I3c_Get_DefaultConfig( &config );

    config.PeriphId        = periphId;
    config.Role            = role;
    config.ClkSrc          = I3C_CLK_SRC_I3C1_HSI;
    config.PushPullFreq    = IT_I3C_PP_FREQ;
    config.OpenDrainFreq   = IT_I3C_OD_FREQ;

#if defined(I3C2)
    if( IT_I3C_TGT_PERIPH == periphId )
    {
        config.ClkSrc = I3C_CLK_SRC_I3C2_HSI;
    }
    else
    {
        /* The first instance uses its own kernel clock source, no action required */
    }
#endif

    config.TargetAddressCallback = ItI3c_TargetAddressCb;
    config.TargetWriteCallback   = ItI3c_TargetWriteCb;
    config.TargetReadCallback    = ItI3c_TargetReadCb;
    config.TargetCccCallback     = ItI3c_TargetCccCb;
    config.IbiCallback           = ItI3c_IbiCb;

    return ( config );
}


/** Ignores the test when the board has no fixture I3C_PAIR (or the line has no second instance) */
static void ItI3c_Require_Pair( void )
{
#if defined(I3C2)
#if defined(IT_FEATURE_I3C_PAIR) || \
    !defined(IT_FIXTURE)
    TEST_ASSERT_EQUAL_UINT32( 1u, IT_I3C_PAIR_LINE );
#else
    TEST_IGNORE_MESSAGE( "Board without the fixture I3C_PAIR (I3C1 and I3C2 wired together)" );
#endif
#else
    TEST_IGNORE_MESSAGE( "Line without the second I3C instance" );
#endif
}


/** Initializes the controller (I3C1) and the target (I3C2) of the fixture and configures the target */
static void ItI3c_Setup_Pair( void )
{
#if defined(I3C2)
    i3c_Config_t       ctrlConfig = ItI3c_Get_Config( I3C_ROLE_CONTROLLER, I3C_PERIPH_1 );
    i3c_Config_t       tgtConfig  = ItI3c_Get_Config( I3C_ROLE_TARGET, IT_I3C_TGT_PERIPH );
    i3c_TargetConfig_t targetConfig = { 0 };

    ctrlConfig.SclPin = IT_I3C_CTRL_SCL_PIN;
    ctrlConfig.SdaPin = IT_I3C_CTRL_SDA_PIN;
    tgtConfig.SclPin  = IT_I3C_TGT_SCL_PIN;
    tgtConfig.SdaPin  = IT_I3C_TGT_SDA_PIN;

    targetConfig.InstanceId    = IT_I3C_TGT_INSTANCE_ID;
    targetConfig.Bcr           = ( I3C_BCR_IBI_CAPABLE | I3C_BCR_IBI_PAYLOAD );
    targetConfig.Dcr           = IT_I3C_TGT_DCR;
    targetConfig.MaxReadLen    = IT_I3C_TGT_MAX_LEN;
    targetConfig.MaxWriteLen   = IT_I3C_TGT_MAX_LEN;
    targetConfig.MaxIbiPayload = IT_I3C_TGT_MAX_IBI;
    targetConfig.IbiState      = I3C_FUNCTION_ACTIVE;

    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Init( &tgtConfig ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Set_TargetConfig( IT_I3C_TGT_PERIPH, &targetConfig ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Init( &ctrlConfig ) );
#endif
}


/** Deinitializes both instances of the fixture */
static void ItI3c_Release_Pair( void )
{
#if defined(I3C2)
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Deinit( I3C_PERIPH_1 ) );
    TEST_ASSERT_EQUAL( I3C_REQUEST_OK, I3c_Deinit( IT_I3C_TGT_PERIPH ) );
#endif
}


/** Serves the instances until the request of the controller ends (bounded loop) */
static i3c_XferState_t ItI3c_Wait_Request( void )
{
    i3c_XferState_t xferState = I3C_XFER_STATE_BUSY;

    for( uint32_t idx = 0u;
         ( IT_I3C_WAIT_LOOPS > idx ) &&
         ( I3C_XFER_STATE_BUSY == xferState );
         idx++ )
    {
        I3c_Task();
        (void)I3c_Get_XferState( I3C_PERIPH_1, &xferState );
    }

    return ( xferState );
}


/** Serves the instances until a callback counter is not zero (bounded loop) */
static void ItI3c_Wait_Counter( const volatile uint32_t * const counter )
{
    for( uint32_t idx = 0u;
         ( IT_I3C_WAIT_LOOPS > idx ) &&
         ( 0u == *counter );
         idx++ )
    {
        I3c_Task();
    }

    TEST_ASSERT_TRUE( 0u < *counter );
}


static void ItI3c_TargetAddressCb( i3c_PeriphId_t periphId, i3c_Address_t address )
{
    (void)periphId;

    itI3c_TgtAddr = address;
    itI3c_TgtAddrCnt++;
}


static void ItI3c_TargetWriteCb( i3c_PeriphId_t periphId, i3c_DataCnt_t xferCnt )
{
    (void)periphId;

    itI3c_TgtWriteLast = xferCnt;
    itI3c_TgtWriteCnt++;
}


static void ItI3c_TargetReadCb( i3c_PeriphId_t periphId, i3c_DataCnt_t xferCnt )
{
    (void)periphId;
    (void)xferCnt;

    itI3c_TgtReadCnt++;
}


static void ItI3c_TargetCccCb( i3c_PeriphId_t periphId, i3c_CccEvent_t event )
{
    (void)periphId;

    itI3c_TgtCccEvent = event;
    itI3c_TgtCccCnt++;
}


/**
 * \brief Error callback of the controller, records the error mask and counts the calls.
 *
 * \param periphId  [in]: Instance (not evaluated)
 * \param errorMask [in]: Error mask of the failed request
 */
static void ItI3c_ErrorCb( i3c_PeriphId_t periphId, i3c_XferError_t errorMask )
{
    (void)periphId;

    itI3c_ErrorMask = errorMask;
    itI3c_ErrorCnt++;
}


static void ItI3c_IbiCb( i3c_PeriphId_t periphId,
                         i3c_Address_t address,
                         const i3c_Data_t * const payload,
                         i3c_DataCnt_t payloadCnt )
{
    (void)periphId;

    itI3c_IbiAddr = address;

    if( 0u < payloadCnt )
    {
        itI3c_IbiMdb = payload[ 0u ];
    }
    else
    {
        /* No payload, the mandatory data byte stays zero */
    }

    itI3c_IbiCnt++;
}
