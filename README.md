# I3C Peripheral Driver

This module provides an abstraction layer for the **improved inter-integrated circuit interface** (`I3Cx`) of STM32 MCUs - the MIPI I3C
bus: the controller and the target role, the private read and write transfers, the **dynamic address assignment**, the common
command codes (CCC), the **in-band interrupts** and the hot-join, and the coexistence with the legacy I2C devices on the same wires. It
offers the data transfers in DMA, interrupt or polling mode. It is a low-level MCAL module: it owns the I3C hardware, the
application and the other modules never touch the registers.

The public interface (`I3c_Port.h`, `I3c_Types.h`) is **identical for all supported families**. The implementation
for a family lives in its own branch (`Dev/<Family>` for development, `Releases/<Family>` for the released versions), so
a module or an application written against the interface does not change when the MCU family changes.

This README is the landing page and the **specification of the interface** (master). The family branches contain their
own README with everything that is specific to the family: the instances of the device, their settings and the status of
the implementation.

> **Status: STM32H5 implemented** (branch `Dev/STM32H5`, controller and target role, POLL / ISR / DMA mode, unit tests on the 12 STM32H5
> lines, integration tests compiled but not run on hardware). The other families are planned. The details and the deviations from the
> interface below are in the section "STM32H5 implementation status".

---

## Intent and purpose

- I3C is the successor of I2C for the sensors and the small peripherals: up to 12.5 MHz in the push-pull SDR mode, the devices get a **dynamic
  address** from the controller (no address clashes, no jumpers), a device can **interrupt the controller over the data wires** (the in-band
  interrupt, no extra interrupt pin) and the legacy I2C devices (Sm / Fm / Fm+) can stay on the same bus. A modern sensor hub (IMUs, pressure
  and temperature sensors, a PMIC) is an I3C device.
- The protocol is a **bus management job** more than a byte transfer: the controller resets the dynamic addresses, discovers the
  devices (the `ENTDAA` procedure returns for each device its 48-bit provisional ID, the bus characteristic and the device
  characteristic), assigns the addresses, enables the events (the IBI, the hot-join, the controller-role request), and the
  transfers use the commands (CCC) for it. The module owns the sequences and offers the functions at the level of *"discover
  the bus"*, *"read N bytes from the target 0x30"*, *"send the broadcast CCC RSTDAA"*.
- The module serves **both roles**: the controller (primary, secondary) and the target (the MCU is a device on the I3C bus of another
  controller, with a static I2C address or a dynamic address, answering to the CCCs and to the private transfers).
- The module hides the differences between the families: the number of the instances, the FIFOs and the DMA channels, the supported
  data rates and the role support differ, the interface stays the same.

What the module is **not**: it does not implement the drivers of the sensors (they are middlewares that use `I3c_Set_Read` / `I3c_Set_Write`)
and it does not implement the HDR modes of the bus (the hardware supports the SDR mode only).

---

## Concept

An instance is identified by `i3c_PeriphId_t`. The role is selected in the configuration (`I3C_ROLE_CONTROLLER`, `I3C_ROLE_TARGET`).

**Controller.** The module keeps a table of the devices on the bus (`i3c_Device_t`):

| Item | Meaning |
|---|---|
| **Type** | `I3C_DEVICE_I3C` (a dynamic address), `I3C_DEVICE_I2C` (a legacy device with a static 7-bit address, the speed class). |
| **Address** | The static address (I2C or the initial I3C one) and the dynamic address (assigned). |
| **Identity** | The 48-bit provisional ID, the bus characteristics (BCR) and the device characteristics (DCR) read in the discovery. |
| **Events** | Whether the IBI (with the mandatory data byte) and the hot-join of the device are accepted. |

Operations:

| Group | Functions | Meaning |
|---|---|---|
| Bus bring-up | `I3c_Set_BusInit`, `I3c_Set_DynamicAddressAssign` | Reset the dynamic addresses (`RSTDAA`), run the `ENTDAA` discovery, assign the addresses from a free pool or by the table, fill the device table. |
| Transfers | `I3c_Set_Write`, `I3c_Set_Read`, `I3c_Set_WriteRead` | Private SDR messages to an I3C target; the same functions with an I2C device type are the legacy messages (Fm / Fm+). The DMA / ISR / POLL mode and the callbacks as in the I2c module. |
| Commands | `I3c_Set_BroadcastCcc`, `I3c_Set_DirectCcc` | Broadcast and direct CCCs with the data (`ENEC`, `DISEC`, `SETMWL`, `GETPID`, `GETSTATUS` ...). |
| Events | `I3c_Set_IbiConfig`, the `IbiCallback`, `HotJoinCallback` | In-band interrupts (the address of the device and the payload) and the hot-join requests. |

**Target.** `I3c_Set_TargetConfig` sets the static address, the identity (PID, BCR, DCR) and the maximum read / write lengths;
the callbacks report the assignment of the dynamic address, a read request (the controller wants data: the application provides a
buffer), a write (data received), a CCC received, and `I3c_Set_IbiRequest` asks the controller for service.

Rules:

1. `I3c_Init` configures the clock (the push-pull SCL frequency up to 12.5 MHz and the open-drain frequency), the timing of the bus (the
   free, available and idle time of the bus characteristics), the FIFO and the DMA, the pins, and leaves the bus idle. The bus bring-up
   is a separate call, after the targets are powered.
2. A transfer is a **request**: it returns at once and the callback reports the end; the same request and the same callbacks are used in all
   transfer modes. A transfer to a device that is not in the table is rejected.
3. The frequency of the open-drain phase is limited by the legacy I2C devices on the bus (the slowest class in the table); the module
   lowers it and reports the achieved value.
4. The error reporting covers the NACK of the address, the NACK of the data, the parity / CRC error of the SDR, the overrun /
   underrun of the FIFO, the frame error and the bus-level errors (the SDA stuck); the callback gets the error mask and the module
   recovers the bus (the stop condition) where it can.
5. A request while another is running returns an error; the module does not queue (a queue of requests is the task of the sensor layer).
6. All parameters are checked before a register is written.

---

## Supported families

Every family gets its own story, its own tasks and its own branches `Dev/<Family>` and `Releases/<Family>` cut from this
master.

| Family | Branch | Interface | Status |
|---|---|---|---|
| STM32H5 | `Dev/STM32H5` | `I3C1`, `I3C2` (one instance on STM32H562 / H563 / H573, two on the others): the MIPI I3C v1.1 SDR primary / secondary controller and target, the legacy I2C messages Sm / Fm / Fm+, FIFOs with the DMA, the CCCs, IBI and the hot-join, up to 12.5 MHz | implemented (`Dev/STM32H5`, hardware tests pending) |
| STM32U5 | `Dev/STM32U5` | `I3C1` on the lines that have it | planned |
| STM32U3 | `Dev/STM32U3` | device dependent | planned |
| STM32H7 | `Dev/STM32H7` | I3C on the H7R / H7S lines only | planned |
| STM32F4, STM32F7, STM32G4, STM32L4 | `Dev/STM32F4` ... | no I3C on the device - the module is a stub that returns `I3C_REQUEST_ERROR` | planned |

A function without a meaning in a family stays in the interface, returns `I3C_REQUEST_ERROR` and suppresses the unused
parameters (for example the target role on an instance without it, the DMA channel of a FIFO the instance does not have).

### STM32H5 implementation status

Branch `Dev/STM32H5`. Files: `I3c.c` (module management, bus timing, device table, bus bring-up), `I3c_Ctrl.c` (controller engine),
`I3c_Xfer.c` (private and legacy messages, CCCs, in-band interrupt and hot-join), `I3c_Tgt.c` (target role), `I3c_Dma.c` (Gpdma channels of the
data FIFOs), `I3c_Reg.c` (register access of the instances), `I3c_Pins.h` (SCL and SDA pin lists generated from the STM32CubeMX database).
The unit tests run on the 12 STM32H5 lines (host, Unity + CMock + RegMem), the integration tests are compiled and linked, they were not run
on hardware.

| Item | STM32H5 |
|---|---|
| Instances | `I3C1`, `I3C2`; `I3C2` is missing on STM32H562 / H563 / H573 |
| Kernel clock | PCLK1 / PLL3R (PLL2R on STM32H503) / HSI for `I3C1`, PCLK3 / PLL3R (PLL2R) / HSI for `I3C2`; at least twice the push-pull frequency |
| Timing | durations rounded up to whole kernel clock cycles, the achieved frequencies (`I3c_Get_BusFreq`) are equal to or lower than the requested ones; the open-drain phase follows the slowest legacy I2C class of the device table |
| Device table | four entries (the hardware has four target device registers, `I3C_DEVICE_CNT_MAX`); an I3C device needs a valid dynamic address to be addressed |
| Static address of a target | only on `I3C1` of STM32H523 / H533 / H543 / H553 / H562 / H563 / H573 (register `I3C_I2CCFGR` at offset 0x98, not in the CMSIS type) |
| Wake-up | `EXTI_LINE_I3C1_WKUP` / `EXTI_LINE_I3C2_WKUP` exist in the Exti module; the I3C wake-up from Stop mode is not used by this module |

Deviations from the interface proposal (with the reason):

- `i3c_TargetConfig_t` holds the 4-bit `InstanceId` instead of the PID: the provisioned ID of the STM32H5 target is fixed except for the
  instance ID (RM0481 54.15.30).
- The DMA mode serves the transmit and the receive FIFO (`TxDma*`, `RxDma*` in the configuration). The control and the status FIFO are served
  by the CPU: a frame has two control words at most and the C-FIFO has two words. A read that the target ends early is reported with the bytes
  received (`I3C_SR.XDCNT`), the DMA channel is stopped.
- Transfer functions return at once in all modes; `I3c_Set_BusInit` and `I3c_Set_DynamicAddressAssign` wait for the end (bounded loop) because they
  return the result.
- `I3c_Set_DeviceTable` accepts the I3C devices with a dynamic address (and the PID that the assignment matches) and the legacy I2C devices with a
  static address and a speed class. A SETDASA does not update the table, the application stores the assigned address with `I3c_Set_DeviceTable`.
- The CCCs with a defining byte other than RSTACT are not offered (GETSTATUS format 2, GETCAPS format 2 ...); `ENTDAA` has its own function.
- The hot-join acknowledge is global (`I3c_Set_HotJoinActive`), not per device; the in-band interrupt acceptance and its mandatory data byte are
  per device (`I3c_Set_IbiConfig`, the device table).
- The target has no event for the start of a read of the controller, the data has to be armed in advance (`I3c_Set_TargetReadBuffer`, preload). The
  read request callback is called when a read finds no data. A write of the controller needs `I3c_Set_TargetWriteBuffer`, bytes beyond the buffer are dropped.
- `I3c_Get_BusFreq` and `I3c_Get_OwnAddress` were added to the interface, `I3c_Set_HotJoinRequest` (target) as the counterpart of the hot-join
  acknowledge of the controller.
- The secondary controller role and the controller-role hand-off are not implemented (a controller-role request of a target is not acknowledged).

Errata (the workarounds are applied by the module): the first frame with the address 0x7F after the enable of the controller (SDA pull-up until the line reads
high, then the controller is initialized and enabled; ES0561 2.10.3, ES0621 2.13.3, ES0565 2.15.3, bug AB#2093; the workaround
sequence is ES0561 2.11, ES0621 2.14, ES0565 2.16) and the SCL stalling in the
address phase (the arbitrable header stays enabled, STALLA is never set, SCL stalling is not used; ES0561 2.10.2, ES0621 2.13.2, ES0565 2.15.2, bug AB#2092). No errata document was
available for the STM32H5E4 / H5E5 / H5F4 / H5F5 lines, the same workarounds are applied.

Other errata of the I3C controller (checked in ES0561 Rev 4, ES0621 Rev 2 and ES0565 Rev 7): "unexpected read data bytes during a legacy I2C
read" (ES0561 2.10.1, ES0621 2.13.1, ES0565 2.15.1, bug AB#2091) is avoided because the module never sets STALLT in I3C_TIMINGR2;
"no timestamp on IBI acknowledge when timing control is used in Asynchronous mode 0" (ES0561 2.10.4, ES0621 2.13.4, ES0565 2.15.4) is
not applicable, the module offers no timing control (no SETXTIME CCC, no timestamp), so no bug exists for it. The ES0621 summary line
"Deadlock or write-data corruption after spurious write to a misaligned address" (ES0621 2.4.5) belongs to the OCTOSPI (OCTOSPI_AR
register) and does not concern the I3C module.

---

## Public interface in short (proposal)

| Group | Functions |
|---|---|
| Module management | `I3c_Get_ModuleVersion`, `I3c_Init`, `I3c_Deinit`, `I3c_Task`, `I3c_Get_DefaultConfig` |
| Bus (controller) | `I3c_Set_BusInit`, `I3c_Set_DynamicAddressAssign`, `I3c_Get_DeviceTable`, `I3c_Set_DeviceTable` |
| Transfers | `I3c_Set_Write`, `I3c_Set_Read`, `I3c_Set_WriteRead`, `I3c_Set_XferStop`, `I3c_Get_XferState`, `I3c_Get_XferError` |
| Commands | `I3c_Set_BroadcastCcc`, `I3c_Set_DirectCcc` |
| Events | `I3c_Set_IbiConfig`, `I3c_Set_HotJoinActive`, `I3c_Set_HotJoinInactive` |
| Target | `I3c_Set_TargetConfig`, `I3c_Set_IbiRequest`, `I3c_Set_TargetReadBuffer`, `I3c_Set_TargetWriteBuffer` |
| Interrupts | `I3c_Set_IrqPriority` / `Get`, the callbacks are a part of the configuration |
| Pins | `I3c_InitSclGpio`, `I3c_InitSdaGpio` |

```c
i3c_ModuleVersion_t I3c_Get_ModuleVersion          ( void );

i3c_RequestState_t  I3c_Init                       ( const i3c_Config_t * const i3cConfig );
i3c_RequestState_t  I3c_Deinit                     ( i3c_PeriphId_t periphId );
void                I3c_Task                       ( void );
i3c_RequestState_t  I3c_Get_DefaultConfig          ( i3c_Config_t * const i3cConfig );

i3c_RequestState_t  I3c_Set_BusInit                ( i3c_PeriphId_t periphId );
i3c_RequestState_t  I3c_Set_DynamicAddressAssign   ( i3c_PeriphId_t periphId, i3c_DeviceCnt_t * const discoveredCnt );
i3c_RequestState_t  I3c_Get_DeviceTable            ( i3c_PeriphId_t periphId, i3c_Device_t * const devices, i3c_DeviceCnt_t * const deviceCnt );

i3c_RequestState_t  I3c_Set_Write                  ( i3c_PeriphId_t periphId, i3c_Address_t address, const i3c_Data_t * const data, i3c_DataCnt_t dataCnt );
i3c_RequestState_t  I3c_Set_Read                   ( i3c_PeriphId_t periphId, i3c_Address_t address, i3c_Data_t * const data, i3c_DataCnt_t dataCnt );
i3c_RequestState_t  I3c_Set_BroadcastCcc           ( i3c_PeriphId_t periphId, i3c_Ccc_t ccc, const i3c_Data_t * const data, i3c_DataCnt_t dataCnt );
i3c_RequestState_t  I3c_Set_DirectCcc              ( i3c_PeriphId_t periphId, i3c_Ccc_t ccc, i3c_Address_t address, i3c_Data_t * const data, i3c_DataCnt_t dataCnt, i3c_Direction_t direction );

i3c_RequestState_t  I3c_Set_IbiConfig              ( i3c_PeriphId_t periphId, i3c_Address_t address, i3c_FunctionState_t ibiState );
i3c_RequestState_t  I3c_Set_TargetConfig           ( i3c_PeriphId_t periphId, const i3c_TargetConfig_t * const targetConfig );
i3c_RequestState_t  I3c_Set_IbiRequest             ( i3c_PeriphId_t periphId, const i3c_Data_t * const payload, i3c_DataCnt_t payloadCnt );
```

Every function returns `I3C_REQUEST_OK` / `I3C_REQUEST_ERROR` and checks its parameters and the state of the module
before it touches a register.

| Type | Meaning |
|---|---|
| `i3c_Config_t` | Instance, role, kernel clock source, push-pull and open-drain frequency, bus timing, transfer mode (`I3C_XFER_MODE_POLL` / `ISR` / `DMA`), DMA channels, pins, callbacks, interrupt priority |
| `i3c_Device_t` | Type, static and dynamic address, PID, BCR, DCR, the IBI and hot-join permission, the speed class of a legacy device |
| `i3c_Ccc_t` | The CCC codes (`I3C_CCC_ENEC`, `I3C_CCC_DISEC`, `I3C_CCC_RSTDAA`, `I3C_CCC_GETPID` ...) |
| `i3c_TargetConfig_t` | Static address, PID, BCR, DCR, the maximum read and write length |
| `i3c_XferError_t` | Bit mask: address NACK, data NACK, parity / CRC, FIFO overrun / underrun, frame, bus error |
| Callbacks | `XferCompleteCallback`, `ErrorCallback`, `IbiCallback( address, payload, cnt )`, `HotJoinCallback`, the target callbacks |

---

## Usage

### Bring-up of the bus and a read of a sensor register

```c
#include "I3c_Port.h"

i3c_Config_t      i3cConfig;
i3c_DeviceCnt_t   discoveredCnt = 0u;
static i3c_Data_t sensorData[ 6u ];

(void)I3c_Get_DefaultConfig( &i3cConfig );

i3cConfig.PeriphId       = I3C_PERIPH_1;
i3cConfig.Role           = I3C_ROLE_CONTROLLER;
i3cConfig.PushPullFreq   = 12500000u;
i3cConfig.OpenDrainFreq  = 1000000u;
i3cConfig.XferMode       = I3C_XFER_MODE_DMA;
i3cConfig.SclPin         = I3C_SCL_PIN_I3C1_PB6;
i3cConfig.SdaPin         = I3C_SDA_PIN_I3C1_PB9;

(void)I3c_Init( &i3cConfig );
(void)I3c_Set_BusInit( I3C_PERIPH_1 );                                   /* RSTDAA */
(void)I3c_Set_DynamicAddressAssign( I3C_PERIPH_1, &discoveredCnt );      /* ENTDAA: PID, BCR, DCR, addresses */

(void)I3c_Set_Read( I3C_PERIPH_1, dynamicAddress, sensorData, sizeof( sensorData ) );   /* the callback reports the end */
```

### A target that asks for service

```c
const i3c_Data_t eventCode[ 1u ] = { SENSOR_DATA_READY };

(void)I3c_Set_IbiRequest( I3C_PERIPH_1, eventCode, 1u );      /* the controller reads the IBI payload and serves the target */
```

---

## Behaviour and limits

- **Pull-ups and speed.** The open-drain phases (the address, the discovery) are limited to about 1 MHz and need the pull-ups of the bus; with legacy I2C
  devices on the bus the I3C addressing runs at the I2C-compatible timing and the rate of the push-pull phase is not affected.
- **Dynamic addresses are volatile.** A target that is power-cycled loses its address; the controller re-runs the discovery (or the
  hot-join event does it). The module keeps the device table and does not assign an address that is in use.
- **Latency.** An in-band interrupt arrives as a bus event: the callback runs in the interrupt of the instance; the read of the payload and the
  follow-up transfer are started from the task level.
- **Errors.** A stuck bus (SDA held low by a device) is reported as a bus error; the recovery by the clock pulses is a task of the sensor
  layer (the module only releases its side).
- **Not part of the first version:** the HDR modes, the multi-controller arbitration beyond the secondary controller role request, the
  I3C timing of the mixed fast bus with a lot of the devices (the table of the analysis is the application's), the CCC vendor
  extensions beyond the code and data pass-through.

---

## Dependencies

- **Rcc** - peripheral clock, kernel clock source selection and frequency, reset
- **Nvic** - interrupt of the instance
- **Gpio** - SCL and SDA pins
- **Gpdma** - DMA transfers of the data and the control FIFOs
- **RAL (Register Abstraction Layer)** - used internally to access the registers

All mandatory RCC, NVIC, GPIO and DMA configurations are handled internally.

---

## Testing

- **Unit tests** (host, Unity + CMock + RegMem): the clock and the bus timing calculation, the discovery sequence against a simulated
  set of targets (one, several, a clash, none), the device table, the transfer flow with every error, the CCC encoding, the IBI and hot-join paths,
  the target callbacks, the rejected requests.
- **Integration tests** (target, probe-rs): a controller instance and a target instance of one device (two buses wired together) - the
  discovery, the private transfers, the CCCs, the in-band interrupt, a real I3C sensor on the bus, a legacy I2C device on the mixed bus.

---

## Interface conventions

The module follows the common rules of the MCAL layer, so that it behaves like the other modules:

- **Lists instead of free members.** Everything that the hardware restricts (the pins, the kernel clock sources, the DMA
  channels, the triggers and the other selectable sources) is a `typedef enum` list with exactly the items that are valid
  for the device line; a pin is an item `I3C_<SIGNAL>_PIN_<INSTANCE>_P<port><pin>` and there is an `*_UNUSED` item. A choice
  that is not in the list cannot be written.
- **Own peripheral only.** The module calls only the registers / LL functions of its own peripheral. The clocks, the
  interrupts, the pins, the DMA and the EXTI lines go through the API of the Rcc, Nvic, Gpio, Gpdma and Exti modules; a
  missing function is added to that module, it is not bypassed.
- **Init verifies itself.** `I3c_Init` checks the result at the end; when it is not as required, the module is
  deinitialized and an error is returned. The waiting for the hardware is a loop on the register state with a timeout,
  never a blind delay.
- **Transfer modes.** Where the module moves data, the POLL (served by `I3c_Task`), ISR and DMA modes have the same
  configuration structure, the same request and the same callbacks (error, half transfer, transfer complete).
- **Flags and states are enums of the module** (`i3c_FunctionState_t`, `i3c_FlagState_t`), the limits and the constants are
  named, there are no magic numbers in the interface.
- **Unsupported functions stay.** A function without a meaning in a family keeps its signature, suppresses the unused
  parameters and returns `I3C_REQUEST_ERROR`.
- **Documentation.** Every function and type has a Doxygen description, the main header has the `\defgroup`.

---

## Repository layout and branches

The master holds this README and the license. The family branches (`Dev/<Family>`, `Releases/<Family>`) hold:

| Path | Content |
|---|---|
| `README.md` | README of the family: the instances of the device, specifics, errata, status |
| `I3c_Port.h`, `I3c_Types.h` | Public interface (identical in all families) |
| `I3c*.c`, `I3c.h` | Implementation of the family |
| `CMakeLists.txt` | Library `I3c_Lib` of the EmBi platform |
| `Tests/UnitTests` | Unit tests (Unity + CMock + RegMem) |
| `Tests/IntegrationTests` | Integration tests on the target (probe-rs) |

A change of the public interface is made in all families.

---

## CMake integration

```cmake
target_link_libraries( User_Lib PRIVATE I3c_Lib )
```

1. Link `I3c_Lib` to your CMake library.
2. Include `I3c_Port.h` in your project.
3. Configure the module as needed for your hardware.

---

## License

This project is licensed under the **Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0)**.

You are free to use, modify, and share this work for **non-commercial purposes**, provided appropriate credit is given.

See [LICENSE.md](LICENSE.md) for full terms or visit [creativecommons.org/licenses/by-nc/4.0](https://creativecommons.org/licenses/by-nc/4.0/).

---

## Authors

- **Mr.Nobody** - [embedbits.com](https://embedbits.com)

Contributions are welcome! Please open a pull request.
