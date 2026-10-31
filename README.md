# FMC Peripheral Driver

This module provides an abstraction layer for the **flexible memory controller** (`FMC`, on some families `FSMC`) of STM32 MCUs: the
parallel bus to the **external static memories** (SRAM, PSRAM, NOR flash, FRAM), the **NAND flash** with the hardware ECC, the
**SDRAM** (including the initialization sequence, the refresh and the power-saving modes) and the **parallel LCD modules** in the
Intel 8080 and Motorola 6800 modes. The external memory appears in the address space of the CPU. It is a low-level MCAL
module: it owns the FMC hardware, the application and the other modules never touch the registers.

The public interface (`Fmc_Port.h`, `Fmc_Types.h`) is **identical for all supported families**. The implementation
for a family lives in its own branch (`Dev/<Family>` for development, `Releases/<Family>` for the released versions), so
a module or an application written against the interface does not change when the MCU family changes.

This README is the landing page and the **specification of the interface** (master). The family branches contain their
own README with everything that is specific to the family: the instances of the device, their settings and the status of
the implementation.

> **Status: STM32H5 implementation in progress** (branch `Dev/STM32H5`, see the section STM32H5 implementation).
> The interface below is the specification (identical for all families); the other families are planned.

---

## Intent and purpose

- An external parallel memory gives the MCU **megabytes of RAM or flash** with a simple, fast, random access bus: an SDRAM as the frame
  buffer and the heap of a graphical product, a PSRAM as a big buffer, a NOR flash for the code executed in place, a NAND flash for the storage
  of data. A parallel LCD with the 8080 interface is driven by the same signals as a static memory.
- The controller has **banks** that decode the address ranges and drive their own chip select and timing: the NOR / PSRAM banks (up to four
  chip selects), the NAND bank, the SDRAM banks. Each bank has its own memory type, data width, timing and options. The timing is the
  hard part: the user knows the memory in **nanoseconds** (the setup, hold and access times from the datasheet), the controller counts
  the **clock cycles** of the bus. The module takes the timing in ns, converts it to the cycles for the actual bus clock, adds the
  controller's own latencies, and rejects a combination that the clock cannot meet.
- The SDRAM is a state machine of its own: the clock enable, the precharge of all banks, the auto-refresh cycles, the load of the mode register (the
  CAS latency, the burst), the refresh counter. `Fmc_Set_SdramInit` runs the sequence in the right order with the delays; the refresh rate, the self-refresh
  and the power-down are controlled by functions.
- The NAND needs the **ECC** to be useful: the controller calculates the code for a page, the module returns it for the comparison with the stored
  one (the correction itself belongs to the NAND driver on the top).
- The module hides the differences between the families: the number of the banks and chip selects, the bus widths, the SDRAM support, the
  synchronous (burst) NOR / PSRAM modes and the write FIFO differ, the interface stays the same.

What the module is **not**: it does not know the commands of a NAND or a NOR flash (the memory drivers do), it does not test the
memory (a memory test is a middleware) and it does not protect the window (the MPU attributes and the Gtzc watermark are configured elsewhere).

---

## Concept

A bank is identified by `fmc_Bank_t` (`FMC_BANK_NOR_PSRAM_1` ... `_4`, `FMC_BANK_NAND`, `FMC_BANK_SDRAM_1`, `_2` - the banks of the device) and configured by:

| Kind | Items |
|---|---|
| **NOR / PSRAM / SRAM** (`fmc_NorSramConfig_t`) | The memory type, the data width (8 / 16), the address / data multiplexing, the burst access, the wait signal polarity and timing, the write enable, the extended mode (separate read and write timing), the page size for the PSRAM, the access mode A / B / C / D, the timings: address setup, address hold, data setup, bus turnaround, clock divider and data latency for the synchronous access (in ns / cycles, converted by the module). |
| **NAND** (`fmc_NandConfig_t`) | The data width, the ECC enable and the page size, the timing of the common and the attribute space (setup, wait, hold, high-Z). |
| **SDRAM** (`fmc_SdramConfig_t`) | The number of the columns and rows, the data width, the internal banks, the CAS latency, the read burst and the pipe, the timings (`tMRD`, `tXSR`, `tRAS`, `tRC`, `tWR`, `tRP`, `tRCD`), the refresh rate (the time for the whole array), the clock divider (the SDRAM clock is `HCLK / 2` or `/ 3`). |
| **LCD** | A NOR / PSRAM bank with the 8080 / 6800 timing; the command and the data differ by one address line. |

Rules:

1. `Fmc_Init` enables the controller; every bank is configured by its function (`Fmc_Set_NorSramConfig`, `Fmc_Set_NandConfig`, `Fmc_Set_SdramConfig`) and enabled
   by `Fmc_Set_BankActive`. The bank configuration is changed only while the bank is disabled.
2. The memory becomes visible **after** the bank is enabled (and, for the SDRAM, after `Fmc_Set_SdramInit` succeeded). `Fmc_Get_BankBaseAddress` returns the
   base address of the window.
3. The timing is given in **nanoseconds**; the module reads the bus clock from the RCC module and converts. A timing that needs more cycles than the
   register field has, or fewer than the controller needs, is rejected before a register is written.
4. `Fmc_Set_SdramInit` runs the standard initialization (the clock enable, the delay of at least 100 us, the precharge all, the auto-refresh cycles, the
   load of the mode register, the refresh rate) and returns when the SDRAM is ready (the status flag with a timeout). Self-refresh and power-down are
   `Fmc_Set_SdramSelfRefresh`, `Fmc_Set_SdramPowerDown`; `Fmc_Set_SdramCommand` sends a raw command.
5. The NAND ECC: `Fmc_Set_NandEccActive` starts the calculation for the next page transfer, `Fmc_Get_NandEcc` returns the code after the page was transferred;
   `Fmc_Get_NandReadyState` reads the ready / busy signal.
6. The write protection of a bank (`Fmc_Set_BankWriteProtection`) makes the SDRAM bank read-only (the hardware write-protect bit).
7. All parameters are checked before a register is written.

---

## Supported families

Every family gets its own story, its own tasks and its own branches `Dev/<Family>` and `Releases/<Family>` cut from this
master.

| Family | Branch | Controller | Status |
|---|---|---|---|
| STM32H5 | `Dev/STM32H5` | `FMC` on STM32H523, H533, H543, H553, H562, H563, H573 and the H5E / H5F lines: SRAM / PSRAM (4 banks) / NOR / FRAM, NAND with ECC (up to 8 KB), SDRAM / mobile LPSDR (not on STM32H523 / H533), 8080 / 6800 LCD, 8- and 16-bit bus (32-bit on the H5E / H5F lines), burst, the write FIFO of 16 words and the read FIFO of 6 words for the SDRAM; not on STM32H503 | implementation in progress (unit tests on 12 lines, integration tests compiled, not run on hardware) |
| STM32U5 | `Dev/STM32U5` | `FMC` on the lines that have it | planned |
| STM32H7 | `Dev/STM32H7` | `FMC` with the SDRAM banks | planned |
| STM32F7 | `Dev/STM32F7` | `FMC` with the SDRAM banks | planned |
| STM32F4 | `Dev/STM32F4` | `FMC` (the SDRAM lines) or `FSMC` (the lines without the SDRAM) | planned |
| STM32G4, STM32L4 | `Dev/STM32G4`, `Dev/STM32L4` | `FMC` on the lines that have it (static memories and NAND; SDRAM where present) | planned |
| STM32U3 | `Dev/STM32U3` | device dependent | planned |

A function without a meaning in a family stays in the interface, returns `FMC_REQUEST_ERROR` and suppresses the unused
parameters (for example the SDRAM on a controller without it, the synchronous burst mode of the NOR on a device that has the
asynchronous access only, a bank the device does not have).

---

## Public interface in short (proposal)

| Group | Functions |
|---|---|
| Module management | `Fmc_Get_ModuleVersion`, `Fmc_Init`, `Fmc_Deinit`, `Fmc_Task`, `Fmc_Get_DefaultConfig` |
| Banks | `Fmc_Set_NorSramConfig`, `Fmc_Set_NandConfig`, `Fmc_Set_SdramConfig`, `Fmc_Set_BankActive`, `Fmc_Set_BankInactive`, `Fmc_Get_BankBaseAddress`, `Fmc_Set_BankWriteProtection` |
| SDRAM | `Fmc_Set_SdramInit`, `Fmc_Set_SdramRefreshRate`, `Fmc_Set_SdramSelfRefresh`, `Fmc_Set_SdramPowerDown`, `Fmc_Set_SdramCommand`, `Fmc_Get_SdramState` |
| NAND | `Fmc_Set_NandEccActive`, `Fmc_Get_NandEcc`, `Fmc_Get_NandReadyState` |
| Interrupts | `Fmc_Set_IrqPriority` / `Get`, the callbacks are a part of the configuration |
| Pins | `Fmc_InitAddressGpio`, `Fmc_InitDataGpio`, `Fmc_InitControlGpio`, `Fmc_InitSdramGpio` |

```c
fmc_ModuleVersion_t Fmc_Get_ModuleVersion      ( void );

fmc_RequestState_t  Fmc_Init                   ( const fmc_Config_t * const fmcConfig );
fmc_RequestState_t  Fmc_Deinit                 ( void );
void                Fmc_Task                   ( void );
fmc_RequestState_t  Fmc_Get_DefaultConfig      ( fmc_Config_t * const fmcConfig );

fmc_RequestState_t  Fmc_Set_NorSramConfig      ( fmc_Bank_t bank, const fmc_NorSramConfig_t * const norSramConfig );
fmc_RequestState_t  Fmc_Set_NandConfig         ( const fmc_NandConfig_t * const nandConfig );
fmc_RequestState_t  Fmc_Set_SdramConfig        ( fmc_Bank_t bank, const fmc_SdramConfig_t * const sdramConfig );
fmc_RequestState_t  Fmc_Set_BankActive         ( fmc_Bank_t bank );
fmc_RequestState_t  Fmc_Set_BankInactive       ( fmc_Bank_t bank );
fmc_RequestState_t  Fmc_Get_BankBaseAddress    ( fmc_Bank_t bank, fmc_Address_t * const baseAddress );

fmc_RequestState_t  Fmc_Set_SdramInit          ( fmc_Bank_t bank );
fmc_RequestState_t  Fmc_Set_SdramSelfRefresh   ( fmc_Bank_t bank );
fmc_RequestState_t  Fmc_Get_SdramState         ( fmc_Bank_t bank, fmc_SdramState_t * const sdramState );

fmc_RequestState_t  Fmc_Set_NandEccActive      ( void );
fmc_RequestState_t  Fmc_Get_NandEcc            ( fmc_Ecc_t * const ecc );
```

Every function returns `FMC_REQUEST_OK` / `FMC_REQUEST_ERROR` and checks its parameters and the state of the module
before it touches a register.

| Type | Meaning |
|---|---|
| `fmc_Config_t` | The write FIFO and the clock settings, the table of the banks to configure at the start, the pins, the callbacks, the interrupt priority |
| `fmc_Time_ns_t` | A time in nanoseconds - the unit of every timing field |
| `fmc_SdramState_t` | `FMC_SDRAM_NORMAL`, `FMC_SDRAM_SELF_REFRESH`, `FMC_SDRAM_POWER_DOWN`, `FMC_SDRAM_BUSY` |
| `fmc_Ecc_t` | The error correcting code of a page |
| Callbacks | `NandReadyCallback`, `RefreshErrorCallback`, `ErrorCallback` |

---

## Usage

### A 16-bit SDRAM as the frame buffer

```c
#include "Fmc_Port.h"

fmc_Config_t      fmcConfig;
fmc_SdramConfig_t sdram =
{
    .ColumnBits = 9u,  .RowBits = 13u,  .DataWidth = FMC_DATA_WIDTH_16,  .InternalBanks = 4u,
    .CasLatency = FMC_CAS_LATENCY_3,
    .LoadToActiveDelay_ns = 14u, .ExitSelfRefreshDelay_ns = 70u, .SelfRefreshTime_ns = 42u,
    .RowCycleDelay_ns = 63u,     .WriteRecoveryTime_ns = 14u,    .RpDelay_ns = 14u, .RcdDelay_ns = 14u,
    .RefreshPeriod_ms = 64u,     .RefreshRows = 8192u,
};
fmc_Address_t     sdramBase = 0u;

(void)Fmc_Get_DefaultConfig( &fmcConfig );
(void)Fmc_Init( &fmcConfig );
(void)Fmc_Set_SdramConfig( FMC_BANK_SDRAM_1, &sdram );
(void)Fmc_Set_SdramInit( FMC_BANK_SDRAM_1 );                    /* clock enable, precharge, refresh, mode register */
(void)Fmc_Get_BankBaseAddress( FMC_BANK_SDRAM_1, &sdramBase );  /* the memory is used as normal RAM from here */
```

### An 8080 LCD on a NOR / PSRAM bank

```c
fmc_NorSramConfig_t lcd =
{
    .MemoryType = FMC_MEMORY_SRAM, .DataWidth = FMC_DATA_WIDTH_16, .AddressSetup_ns = 10u, .DataSetup_ns = 30u,
};

(void)Fmc_Set_NorSramConfig( FMC_BANK_NOR_PSRAM_1, &lcd );
(void)Fmc_Set_BankActive( FMC_BANK_NOR_PSRAM_1 );
```

---

## Behaviour and limits

- **Timing.** The bus clock limits the resolution of the timing; a memory that needs a timing the clock cannot make is rejected, not approximated
  to a shorter one. The access times include the board delay - the margin is added by the caller.
- **Memory attributes.** The external window should be a normal, cacheable (or device) memory in the MPU as the use requires; the SDRAM is not
  coherent with the DMA masters without the cache maintenance.
- **SDRAM sequence.** The initialization is sequential and takes at least the power-up delay; the refresh rate is calculated from the
  total refresh time and the number of the rows and applies the safety margin that the reference manual prescribes.
- **NAND.** The ready / busy line is a signal the NAND driver waits for (the callback or the poll); the module does not interpret the commands.
- **Pins.** A large pin count (address, data, control) is configured from the lists of the device; the pins that the bank does not need are left alone.
- **Not part of the first version:** the memory test, the NOR flash CFI parsing, the software ECC correction, the byte-wise write protection of the SRAM windows (the Gtzc module).

---

## Dependencies

- **Rcc** - peripheral clock, the bus clock frequency for the timing conversion
- **Nvic** - interrupt
- **Gpio** - address, data and control pins
- **Gtzc** - the watermark protection of the external window (configured by its module)
- **RAL (Register Abstraction Layer)** - used internally to access the registers

All mandatory RCC, NVIC and GPIO configurations are handled internally.

---

## Testing

- **Unit tests** (host, Unity + CMock + RegMem): the conversion of the ns timings to the cycles for the bus clocks (limits, rounding up,
  rejection), the register mapping of the NOR / SRAM, NAND and SDRAM configurations, the SDRAM init sequence order and its timeouts, the refresh counter
  calculation, the self-refresh and the power-down, the ECC read, the rejected changes of an enabled bank.
- **Integration tests** (target, probe-rs): a write - read pattern test of an SRAM / PSRAM and an SDRAM at several clock rates, the SDRAM refresh
  retention over a longer time, a NAND page ECC, an LCD register read / write.

---

## Interface conventions

The module follows the common rules of the MCAL layer, so that it behaves like the other modules:

- **Lists instead of free members.** Everything that the hardware restricts (the pins, the kernel clock sources, the DMA
  channels, the triggers and the other selectable sources) is a `typedef enum` list with exactly the items that are valid
  for the device line; a pin is an item `FMC_<SIGNAL>_PIN_<INSTANCE>_P<port><pin>` and there is an `*_UNUSED` item. A choice
  that is not in the list cannot be written.
- **Own peripheral only.** The module calls only the registers / LL functions of its own peripheral. The clocks, the
  interrupts, the pins, the DMA and the EXTI lines go through the API of the Rcc, Nvic, Gpio, Gpdma and Exti modules; a
  missing function is added to that module, it is not bypassed.
- **Init verifies itself.** `Fmc_Init` checks the result at the end; when it is not as required, the module is
  deinitialized and an error is returned. The waiting for the hardware is a loop on the register state with a timeout,
  never a blind delay.
- **Transfer modes.** Where the module moves data, the POLL (served by `Fmc_Task`), ISR and DMA modes have the same
  configuration structure, the same request and the same callbacks (error, half transfer, transfer complete).
- **Flags and states are enums of the module** (`fmc_FunctionState_t`, `fmc_FlagState_t`), the limits and the constants are
  named, there are no magic numbers in the interface.
- **Unsupported functions stay.** A function without a meaning in a family keeps its signature, suppresses the unused
  parameters and returns `FMC_REQUEST_ERROR`.
- **Documentation.** Every function and type has a Doxygen description, the main header has the `\defgroup`.

---

## Repository layout and branches

The master holds this README and the license. The family branches (`Dev/<Family>`, `Releases/<Family>`) hold:

| Path | Content |
|---|---|
| `README.md` | README of the family: the instances of the device, specifics, errata, status |
| `Fmc_Port.h`, `Fmc_Types.h` | Public interface (identical in all families) |
| `Fmc*.c`, `Fmc.h` | Implementation of the family |
| `CMakeLists.txt` | Library `Fmc_Lib` of the EmBi platform |
| `Tests/UnitTests` | Unit tests (Unity + CMock + RegMem) |
| `Tests/IntegrationTests` | Integration tests on the target (probe-rs) |

A change of the public interface is made in all families.

---

## CMake integration

```cmake
target_link_libraries( User_Lib PRIVATE Fmc_Lib )
```

1. Link `Fmc_Lib` to your CMake library.
2. Include `Fmc_Port.h` in your project.
3. Configure the module as needed for your hardware.

---

## STM32H5 implementation

Branch `Dev/STM32H5`, library `Fmc_Lib`. Status: the NOR / PSRAM / SRAM banks, the interrupt and the pins (`Fmc.c`), the SDRAM
banks (`Fmc_Sdram.c`) and the NAND bank with the ECC (`Fmc_Nand.c`) are implemented. The unit tests (73 cases) pass on all 12 lines;
the integration tests are only compiled and linked (STM32H503, STM32H563), nothing was run on hardware.

### Device lines

| Line | NOR / PSRAM / SRAM banks (4) | NAND bank | SDRAM banks (2) | Data bus |
|---|---|---|---|---|
| STM32H503 | - (every function returns `FMC_REQUEST_ERROR`) | - | - | - |
| STM32H523, H533 | yes | yes | - | 8 / 16 bit |
| STM32H543, H553, H562, H563, H573 | yes | yes | yes | 8 / 16 bit |
| STM32H5E4, H5E5, H5F4, H5F5 | yes | yes | yes | 8 / 16 / 32 bit (`FMC_DATA_WIDTH_32`, not for the NAND) |

The banks of `fmc_Bank_t` and the 32-bit width exist only on the lines that have them. The kernel clock of the controller is
the bus clock `HCLK`, read from the Rcc module (`RCC_PERIPH_FMC`).

### Files

| File | Content |
|---|---|
| `Fmc_Types.h`, `Fmc_Port.h` | Public interface |
| `Fmc_Pins.h` | Pin lists (address, data, control, SDRAM), generated from the STM32CubeMX database, guarded per device line |
| `Fmc.h` | Private declarations: run-time state, conversions of the timing to register values (pure functions) |
| `Fmc.c` | Module management, NOR / PSRAM / SRAM banks, bank activation, write protection, interrupt, pins |
| `Fmc_Sdram.c` | SDRAM banks: configuration, initialization sequence, refresh rate, self-refresh, power-down, raw command, state |
| `Fmc_Nand.c` | NAND bank: configuration, ECC, ready / busy signal and its events |
| `Fmc_Reg.c`, `Fmc_Reg.h` | The only place that touches the registers: bounds checked accessors, the controller enable protocol |

### Timing and the controller enable

- Every time is converted to cycles of the bus clock by `ceil( time * HCLK / 10^9 )` (a time is never approximated to a shorter
  one) and raised to the smallest value the controller can generate (`DATAST` and `ADDHLD` at least one cycle, `ADDSET` at least
  one cycle for a multiplexed bus, a PSRAM and the access mode D). A time that needs more cycles than its register field holds is
  rejected before any register is written, the same for the data latency (2 to 17 `FMC_CLK` cycles) and the `FMC_CLK` period
  (1 to 16 bus clock cycles).
- The register offsets of RM0481 are applied by the module: the bus turnaround is `BUSTURN + 1` cycles, the data hold of a write
  is `DATAHLD + 1` cycles (the read data hold is the register value, so one `DATAHLD` serves both when the extended mode is not used).
- The controller registers except the on-the-fly bits are changed with the controller disabled (RM0481 23.2): a configuration
  function disables `FMCEN`, writes the registers, enables it again and verifies the registers by read-back. No bank may be accessed
  while such a function runs (an access to a bank of the disabled controller is a bus error).
- `Fmc_Init` disables all banks (bank 1 is enabled after the reset), sets the write FIFO and the continuous clock, enables the
  controller and verifies `BCR1` by read-back; on a failure the module is deinitialized.

### SDRAM

- `Fmc_Set_SdramConfig` converts every time to cycles of the SDRAM clock (`HCLK` divided by `ClockDivider`, exact `ceil( time * HCLK /
  ( divider * 10^9 ) )`), raises it to the smallest value of the controller (tMRD 3 cycles, the other timings 2 cycles) and rejects a time
  above the register field (tMRD 18 cycles, the others 17). tWR is raised to satisfy `tWR >= tRAS - tRCD` and `tWR >= tRC - tRCD - tRP`
  (RM0481 23.8.5); the call is rejected when that needs more than 17 cycles. The refresh period must be meetable by the clock too (the
  refresh count has to be 41 .. 8191), otherwise nothing is written.
- Two SDRAM banks: the clock divider, the read burst and the read pipe are common (they live in `FMC_SDCR1`), the second bank must carry the
  same values. The timings the controller shares between the devices (tRP and tRC in `FMC_SDTR1`, tWR and tXSR in both registers, tMRD and
  tRAS of bank 1 for the commands to both banks) are the slowest of both devices; the register of a bank that is not used keeps its reset value.
  Both banks are configured before one of them is activated; any new configuration makes a new `Fmc_Set_SdramInit` necessary.
- `Fmc_Set_SdramInit( bank )` runs the sequence of RM0481 23.8.3 for **every configured SDRAM bank at once** (the precharge all and the
  auto-refresh must reach both devices simultaneously): clock enable, power-up delay, precharge all, 8 auto-refresh cycles, load of the mode
  register of each bank (burst length 1, sequential, the CAS latency of the bank, standard operation, single location write), refresh count.
  Every wait for the controller is a bounded loop on `FMC_SDSR.BUSY`. The power-up delay is a time requirement of the memory, not a flag of
  the controller: it is a busy loop derived from the bus clock (never shorter than requested), at least 100 us (`PowerUpDelay_us` of the
  configuration extends it).
- Refresh: `COUNT = floor( RefreshPeriod_ms * SDRAM clock / RefreshRows / 1000 ) - 20` (rounded down, the safety margin of the reference
  manual), decreased by one when it equals the sum of the timings plus four cycles (RM0481 note); `Fmc_Set_SdramRefreshRate` writes it
  again. The refresh error interrupt is enabled when the event mode is `FMC_EVENT_MODE_ISR` and a refresh error or error callback exists, in
  `FMC_EVENT_MODE_POLL` `Fmc_Task` reads and clears the flag.
- `Fmc_Set_BankActive` of an SDRAM bank sends the normal mode command (also the exit of the self-refresh and the power-down) and needs the
  initialization; `Fmc_Set_SdramSelfRefresh` and `Fmc_Set_SdramPowerDown` wait for the mode reported by the controller. `Fmc_Set_SdramCommand`
  sends one raw command to one bank. `Fmc_Get_SdramState` has no side effect.
- Limits: the extended mode register of a mobile LPSDR SDRAM (dummy read with BA1 = 1) is not programmed by the module. A precharge all
  command to bank 1 is needed before an access to bank 2 (RM0481 23.8.4), the application issues `FMC_SDRAM_COMMAND_PRECHARGE_ALL` for it.
  The module does not know the board: the write recovery and the other times include no board delay.

### NAND

- `Fmc_Set_NandConfig` converts the timing of the common and of the attribute memory space (setup, wait, hold and data bus high-Z time)
  to bus clock cycles (setup 1 .. 255, wait 2 .. 255, hold 1 .. 254, high-Z 1 .. 255 cycles; the registers hold cycles - 1, cycles - 1,
  cycles and cycles - 1) and the delays of ALE and CLE to the read strobe to the `TAR` / `TCLR` fields (`delay = ( field + setup field + 2 )`
  cycles, the smaller setup field of both spaces, field 0 .. 15). Nothing is written when a time does not fit. The hold time is converted
  for the write access (a read has two more cycles of hold), so it is met for both. The data width is 8 or 16 bits.
- ECC: `Fmc_Set_NandEccActive` resets and enables the ECC logic (the NAND bank must be configured with `EccEnable`), `Fmc_Get_NandEcc`
  waits (bounded) until the write FIFO of the controller is empty, reads `FMC_ECCR`, keeps the bits valid for the page size (22 bits for 256
  bytes, two more for every doubling up to 32 bits for 8192 bytes) and stops the calculation, as the reference manual prescribes (the
  code is read once per calculation). The comparison with the stored code and the correction belong to the NAND driver.
- `Fmc_Get_NandReadyState` reads the ready / busy signal through the high level detection of the controller (it enables the detection
  for the call, clears the flag and polls it a bounded number of times; the flag of the high level event is cleared by the call).
  The events (rising edge, falling edge, high level) are reported by `NandReadyCallback` from `Fmc_Task` or the interrupt; the high level
  event repeats while the signal stays high.
- Not verified: the behaviour of the high level flag against a real NAND (the module relies on the flag being set by the hardware while the
  signal is high) and the ECC bit widths of the reference manual table (the text of the table is damaged in the extracted manual and the
  widths are taken from its pattern); both need a board.

### Notes and deviations from the specification

- The specification table lists no FMC on STM32H543 / H553. The CMSIS device headers, the STM32CubeMX database and RM0481 have the FMC
  (with the SDRAM controller) on these lines, so they are implemented; the datasheets were not available to confirm it.
- A time below the controller minimum is raised to the minimum, not rejected (a minimum time of a memory is still met); only a time
  above the register field is rejected.
- A configuration field is a list or a typed value: `ColumnBits`, `RowBits`, `InternalBanks`, `DataWidth` and `CasLatency` are lists whose
  items carry the number (`FMC_SDRAM_COLUMN_BITS_9 = 9u`), so a plain number from the specification example is accepted when it is an item.
- `fmc_NorSramConfig_t` adds `DataHold_ns`, `ByteLaneSetup_ns`, `ChipSelectMaxLow_ns` and `WriteTiming` (the separate write timing of the
  extended mode); a zero-initialized structure selects the safe defaults (SRAM, writes enabled, asynchronous, no wait signal).
- `fmc_Config_t` holds the pointers of the bank configurations to apply at the start; they are read by `Fmc_Init` only. The banks are
  configured, not activated: `Fmc_Set_BankActive` enables a bank.
- `Fmc_Set_BankInactive` rejects an SDRAM bank (the controller has no enable bit); the self-refresh and the power-down end with
  `Fmc_Set_BankActive`.
- The SDRAM clock divider list is `FMC_SDRAM_CLOCK_DIV_2` (default, value 0), `_3` and `_1` (the controller of STM32H5 also allows the
  bus clock itself, the specification names `/ 2` and `/ 3`). `Fmc_Set_SdramRefreshRate` takes the refresh period and the number of
  rows, `Fmc_Set_SdramCommand` takes a command structure (`fmc_SdramCommand_t`); the configuration adds `PowerUpDelay_us`.
- `Fmc_Set_BankWriteProtection` takes the protection state as a second parameter and also protects a NOR / PSRAM / SRAM bank (`WREN`).
- The pins: `fmc_AddressPin_t`, `fmc_DataPin_t`, `fmc_ControlPin_t`, `fmc_SdramPin_t` list every valid pad (128 items). The multiplexed
  `FMC_DA0` .. `FMC_DA15` are the pads of `FMC_D0` .. `FMC_D15` and have no list of their own; `FMC_NBL0` .. `FMC_NBL3` (also the SDRAM
  byte mask) are in the control list. `Fmc_Init` configures the pins of the lists of `fmc_Config_t`.

---

## License

This project is licensed under the **Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0)**.

You are free to use, modify, and share this work for **non-commercial purposes**, provided appropriate credit is given.

See [LICENSE.md](LICENSE.md) for full terms or visit [creativecommons.org/licenses/by-nc/4.0](https://creativecommons.org/licenses/by-nc/4.0/).

---

## Authors

- **Mr.Nobody** - [embedbits.com](https://embedbits.com)

Contributions are welcome! Please open a pull request.
