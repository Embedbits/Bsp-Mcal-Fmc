/**
 * \author Mr.Nobody
 * \defgroup Fmc Fmc
 * \brief Fmc module
 */

/**
 * \author Mr.Nobody
 * \file Fmc_Types.h
 * \ingroup Fmc
 * \brief Fmc module global types definition
 *
 * This file contains the types definitions used across the module and are
 * available for other modules through the Port file. STM32H5 has the flexible
 * memory controller on every line except STM32H503 (no FMC at all, every function
 * returns an error). The NOR / PSRAM / SRAM banks and the NAND bank exist on all
 * the other lines, the SDRAM banks from STM32H543 on (not on STM32H523 / H533). The
 * 32-bit data bus exists on the STM32H5E / H5F lines only.
 *
 */

#ifndef FMC_FMC_TYPES_H
#define FMC_FMC_TYPES_H
/* ============================== INCLUDES ================================== */
#include "stdint.h"                         /* Module types definition        */
#include "Gpio_Port.h"                      /* GPIO Port module definitions   */
#include "Nvic_Port.h"                      /* NVIC Port module definitions   */
/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Null pointer definition */
#define FMC_NULL_PTR                          ( ( void* ) 0u )

/** Number of the NOR / PSRAM / SRAM banks (chip selects NE1 .. NE4) */
#define FMC_NOR_BANK_CNT                      ( 4u )

/** Number of the SDRAM banks (SDNE0 / SDNE1) */
#define FMC_SDRAM_BANK_CNT                    ( 2u )

/** Number of the address lines (A0 .. A25), size of the address pin array of fmc_Config_t */
#define FMC_ADDRESS_LINE_CNT                  ( 26u )

/** Number of the data lines (D0 .. D31), size of the data pin array of fmc_Config_t */
#define FMC_DATA_LINE_CNT                     ( 32u )

/** Number of the control signals, size of the control pin array of fmc_Config_t */
#define FMC_CONTROL_PIN_CNT                   ( 17u )

/** Number of the SDRAM signals, size of the SDRAM pin array of fmc_Config_t */
#define FMC_SDRAM_PIN_CNT                     ( 10u )

/** Highest value of the SDRAM mode register (13 bits) */
#define FMC_SDRAM_MODE_REGISTER_MAX           ( 0x1FFFu )

/** Highest count of the consecutive auto-refresh commands of a raw SDRAM command */
#define FMC_SDRAM_AUTO_REFRESH_CNT_MAX        ( 16u )

/** Alternate function identification bit offset in an encoded pin value */
#define FMC_PIN_BIT_MASK_AF_BIT_OFFSET        ( 0u )

/** Pin identification bit offset in an encoded pin value */
#define FMC_PIN_BIT_MASK_PIN_BIT_OFFSET       ( 5u )

/** Port identification bit offset in an encoded pin value */
#define FMC_PIN_BIT_MASK_PORT_BIT_OFFSET      ( 10u )

/** Signal identification bit offset in an encoded pin value */
#define FMC_PIN_BIT_MASK_SIGNAL_BIT_OFFSET    ( 15u )

/* ========================== EXPORTED MACROS =============================== */

/** Encode a pin configuration (signal, port, pin, alternate function) into a single bit-mask */
#define FMC_PIN_BIT_MASK_ENCODE( SIGNAL_ID, PORT_ID, PIN_ID, AF_ID ) \
            ( ( (uint32_t)(SIGNAL_ID) << FMC_PIN_BIT_MASK_SIGNAL_BIT_OFFSET ) | \
              ( (uint32_t)(PORT_ID)   << FMC_PIN_BIT_MASK_PORT_BIT_OFFSET   ) | \
              ( (uint32_t)(PIN_ID)    << FMC_PIN_BIT_MASK_PIN_BIT_OFFSET    ) | \
              ( (uint32_t)(AF_ID)     << FMC_PIN_BIT_MASK_AF_BIT_OFFSET     )   )

/** Encode a pin of the generated lists, the signal and the GPIO identifications are given without their prefix */
#define FMC_PIN_ENCODE( SIGNAL, PORT, PIN, AF ) \
            FMC_PIN_BIT_MASK_ENCODE( FMC_SIGNAL_ ## SIGNAL, \
                                     GPIO_PORT_ ## PORT, \
                                     GPIO_PIN_ID_ ## PIN, \
                                     GPIO_ALT_FUNC_ ## AF )

/** Value of the *_PIN_UNUSED items of the pin lists */
#define FMC_PIN_UNUSED \
            FMC_PIN_BIT_MASK_ENCODE( FMC_SIGNAL_CNT, GPIO_PORT_CNT, GPIO_PIN_ID_CNT, GPIO_ALT_FUNC_CNT )

/** Extract the signal identification from an encoded pin value */
#define FMC_PIN_BIT_MASK_DECODE_SIGNAL( CODED_VAL ) \
            ( ( (uint32_t)(CODED_VAL) >> FMC_PIN_BIT_MASK_SIGNAL_BIT_OFFSET ) & 0x7Fu )

/** Extract the port identification from an encoded pin value */
#define FMC_PIN_BIT_MASK_DECODE_PORT( CODED_VAL ) \
            ( ( (uint32_t)(CODED_VAL) >> FMC_PIN_BIT_MASK_PORT_BIT_OFFSET ) & 0x1Fu )

/** Extract the pin identification from an encoded pin value */
#define FMC_PIN_BIT_MASK_DECODE_PIN( CODED_VAL ) \
            ( ( (uint32_t)(CODED_VAL) >> FMC_PIN_BIT_MASK_PIN_BIT_OFFSET ) & 0x1Fu )

/** Extract the alternate function identification from an encoded pin value */
#define FMC_PIN_BIT_MASK_DECODE_AF( CODED_VAL ) \
            ( ( (uint32_t)(CODED_VAL) >> FMC_PIN_BIT_MASK_AF_BIT_OFFSET ) & 0x1Fu )

/* ============================== TYPEDEFS ================================== */

/** Major version number of the module */
typedef uint8_t fmc_MajorVersion_t;

/** Minor version number of the module */
typedef uint8_t fmc_MinorVersion_t;

/** Patch version number of the module */
typedef uint8_t fmc_PatchVersion_t;

/** Module version */
typedef struct
{
    fmc_MajorVersion_t Major; /**< Major version number */
    fmc_MinorVersion_t Minor; /**< Minor version number */
    fmc_PatchVersion_t Patch; /**< Patch version number */
}   fmc_ModuleVersion_t;

/** Result of a module request */
typedef enum
{
    FMC_REQUEST_OK = 0u, /**< The request succeeded                                           */
    FMC_REQUEST_ERROR    /**< The request failed (bad parameter, bad state, hardware timeout) */
}   fmc_RequestState_t;

/** Generic active / inactive function state of the module */
typedef enum
{
    FMC_FUNCTION_INACTIVE = 0u, /**< The function is inactive */
    FMC_FUNCTION_ACTIVE         /**< The function is active   */
}   fmc_FunctionState_t;

/** Generic active / inactive flag state of the module */
typedef enum
{
    FMC_FLAG_INACTIVE = 0u, /**< The flag is not set */
    FMC_FLAG_ACTIVE         /**< The flag is set     */
}   fmc_FlagState_t;

/** List of the banks of the device line (the NOR / PSRAM / SRAM banks and the NAND bank on every line with the FMC,
 *  the SDRAM banks from STM32H543 on) */
typedef enum
{
#if defined(FMC_Bank1_R)
    FMC_BANK_NOR_PSRAM_1 = 0u, /**< NOR / PSRAM / SRAM bank 1 (chip select NE1) */
    FMC_BANK_NOR_PSRAM_2,      /**< NOR / PSRAM / SRAM bank 2 (chip select NE2) */
    FMC_BANK_NOR_PSRAM_3,      /**< NOR / PSRAM / SRAM bank 3 (chip select NE3) */
    FMC_BANK_NOR_PSRAM_4,      /**< NOR / PSRAM / SRAM bank 4 (chip select NE4) */
    FMC_BANK_NAND,             /**< NAND flash bank (bank 3 of the address map)  */
#endif
#if defined(FMC_Bank5_6_R)
    FMC_BANK_SDRAM_1,          /**< SDRAM bank 1 (chip select SDNE0)             */
    FMC_BANK_SDRAM_2,          /**< SDRAM bank 2 (chip select SDNE1)             */
#endif
    FMC_BANK_CNT               /**< Number of banks of the device line           */
}   fmc_Bank_t;

/** Type of the memory on a NOR / PSRAM / SRAM bank */
typedef enum
{
    FMC_MEMORY_SRAM = 0u, /**< SRAM or ROM                                                       */
    FMC_MEMORY_PSRAM,     /**< PSRAM (CellularRAM), asynchronous or burst                        */
    FMC_MEMORY_NOR,       /**< NOR flash, asynchronous or burst                                  */
    FMC_MEMORY_FRAM,      /**< FRAM (like an SRAM, the bus turnaround time is the precharge time) */
    FMC_MEMORY_CNT        /**< Number of memory type items                                       */
}   fmc_MemoryType_t;

/** Data bus width of a memory (the value is the width in bits) */
typedef enum
{
    FMC_DATA_WIDTH_8  = 8u,  /**< 8-bit data bus  */
    FMC_DATA_WIDTH_16 = 16u, /**< 16-bit data bus */
#if defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
    FMC_DATA_WIDTH_32 = 32u, /**< 32-bit data bus (the STM32H5E / H5F lines only, not for the NAND) */
#endif
}   fmc_DataWidth_t;

/** Asynchronous access mode of the NOR / PSRAM / SRAM controller (with the extended mode) */
typedef enum
{
    FMC_ACCESS_MODE_A = 0u, /**< Access mode A (output enable toggles with the chip select low)  */
    FMC_ACCESS_MODE_B,      /**< Access mode B (NOR flash)                                       */
    FMC_ACCESS_MODE_C,      /**< Access mode C (NOR flash, output enable delay)                  */
    FMC_ACCESS_MODE_D,      /**< Access mode D (address hold phase, extended address)            */
    FMC_ACCESS_MODE_CNT     /**< Number of access mode items                                     */
}   fmc_AccessMode_t;

/** Page size of a PSRAM (CellularRAM 1.5), a burst is split on the page boundary */
typedef enum
{
    FMC_PAGE_SIZE_NONE = 0u, /**< No split of a burst on a page boundary */
    FMC_PAGE_SIZE_128,       /**< 128 bytes                              */
    FMC_PAGE_SIZE_256,       /**< 256 bytes                              */
    FMC_PAGE_SIZE_512,       /**< 512 bytes                              */
    FMC_PAGE_SIZE_1024,      /**< 1024 bytes                             */
    FMC_PAGE_SIZE_CNT        /**< Number of page size items              */
}   fmc_PageSize_t;

/** Polarity of the wait signal NWAIT of the memory */
typedef enum
{
    FMC_WAIT_POLARITY_LOW = 0u, /**< NWAIT active low  */
    FMC_WAIT_POLARITY_HIGH,     /**< NWAIT active high */
    FMC_WAIT_POLARITY_CNT       /**< Number of wait polarity items */
}   fmc_WaitPolarity_t;

/** Timing of the wait signal NWAIT of a synchronous memory */
typedef enum
{
    FMC_WAIT_TIMING_BEFORE_STATE = 0u, /**< NWAIT is active one data cycle before the wait state */
    FMC_WAIT_TIMING_DURING_STATE,      /**< NWAIT is active during the wait state                 */
    FMC_WAIT_TIMING_CNT                /**< Number of wait timing items                           */
}   fmc_WaitTiming_t;

/** Page size of the NAND flash for the ECC calculation (one code per page) */
typedef enum
{
    FMC_ECC_PAGE_SIZE_256 = 0u, /**< 256 bytes  */
    FMC_ECC_PAGE_SIZE_512,      /**< 512 bytes  */
    FMC_ECC_PAGE_SIZE_1024,     /**< 1024 bytes */
    FMC_ECC_PAGE_SIZE_2048,     /**< 2048 bytes */
    FMC_ECC_PAGE_SIZE_4096,     /**< 4096 bytes */
    FMC_ECC_PAGE_SIZE_8192,     /**< 8192 bytes */
    FMC_ECC_PAGE_SIZE_CNT       /**< Number of ECC page size items */
}   fmc_EccPageSize_t;

/** Number of the column address bits of an SDRAM (the value is the number of bits) */
typedef enum
{
    FMC_SDRAM_COLUMN_BITS_8  = 8u,  /**< 8 column address bits  */
    FMC_SDRAM_COLUMN_BITS_9  = 9u,  /**< 9 column address bits  */
    FMC_SDRAM_COLUMN_BITS_10 = 10u, /**< 10 column address bits */
    FMC_SDRAM_COLUMN_BITS_11 = 11u  /**< 11 column address bits */
}   fmc_SdramColumnBits_t;

/** Number of the row address bits of an SDRAM (the value is the number of bits) */
typedef enum
{
    FMC_SDRAM_ROW_BITS_11 = 11u, /**< 11 row address bits */
    FMC_SDRAM_ROW_BITS_12 = 12u, /**< 12 row address bits */
    FMC_SDRAM_ROW_BITS_13 = 13u  /**< 13 row address bits */
}   fmc_SdramRowBits_t;

/** Number of the internal banks of an SDRAM (the value is the number of banks) */
typedef enum
{
    FMC_SDRAM_INTERNAL_BANKS_2 = 2u, /**< Two internal banks   */
    FMC_SDRAM_INTERNAL_BANKS_4 = 4u  /**< Four internal banks  */
}   fmc_SdramInternalBanks_t;

/** CAS latency of an SDRAM (the value is the number of memory clock cycles) */
typedef enum
{
    FMC_CAS_LATENCY_1 = 1u, /**< CAS latency of 1 cycle  */
    FMC_CAS_LATENCY_2 = 2u, /**< CAS latency of 2 cycles */
    FMC_CAS_LATENCY_3 = 3u  /**< CAS latency of 3 cycles */
}   fmc_CasLatency_t;

/** Delay of the read data after the CAS latency (read pipe), the value is the delay in memory clock cycles */
typedef enum
{
    FMC_READ_PIPE_0 = 0u, /**< No delay of the read data      */
    FMC_READ_PIPE_1,      /**< One cycle delay of the read data */
    FMC_READ_PIPE_2,      /**< Two cycles delay of the read data */
    FMC_READ_PIPE_CNT     /**< Number of read pipe items       */
}   fmc_ReadPipe_t;

/** Period of the SDRAM clock as a multiple of the bus clock (HCLK) period */
typedef enum
{
    FMC_SDRAM_CLOCK_DIV_2 = 0u, /**< SDRAM clock is HCLK / 2          */
    FMC_SDRAM_CLOCK_DIV_3,      /**< SDRAM clock is HCLK / 3          */
    FMC_SDRAM_CLOCK_DIV_1,      /**< SDRAM clock is HCLK              */
    FMC_SDRAM_CLOCK_DIV_CNT     /**< Number of SDRAM clock divider items */
}   fmc_SdramClockDiv_t;

/** Command of a raw SDRAM command (the value is the MODE field of the command register) */
typedef enum
{
    FMC_SDRAM_COMMAND_NORMAL = 0u,        /**< Normal mode (exit of the self-refresh and the power-down) */
    FMC_SDRAM_COMMAND_CLOCK_ENABLE,       /**< Clock configuration enable, SDCKE is driven high          */
    FMC_SDRAM_COMMAND_PRECHARGE_ALL,      /**< Precharge all banks                                       */
    FMC_SDRAM_COMMAND_AUTO_REFRESH,       /**< Auto-refresh (the number of the cycles is a parameter)    */
    FMC_SDRAM_COMMAND_LOAD_MODE_REGISTER, /**< Load the mode register of the SDRAM                      */
    FMC_SDRAM_COMMAND_SELF_REFRESH,       /**< Self-refresh                                              */
    FMC_SDRAM_COMMAND_POWER_DOWN,         /**< Power-down                                                */
    FMC_SDRAM_COMMAND_CNT                 /**< Number of SDRAM command items                             */
}   fmc_SdramCommandMode_t;

/** State of an SDRAM bank */
typedef enum
{
    FMC_SDRAM_NORMAL = 0u,  /**< Normal mode                                       */
    FMC_SDRAM_SELF_REFRESH, /**< Self-refresh mode                                 */
    FMC_SDRAM_POWER_DOWN,   /**< Power-down mode                                   */
    FMC_SDRAM_BUSY,         /**< The controller executes a command (not ready)     */
    FMC_SDRAM_STATE_CNT     /**< Number of SDRAM state items                       */
}   fmc_SdramState_t;

/** Ready / busy state of the NAND flash (the signal FMC_INT / NWAIT) */
typedef enum
{
    FMC_NAND_BUSY = 0u, /**< The NAND flash is busy (R/B low) */
    FMC_NAND_READY,     /**< The NAND flash is ready (R/B high) */
    FMC_NAND_STATE_CNT  /**< Number of NAND ready state items */
}   fmc_NandReadyState_t;

/** Event of the NAND ready / busy signal reported by the callback */
typedef enum
{
    FMC_NAND_EVENT_RISING_EDGE = 0u, /**< Rising edge of the ready signal (the NAND became ready) */
    FMC_NAND_EVENT_FALLING_EDGE,     /**< Falling edge of the ready signal (the NAND became busy)  */
    FMC_NAND_EVENT_HIGH_LEVEL,       /**< High level of the ready signal                           */
    FMC_NAND_EVENT_CNT               /**< Number of NAND event items                               */
}   fmc_NandEvent_t;

/** Error reported by the error callback */
typedef enum
{
    FMC_ERROR_SDRAM_REFRESH = 0u, /**< An SDRAM auto-refresh request was not served in time (refresh error) */
    FMC_ERROR_CNT                 /**< Number of error items                                                 */
}   fmc_ErrorId_t;

/** Serving mode of the events (the NAND ready signal and the refresh error) */
typedef enum
{
    FMC_EVENT_MODE_POLL = 0u, /**< Served by Fmc_Task, the interrupt of the module is not enabled */
    FMC_EVENT_MODE_ISR,       /**< Served by the interrupt of the module                          */
    FMC_EVENT_MODE_CNT        /**< Number of event mode items                                     */
}   fmc_EventMode_t;

/** Time in nanoseconds, the unit of every timing field */
typedef uint32_t fmc_Time_ns_t;

/** Time in microseconds */
typedef uint32_t fmc_Time_us_t;

/** Time in milliseconds */
typedef uint32_t fmc_Time_ms_t;

/** Frequency in hertz */
typedef uint32_t fmc_Frequency_Hz_t;

/** Address of a memory window in the address space of the CPU */
typedef uint32_t fmc_Address_t;

/** Error correcting code of a NAND page (the valid bits depend on the page size) */
typedef uint32_t fmc_Ecc_t;

/** Number of the rows of an SDRAM (refresh cycles of the whole array) */
typedef uint32_t fmc_RowCnt_t;

/** Count of clock cycles of the memory */
typedef uint8_t fmc_ClockCycleCnt_t;

/** Count of the consecutive auto-refresh commands */
typedef uint8_t fmc_RefreshCnt_t;

/** Content of the SDRAM mode register (13 bits) */
typedef uint16_t fmc_ModeRegister_t;

/** Interrupt priority of the module (forwarded to Nvic) */
typedef nvic_IrqPrio_t fmc_IrqPrio_t;

/* ========================= PIN ENUMERATIONS, EARLY ========================= */
/* Fmc_Pins.h uses the macros FMC_PIN_ENCODE / FMC_PIN_UNUSED defined just above, and its items
 * (fmc_AddressPin_t, fmc_DataPin_t, fmc_ControlPin_t, fmc_SdramPin_t) are used by fmc_Config_t
 * below, so it is included here rather than at the end of this file. */
#include "Fmc_Pins.h"                       /* Generated pin enumerations     */

/** Write timing of a NOR / PSRAM / SRAM bank in the extended mode (the write accesses only) */
typedef struct
{
    fmc_Time_ns_t AddressSetup_ns;  /**< Address setup time (address valid to the write enable low)        */
    fmc_Time_ns_t AddressHold_ns;   /**< Address hold time (multiplexed bus and access mode D)             */
    fmc_Time_ns_t DataSetup_ns;     /**< Data setup time (the width of the write enable pulse)             */
    fmc_Time_ns_t DataHold_ns;      /**< Data hold time after the write enable rising edge                 */
    fmc_Time_ns_t BusTurnaround_ns; /**< Time between the end of a write and the next access of the bank   */
}   fmc_NorSramWriteTiming_t;

/** Configuration of a NOR / PSRAM / SRAM bank, see Fmc_Set_NorSramConfig */
typedef struct
{
    fmc_MemoryType_t         MemoryType;          /**< Type of the memory                                          */
    fmc_DataWidth_t          DataWidth;           /**< Data bus width of the memory                                */
    fmc_FunctionState_t      AddressDataMux;      /**< Address and data multiplexed on the data bus (NOR, PSRAM)   */
    fmc_FunctionState_t      BurstAccess;         /**< Synchronous burst read access (NOR, PSRAM)                  */
    fmc_FunctionState_t      WriteBurst;          /**< Synchronous burst write access (PSRAM)                      */
    fmc_FunctionState_t      WriteProtect;        /**< Write accesses are rejected by the controller (bus error)   */
    fmc_FunctionState_t      WaitSignal;          /**< NWAIT inserts wait states of a synchronous access           */
    fmc_FunctionState_t      AsyncWait;           /**< NWAIT is used by an asynchronous access too                 */
    fmc_WaitPolarity_t       WaitPolarity;        /**< Polarity of NWAIT                                           */
    fmc_WaitTiming_t         WaitTiming;          /**< Timing of NWAIT of a synchronous access                     */
    fmc_FunctionState_t      ExtendedMode;        /**< Separate write timing and access mode (WriteTiming)         */
    fmc_AccessMode_t         AccessMode;          /**< Access mode of the extended mode                            */
    fmc_PageSize_t           PageSize;            /**< Page size of a PSRAM burst                                  */
    fmc_Time_ns_t            AddressSetup_ns;     /**< Address setup time (the read timing when ExtendedMode)      */
    fmc_Time_ns_t            AddressHold_ns;      /**< Address hold time (multiplexed bus and access mode D)       */
    fmc_Time_ns_t            DataSetup_ns;        /**< Data setup time (the width of the output enable pulse)      */
    fmc_Time_ns_t            DataHold_ns;         /**< Data hold time after the output enable rising edge          */
    fmc_Time_ns_t            BusTurnaround_ns;    /**< Time between the end of an access and the next one          */
    fmc_Time_ns_t            ByteLaneSetup_ns;    /**< Byte lane (NBL) setup time before the chip select           */
    fmc_Time_ns_t            ClockPeriod_ns;      /**< Shortest period of FMC_CLK the memory accepts (synchronous) */
    fmc_ClockCycleCnt_t      DataLatency;         /**< Data latency of a synchronous access in FMC_CLK cycles      */
    fmc_Time_ns_t            ChipSelectMaxLow_ns; /**< Longest low time of the chip select (PSRAM), 0 = no limit */
    fmc_NorSramWriteTiming_t WriteTiming;         /**< Write timing of the extended mode                           */
}   fmc_NorSramConfig_t;

/** Timing of one memory space (common or attribute) of the NAND bank */
typedef struct
{
    fmc_Time_ns_t Setup_ns; /**< Address setup time before the command strobe                    */
    fmc_Time_ns_t Wait_ns;  /**< Width of the command strobe (NWE / NOE)                         */
    fmc_Time_ns_t Hold_ns;  /**< Address and data hold time after the command strobe             */
    fmc_Time_ns_t HiZ_ns;   /**< Time the data bus stays in high impedance after a write starts  */
}   fmc_NandTiming_t;

/** Configuration of the NAND bank, see Fmc_Set_NandConfig */
typedef struct
{
    fmc_DataWidth_t     DataWidth;       /**< Data bus width of the NAND (8 or 16 bits)                       */
    fmc_FunctionState_t WaitFeature;     /**< The wait signal (ready / busy) extends the command strobe       */
    fmc_FunctionState_t EccEnable;       /**< The ECC calculation is available (Fmc_Set_NandEccActive)        */
    fmc_EccPageSize_t   EccPageSize;     /**< Page size of the ECC calculation                                */
    fmc_NandTiming_t    CommonTiming;    /**< Timing of the common memory space (data, commands, addresses)   */
    fmc_NandTiming_t    AttributeTiming; /**< Timing of the attribute memory space                            */
    fmc_Time_ns_t       AleToReDelay_ns; /**< Delay from ALE low to the read strobe                           */
    fmc_Time_ns_t       CleToReDelay_ns; /**< Delay from CLE low to the read strobe                           */
    fmc_FunctionState_t ReadyRisingIrq;  /**< Event on the rising edge of the ready signal                    */
    fmc_FunctionState_t ReadyFallingIrq; /**< Event on the falling edge of the ready signal                   */
    fmc_FunctionState_t ReadyHighIrq;    /**< Event on the high level of the ready signal                     */
}   fmc_NandConfig_t;

/** Configuration of an SDRAM bank, see Fmc_Set_SdramConfig */
typedef struct
{
    fmc_SdramColumnBits_t    ColumnBits;              /**< Number of the column address bits                    */
    fmc_SdramRowBits_t       RowBits;                 /**< Number of the row address bits                       */
    fmc_DataWidth_t          DataWidth;               /**< Data bus width of the SDRAM                          */
    fmc_SdramInternalBanks_t InternalBanks;           /**< Number of the internal banks                         */
    fmc_CasLatency_t         CasLatency;              /**< CAS latency                                          */
    fmc_FunctionState_t      ReadBurst;               /**< Single reads are managed as bursts (common to banks) */
    fmc_ReadPipe_t           ReadPipe;                /**< Delay of the read data (common to the banks)         */
    fmc_SdramClockDiv_t      ClockDivider;            /**< SDRAM clock divider (common to the banks)            */
    fmc_FunctionState_t      WriteProtect;            /**< The bank is read only                                */
    fmc_Time_ns_t            LoadToActiveDelay_ns;    /**< tMRD: load mode register to active                   */
    fmc_Time_ns_t            ExitSelfRefreshDelay_ns; /**< tXSR: exit self-refresh to active                    */
    fmc_Time_ns_t            SelfRefreshTime_ns;      /**< tRAS: shortest time in the self-refresh              */
    fmc_Time_ns_t            RowCycleDelay_ns;        /**< tRC: refresh to activate, refresh to refresh         */
    fmc_Time_ns_t            WriteRecoveryTime_ns;    /**< tWR: write to precharge                              */
    fmc_Time_ns_t            RpDelay_ns;              /**< tRP: precharge to another command                    */
    fmc_Time_ns_t            RcdDelay_ns;             /**< tRCD: activate to read / write                       */
    fmc_Time_ms_t            RefreshPeriod_ms;        /**< Time in which the whole array must be refreshed      */
    fmc_RowCnt_t             RefreshRows;             /**< Number of the rows refreshed in RefreshPeriod_ms     */
    fmc_Time_us_t            PowerUpDelay_us;         /**< Power-up delay after the clock start (at least 100)  */
}   fmc_SdramConfig_t;

/** Raw command of the SDRAM, see Fmc_Set_SdramCommand */
typedef struct
{
    fmc_SdramCommandMode_t Mode;           /**< Command                                                    */
    fmc_RefreshCnt_t       AutoRefreshCnt; /**< Number of the auto-refresh cycles (1..16, auto-refresh only) */
    fmc_ModeRegister_t     ModeRegister;   /**< Content of the mode register (load mode register only)    */
}   fmc_SdramCommand_t;

/** Callback reporting an event of the NAND ready signal */
typedef void ( *fmc_NandReadyCallback_t )( fmc_NandEvent_t nandEvent );

/** Callback reporting an SDRAM refresh error */
typedef void ( *fmc_RefreshErrorCallback_t )( void );

/** Callback reporting an error of the controller */
typedef void ( *fmc_ErrorCallback_t )( fmc_ErrorId_t errorId );

/** Configuration of the module, see Fmc_Init / Fmc_Get_DefaultConfig. The bank configurations and the pin lists
 *  are read by Fmc_Init only, the pointers are not kept. */
typedef struct
{
    fmc_FunctionState_t         WriteFifo;                          /**< Write FIFO of the controller    */
    fmc_FunctionState_t         ContinuousClock;                    /**< FMC_CLK is generated always     */
    fmc_EventMode_t             EventMode;                          /**< Serving mode of the events      */
    fmc_IrqPrio_t               IrqPriority;                        /**< NVIC priority of the interrupt  */
    const fmc_NorSramConfig_t * NorSramConfig[ FMC_NOR_BANK_CNT ];  /**< NOR / PSRAM / SRAM banks, or null */
    const fmc_NandConfig_t    * NandConfig;                         /**< NAND bank, or null              */
    const fmc_SdramConfig_t   * SdramConfig[ FMC_SDRAM_BANK_CNT ];  /**< SDRAM banks, or null            */
    fmc_AddressPin_t            AddressPin[ FMC_ADDRESS_LINE_CNT ]; /**< Address pins to configure       */
    fmc_DataPin_t               DataPin[ FMC_DATA_LINE_CNT ];       /**< Data pins to configure          */
    fmc_ControlPin_t            ControlPin[ FMC_CONTROL_PIN_CNT ];  /**< Control pins to configure       */
    fmc_SdramPin_t              SdramPin[ FMC_SDRAM_PIN_CNT ];      /**< SDRAM pins to configure         */
    fmc_NandReadyCallback_t     NandReadyCallback;                  /**< NAND ready event, optional      */
    fmc_RefreshErrorCallback_t  RefreshErrorCallback;               /**< SDRAM refresh error, optional   */
    fmc_ErrorCallback_t         ErrorCallback;                      /**< Controller error, optional      */
}   fmc_Config_t;

#endif /* FMC_FMC_TYPES_H */
