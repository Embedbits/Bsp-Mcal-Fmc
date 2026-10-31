/**
 * \author Mr.Nobody
 * \file Test_Fmc.c
 * \ingroup Fmc
 * \brief Unit tests of the Fmc module (STM32H5).
 *
 * Fmc.c, Fmc_Sdram.c, Fmc_Nand.c and Fmc_Reg.c are compiled unchanged, the registers of the flexible
 * memory controller are emulated by RegMem, Rcc, Nvic and Gpio are mocked by CMock. The bus clock is
 * 200 MHz (a period of 5 ns) unless a test says otherwise, so a time in nanoseconds divided by five and
 * rounded up is the number of cycles. The conversions of the timing in nanoseconds to the register values
 * are pure functions and are tested directly (limits, rounding up, minimum, rejection); the register
 * mapping of the NOR / PSRAM / SRAM, SDRAM and NAND configurations is checked through the registers.
 *
 * \note The registers are plain memory: flags that the hardware sets (BUSY, the mode status, FIFO empty,
 *       the ready level) are preset by the tests or emulated by an HW model; the write-0-to-clear
 *       behaviour of the NAND flags is not emulated (a test presets the flag and checks that the module
 *       cleared it). The SDRAM initialization order is observed by an HW model that logs every change of
 *       FMC_SDCMR. The behaviour against a real memory is checked by the integration tests only.
 *
 * \note Each test case runs in its own process, the static state of the module starts clean in every
 *       test. On STM32H503 (no FMC) the tests check that every function returns an error; the SDRAM
 *       tests do the same on STM32H523 / H533 (no SDRAM controller).
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "RegMem.h"                         /* Register memory emulation      */
#include "Fmc.h"                            /* Module under test (private)    */
#include "Fmc_Reg.h"                        /* Module Reg component           */
#include "Stm32.h"                          /* Device registers definition    */
#include "MockRcc_Port.h"                   /* RCC module mock                */
#include "MockNvic_Port.h"                  /* NVIC module mock              */
#include "MockGpio_Port.h"                  /* GPIO module mock               */
#include <string.h>                         /* memset                         */
/* ============================= TYPEDEFS =================================== */
/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Message of a test that has nothing to check on a line without the FMC */
#define UT_FMC_NO_FMC_MSG                     "No FMC on this line."

/** Message of a test that has nothing to check on a line without the SDRAM controller */
#define UT_FMC_NO_SDRAM_MSG                   "No SDRAM controller on this line."

/** Bus clock of the tests (period 5 ns) */
#define UT_FMC_HCLK_200MHZ                    ( 200000000u )

/** Bus clock of the tests that need an exact fraction of a period (period 4 ns) */
#define UT_FMC_HCLK_250MHZ                    ( 250000000u )

/** Highest value of a 32-bit time and clock */
#define UT_FMC_UINT32_MAX                     ( 0xFFFFFFFFu )

/** Width item that is not in the list on a line without the 32-bit bus */
#define UT_FMC_WIDTH_32                       ( 32u )

/** Width value that is in no list */
#define UT_FMC_WIDTH_INVALID                  ( 12u )

/** Value of FMC_BWTRx after the reset (extended mode not used) */
#define UT_FMC_BWTR_RESET                     ( 0x0FFFFFFFu )

/** Value of FMC_SDTRx after the reset (bank not used) */
#define UT_FMC_SDTR_RESET                     ( 0x0FFFFFFFu )

/** Time of 11 ns, 3 cycles at 200 MHz (rounded up) */
#define UT_FMC_TIME_11NS                      ( 11u )

/** Cycles of 11 ns at 200 MHz */
#define UT_FMC_TIME_11NS_CYCLES               ( 3u )

/** Time of 10 ns, exactly 2 cycles at 200 MHz */
#define UT_FMC_ADDSET_NS                      ( 10u )

/** Cycles of the address setup time of the tests */
#define UT_FMC_ADDSET_CYC                     ( 2u )

/** Time of 20 ns, 4 cycles at 200 MHz */
#define UT_FMC_ADDHLD_NS                      ( 20u )

/** Cycles of the address hold time of the tests */
#define UT_FMC_ADDHLD_CYC                     ( 4u )

/** Time of 30 ns, 6 cycles at 200 MHz */
#define UT_FMC_DATAST_NS                      ( 30u )

/** Cycles of the data setup time of the tests */
#define UT_FMC_DATAST_CYC                     ( 6u )

/** Time of 20 ns, 4 cycles at 200 MHz, BUSTURN register value is cycles minus one */
#define UT_FMC_TURN_NS                        ( 20u )

/** BUSTURN field of the bus turnaround time of the tests (4 cycles) */
#define UT_FMC_TURN_FIELD                     ( 3u )

/** Time of 15 ns, 3 cycles at 200 MHz (data hold of a read) */
#define UT_FMC_DATAHLD_NS                     ( 15u )

/** Cycles of the data hold time of the tests */
#define UT_FMC_DATAHLD_CYC                    ( 3u )

/** DATAHLD field of a write of 3 cycles (the register holds the cycles minus one) */
#define UT_FMC_DATAHLD_WR_FIELD               ( 2u )

/** Time of 5 ns, 1 cycle at 200 MHz */
#define UT_FMC_ONE_CYCLE_NS                   ( 5u )

/** Time of 10 ns, 2 cycles at 200 MHz (byte lane setup) */
#define UT_FMC_NBL_NS                         ( 10u )

/** Cycles of the byte lane setup time of the tests */
#define UT_FMC_NBL_CYC                        ( 2u )

/** Time of 15 ns, 3 cycles of the FMC_CLK period at 200 MHz (CLKDIV field is cycles minus one) */
#define UT_FMC_CLKPER_NS                      ( 15u )

/** CLKDIV field of the FMC_CLK period of the tests */
#define UT_FMC_CLKDIV_FIELD                   ( 2u )

/** Data latency of the tests in FMC_CLK cycles */
#define UT_FMC_DATLAT                         ( 4u )

/** DATLAT field of the data latency of the tests (the register holds the cycles minus two) */
#define UT_FMC_DATLAT_FIELD                   ( 2u )

/** Time of 4000 ns, 800 cycles at 200 MHz (chip select counter) */
#define UT_FMC_CS_NS                          ( 4000u )

/** Chip select counter value of UT_FMC_CS_NS */
#define UT_FMC_CS_CYC                         ( 800u )

/** Data setup time that needs 260 cycles (the field holds 255) */
#define UT_FMC_DATAST_TOO_LONG_NS             ( 1300u )

/** Address setup time that needs 16 cycles (the field holds 15) */
#define UT_FMC_ADDSET_TOO_LONG_NS             ( 80u )

/** Bus turnaround time that needs 17 cycles (the field holds 16) */
#define UT_FMC_TURN_TOO_LONG_NS               ( 85u )

/** Data hold time of a read that needs 4 cycles (the field holds 3) */
#define UT_FMC_DATAHLD_TOO_LONG_NS            ( 20u )

/** Data hold time of a write that needs 5 cycles (the field holds 4) */
#define UT_FMC_DATAHLD_WR_TOO_LONG_NS         ( 25u )

/** FMC_CLK period that needs 17 cycles (the field holds 16) */
#define UT_FMC_CLKPER_TOO_LONG_NS             ( 85u )

/** Byte lane setup time that needs 5 cycles (the field holds 3) */
#define UT_FMC_NBL_TOO_LONG_NS                ( 25u )

/** Chip select low time that needs 80000 cycles (the counter holds 65535) */
#define UT_FMC_CS_TOO_LONG_NS                 ( 400000u )

/** Data latency below the smallest value */
#define UT_FMC_DATLAT_TOO_SHORT               ( 1u )

/** Data latency above the largest value */
#define UT_FMC_DATLAT_TOO_LONG                ( 18u )

/** Write timing of the extended mode: address setup 5 ns, 1 cycle */
#define UT_FMC_WR_ADDSET_NS                   ( 5u )

/** Write timing of the extended mode: data setup 15 ns, 3 cycles */
#define UT_FMC_WR_DATAST_NS                   ( 15u )

/** Write timing of the extended mode: data setup in cycles */
#define UT_FMC_WR_DATAST_CYC                  ( 3u )

/** Access mode item B as the value of the ACCMOD field */
#define UT_FMC_ACCMOD_B_FIELD                 ( 1u )

/** Access mode item D as the value of the ACCMOD field */
#define UT_FMC_ACCMOD_D_FIELD                 ( 3u )

/** MTYP field of a PSRAM */
#define UT_FMC_MTYP_PSRAM_FIELD               ( 1u )

/** MTYP field of a NOR flash */
#define UT_FMC_MTYP_NOR_FIELD                 ( 2u )

/** MWID field of a 16-bit bus */
#define UT_FMC_MWID_16_FIELD                  ( 1u )

/** MWID field of a 32-bit bus */
#define UT_FMC_MWID_32_FIELD                  ( 2u )

/** CPSIZE field of a page of 256 bytes */
#define UT_FMC_CPSIZE_256_FIELD               ( 2u )

/** Index of bank 2 in the CMSIS array BTCR (control register) */
#define UT_FMC_BTCR_BCR2                      ( 2u )

/** Index of bank 2 in the CMSIS array BTCR (timing register) */
#define UT_FMC_BTCR_BTR2                      ( 3u )

/** Index of bank 2 in the CMSIS array BWTR (write timing register) */
#define UT_FMC_BWTR_2                         ( 2u )

/** Index of bank 3 in the CMSIS array BTCR (control register) */
#define UT_FMC_BTCR_BCR3                      ( 4u )

/** Index of bank 4 in the CMSIS array BTCR (control register) */
#define UT_FMC_BTCR_BCR4                      ( 6u )

/** Index of the first NOR / PSRAM bank */
#define UT_FMC_BANK_IDX_1                     ( 0u )

/** Index of bank 2 as an item value of fmc_Bank_t */
#define UT_FMC_BANK_IDX_2                     ( 1u )

/** Interrupt priority of the tests */
#define UT_FMC_IRQ_PRIO                       ( 5u )

/** Item of fmc_Bank_t that is not a bank of any line */
#define UT_FMC_BANK_INVALID                   ( 99u )

/** Number of the FMC_BANK_NOR_PSRAM_x banks */
#define UT_FMC_NOR_BANK_CNT                   ( 4u )

/** Number of the calls of the Gpio stub that the tests with several pins expect */
#define UT_FMC_PIN_CALLS                      ( 3u )

/** Size of the log of the SDRAM command register */
#define UT_FMC_CMR_LOG_SIZE                   ( 16u )

/** Number of the commands of the SDRAM initialization sequence (clock enable, precharge, refresh, load mode) */
#define UT_FMC_INIT_COMMANDS                  ( 4u )

/** Index of the second command of the initialization sequence */
#define UT_FMC_CMD_IDX_2                      ( 1u )

/** Index of the third command of the initialization sequence */
#define UT_FMC_CMD_IDX_3                      ( 2u )

/** Index of the fourth command of the initialization sequence */
#define UT_FMC_CMD_IDX_4                      ( 3u )

/** Number of the auto-refresh cycles of the initialization sequence minus one (NRFS field) */
#define UT_FMC_NRFS_INIT_FIELD                ( 7u )

/** Content of the mode register of an SDRAM with CAS latency 3 (burst length 1, sequential, single write) */
#define UT_FMC_MODE_REG_CAS3                  ( 0x0230u )

/** Content of the mode register of an SDRAM with CAS latency 2 */
#define UT_FMC_MODE_REG_CAS2                  ( 0x0220u )

/** Number of the NAND events */
#define UT_FMC_NAND_EVENT_CNT                 ( 3u )

/* ---------------------------- SDRAM of the tests ---------------------------- */

/** Rows of the SDRAM of the tests */
#define UT_FMC_SDRAM_ROWS                     ( 8192u )

/** Refresh period of the SDRAM of the tests in milliseconds */
#define UT_FMC_SDRAM_PERIOD_MS                ( 64u )

/** Refresh count of the SDRAM of the tests: 64 ms / 8192 rows at 100 MHz is 781 cycles, minus the margin of 20 */
#define UT_FMC_SDRAM_COUNT                    ( 761u )

/** SDCLK field of the divider 2 */
#define UT_FMC_SDCLK_DIV2_FIELD               ( 2u )

/** SDCLK field of the divider 3 */
#define UT_FMC_SDCLK_DIV3_FIELD               ( 3u )

/** CAS latency of the SDRAM of the tests as the register field */
#define UT_FMC_CAS3_FIELD                     ( 3u )

/** NC field of 9 column bits */
#define UT_FMC_NC_9_FIELD                     ( 1u )

/** NR field of 13 row bits */
#define UT_FMC_NR_13_FIELD                    ( 2u )

/** tMRD of the SDRAM of the tests: 14 ns is 2 cycles at 100 MHz, raised to the smallest value of 3 cycles (field 0) */
#define UT_FMC_TMRD_NS                        ( 14u )

/** tXSR of the SDRAM of the tests: 70 ns, 7 cycles at 100 MHz */
#define UT_FMC_TXSR_NS                        ( 70u )

/** tXSR in cycles */
#define UT_FMC_TXSR_CYC                       ( 7u )

/** tRAS of the SDRAM of the tests: 42 ns, 5 cycles at 100 MHz */
#define UT_FMC_TRAS_NS                        ( 42u )

/** tRAS in cycles */
#define UT_FMC_TRAS_CYC                       ( 5u )

/** tRC of the SDRAM of the tests: 63 ns, 7 cycles at 100 MHz */
#define UT_FMC_TRC_NS                         ( 63u )

/** tRC in cycles */
#define UT_FMC_TRC_CYC                        ( 7u )

/** tWR, tRP and tRCD of the SDRAM of the tests: 14 ns, 2 cycles at 100 MHz */
#define UT_FMC_TWR_NS                         ( 14u )

/** Smallest SDRAM timing in cycles (tRP and tRCD of the tests) */
#define UT_FMC_SDRAM_TIME_2CYC                ( 2u )

/** tMRD in cycles after the raise to the smallest value */
#define UT_FMC_TMRD_CYC                       ( 3u )

/** tWR in cycles after the raise by tRAS - tRCD (5 - 2) and tRC - tRCD - tRP (7 - 2 - 2) */
#define UT_FMC_TWR_CYC                        ( 3u )

/** TXSR field (7 cycles minus 2) */
#define UT_FMC_TXSR_FIELD                     ( 5u )

/** TRAS field (5 cycles minus 2) */
#define UT_FMC_TRAS_FIELD                     ( 3u )

/** TRC field (7 cycles minus 2) */
#define UT_FMC_TRC_FIELD                      ( 5u )

/** TWR field (3 cycles minus 2) */
#define UT_FMC_TWR_FIELD                      ( 1u )

/** Power-up delay that extends the minimum of 100 us */
#define UT_FMC_POWER_UP_US                    ( 200u )

/** Refresh period that gives a count below the smallest value (1 ms, 8192 rows, 100 MHz: 12 cycles) */
#define UT_FMC_PERIOD_SHORT_MS                ( 1u )

/** Refresh period that gives a count above the largest value (1000 ms, 1 row, 100 MHz) */
#define UT_FMC_PERIOD_LONG_MS                 ( 1000u )

/** One row */
#define UT_FMC_ONE_ROW                        ( 1u )

/** Sum of tWR, tRP, tRC and tRCD that makes the refresh count of UT_FMC_SDRAM_COUNT collide (761 - 4) */
#define UT_FMC_SUM_COLLIDE                    ( 757u )

/** Refresh count after the collision with the sum of the timings */
#define UT_FMC_SDRAM_COUNT_COLLIDE            ( 760u )

/** Value of FMC_SDSR with the mode status of bank 1 in the self-refresh mode */
#define UT_FMC_MODES_SELF                     ( 1u )

/** Value of the mode status in the power-down mode */
#define UT_FMC_MODES_POWER_DOWN               ( 2u )

/** Value of the mode status that the module does not know */
#define UT_FMC_MODES_RESERVED                 ( 3u )

/* ----------------------------- NAND of the tests ---------------------------- */

/** Common space: setup 10 ns = 2 cycles, the register holds 1 */
#define UT_FMC_NAND_C_SETUP_NS                ( 10u )

/** Common space: wait 30 ns = 6 cycles, the register holds 5 */
#define UT_FMC_NAND_C_WAIT_NS                 ( 30u )

/** Common space: hold 20 ns = 4 cycles, the register holds 4 */
#define UT_FMC_NAND_C_HOLD_NS                 ( 20u )

/** Common space: high-Z 15 ns = 3 cycles, the register holds 2 */
#define UT_FMC_NAND_C_HIZ_NS                  ( 15u )

/** MEMSET field of the tests */
#define UT_FMC_NAND_C_SETUP_FIELD             ( 1u )

/** MEMWAIT field of the tests */
#define UT_FMC_NAND_C_WAIT_FIELD              ( 5u )

/** MEMHOLD field of the tests */
#define UT_FMC_NAND_C_HOLD_FIELD              ( 4u )

/** MEMHIZ field of the tests */
#define UT_FMC_NAND_C_HIZ_FIELD               ( 2u )

/** Attribute space: wait 10 ns = 2 cycles, the register holds 1 */
#define UT_FMC_NAND_A_WAIT_NS                 ( 10u )

/** ALE to read strobe delay 40 ns = 8 cycles, TAR = 8 - ( setup field 0 + 2 ) = 6 */
#define UT_FMC_NAND_ALE_NS                    ( 40u )

/** TAR field of the tests */
#define UT_FMC_NAND_TAR_FIELD                 ( 6u )

/** Setup time that needs 256 cycles (the field holds 255 cycles) */
#define UT_FMC_NAND_SETUP_TOO_LONG_NS         ( 1280u )

/** Hold time that needs 255 cycles (the field holds 254 cycles) */
#define UT_FMC_NAND_HOLD_TOO_LONG_NS          ( 1275u )

/** ALE delay that needs 19 cycles (a field of 17, the field holds 15) */
#define UT_FMC_NAND_ALE_TOO_LONG_NS           ( 95u )

/** ECC page size item 512 bytes: valid bits ECC[23:0] */
#define UT_FMC_ECC_MASK_512                   ( 0x00FFFFFFu )

/** ECC page size item 256 bytes: valid bits ECC[21:0] */
#define UT_FMC_ECC_MASK_256                   ( 0x003FFFFFu )

/** ECC page size item 8192 bytes: valid bits ECC[31:0] */
#define UT_FMC_ECC_MASK_8192                  ( 0xFFFFFFFFu )

/** Attribute space: ATTSET field of 1 cycle (the register holds 0) */
#define UT_FMC_NAND_A_SETUP_FIELD             ( 0u )

/** Attribute space: ATTWAIT field of 2 cycles (the register holds 1) */
#define UT_FMC_NAND_A_WAIT_FIELD              ( 1u )

/** Attribute space: ATTHOLD field of 1 cycle */
#define UT_FMC_NAND_A_HOLD_FIELD              ( 1u )

/** Attribute space: ATTHIZ field of 1 cycle (the register holds 0) */
#define UT_FMC_NAND_A_HIZ_FIELD               ( 0u )

/** PWID field of the 16-bit NAND bus */
#define UT_FMC_NAND_PWID_16                   ( 1u )

/** Function state that is in no list */
#define UT_FMC_STATE_INVALID                  ( 7u )

/** Number of the ready events enabled in the event test */
#define UT_FMC_NAND_EVENT_ENABLED_CNT         ( 2u )

/** ECC result register of the tests (every bit set) */
#define UT_FMC_ECC_ALL_SET                    ( 0xFFFFFFFFu )

/* ========================= EXPORTED MACROS ================================ */

/** Expected value of FMC_BTRx from the register fields (the fields are named constants of the tests) */
#define UT_FMC_BTR( ADDSET, ADDHLD, DATAST, BUSTURN, CLKDIV, DATLAT, ACCMOD, DATAHLD ) \
            ( ( (uint32_t)(ADDSET)  << FMC_BTRx_ADDSET_Pos  ) | \
              ( (uint32_t)(ADDHLD)  << FMC_BTRx_ADDHLD_Pos  ) | \
              ( (uint32_t)(DATAST)  << FMC_BTRx_DATAST_Pos  ) | \
              ( (uint32_t)(BUSTURN) << FMC_BTRx_BUSTURN_Pos ) | \
              ( (uint32_t)(CLKDIV)  << FMC_BTRx_CLKDIV_Pos  ) | \
              ( (uint32_t)(DATLAT)  << FMC_BTRx_DATLAT_Pos  ) | \
              ( (uint32_t)(ACCMOD)  << FMC_BTRx_ACCMOD_Pos  ) | \
              ( (uint32_t)(DATAHLD) << FMC_BTRx_DATAHLD_Pos ) )

/** Expected value of FMC_BWTRx from the register fields */
#define UT_FMC_BWTR( ADDSET, ADDHLD, DATAST, BUSTURN, ACCMOD, DATAHLD ) \
            ( ( (uint32_t)(ADDSET)  << FMC_BWTRx_ADDSET_Pos  ) | \
              ( (uint32_t)(ADDHLD)  << FMC_BWTRx_ADDHLD_Pos  ) | \
              ( (uint32_t)(DATAST)  << FMC_BWTRx_DATAST_Pos  ) | \
              ( (uint32_t)(BUSTURN) << FMC_BWTRx_BUSTURN_Pos ) | \
              ( (uint32_t)(ACCMOD)  << FMC_BWTRx_ACCMOD_Pos  ) | \
              ( (uint32_t)(DATAHLD) << FMC_BWTRx_DATAHLD_Pos ) )

/** Expected value of FMC_SDTRx from the register fields */
#define UT_FMC_SDTR( TMRD, TXSR, TRAS, TRC, TWR, TRP, TRCD ) \
            ( ( (uint32_t)(TMRD) << FMC_SDTRx_TMRD_Pos ) | \
              ( (uint32_t)(TXSR) << FMC_SDTRx_TXSR_Pos ) | \
              ( (uint32_t)(TRAS) << FMC_SDTRx_TRAS_Pos ) | \
              ( (uint32_t)(TRC)  << FMC_SDTRx_TRC_Pos  ) | \
              ( (uint32_t)(TWR)  << FMC_SDTRx_TWR_Pos  ) | \
              ( (uint32_t)(TRP)  << FMC_SDTRx_TRP_Pos  ) | \
              ( (uint32_t)(TRCD) << FMC_SDTRx_TRCD_Pos ) )

/** Expected value of FMC_PMEM / FMC_PATT from the register fields (the positions of both are equal) */
#define UT_FMC_NAND_SPACE( SET, WAIT, HOLD, HIZ ) \
            ( ( (uint32_t)(SET)  << FMC_PMEM_MEMSET_Pos  ) | \
              ( (uint32_t)(WAIT) << FMC_PMEM_MEMWAIT_Pos ) | \
              ( (uint32_t)(HOLD) << FMC_PMEM_MEMHOLD_Pos ) | \
              ( (uint32_t)(HIZ)  << FMC_PMEM_MEMHIZ_Pos  ) )

/* ========================== LOCAL VARIABLES =============================== */

/** Bus clock returned by the Rcc stub (0 = the clock is not available) */
static rcc_FreqHz_t utFmc_BusClk = UT_FMC_HCLK_200MHZ;

/** Interrupt service routine registered in Nvic */
static nvic_IsrCallback_t utFmc_Isr = NULL;

/** Number of the Gpio_Init calls */
static uint32_t utFmc_GpioCnt = 0u;

/** Configuration of the last Gpio_Init call */
static gpio_Config_t utFmc_GpioConfig;

/** Result of the Gpio stub */
static gpio_RequestState_t utFmc_GpioResult = GPIO_REQUEST_OK;

/** Number of the NAND ready events reported by the callback */
static uint32_t utFmc_NandEventCnt = 0u;

/** Last NAND ready event */
static fmc_NandEvent_t utFmc_NandEvent = FMC_NAND_EVENT_CNT;

/** Number of the refresh error callbacks */
static uint32_t utFmc_RefreshCnt = 0u;

/** Number of the error callbacks */
static uint32_t utFmc_ErrorCnt = 0u;

/** Last error reported by the error callback */
static fmc_ErrorId_t utFmc_ErrorId = FMC_ERROR_CNT;

/** Log of the changes of FMC_SDCMR (HW model) */
static uint32_t utFmc_CmrLog[ UT_FMC_CMR_LOG_SIZE ];

/** Number of the entries of the log */
static uint32_t utFmc_CmrCnt = 0u;

/** Last value of FMC_SDCMR seen by the HW model */
static uint32_t utFmc_CmrLast = 0u;

/* ======================== FORWARD DECLARATIONS ============================ */

static rcc_RequestState_t   UtFmc_RccGetClkStub     ( rcc_PeriphId_t periphId,
                                                      rcc_FreqHz_t * const periphClk,
                                                      int callCnt );
static nvic_RequestState_t  UtFmc_NvicSetHandlerStub( nvic_PeriphIrqList_t irqId,
                                                      const nvic_IsrCallback_t irqHandler,
                                                      int callCnt );
static gpio_RequestState_t  UtFmc_GpioInitStub      ( gpio_Config_t * gpioConfig, int callCnt );

/* ========================== LOCAL FUNCTIONS =============================== */

/**
 * \brief Rcc_Get_PeriphClk stub: returns the bus clock of the test.
 */
static rcc_RequestState_t UtFmc_RccGetClkStub( rcc_PeriphId_t periphId, rcc_FreqHz_t * const periphClk, int callCnt )
{
    rcc_RequestState_t retState = RCC_REQUEST_ERROR;

    (void)periphId;
    (void)callCnt;

    if( 0u != utFmc_BusClk )
    {
        *periphClk = utFmc_BusClk;
        retState   = RCC_REQUEST_OK;
    }
    else
    {
        /* The clock is not available */
        *periphClk = 0u;
        retState   = RCC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Nvic_Set_PeriphIrq_Handler stub: keeps the interrupt service routine of the module.
 */
static nvic_RequestState_t UtFmc_NvicSetHandlerStub( nvic_PeriphIrqList_t irqId,
                                                     const nvic_IsrCallback_t irqHandler,
                                                     int callCnt )
{
    (void)irqId;
    (void)callCnt;
    utFmc_Isr = irqHandler;

    return ( NVIC_REQUEST_OK );
}


/**
 * \brief Gpio_Init stub: counts the calls and keeps the configuration of the last one.
 */
static gpio_RequestState_t UtFmc_GpioInitStub( gpio_Config_t * gpioConfig, int callCnt )
{
    (void)callCnt;
    utFmc_GpioConfig = *gpioConfig;
    utFmc_GpioCnt++;

    return ( utFmc_GpioResult );
}


/**
 * \brief NAND ready callback of the tests.
 */
static void UtFmc_NandReadyCb( fmc_NandEvent_t nandEvent )
{
    utFmc_NandEvent = nandEvent;
    utFmc_NandEventCnt++;
}


/**
 * \brief Refresh error callback of the tests.
 */
static void UtFmc_RefreshErrorCb( void )
{
    utFmc_RefreshCnt++;
}


/**
 * \brief Error callback of the tests.
 */
static void UtFmc_ErrorCb( fmc_ErrorId_t errorId )
{
    utFmc_ErrorId = errorId;
    utFmc_ErrorCnt++;
}


/**
 * \brief Sets the mocks of Rcc, Nvic and Gpio that the module uses (every call is accepted).
 */
static void UtFmc_Setup_Mocks( void )
{
    Rcc_Get_PeriphClk_StubWithCallback( UtFmc_RccGetClkStub );
    Rcc_Set_PeriphActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_PeriphInactive_IgnoreAndReturn( RCC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Handler_StubWithCallback( UtFmc_NvicSetHandlerStub );
    Nvic_Set_PeriphIrq_Prio_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Inactive_IgnoreAndReturn( NVIC_REQUEST_OK );
    Gpio_Init_StubWithCallback( UtFmc_GpioInitStub );
}


/**
 * \brief Fills a configuration of a 16-bit asynchronous SRAM: address setup 10 ns, data setup 30 ns, the other
 *        times zero (the controller minimum applies).
 */
static void UtFmc_Get_SramConfig( fmc_NorSramConfig_t * const config )
{
    (void)memset( config, 0, sizeof( fmc_NorSramConfig_t ) );
    config->MemoryType      = FMC_MEMORY_SRAM;
    config->DataWidth       = FMC_DATA_WIDTH_16;
    config->AddressSetup_ns = UT_FMC_ADDSET_NS;
    config->DataSetup_ns    = UT_FMC_DATAST_NS;
}


/**
 * \brief Fills a configuration of a 16-bit PSRAM with the synchronous burst access.
 */
static void UtFmc_Get_PsramConfig( fmc_NorSramConfig_t * const config )
{
    UtFmc_Get_SramConfig( config );
    config->MemoryType     = FMC_MEMORY_PSRAM;
    config->BurstAccess    = FMC_FUNCTION_ACTIVE;
    config->ClockPeriod_ns = UT_FMC_CLKPER_NS;
    config->DataLatency    = UT_FMC_DATLAT;
}


/**
 * \brief Fills the configuration of the SDRAM of the specification example (16 bit, 9 columns, 13 rows, 4 banks,
 *        CAS latency 3, 64 ms for 8192 rows).
 */
static void UtFmc_Get_SdramConfig( fmc_SdramConfig_t * const config )
{
    (void)memset( config, 0, sizeof( fmc_SdramConfig_t ) );
    config->ColumnBits              = FMC_SDRAM_COLUMN_BITS_9;
    config->RowBits                 = FMC_SDRAM_ROW_BITS_13;
    config->DataWidth               = FMC_DATA_WIDTH_16;
    config->InternalBanks           = FMC_SDRAM_INTERNAL_BANKS_4;
    config->CasLatency              = FMC_CAS_LATENCY_3;
    config->LoadToActiveDelay_ns    = UT_FMC_TMRD_NS;
    config->ExitSelfRefreshDelay_ns = UT_FMC_TXSR_NS;
    config->SelfRefreshTime_ns      = UT_FMC_TRAS_NS;
    config->RowCycleDelay_ns        = UT_FMC_TRC_NS;
    config->WriteRecoveryTime_ns    = UT_FMC_TWR_NS;
    config->RpDelay_ns              = UT_FMC_TWR_NS;
    config->RcdDelay_ns             = UT_FMC_TWR_NS;
    config->RefreshPeriod_ms        = UT_FMC_SDRAM_PERIOD_MS;
    config->RefreshRows             = UT_FMC_SDRAM_ROWS;
}


/**
 * \brief Fills a configuration of a 16-bit NAND with the ECC of 512 bytes pages.
 */
static void UtFmc_Get_NandConfig( fmc_NandConfig_t * const config )
{
    (void)memset( config, 0, sizeof( fmc_NandConfig_t ) );
    config->DataWidth                = FMC_DATA_WIDTH_16;
    config->EccEnable                = FMC_FUNCTION_ACTIVE;
    config->EccPageSize              = FMC_ECC_PAGE_SIZE_512;
    config->CommonTiming.Setup_ns    = UT_FMC_NAND_C_SETUP_NS;
    config->CommonTiming.Wait_ns     = UT_FMC_NAND_C_WAIT_NS;
    config->CommonTiming.Hold_ns     = UT_FMC_NAND_C_HOLD_NS;
    config->CommonTiming.HiZ_ns      = UT_FMC_NAND_C_HIZ_NS;
    config->AttributeTiming.Setup_ns = UT_FMC_ONE_CYCLE_NS;
    config->AttributeTiming.Wait_ns  = UT_FMC_NAND_A_WAIT_NS;
    config->AttributeTiming.Hold_ns  = UT_FMC_ONE_CYCLE_NS;
    config->AttributeTiming.HiZ_ns   = UT_FMC_ONE_CYCLE_NS;
    config->AleToReDelay_ns          = UT_FMC_NAND_ALE_NS;
}


/**
 * \brief Sets the mocks and initializes the module with the default configuration (all banks disabled, the
 *        controller enabled, the events served by Fmc_Task).
 */
static void UtFmc_Init_Default( void )
{
    fmc_Config_t config;

    UtFmc_Setup_Mocks();
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_DefaultConfig( &config ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Init( &config ) );
}


/**
 * \brief HW model of the SDRAM controller: logs every change of FMC_SDCMR (the order of the commands).
 */
static void UtFmc_SdramModel( void )
{
#if defined(FMC_Bank5_6_R)
    const uint32_t commandReg = FMC_Bank5_6_R->SDCMR;

    if( ( UT_FMC_CMR_LOG_SIZE > utFmc_CmrCnt ) &&
        ( commandReg         != utFmc_CmrLast )    )
    {
        utFmc_CmrLog[utFmc_CmrCnt] = commandReg;
        utFmc_CmrCnt++;
        utFmc_CmrLast = commandReg;
    }
    else
    {
        /* The command register did not change */
    }
#endif
}


/**
 * \brief HW model of the NAND ready signal: the signal is high, so the high level flag is set while the detection
 *        of the high level is enabled.
 */
static void UtFmc_NandReadyModel( void )
{
#if defined(FMC_Bank1_R)
    if( 0u != ( FMC_Bank3_R->SR & FMC_SR_ILEN ) )
    {
        FMC_Bank3_R->SR |= FMC_SR_ILS;
    }
    else
    {
        /* The detection is off, the flag stays as it is */
    }
#endif
}

/* ============================ TEST FIXTURE ================================ */

void setUp( void )
{
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );

    utFmc_BusClk       = UT_FMC_HCLK_200MHZ;
    utFmc_Isr          = NULL;
    utFmc_GpioCnt      = 0u;
    utFmc_GpioResult   = GPIO_REQUEST_OK;
    utFmc_NandEventCnt = 0u;
    utFmc_NandEvent    = FMC_NAND_EVENT_CNT;
    utFmc_RefreshCnt   = 0u;
    utFmc_ErrorCnt     = 0u;
    utFmc_ErrorId      = FMC_ERROR_CNT;
    utFmc_CmrCnt       = 0u;
    utFmc_CmrLast      = 0u;
}


void tearDown( void )
{
    (void)RegMem_Set_ModelInactive();
}

/* ====================== CONSTANTS OF THE CONVERSION TESTS =================== */

/** Time of 31 ns, 3 cycles of a clock of 200 MHz / 3 (a period of 15 ns), 31 / 15 rounded up */
#define UT_FMC_TIME_31NS                      ( 31u )

/** Cycles of 31 ns at 200 MHz / 3 */
#define UT_FMC_TIME_31NS_CYCLES               ( 3u )

/** Divider of the memory clock of the conversion tests */
#define UT_FMC_DIVIDER_3                      ( 3u )

/** Smallest number of the cycles of the phase limit of the tests */
#define UT_FMC_LIMIT_MIN                      ( 2u )

/** Largest number of the cycles of the phase limit of the tests */
#define UT_FMC_LIMIT_MAX                      ( 5u )

/** Time that needs 4 cycles at 200 MHz */
#define UT_FMC_TIME_20NS_CYCLES               ( 4u )

/** Time that needs 5 cycles at 200 MHz (the largest value of the limit of the tests) */
#define UT_FMC_TIME_25NS                      ( 25u )

/** Time that needs 6 cycles at 200 MHz (above the limit of the tests) */
#define UT_FMC_TIME_30NS                      ( 30u )

/** Data setup time of 255 cycles at 200 MHz (the largest value of the field) */
#define UT_FMC_DATAST_MAX_NS                  ( 1275u )

/** Address setup time of 15 cycles at 200 MHz (the largest value of the field) */
#define UT_FMC_ADDSET_MAX_NS                  ( 75u )

/** Bus turnaround time of 16 cycles at 200 MHz (the largest value of the field) */
#define UT_FMC_TURN_MAX_NS                    ( 80u )

/** Value that proves that a function wrote the output */
#define UT_FMC_SENTINEL                       ( 0xA5A5A5A5u )

/* ========================== MODULE VERSION ================================ */

/**
 * \brief   Fmc_Get_ModuleVersion() returns the version of the module.
 *
 * \par Expected results
 * - Version is 1.0.0.
 */
void Ut_Fmc_Get_ModuleVersion_Call_ReturnsVersion( void )
{
    const fmc_ModuleVersion_t version = Fmc_Get_ModuleVersion();

    TEST_ASSERT_EQUAL_UINT8( 1u, version.Major );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Minor );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Patch );
}

/* ========================= DEFAULT CONFIGURATION =========================== */

/**
 * \brief   Fmc_Get_DefaultConfig() rejects a null pointer.
 *
 * \par Expected results
 * - FMC_REQUEST_ERROR is returned.
 */
void Ut_Fmc_Get_DefaultConfig_NullPtr_ReturnsError( void )
{
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Get_DefaultConfig( FMC_NULL_PTR ) );
}


/**
 * \brief   Fmc_Get_DefaultConfig() fills a configuration with the write FIFO enabled and nothing else.
 *
 * \par Expected results
 * - FMC_REQUEST_OK is returned.
 * - The write FIFO is active, the continuous clock is inactive, the events are served by Fmc_Task.
 * - No bank configuration, every pin is the unused item, no callback.
 */
void Ut_Fmc_Get_DefaultConfig_Call_FillsSafeDefaults( void )
{
    fmc_Config_t config;

#if defined(FMC_Bank1_R)
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_DefaultConfig( &config ) );
    TEST_ASSERT_EQUAL( FMC_FUNCTION_ACTIVE, config.WriteFifo );
    TEST_ASSERT_EQUAL( FMC_FUNCTION_INACTIVE, config.ContinuousClock );
    TEST_ASSERT_EQUAL( FMC_EVENT_MODE_POLL, config.EventMode );
    TEST_ASSERT_NULL( config.NandConfig );
    for( uint32_t idx = 0u; ( FMC_NOR_BANK_CNT > idx ); idx++ )
    {
        TEST_ASSERT_NULL( config.NorSramConfig[idx] );
    }
    for( uint32_t idx = 0u; ( FMC_SDRAM_BANK_CNT > idx ); idx++ )
    {
        TEST_ASSERT_NULL( config.SdramConfig[idx] );
    }
    for( uint32_t idx = 0u; ( FMC_ADDRESS_LINE_CNT > idx ); idx++ )
    {
        TEST_ASSERT_EQUAL( FMC_ADDRESS_PIN_UNUSED, config.AddressPin[idx] );
    }
    for( uint32_t idx = 0u; ( FMC_DATA_LINE_CNT > idx ); idx++ )
    {
        TEST_ASSERT_EQUAL( FMC_DATA_PIN_UNUSED, config.DataPin[idx] );
    }
    for( uint32_t idx = 0u; ( FMC_CONTROL_PIN_CNT > idx ); idx++ )
    {
        TEST_ASSERT_EQUAL( FMC_CONTROL_PIN_UNUSED, config.ControlPin[idx] );
    }
    for( uint32_t idx = 0u; ( FMC_SDRAM_PIN_CNT > idx ); idx++ )
    {
        TEST_ASSERT_EQUAL( FMC_SDRAM_PIN_UNUSED, config.SdramPin[idx] );
    }
    TEST_ASSERT_NULL( (void *)config.NandReadyCallback );
    TEST_ASSERT_NULL( (void *)config.RefreshErrorCallback );
    TEST_ASSERT_NULL( (void *)config.ErrorCallback );
#else
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Get_DefaultConfig( &config ) );
#endif /* FMC_Bank1_R */
}

/* ========================== LINE WITHOUT THE FMC ============================ */

/**
 * \brief   Every function of the interface returns an error on a line without the FMC (STM32H503).
 *
 * \par Expected results
 * - On STM32H503 every function returns FMC_REQUEST_ERROR and the pin lists hold the unused item only.
 * - On the other lines the test has nothing to check.
 */
void Ut_Fmc_AllFunctions_NoFmcOnLine_ReturnError( void )
{
#if !defined(FMC_Bank1_R)
    fmc_Config_t        config;
    fmc_NorSramConfig_t norSram;
    fmc_SdramConfig_t   sdram;
    fmc_NandConfig_t    nand;
    fmc_SdramCommand_t  command;
    fmc_Address_t       address = 0u;
    fmc_SdramState_t    sdramState;
    fmc_NandReadyState_t readyState;
    fmc_Ecc_t           ecc = 0u;
    fmc_IrqPrio_t       prio = 0u;

    (void)memset( &config, 0, sizeof( config ) );
    (void)memset( &norSram, 0, sizeof( norSram ) );
    (void)memset( &sdram, 0, sizeof( sdram ) );
    (void)memset( &nand, 0, sizeof( nand ) );
    (void)memset( &command, 0, sizeof( command ) );

    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Init( &config ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Deinit() );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_NorSramConfig( (fmc_Bank_t)UT_FMC_BANK_IDX_1, &norSram ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_NandConfig( &nand ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramConfig( (fmc_Bank_t)UT_FMC_BANK_IDX_1, &sdram ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_BankActive( (fmc_Bank_t)UT_FMC_BANK_IDX_1 ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_BankInactive( (fmc_Bank_t)UT_FMC_BANK_IDX_1 ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Get_BankBaseAddress( (fmc_Bank_t)UT_FMC_BANK_IDX_1, &address ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR,
                       Fmc_Set_BankWriteProtection( (fmc_Bank_t)UT_FMC_BANK_IDX_1, FMC_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramInit( (fmc_Bank_t)UT_FMC_BANK_IDX_1 ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramRefreshRate( UT_FMC_SDRAM_PERIOD_MS, UT_FMC_SDRAM_ROWS ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramSelfRefresh( (fmc_Bank_t)UT_FMC_BANK_IDX_1 ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramPowerDown( (fmc_Bank_t)UT_FMC_BANK_IDX_1 ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramCommand( (fmc_Bank_t)UT_FMC_BANK_IDX_1, &command ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Get_SdramState( (fmc_Bank_t)UT_FMC_BANK_IDX_1, &sdramState ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_NandEccActive() );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Get_NandEcc( &ecc ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Get_NandReadyState( &readyState ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_IrqPriority( UT_FMC_IRQ_PRIO ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Get_IrqPriority( &prio ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_InitAddressGpio( FMC_ADDRESS_PIN_UNUSED ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_InitDataGpio( FMC_DATA_PIN_UNUSED ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_InitControlGpio( FMC_CONTROL_PIN_UNUSED ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_InitSdramGpio( FMC_SDRAM_PIN_UNUSED ) );
    Fmc_Task();
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}

/* ============================ TIME TO CYCLES =============================== */

/**
 * \brief   FmcPriv_Get_Cycles() rounds a time up to whole cycles.
 *
 * \par Expected results
 * - 11 ns at 200 MHz are 3 cycles (2.2 rounded up), 10 ns are exactly 2 cycles, 0 ns are 0 cycles.
 */
void Ut_Fmc_Get_Cycles_TimeNotMultiple_RoundsUp( void )
{
#if defined(FMC_Bank1_R)
    fmc_CycleCnt_t cycles = UT_FMC_SENTINEL;

    TEST_ASSERT_EQUAL( FMC_REQUEST_OK,
                       FmcPriv_Get_Cycles( UT_FMC_TIME_11NS, UT_FMC_HCLK_200MHZ, FMC_NO_DIVIDER, &cycles ) );
    TEST_ASSERT_EQUAL_UINT32( UT_FMC_TIME_11NS_CYCLES, cycles );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK,
                       FmcPriv_Get_Cycles( UT_FMC_ADDSET_NS, UT_FMC_HCLK_200MHZ, FMC_NO_DIVIDER, &cycles ) );
    TEST_ASSERT_EQUAL_UINT32( UT_FMC_ADDSET_CYC, cycles );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, FmcPriv_Get_Cycles( 0u, UT_FMC_HCLK_200MHZ, FMC_NO_DIVIDER, &cycles ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, cycles );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   FmcPriv_Get_Cycles() converts for a memory clock that is the bus clock divided by a divider.
 *
 * \par Expected results
 * - 31 ns at 200 MHz / 3 (a period of 15 ns) are 3 cycles.
 */
void Ut_Fmc_Get_Cycles_Divider_UsesDividedClock( void )
{
#if defined(FMC_Bank1_R)
    fmc_CycleCnt_t cycles = UT_FMC_SENTINEL;

    TEST_ASSERT_EQUAL( FMC_REQUEST_OK,
                       FmcPriv_Get_Cycles( UT_FMC_TIME_31NS, UT_FMC_HCLK_200MHZ, UT_FMC_DIVIDER_3, &cycles ) );
    TEST_ASSERT_EQUAL_UINT32( UT_FMC_TIME_31NS_CYCLES, cycles );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   FmcPriv_Get_Cycles() rejects a result that does not fit and invalid parameters.
 *
 * \par Expected results
 * - The largest time at the largest clock needs more than 32 bits: FMC_REQUEST_ERROR.
 * - A null result pointer, a clock of 0 Hz and a divider of 0: FMC_REQUEST_ERROR.
 */
void Ut_Fmc_Get_Cycles_InvalidInput_ReturnsError( void )
{
#if defined(FMC_Bank1_R)
    fmc_CycleCnt_t cycles = 0u;

    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR,
                       FmcPriv_Get_Cycles( UT_FMC_UINT32_MAX, UT_FMC_UINT32_MAX, FMC_NO_DIVIDER, &cycles ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR,
                       FmcPriv_Get_Cycles( UT_FMC_ADDSET_NS, UT_FMC_HCLK_200MHZ, FMC_NO_DIVIDER, FMC_NULL_PTR ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_Cycles( UT_FMC_ADDSET_NS, 0u, FMC_NO_DIVIDER, &cycles ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_Cycles( UT_FMC_ADDSET_NS, UT_FMC_HCLK_200MHZ, 0u, &cycles ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   FmcPriv_Get_PhaseCycles() raises a time to the smallest value and rejects one above the largest.
 *
 * \par Expected results
 * - 5 ns (1 cycle) become the smallest value of 2 cycles, 20 ns give 4 cycles, 25 ns give the largest value of 5.
 * - 30 ns (6 cycles) are rejected and the result is not written.
 */
void Ut_Fmc_Get_PhaseCycles_Limits_RaisedOrRejected( void )
{
#if defined(FMC_Bank1_R)
    const fmc_Clock_t      clock  = { .Clk = UT_FMC_HCLK_200MHZ, .Divider = FMC_NO_DIVIDER };
    const fmc_PhaseLimit_t limit  = { .Min = UT_FMC_LIMIT_MIN, .Max = UT_FMC_LIMIT_MAX };
    fmc_CycleCnt_t         cycles = UT_FMC_SENTINEL;

    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, FmcPriv_Get_PhaseCycles( UT_FMC_ONE_CYCLE_NS, &clock, &limit, &cycles ) );
    TEST_ASSERT_EQUAL_UINT32( UT_FMC_LIMIT_MIN, cycles );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, FmcPriv_Get_PhaseCycles( UT_FMC_ADDHLD_NS, &clock, &limit, &cycles ) );
    TEST_ASSERT_EQUAL_UINT32( UT_FMC_TIME_20NS_CYCLES, cycles );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, FmcPriv_Get_PhaseCycles( UT_FMC_TIME_25NS, &clock, &limit, &cycles ) );
    TEST_ASSERT_EQUAL_UINT32( UT_FMC_LIMIT_MAX, cycles );

    cycles = UT_FMC_SENTINEL;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_PhaseCycles( UT_FMC_TIME_30NS, &clock, &limit, &cycles ) );
    TEST_ASSERT_EQUAL_UINT32( UT_FMC_SENTINEL, cycles );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR,
                       FmcPriv_Get_PhaseCycles( UT_FMC_ONE_CYCLE_NS, FMC_NULL_PTR, &limit, &cycles ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR,
                       FmcPriv_Get_PhaseCycles( UT_FMC_ONE_CYCLE_NS, &clock, FMC_NULL_PTR, &cycles ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR,
                       FmcPriv_Get_PhaseCycles( UT_FMC_ONE_CYCLE_NS, &clock, &limit, FMC_NULL_PTR ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   FmcPriv_Get_Phases() converts every phase of the list, one failing phase fails the call.
 *
 * \par Expected results
 * - A list with two valid phases: FMC_REQUEST_OK and both results are written.
 * - A list with a phase above its limit: FMC_REQUEST_ERROR, the other phase is converted anyway.
 * - A null list: FMC_REQUEST_ERROR.
 */
void Ut_Fmc_Get_Phases_OneInvalidPhase_ReturnsErrorConvertsOthers( void )
{
#if defined(FMC_Bank1_R)
    const fmc_Clock_t      clock   = { .Clk = UT_FMC_HCLK_200MHZ, .Divider = FMC_NO_DIVIDER };
    const fmc_PhaseLimit_t limit   = { .Min = UT_FMC_LIMIT_MIN, .Max = UT_FMC_LIMIT_MAX };
    fmc_CycleCnt_t         first   = 0u;
    fmc_CycleCnt_t         second  = 0u;
    const fmc_Phase_t      valid[] =
    {
        { UT_FMC_ADDHLD_NS, &limit, &first  },
        { UT_FMC_TIME_25NS, &limit, &second },
    };
    const fmc_Phase_t      invalid[] =
    {
        { UT_FMC_TIME_30NS, &limit, &first  },
        { UT_FMC_ADDHLD_NS, &limit, &second },
    };

    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, FmcPriv_Get_Phases( &clock, valid, FMC_PHASE_CNT( valid ) ) );
    TEST_ASSERT_EQUAL_UINT32( UT_FMC_TIME_20NS_CYCLES, first );
    TEST_ASSERT_EQUAL_UINT32( UT_FMC_LIMIT_MAX, second );

    second = 0u;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_Phases( &clock, invalid, FMC_PHASE_CNT( invalid ) ) );
    TEST_ASSERT_EQUAL_UINT32( UT_FMC_TIME_20NS_CYCLES, second );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_Phases( &clock, FMC_NULL_PTR, 0u ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}

/* ===================== NOR / PSRAM / SRAM REGISTER VALUES ================== */

/**
 * \brief   FmcPriv_Get_NorSramRegs() converts a 16-bit SRAM: the times in cycles, the controller minimum, writes
 *          enabled, the write timing register at its reset value.
 *
 * \par Expected results
 * - BCR holds the 16-bit width and WREN, nothing else.
 * - BTR holds ADDSET 2 (10 ns), ADDHLD 1 (the smallest value), DATAST 6 (30 ns), BUSTURN 0 (the smallest value).
 * - BWTR is the reset value, the chip select counter is not used.
 */
void Ut_Fmc_Get_NorSramRegs_Sram16Bit_MapsTimes( void )
{
#if defined(FMC_Bank1_R)
    fmc_NorSramConfig_t config;
    fmc_NorSramRegs_t   regs;

    UtFmc_Get_SramConfig( &config );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );
    TEST_ASSERT_EQUAL_HEX32( ( ( UT_FMC_MWID_16_FIELD << FMC_BCRx_MWID_Pos ) | FMC_BCRx_WREN ), regs.Bcr );
    TEST_ASSERT_EQUAL_HEX32( UT_FMC_BTR( UT_FMC_ADDSET_CYC, 1u, UT_FMC_DATAST_CYC, 0u, 0u, 0u, 0u, 0u ), regs.Btr );
    TEST_ASSERT_EQUAL_HEX32( UT_FMC_BWTR_RESET, regs.Bwtr );
    TEST_ASSERT_EQUAL( FMC_FUNCTION_INACTIVE, regs.CsCounter );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   FmcPriv_Get_NorSramRegs() converts a multiplexed 8-bit NOR flash: the NOR type, the flash access enable,
 *          the multiplexing, the address hold time and the bus turnaround time with its offset.
 *
 * \par Expected results
 * - BCR holds MTYP NOR, MWID 8 bit (0), MUXEN, FACCEN and WREN.
 * - BTR holds ADDSET 1 (a multiplexed bus needs at least one cycle), ADDHLD 4, DATAST 6, BUSTURN 3 (4 cycles).
 */
void Ut_Fmc_Get_NorSramRegs_MultiplexedNor8Bit_MapsTimes( void )
{
#if defined(FMC_Bank1_R)
    fmc_NorSramConfig_t config;
    fmc_NorSramRegs_t   regs;

    UtFmc_Get_SramConfig( &config );
    config.MemoryType       = FMC_MEMORY_NOR;
    config.DataWidth        = FMC_DATA_WIDTH_8;
    config.AddressDataMux   = FMC_FUNCTION_ACTIVE;
    config.AddressSetup_ns  = 0u;
    config.AddressHold_ns   = UT_FMC_ADDHLD_NS;
    config.BusTurnaround_ns = UT_FMC_TURN_NS;

    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );
    TEST_ASSERT_EQUAL_HEX32( ( ( UT_FMC_MTYP_NOR_FIELD << FMC_BCRx_MTYP_Pos ) | FMC_BCRx_MUXEN | FMC_BCRx_FACCEN |
                               FMC_BCRx_WREN ),
                             regs.Bcr );
    TEST_ASSERT_EQUAL_HEX32( UT_FMC_BTR( 1u, UT_FMC_ADDHLD_CYC, UT_FMC_DATAST_CYC, UT_FMC_TURN_FIELD, 0u, 0u, 0u, 0u ),
                             regs.Btr );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   FmcPriv_Get_NorSramRegs() raises the address setup time of a PSRAM to one cycle.
 *
 * \par Expected results
 * - The address setup time of 0 ns gives ADDSET 1 for a PSRAM (an SRAM may have 0).
 */
void Ut_Fmc_Get_NorSramRegs_PsramZeroAddressSetup_RaisedToOneCycle( void )
{
#if defined(FMC_Bank1_R)
    fmc_NorSramConfig_t config;
    fmc_NorSramRegs_t   regs;

    UtFmc_Get_SramConfig( &config );
    config.AddressSetup_ns = 0u;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );
    TEST_ASSERT_EQUAL_HEX32( UT_FMC_BTR( 0u, 1u, UT_FMC_DATAST_CYC, 0u, 0u, 0u, 0u, 0u ), regs.Btr );

    config.MemoryType = FMC_MEMORY_PSRAM;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );
    TEST_ASSERT_EQUAL_HEX32( UT_FMC_BTR( 1u, 1u, UT_FMC_DATAST_CYC, 0u, 0u, 0u, 0u, 0u ), regs.Btr );
    TEST_ASSERT_EQUAL_HEX32( ( ( UT_FMC_MWID_16_FIELD << FMC_BCRx_MWID_Pos ) |
                               ( UT_FMC_MTYP_PSRAM_FIELD << FMC_BCRx_MTYP_Pos ) |
                               FMC_BCRx_WREN ),
                             regs.Bcr );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   FmcPriv_Get_NorSramRegs() converts the extended mode: the access mode in both timing registers, the
 *          separate write timing with the data hold of a write (cycles minus one) and the EXTMOD bit.
 *
 * \par Expected results
 * - BCR holds EXTMOD, BTR holds ACCMOD B (the read timing is converted as without the extended mode).
 * - BWTR holds ADDSET 1, ADDHLD 1, DATAST 3, BUSTURN 3, DATAHLD 2 (3 cycles) and ACCMOD B.
 */
void Ut_Fmc_Get_NorSramRegs_ExtendedMode_MapsWriteTiming( void )
{
#if defined(FMC_Bank1_R)
    fmc_NorSramConfig_t config;
    fmc_NorSramRegs_t   regs;

    UtFmc_Get_SramConfig( &config );
    config.ExtendedMode                  = FMC_FUNCTION_ACTIVE;
    config.AccessMode                    = FMC_ACCESS_MODE_B;
    config.WriteTiming.AddressSetup_ns   = UT_FMC_WR_ADDSET_NS;
    config.WriteTiming.DataSetup_ns      = UT_FMC_WR_DATAST_NS;
    config.WriteTiming.DataHold_ns       = UT_FMC_DATAHLD_NS;
    config.WriteTiming.BusTurnaround_ns  = UT_FMC_TURN_NS;

    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );
    TEST_ASSERT_EQUAL_HEX32( ( ( UT_FMC_MWID_16_FIELD << FMC_BCRx_MWID_Pos ) | FMC_BCRx_EXTMOD | FMC_BCRx_WREN ),
                             regs.Bcr );
    TEST_ASSERT_EQUAL_HEX32( UT_FMC_BTR( UT_FMC_ADDSET_CYC,
                                         1u,
                                         UT_FMC_DATAST_CYC,
                                         0u,
                                         0u,
                                         0u,
                                         UT_FMC_ACCMOD_B_FIELD,
                                         0u ),
                             regs.Btr );
    TEST_ASSERT_EQUAL_HEX32( UT_FMC_BWTR( 1u,
                                          1u,
                                          UT_FMC_WR_DATAST_CYC,
                                          UT_FMC_TURN_FIELD,
                                          UT_FMC_ACCMOD_B_FIELD,
                                          UT_FMC_DATAHLD_WR_FIELD ),
                             regs.Bwtr );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   FmcPriv_Get_NorSramRegs() raises the address setup time of the access mode D (read and write) to one cycle.
 *
 * \par Expected results
 * - With the extended mode and the access mode D the zero address setup times give ADDSET 1 in BTR and in BWTR.
 */
void Ut_Fmc_Get_NorSramRegs_AccessModeD_RaisesAddressSetup( void )
{
#if defined(FMC_Bank1_R)
    fmc_NorSramConfig_t config;
    fmc_NorSramRegs_t   regs;

    UtFmc_Get_SramConfig( &config );
    config.AddressSetup_ns = 0u;
    config.ExtendedMode    = FMC_FUNCTION_ACTIVE;
    config.AccessMode      = FMC_ACCESS_MODE_D;
    config.WriteTiming.DataSetup_ns = UT_FMC_WR_DATAST_NS;

    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );
    TEST_ASSERT_EQUAL_HEX32( UT_FMC_BTR( 1u, 1u, UT_FMC_DATAST_CYC, 0u, 0u, 0u, UT_FMC_ACCMOD_D_FIELD, 0u ), regs.Btr );
    TEST_ASSERT_EQUAL_HEX32( UT_FMC_BWTR( 1u, 1u, UT_FMC_WR_DATAST_CYC, 0u, UT_FMC_ACCMOD_D_FIELD, 0u ), regs.Bwtr );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   FmcPriv_Get_NorSramRegs() converts a PSRAM with the synchronous burst access: the clock divider, the data
 *          latency, the page size, the wait signal and the write burst.
 *
 * \par Expected results
 * - BCR holds MTYP PSRAM, MWID 16, BURSTEN, CBURSTRW, WAITEN, WAITPOL, WAITCFG, WREN and CPSIZE 256 bytes.
 * - BTR holds CLKDIV 2 (3 cycles of 15 ns) and DATLAT 2 (4 cycles).
 */
void Ut_Fmc_Get_NorSramRegs_PsramBurst_MapsClockAndLatency( void )
{
#if defined(FMC_Bank1_R)
    fmc_NorSramConfig_t config;
    fmc_NorSramRegs_t   regs;

    UtFmc_Get_PsramConfig( &config );
    config.WriteBurst   = FMC_FUNCTION_ACTIVE;
    config.WaitSignal   = FMC_FUNCTION_ACTIVE;
    config.WaitPolarity = FMC_WAIT_POLARITY_HIGH;
    config.WaitTiming   = FMC_WAIT_TIMING_DURING_STATE;
    config.PageSize     = FMC_PAGE_SIZE_256;

    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );
    TEST_ASSERT_EQUAL_HEX32( ( ( UT_FMC_MWID_16_FIELD << FMC_BCRx_MWID_Pos ) |
                               ( UT_FMC_MTYP_PSRAM_FIELD << FMC_BCRx_MTYP_Pos ) |
                               ( UT_FMC_CPSIZE_256_FIELD << FMC_BCRx_CPSIZE_Pos ) |
                               FMC_BCRx_BURSTEN |
                               FMC_BCRx_CBURSTRW |
                               FMC_BCRx_WAITEN |
                               FMC_BCRx_WAITPOL |
                               FMC_BCRx_WAITCFG |
                               FMC_BCRx_WREN ),
                             regs.Bcr );
    TEST_ASSERT_EQUAL_HEX32( UT_FMC_BTR( UT_FMC_ADDSET_CYC,
                                         1u,
                                         UT_FMC_DATAST_CYC,
                                         0u,
                                         UT_FMC_CLKDIV_FIELD,
                                         UT_FMC_DATLAT_FIELD,
                                         0u,
                                         0u ),
                             regs.Btr );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   FmcPriv_Get_NorSramRegs() maps the write protection, the byte lane setup, the NWAIT use in the asynchronous
 *          access and the chip select counter.
 *
 * \par Expected results
 * - WriteProtect clears WREN, AsyncWait sets ASYNCWAIT, the byte lane setup of 10 ns gives NBLSET 2.
 * - A PSRAM with a chip select low time of 4000 ns uses the counter with 800 cycles.
 */
void Ut_Fmc_Get_NorSramRegs_ProtectNblAsyncWaitCounter_MapsBits( void )
{
#if defined(FMC_Bank1_R)
    fmc_NorSramConfig_t config;
    fmc_NorSramRegs_t   regs;

    UtFmc_Get_SramConfig( &config );
    config.WriteProtect     = FMC_FUNCTION_ACTIVE;
    config.AsyncWait        = FMC_FUNCTION_ACTIVE;
    config.ByteLaneSetup_ns = UT_FMC_NBL_NS;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );
    TEST_ASSERT_EQUAL_HEX32( ( ( UT_FMC_MWID_16_FIELD << FMC_BCRx_MWID_Pos ) | FMC_BCRx_ASYNCWAIT |
                               ( UT_FMC_NBL_CYC << FMC_BCRx_NBLSET_Pos ) ),
                             regs.Bcr );

    UtFmc_Get_SramConfig( &config );
    config.MemoryType          = FMC_MEMORY_PSRAM;
    config.ChipSelectMaxLow_ns = UT_FMC_CS_NS;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );
    TEST_ASSERT_EQUAL( FMC_FUNCTION_ACTIVE, regs.CsCounter );
    TEST_ASSERT_EQUAL_UINT32( UT_FMC_CS_CYC, regs.CsCount );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   FmcPriv_Get_NorSramRegs() converts the 32-bit data bus on the STM32H5E / H5F lines and rejects it on the
 *          others.
 *
 * \par Expected results
 * - STM32H5E / H5F: FMC_REQUEST_OK and MWID 32 bit.
 * - The other lines: a width of 32 bits and a width that is in no list give FMC_REQUEST_ERROR.
 */
void Ut_Fmc_Get_NorSramRegs_Width32_OnlyOnH5eH5f( void )
{
#if defined(FMC_Bank1_R)
    fmc_NorSramConfig_t config;
    fmc_NorSramRegs_t   regs;

    UtFmc_Get_SramConfig( &config );
    config.DataWidth = (fmc_DataWidth_t)UT_FMC_WIDTH_32;
#if defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );
    TEST_ASSERT_EQUAL_HEX32( ( ( UT_FMC_MWID_32_FIELD << FMC_BCRx_MWID_Pos ) | FMC_BCRx_WREN ), regs.Bcr );
#else
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );
#endif
    config.DataWidth = (fmc_DataWidth_t)UT_FMC_WIDTH_INVALID;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   FmcPriv_Get_NorSramRegs() rejects a time that needs more cycles than its register field holds.
 *
 * \par Expected results
 * - FMC_REQUEST_ERROR for a data setup time of 260 cycles, an address setup time of 16 cycles, a bus turnaround time
 *   of 17 cycles, a read data hold of 4 cycles, a byte lane setup of 5 cycles, a write data hold of 5 cycles in the
 *   extended mode, an FMC_CLK period of 17 cycles and a chip select low time of 80000 cycles.
 * - The boundary values (255, 15, 16, 3, 3, 4, 16 and 65535 cycles) are accepted.
 */
void Ut_Fmc_Get_NorSramRegs_TimeAboveField_ReturnsError( void )
{
#if defined(FMC_Bank1_R)
    fmc_NorSramConfig_t config;
    fmc_NorSramRegs_t   regs;

    UtFmc_Get_SramConfig( &config );
    config.DataSetup_ns = UT_FMC_DATAST_TOO_LONG_NS;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );

    UtFmc_Get_SramConfig( &config );
    config.AddressSetup_ns = UT_FMC_ADDSET_TOO_LONG_NS;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );

    UtFmc_Get_SramConfig( &config );
    config.BusTurnaround_ns = UT_FMC_TURN_TOO_LONG_NS;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );

    UtFmc_Get_SramConfig( &config );
    config.DataHold_ns = UT_FMC_DATAHLD_TOO_LONG_NS;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );

    UtFmc_Get_SramConfig( &config );
    config.ByteLaneSetup_ns = UT_FMC_NBL_TOO_LONG_NS;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );

    UtFmc_Get_SramConfig( &config );
    config.ExtendedMode                = FMC_FUNCTION_ACTIVE;
    config.WriteTiming.DataSetup_ns    = UT_FMC_WR_DATAST_NS;
    config.WriteTiming.DataHold_ns     = UT_FMC_DATAHLD_WR_TOO_LONG_NS;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );

    UtFmc_Get_PsramConfig( &config );
    config.ClockPeriod_ns = UT_FMC_CLKPER_TOO_LONG_NS;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );

    UtFmc_Get_PsramConfig( &config );
    config.BurstAccess         = FMC_FUNCTION_INACTIVE;
    config.ChipSelectMaxLow_ns = UT_FMC_CS_TOO_LONG_NS;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   FmcPriv_Get_NorSramRegs() rejects a data latency outside 2 .. 17 cycles of a synchronous access and ignores
 *          the latency of an asynchronous one.
 *
 * \par Expected results
 * - Data latency 1 and 18 with the burst access: FMC_REQUEST_ERROR.
 * - The same values without the burst access: FMC_REQUEST_OK.
 */
void Ut_Fmc_Get_NorSramRegs_DataLatencyOutOfRange_ErrorOnlyWhenSynchronous( void )
{
#if defined(FMC_Bank1_R)
    fmc_NorSramConfig_t config;
    fmc_NorSramRegs_t   regs;

    UtFmc_Get_PsramConfig( &config );
    config.DataLatency = UT_FMC_DATLAT_TOO_SHORT;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );
    config.DataLatency = UT_FMC_DATLAT_TOO_LONG;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );

    config.BurstAccess = FMC_FUNCTION_INACTIVE;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );
    config.DataLatency = UT_FMC_DATLAT_TOO_SHORT;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   FmcPriv_Get_NorSramRegs() accepts the largest time of every field.
 *
 * \par Expected results
 * - 255 cycles of data setup, 15 cycles of address setup, 16 cycles of bus turnaround and 3 cycles of data hold and
 *   byte
 *   lane setup fit: FMC_REQUEST_OK and the fields hold their largest values.
 */
void Ut_Fmc_Get_NorSramRegs_LargestTimes_Accepted( void )
{
#if defined(FMC_Bank1_R)
    fmc_NorSramConfig_t config;
    fmc_NorSramRegs_t   regs;

    UtFmc_Get_SramConfig( &config );
    config.DataSetup_ns     = UT_FMC_DATAST_MAX_NS;
    config.AddressSetup_ns  = UT_FMC_ADDSET_MAX_NS;
    config.BusTurnaround_ns = UT_FMC_TURN_MAX_NS;
    config.DataHold_ns      = UT_FMC_DATAHLD_NS;
    config.ByteLaneSetup_ns = UT_FMC_DATAHLD_NS;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );
    TEST_ASSERT_EQUAL_UINT32( ( FMC_BTRx_DATAST_Msk >> FMC_BTRx_DATAST_Pos ),
                              ( ( regs.Btr & FMC_BTRx_DATAST_Msk ) >> FMC_BTRx_DATAST_Pos ) );
    TEST_ASSERT_EQUAL_UINT32( ( FMC_BTRx_ADDSET_Msk >> FMC_BTRx_ADDSET_Pos ),
                              ( ( regs.Btr & FMC_BTRx_ADDSET_Msk ) >> FMC_BTRx_ADDSET_Pos ) );
    TEST_ASSERT_EQUAL_UINT32( ( FMC_BTRx_BUSTURN_Msk >> FMC_BTRx_BUSTURN_Pos ),
                              ( ( regs.Btr & FMC_BTRx_BUSTURN_Msk ) >> FMC_BTRx_BUSTURN_Pos ) );
    TEST_ASSERT_EQUAL_UINT32( ( FMC_BTRx_DATAHLD_Msk >> FMC_BTRx_DATAHLD_Pos ),
                              ( ( regs.Btr & FMC_BTRx_DATAHLD_Msk ) >> FMC_BTRx_DATAHLD_Pos ) );
    TEST_ASSERT_EQUAL_UINT32( ( FMC_BCRx_NBLSET_Msk >> FMC_BCRx_NBLSET_Pos ),
                              ( ( regs.Bcr & FMC_BCRx_NBLSET_Msk ) >> FMC_BCRx_NBLSET_Pos ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   FmcPriv_Get_NorSramRegs() rejects the combinations a memory type does not have and invalid items.
 *
 * \par Expected results
 * - FMC_REQUEST_ERROR for the multiplexed bus and the burst access of an SRAM, the write burst, the page size and the
 *   chip select counter of a memory that is not a PSRAM, a memory type, access mode, page size, wait polarity, wait
 *   timing and function state that are out of their lists.
 * - FMC_REQUEST_ERROR for a null configuration, a null result and a bus clock of 0 Hz.
 */
void Ut_Fmc_Get_NorSramRegs_InvalidCombination_ReturnsError( void )
{
#if defined(FMC_Bank1_R)
    fmc_NorSramConfig_t config;
    fmc_NorSramRegs_t   regs;

    UtFmc_Get_SramConfig( &config );
    config.AddressDataMux = FMC_FUNCTION_ACTIVE;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );

    UtFmc_Get_SramConfig( &config );
    config.BurstAccess = FMC_FUNCTION_ACTIVE;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );

    UtFmc_Get_PsramConfig( &config );
    config.MemoryType = FMC_MEMORY_NOR;
    config.WriteBurst = FMC_FUNCTION_ACTIVE;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );

    UtFmc_Get_SramConfig( &config );
    config.PageSize = FMC_PAGE_SIZE_128;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );

    UtFmc_Get_SramConfig( &config );
    config.ChipSelectMaxLow_ns = UT_FMC_CS_NS;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );

    UtFmc_Get_SramConfig( &config );
    config.MemoryType = FMC_MEMORY_CNT;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );

    UtFmc_Get_SramConfig( &config );
    config.AccessMode = FMC_ACCESS_MODE_CNT;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );

    UtFmc_Get_SramConfig( &config );
    config.PageSize = FMC_PAGE_SIZE_CNT;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );

    UtFmc_Get_SramConfig( &config );
    config.WaitPolarity = FMC_WAIT_POLARITY_CNT;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );

    UtFmc_Get_SramConfig( &config );
    config.WaitTiming = FMC_WAIT_TIMING_CNT;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );

    UtFmc_Get_SramConfig( &config );
    config.WriteProtect = (fmc_FunctionState_t)( FMC_FUNCTION_ACTIVE + 1u );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );

    UtFmc_Get_SramConfig( &config );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NorSramRegs( FMC_NULL_PTR, UT_FMC_HCLK_200MHZ, &regs ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NorSramRegs( &config, UT_FMC_HCLK_200MHZ, FMC_NULL_PTR ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NorSramRegs( &config, 0u, &regs ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}

/** Chip select low time of 8000 ns, 1600 cycles, a count that differs from UT_FMC_CS_CYC */
#define UT_FMC_CS2_NS                         ( 8000u )

/** All the chip select counter enable bits */
#define UT_FMC_CNTBEN_MASK                    ( FMC_PCSCNTR_CNTB1EN | FMC_PCSCNTR_CNTB2EN | \
                                              FMC_PCSCNTR_CNTB3EN | FMC_PCSCNTR_CNTB4EN )

/* ============================== INIT / DEINIT ============================== */

/**
 * \brief   Fmc_Init() rejects a null configuration and a configuration with an item out of its list.
 *
 * \par Expected results
 * - FMC_REQUEST_ERROR for a null pointer, an event mode out of the list and a write FIFO state out of the list.
 */
void Ut_Fmc_Init_InvalidConfig_ReturnsError( void )
{
#if defined(FMC_Bank1_R)
    fmc_Config_t config;

    UtFmc_Setup_Mocks();
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_DefaultConfig( &config ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Init( FMC_NULL_PTR ) );
    config.EventMode = FMC_EVENT_MODE_CNT;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Init( &config ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_DefaultConfig( &config ) );
    config.WriteFifo = (fmc_FunctionState_t)( FMC_FUNCTION_ACTIVE + 1u );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Init( &config ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   Fmc_Init() with the default configuration disables every bank (also the ones enabled by the reset) and
 *          enables the controller with the write FIFO.
 *
 * \par Expected results
 * - FMC_REQUEST_OK, the interrupt service routine is registered in Nvic.
 * - BCR1 holds FMCEN without WFDIS and CCLKEN, MBKEN of the four NOR banks and PBKEN of the NAND bank are cleared.
 */
void Ut_Fmc_Init_DefaultConfig_EnablesControllerDisablesBanks( void )
{
#if defined(FMC_Bank1_R)
    FMC_Bank1_R->BTCR[0]                  = FMC_BCRx_MBKEN;
    FMC_Bank1_R->BTCR[UT_FMC_BTCR_BCR2]   = FMC_BCRx_MBKEN;
    FMC_Bank1_R->BTCR[UT_FMC_BTCR_BCR3]   = FMC_BCRx_MBKEN;
    FMC_Bank1_R->BTCR[UT_FMC_BTCR_BCR4]   = FMC_BCRx_MBKEN;
    FMC_Bank3_R->PCR                      = FMC_PCR_PBKEN;

    UtFmc_Init_Default();

    TEST_ASSERT_NOT_NULL( (void *)utFmc_Isr );
    TEST_ASSERT_EQUAL_HEX32( FMC_BCR1_FMCEN,
                             ( FMC_Bank1_R->BTCR[0] & ( FMC_BCR1_FMCEN | FMC_BCR1_WFDIS | FMC_BCR1_CCLKEN ) ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, ( FMC_Bank1_R->BTCR[0] & FMC_BCRx_MBKEN ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, ( FMC_Bank1_R->BTCR[UT_FMC_BTCR_BCR2] & FMC_BCRx_MBKEN ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, ( FMC_Bank1_R->BTCR[UT_FMC_BTCR_BCR3] & FMC_BCRx_MBKEN ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, ( FMC_Bank1_R->BTCR[UT_FMC_BTCR_BCR4] & FMC_BCRx_MBKEN ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, ( FMC_Bank3_R->PCR & FMC_PCR_PBKEN ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   Fmc_Init() sets the write FIFO disable and the continuous clock bits of BCR1 as configured.
 *
 * \par Expected results
 * - With the write FIFO inactive and the continuous clock active BCR1 holds FMCEN, WFDIS and CCLKEN.
 */
void Ut_Fmc_Init_FifoOffContinuousClock_SetsCommonBits( void )
{
#if defined(FMC_Bank1_R)
    fmc_Config_t config;

    UtFmc_Setup_Mocks();
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_DefaultConfig( &config ) );
    config.WriteFifo       = FMC_FUNCTION_INACTIVE;
    config.ContinuousClock = FMC_FUNCTION_ACTIVE;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Init( &config ) );
    TEST_ASSERT_EQUAL_HEX32( ( FMC_BCR1_FMCEN | FMC_BCR1_WFDIS | FMC_BCR1_CCLKEN ),
                             ( FMC_Bank1_R->BTCR[0] & ( FMC_BCR1_FMCEN | FMC_BCR1_WFDIS | FMC_BCR1_CCLKEN ) ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   Fmc_Init() and Fmc_Deinit() call Rcc and Nvic in the right order with the right arguments (the interrupt is
 *          enabled in the ISR event mode only).
 *
 * \par Expected results
 * - Init: Rcc_Set_PeriphActive( RCC_PERIPH_FMC ), the handler, the priority, then Nvic_Set_PeriphIrq_Active.
 * - Deinit: Nvic_Set_PeriphIrq_Inactive, then Rcc_Set_PeriphInactive( RCC_PERIPH_FMC ); BCR1 FMCEN is cleared.
 */
void Ut_Fmc_Init_IsrMode_CallsRccAndNvicInOrder( void )
{
#if defined(FMC_Bank1_R)
    fmc_Config_t config;

    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_DefaultConfig( &config ) );
    config.EventMode   = FMC_EVENT_MODE_ISR;
    config.IrqPriority = UT_FMC_IRQ_PRIO;

    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_FMC, RCC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Handler_StubWithCallback( UtFmc_NvicSetHandlerStub );
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_FMC, UT_FMC_IRQ_PRIO, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( NVIC_PERIPH_IRQ_FMC, NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Init( &config ) );

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_FMC, NVIC_REQUEST_OK );
    Rcc_Set_PeriphInactive_ExpectAndReturn( RCC_PERIPH_FMC, RCC_REQUEST_OK );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Deinit() );
    TEST_ASSERT_EQUAL_HEX32( 0u, ( FMC_Bank1_R->BTCR[0] & FMC_BCR1_FMCEN ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   Fmc_Init() of an initialized module is rejected, Fmc_Deinit() of an uninitialized one too.
 *
 * \par Expected results
 * - Deinit before Init and the second Init: FMC_REQUEST_ERROR; Deinit after Init: FMC_REQUEST_OK; then Init works
 *   again.
 */
void Ut_Fmc_Init_Twice_SecondReturnsError( void )
{
#if defined(FMC_Bank1_R)
    fmc_Config_t config;

    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Deinit() );
    UtFmc_Init_Default();
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_DefaultConfig( &config ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Init( &config ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Deinit() );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Init( &config ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   Fmc_Init() deinitializes the module and returns an error when Rcc refuses the clock.
 *
 * \par Expected results
 * - FMC_REQUEST_ERROR, Rcc_Set_PeriphInactive is called, BCR1 FMCEN is cleared, the base address of a bank is not
 *   returned any more (the module is not initialized).
 */
void Ut_Fmc_Init_RccRefuses_DeinitializesAndReturnsError( void )
{
#if defined(FMC_Bank1_R)
    fmc_Config_t  config;
    fmc_Address_t address = 0u;

    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_DefaultConfig( &config ) );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_FMC, RCC_REQUEST_ERROR );
    Nvic_Set_PeriphIrq_Handler_StubWithCallback( UtFmc_NvicSetHandlerStub );
    Nvic_Set_PeriphIrq_Prio_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Inactive_IgnoreAndReturn( NVIC_REQUEST_OK );
    Rcc_Set_PeriphInactive_ExpectAndReturn( RCC_PERIPH_FMC, RCC_REQUEST_OK );

    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Init( &config ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, ( FMC_Bank1_R->BTCR[0] & FMC_BCR1_FMCEN ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Get_BankBaseAddress( FMC_BANK_NOR_PSRAM_1, &address ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   Fmc_Init() configures the banks of the configuration (not activated) and Fmc_Set_BankActive() enables them.
 *
 * \par Expected results
 * - After Init bank 2 holds its BTR, its MBKEN is clear.
 * - After Fmc_Set_BankActive( FMC_BANK_NOR_PSRAM_2 ) MBKEN of bank 2 is set.
 */
void Ut_Fmc_Init_WithNorSramConfig_ConfiguresBankNotActive( void )
{
#if defined(FMC_Bank1_R)
    fmc_Config_t        config;
    fmc_NorSramConfig_t sram;

    UtFmc_Setup_Mocks();
    UtFmc_Get_SramConfig( &sram );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_DefaultConfig( &config ) );
    config.NorSramConfig[UT_FMC_BANK_IDX_2] = &sram;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( UT_FMC_BTR( UT_FMC_ADDSET_CYC, 1u, UT_FMC_DATAST_CYC, 0u, 0u, 0u, 0u, 0u ),
                             FMC_Bank1_R->BTCR[UT_FMC_BTCR_BTR2] );
    TEST_ASSERT_EQUAL_HEX32( 0u, ( FMC_Bank1_R->BTCR[UT_FMC_BTCR_BCR2] & FMC_BCRx_MBKEN ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_BankActive( FMC_BANK_NOR_PSRAM_2 ) );
    TEST_ASSERT_EQUAL_HEX32( FMC_BCRx_MBKEN, ( FMC_Bank1_R->BTCR[UT_FMC_BTCR_BCR2] & FMC_BCRx_MBKEN ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   Fmc_Init() deinitializes the module and returns an error when a bank configuration is rejected.
 *
 * \par Expected results
 * - FMC_REQUEST_ERROR for an SRAM with a data setup time above the field, BCR1 FMCEN is cleared, the module is not
 *   initialized afterwards.
 */
void Ut_Fmc_Init_BankConfigRejected_DeinitializesAndReturnsError( void )
{
#if defined(FMC_Bank1_R)
    fmc_Config_t        config;
    fmc_NorSramConfig_t sram;
    fmc_Address_t       address = 0u;

    UtFmc_Setup_Mocks();
    UtFmc_Get_SramConfig( &sram );
    sram.DataSetup_ns = UT_FMC_DATAST_TOO_LONG_NS;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_DefaultConfig( &config ) );
    config.NorSramConfig[0] = &sram;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Init( &config ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, ( FMC_Bank1_R->BTCR[0] & FMC_BCR1_FMCEN ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Get_BankBaseAddress( FMC_BANK_NOR_PSRAM_1, &address ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   Fmc_Init() configures the pins of the lists and fails when Gpio rejects one.
 *
 * \par Expected results
 * - An address, a data and a control pin: Gpio_Init is called three times, the last call is the control pin NE1 on PD7
 *   in the alternate function 12.
 * - With a failing Gpio the result is FMC_REQUEST_ERROR.
 */
void Ut_Fmc_Init_PinLists_ConfiguresPinsOrFails( void )
{
#if defined(FMC_Bank1_R)
    fmc_Config_t config;

    UtFmc_Setup_Mocks();
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_DefaultConfig( &config ) );
    config.AddressPin[0] = FMC_A0_PIN_PF0;
    config.DataPin[0]    = FMC_D0_PIN_PD14;
    config.ControlPin[0] = FMC_NE1_PIN_PD7;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Init( &config ) );
    TEST_ASSERT_EQUAL_UINT32( UT_FMC_PIN_CALLS, utFmc_GpioCnt );
    TEST_ASSERT_EQUAL( GPIO_PORT_D, utFmc_GpioConfig.PortId );
    TEST_ASSERT_EQUAL( GPIO_PIN_ID_7, utFmc_GpioConfig.PinId );
    TEST_ASSERT_EQUAL( GPIO_ALT_FUNC_12, utFmc_GpioConfig.PinAltFunction );

    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Deinit() );
    utFmc_GpioResult = GPIO_REQUEST_ERROR;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Init( &config ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   Fmc_Deinit() disables the banks and the controller.
 *
 * \par Expected results
 * - After an activated bank, Fmc_Deinit() returns FMC_REQUEST_OK, MBKEN and FMCEN are cleared.
 */
void Ut_Fmc_Deinit_ActiveBank_DisablesBanksAndController( void )
{
#if defined(FMC_Bank1_R)
    fmc_NorSramConfig_t sram;

    UtFmc_Init_Default();
    UtFmc_Get_SramConfig( &sram );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_NorSramConfig( FMC_BANK_NOR_PSRAM_1, &sram ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_BankActive( FMC_BANK_NOR_PSRAM_1 ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Deinit() );
    TEST_ASSERT_EQUAL_HEX32( 0u, ( FMC_Bank1_R->BTCR[0] & ( FMC_BCR1_FMCEN | FMC_BCRx_MBKEN ) ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}

/* ========================= NOR / PSRAM / SRAM BANKS ======================== */

/**
 * \brief   Fmc_Set_NorSramConfig() writes the timing, write timing and control registers of the bank and keeps the
 *          controller enabled.
 *
 * \par Expected results
 * - Bank 2: BTR2 and BWTR2 hold the converted values, BCR2 holds the 16-bit width and WREN, MBKEN is clear.
 * - Bank 1: the bits common to the banks (FMCEN) of BCR1 are kept.
 */
void Ut_Fmc_Set_NorSramConfig_Sram_WritesRegistersKeepsController( void )
{
#if defined(FMC_Bank1_R)
    fmc_NorSramConfig_t sram;

    UtFmc_Init_Default();
    UtFmc_Get_SramConfig( &sram );

    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_NorSramConfig( FMC_BANK_NOR_PSRAM_2, &sram ) );
    TEST_ASSERT_EQUAL_HEX32( UT_FMC_BTR( UT_FMC_ADDSET_CYC, 1u, UT_FMC_DATAST_CYC, 0u, 0u, 0u, 0u, 0u ),
                             FMC_Bank1_R->BTCR[UT_FMC_BTCR_BTR2] );
    TEST_ASSERT_EQUAL_HEX32( UT_FMC_BWTR_RESET, FMC_Bank1E_R->BWTR[UT_FMC_BWTR_2] );
    TEST_ASSERT_EQUAL_HEX32( ( ( UT_FMC_MWID_16_FIELD << FMC_BCRx_MWID_Pos ) | FMC_BCRx_WREN ),
                             FMC_Bank1_R->BTCR[UT_FMC_BTCR_BCR2] );

    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_NorSramConfig( FMC_BANK_NOR_PSRAM_1, &sram ) );
    TEST_ASSERT_EQUAL_HEX32( FMC_BCR1_FMCEN, ( FMC_Bank1_R->BTCR[0] & ( FMC_BCR1_FMCEN | FMC_BCRx_MBKEN ) ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   Fmc_Set_NorSramConfig() rejects an invalid parameter or state and writes no register.
 *
 * \par Expected results
 * - Before Fmc_Init, a null configuration, a bank that is not a NOR / PSRAM / SRAM bank, an invalid bank, a bus clock
 *   that is not available and a time above the field: FMC_REQUEST_ERROR and BTR2 stays 0.
 */
void Ut_Fmc_Set_NorSramConfig_InvalidParameterOrState_ReturnsErrorNoWrite( void )
{
#if defined(FMC_Bank1_R)
    fmc_NorSramConfig_t sram;

    UtFmc_Setup_Mocks();
    UtFmc_Get_SramConfig( &sram );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_NorSramConfig( FMC_BANK_NOR_PSRAM_2, &sram ) );

    UtFmc_Init_Default();
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_NorSramConfig( FMC_BANK_NOR_PSRAM_2, FMC_NULL_PTR ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_NorSramConfig( FMC_BANK_NAND, &sram ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_NorSramConfig( (fmc_Bank_t)UT_FMC_BANK_INVALID, &sram ) );

    utFmc_BusClk = 0u;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_NorSramConfig( FMC_BANK_NOR_PSRAM_2, &sram ) );
    utFmc_BusClk = UT_FMC_HCLK_200MHZ;
    sram.DataSetup_ns = UT_FMC_DATAST_TOO_LONG_NS;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_NorSramConfig( FMC_BANK_NOR_PSRAM_2, &sram ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, FMC_Bank1_R->BTCR[UT_FMC_BTCR_BTR2] );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   The configuration of an active NOR / PSRAM / SRAM bank is rejected until the bank is disabled.
 *
 * \par Expected results
 * - After Fmc_Set_BankActive the configuration returns FMC_REQUEST_ERROR, after Fmc_Set_BankInactive FMC_REQUEST_OK.
 */
void Ut_Fmc_Set_NorSramConfig_ActiveBank_RejectedUntilInactive( void )
{
#if defined(FMC_Bank1_R)
    fmc_NorSramConfig_t sram;

    UtFmc_Init_Default();
    UtFmc_Get_SramConfig( &sram );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_NorSramConfig( FMC_BANK_NOR_PSRAM_1, &sram ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_BankActive( FMC_BANK_NOR_PSRAM_1 ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_NorSramConfig( FMC_BANK_NOR_PSRAM_1, &sram ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_BankInactive( FMC_BANK_NOR_PSRAM_1 ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_NorSramConfig( FMC_BANK_NOR_PSRAM_1, &sram ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   The chip select counter is common to the banks: a second bank with another count is rejected.
 *
 * \par Expected results
 * - PSRAM bank 1 with 4000 ns: CNTB1EN and CSCOUNT 800.
 * - Bank 2 with 8000 ns: FMC_REQUEST_ERROR; with 4000 ns: FMC_REQUEST_OK and CNTB2EN set too.
 * - Bank 1 reconfigured without a counter: CNTB1EN is cleared, CNTB2EN stays.
 */
void Ut_Fmc_Set_NorSramConfig_ChipSelectCounter_CommonCountChecked( void )
{
#if defined(FMC_Bank1_R)
    fmc_NorSramConfig_t psram;

    UtFmc_Init_Default();
    UtFmc_Get_SramConfig( &psram );
    psram.MemoryType          = FMC_MEMORY_PSRAM;
    psram.ChipSelectMaxLow_ns = UT_FMC_CS_NS;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_NorSramConfig( FMC_BANK_NOR_PSRAM_1, &psram ) );
    TEST_ASSERT_EQUAL_HEX32( ( ( UT_FMC_CS_CYC << FMC_PCSCNTR_CSCOUNT_Pos ) | FMC_PCSCNTR_CNTB1EN ),
                             FMC_Bank1_R->PCSCNTR );

    psram.ChipSelectMaxLow_ns = UT_FMC_CS2_NS;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_NorSramConfig( FMC_BANK_NOR_PSRAM_2, &psram ) );
    psram.ChipSelectMaxLow_ns = UT_FMC_CS_NS;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_NorSramConfig( FMC_BANK_NOR_PSRAM_2, &psram ) );
    TEST_ASSERT_EQUAL_HEX32( ( FMC_PCSCNTR_CNTB1EN | FMC_PCSCNTR_CNTB2EN ),
                             ( FMC_Bank1_R->PCSCNTR & UT_FMC_CNTBEN_MASK ) );

    psram.ChipSelectMaxLow_ns = 0u;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_NorSramConfig( FMC_BANK_NOR_PSRAM_1, &psram ) );
    TEST_ASSERT_EQUAL_HEX32( FMC_PCSCNTR_CNTB2EN, ( FMC_Bank1_R->PCSCNTR & UT_FMC_CNTBEN_MASK ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}

/* ============================== BANK CONTROL ================================ */

/**
 * \brief   Fmc_Set_BankActive() and Fmc_Set_BankInactive() set and clear MBKEN of a NOR / PSRAM / SRAM bank and PBKEN
 *          of
 *          the NAND bank; a bank that is not configured, an invalid bank and an uninitialized module are rejected.
 *
 * \par Expected results
 * - Before Init and for an unconfigured bank: FMC_REQUEST_ERROR.
 * - A configured bank: MBKEN is set, then cleared; the NAND bank: PBKEN is set, then cleared.
 * - Fmc_Set_BankInactive( invalid bank ): FMC_REQUEST_ERROR.
 */
void Ut_Fmc_Set_BankActive_ConfiguredBanks_SetsAndClearsEnableBits( void )
{
#if defined(FMC_Bank1_R)
    fmc_NorSramConfig_t sram;
    fmc_NandConfig_t    nand;

    UtFmc_Setup_Mocks();
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_BankActive( FMC_BANK_NOR_PSRAM_1 ) );
    UtFmc_Init_Default();
    UtFmc_Get_SramConfig( &sram );
    UtFmc_Get_NandConfig( &nand );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_BankActive( FMC_BANK_NOR_PSRAM_1 ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_BankActive( FMC_BANK_NAND ) );

    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_NorSramConfig( FMC_BANK_NOR_PSRAM_1, &sram ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_NandConfig( &nand ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_BankActive( FMC_BANK_NOR_PSRAM_1 ) );
    TEST_ASSERT_EQUAL_HEX32( FMC_BCRx_MBKEN, ( FMC_Bank1_R->BTCR[0] & FMC_BCRx_MBKEN ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_BankInactive( FMC_BANK_NOR_PSRAM_1 ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, ( FMC_Bank1_R->BTCR[0] & FMC_BCRx_MBKEN ) );

    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_BankActive( FMC_BANK_NAND ) );
    TEST_ASSERT_EQUAL_HEX32( FMC_PCR_PBKEN, ( FMC_Bank3_R->PCR & FMC_PCR_PBKEN ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_BankInactive( FMC_BANK_NAND ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, ( FMC_Bank3_R->PCR & FMC_PCR_PBKEN ) );

    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_BankActive( (fmc_Bank_t)UT_FMC_BANK_INVALID ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_BankInactive( (fmc_Bank_t)UT_FMC_BANK_INVALID ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   Fmc_Get_BankBaseAddress() returns the address of the memory window of every bank.
 *
 * \par Expected results
 * - NOR / PSRAM banks 1 to 4, the NAND bank and (where present) the SDRAM banks give the CMSIS window addresses.
 * - A null pointer, an invalid bank and an uninitialized module: FMC_REQUEST_ERROR.
 */
void Ut_Fmc_Get_BankBaseAddress_AllBanks_ReturnsWindowAddresses( void )
{
#if defined(FMC_Bank1_R)
    fmc_Address_t address = 0u;

    UtFmc_Setup_Mocks();
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Get_BankBaseAddress( FMC_BANK_NOR_PSRAM_1, &address ) );
    UtFmc_Init_Default();

    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_BankBaseAddress( FMC_BANK_NOR_PSRAM_1, &address ) );
    TEST_ASSERT_EQUAL_HEX32( FMC_BANK1_1, address );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_BankBaseAddress( FMC_BANK_NOR_PSRAM_2, &address ) );
    TEST_ASSERT_EQUAL_HEX32( FMC_BANK1_2, address );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_BankBaseAddress( FMC_BANK_NOR_PSRAM_3, &address ) );
    TEST_ASSERT_EQUAL_HEX32( FMC_BANK1_3, address );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_BankBaseAddress( FMC_BANK_NOR_PSRAM_4, &address ) );
    TEST_ASSERT_EQUAL_HEX32( FMC_BANK1_4, address );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_BankBaseAddress( FMC_BANK_NAND, &address ) );
    TEST_ASSERT_EQUAL_HEX32( FMC_BANK3, address );
#if defined(FMC_Bank5_6_R)
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_BankBaseAddress( FMC_BANK_SDRAM_1, &address ) );
    TEST_ASSERT_EQUAL_HEX32( FMC_SDRAM_BANK_1, address );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_BankBaseAddress( FMC_BANK_SDRAM_2, &address ) );
    TEST_ASSERT_EQUAL_HEX32( FMC_SDRAM_BANK_2, address );
#endif
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Get_BankBaseAddress( FMC_BANK_NOR_PSRAM_1, FMC_NULL_PTR ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Get_BankBaseAddress( (fmc_Bank_t)UT_FMC_BANK_INVALID, &address ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   Fmc_Set_BankWriteProtection() clears and sets WREN of a NOR / PSRAM / SRAM bank on the fly.
 *
 * \par Expected results
 * - Protection active: WREN cleared, inactive: WREN set (the bank may be active).
 * - The NAND bank, a bank that is not configured, a state out of the list: FMC_REQUEST_ERROR.
 */
void Ut_Fmc_Set_BankWriteProtection_NorBank_TogglesWren( void )
{
#if defined(FMC_Bank1_R)
    fmc_NorSramConfig_t sram;

    UtFmc_Init_Default();
    UtFmc_Get_SramConfig( &sram );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_BankWriteProtection( FMC_BANK_NOR_PSRAM_1, FMC_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_NorSramConfig( FMC_BANK_NOR_PSRAM_1, &sram ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_BankActive( FMC_BANK_NOR_PSRAM_1 ) );

    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_BankWriteProtection( FMC_BANK_NOR_PSRAM_1, FMC_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, ( FMC_Bank1_R->BTCR[0] & FMC_BCRx_WREN ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_BankWriteProtection( FMC_BANK_NOR_PSRAM_1, FMC_FUNCTION_INACTIVE ) );
    TEST_ASSERT_EQUAL_HEX32( FMC_BCRx_WREN, ( FMC_Bank1_R->BTCR[0] & FMC_BCRx_WREN ) );

    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_BankWriteProtection( FMC_BANK_NAND, FMC_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR,
                       Fmc_Set_BankWriteProtection( FMC_BANK_NOR_PSRAM_1,
                                                    (fmc_FunctionState_t)( FMC_FUNCTION_ACTIVE + 1u ) ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   Fmc_Set_IrqPriority() and Fmc_Get_IrqPriority() forward to Nvic and need an initialized module.
 *
 * \par Expected results
 * - Before Init: FMC_REQUEST_ERROR without a call of Nvic.
 * - After Init the priority is passed to Nvic_Set_PeriphIrq_Prio and read by Nvic_Get_PeriphIrq_Prio.
 * - A null pointer: FMC_REQUEST_ERROR.
 */
void Ut_Fmc_IrqPriority_SetGet_ForwardsToNvic( void )
{
#if defined(FMC_Bank1_R)
    fmc_IrqPrio_t priority = UT_FMC_IRQ_PRIO;

    UtFmc_Setup_Mocks();
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_IrqPriority( UT_FMC_IRQ_PRIO ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Get_IrqPriority( &priority ) );
    UtFmc_Init_Default();

    Nvic_Set_PeriphIrq_Prio_StopIgnore();
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_FMC, UT_FMC_IRQ_PRIO, NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_IrqPriority( UT_FMC_IRQ_PRIO ) );

    priority = 0u;
    Nvic_Get_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_FMC, FMC_NULL_PTR, NVIC_REQUEST_OK );
    Nvic_Get_PeriphIrq_Prio_IgnoreArg_irqPrio();
    Nvic_Get_PeriphIrq_Prio_ReturnThruPtr_irqPrio( &(fmc_IrqPrio_t){ UT_FMC_IRQ_PRIO } );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_IrqPriority( &priority ) );
    TEST_ASSERT_EQUAL_UINT32( UT_FMC_IRQ_PRIO, priority );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Get_IrqPriority( FMC_NULL_PTR ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}

/* ================================== PINS =================================== */

/**
 * \brief   The pin functions decode a pin item and configure the pin through Gpio (alternate function, very high speed,
 *          no pull resistor; pull-up on the wait and ready signals).
 *
 * \par Expected results
 * - A0 on PF0: port F, pin 0, alternate function 12, push-pull, very high speed, no pull.
 * - D0 on PD14 and NOE on PD4 are accepted, NWAIT on PD6 and INT on PG7 get the pull-up.
 * - The unused item, an item of another list and a failing Gpio: FMC_REQUEST_ERROR.
 */
void Ut_Fmc_InitGpio_PinItems_ConfiguredThroughGpio( void )
{
#if defined(FMC_Bank1_R)
    UtFmc_Setup_Mocks();

    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_InitAddressGpio( FMC_A0_PIN_PF0 ) );
    TEST_ASSERT_EQUAL( GPIO_PORT_F, utFmc_GpioConfig.PortId );
    TEST_ASSERT_EQUAL( GPIO_PIN_ID_0, utFmc_GpioConfig.PinId );
    TEST_ASSERT_EQUAL( GPIO_ALT_FUNC_12, utFmc_GpioConfig.PinAltFunction );
    TEST_ASSERT_EQUAL( GPIO_PIN_MODE_ALTERNATE, utFmc_GpioConfig.PinMode );
    TEST_ASSERT_EQUAL( GPIO_PIN_SPEED_VERY_HIGH, utFmc_GpioConfig.PinSpeed );
    TEST_ASSERT_EQUAL( GPIO_PIN_OUTPUT_PUSHPULL, utFmc_GpioConfig.PinOutType );
    TEST_ASSERT_EQUAL( GPIO_PIN_PULL_NONE, utFmc_GpioConfig.PinPull );

    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_InitDataGpio( FMC_D0_PIN_PD14 ) );
    TEST_ASSERT_EQUAL( GPIO_PORT_D, utFmc_GpioConfig.PortId );
    TEST_ASSERT_EQUAL( GPIO_PIN_ID_14, utFmc_GpioConfig.PinId );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_InitControlGpio( FMC_NOE_PIN_PD4 ) );
    TEST_ASSERT_EQUAL( GPIO_PIN_PULL_NONE, utFmc_GpioConfig.PinPull );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_InitControlGpio( FMC_NWAIT_PIN_PD6 ) );
    TEST_ASSERT_EQUAL( GPIO_PIN_PULL_UP, utFmc_GpioConfig.PinPull );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_InitControlGpio( FMC_INT_PIN_PG7 ) );
    TEST_ASSERT_EQUAL( GPIO_PIN_PULL_UP, utFmc_GpioConfig.PinPull );
#if defined(FMC_Bank5_6_R)
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_InitSdramGpio( FMC_SDCLK_PIN_PG8 ) );
    TEST_ASSERT_EQUAL( GPIO_PORT_G, utFmc_GpioConfig.PortId );
    TEST_ASSERT_EQUAL( GPIO_PIN_ID_8, utFmc_GpioConfig.PinId );
#endif
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   The pin functions reject the unused item, an item of another list and a pin that Gpio rejects.
 *
 * \par Expected results
 * - Every unused item: FMC_REQUEST_ERROR without a Gpio call.
 * - A data pin passed as an address pin (a value outside the list): FMC_REQUEST_ERROR without a Gpio call.
 * - A failing Gpio: FMC_REQUEST_ERROR.
 */
void Ut_Fmc_InitGpio_InvalidPin_ReturnsErrorNoGpioCall( void )
{
#if defined(FMC_Bank1_R)
    UtFmc_Setup_Mocks();

    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_InitAddressGpio( FMC_ADDRESS_PIN_UNUSED ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_InitDataGpio( FMC_DATA_PIN_UNUSED ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_InitControlGpio( FMC_CONTROL_PIN_UNUSED ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_InitSdramGpio( FMC_SDRAM_PIN_UNUSED ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_InitAddressGpio( (fmc_AddressPin_t)FMC_D0_PIN_PD14 ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_InitDataGpio( (fmc_DataPin_t)FMC_NE1_PIN_PD7 ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmc_GpioCnt );

    utFmc_GpioResult = GPIO_REQUEST_ERROR;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_InitAddressGpio( FMC_A0_PIN_PF0 ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmc_GpioCnt );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}

/* ====================== CONSTANTS OF THE SDRAM / NAND TESTS ================= */

/** SDRAM command MODE field: clock configuration enable */
#define UT_FMC_MODE_CLK_ENABLE                ( 1u )

/** SDRAM command MODE field: precharge all */
#define UT_FMC_MODE_PALL                      ( 2u )

/** SDRAM command MODE field: auto-refresh */
#define UT_FMC_MODE_AUTO_REFRESH              ( 3u )

/** SDRAM command MODE field: load mode register */
#define UT_FMC_MODE_LOAD_MODE                 ( 4u )

/** SDRAM command MODE field: self-refresh */
#define UT_FMC_MODE_SELF_REFRESH              ( 5u )

/** SDRAM command MODE field: power-down */
#define UT_FMC_MODE_POWER_DOWN                ( 6u )

/** Column bits that are in no list */
#define UT_FMC_COLUMNS_INVALID                ( 7u )

/** Row bits that are in no list */
#define UT_FMC_ROWS_INVALID                   ( 14u )

/** Internal banks that are in no list */
#define UT_FMC_BANKS_INVALID                  ( 3u )

/** CAS latency that is in no list */
#define UT_FMC_CAS_INVALID                    ( 4u )

/** tXSR that needs 18 cycles at 100 MHz (the field holds 17) */
#define UT_FMC_TXSR_TOO_LONG_NS               ( 175u )

/** tMRD that needs 19 cycles at 100 MHz (the field holds 18) */
#define UT_FMC_TMRD_TOO_LONG_NS               ( 181u )

/** tWR of 50 ns: 5 cycles at 100 MHz, above the minimum of the two rules */
#define UT_FMC_TWR_LONG_NS                    ( 50u )

/** tWR of 50 ns in cycles */
#define UT_FMC_TWR_LONG_CYC                   ( 5u )

/** tXSR of 70 ns at 200 MHz / 3 (a period of 15 ns): 5 cycles */
#define UT_FMC_TXSR_DIV3_CYC                  ( 5u )

/** tXSR of 70 ns at 200 MHz (a period of 5 ns): 14 cycles */
#define UT_FMC_TXSR_DIV1_CYC                  ( 14u )

/** tXSR of the second bank: 100 ns, 10 cycles at 100 MHz */
#define UT_FMC_TXSR_BANK2_NS                  ( 100u )

/** TXSR field of 10 cycles */
#define UT_FMC_TXSR_BANK2_FIELD               ( 8u )

/** tRP of the second bank: 40 ns, 4 cycles at 100 MHz */
#define UT_FMC_TRP_BANK2_NS                   ( 40u )

/** TRP field of 4 cycles */
#define UT_FMC_TRP_BANK2_FIELD                ( 2u )

/** Refresh period of the refresh rate tests: 32 ms */
#define UT_FMC_PERIOD_32MS                    ( 32u )

/** Refresh count of 32 ms for 8192 rows at 100 MHz: 390 cycles minus the margin of 20 */
#define UT_FMC_COUNT_32MS                     ( 370u )

/** Mode register of a raw load mode register command */
#define UT_FMC_MODE_REG_RAW                   ( 0x0123u )

/** Auto-refresh cycles of a raw command */
#define UT_FMC_REFRESH_CNT_RAW                ( 4u )

/** NRFS field of 4 auto-refresh cycles */
#define UT_FMC_NRFS_RAW_FIELD                 ( 3u )

/** Auto-refresh cycles above the largest value */
#define UT_FMC_REFRESH_CNT_TOO_MANY           ( 17u )

/** Mode register above 13 bits */
#define UT_FMC_MODE_REG_TOO_LARGE             ( 0x2000u )

/* =========================== SDRAM CONVERSIONS ============================= */

/**
 * \brief   FmcPriv_Get_SdramDivider() converts the divider items to the divider values.
 *
 * \par Expected results
 * - FMC_SDRAM_CLOCK_DIV_2 gives 2, FMC_SDRAM_CLOCK_DIV_3 gives 3, FMC_SDRAM_CLOCK_DIV_1 gives 1.
 * - An item out of the list and a null pointer: FMC_REQUEST_ERROR.
 */
void Ut_Fmc_Get_SdramDivider_Items_ConvertedOrRejected( void )
{
#if defined(FMC_Bank5_6_R)
    fmc_CycleCnt_t divider = 0u;

    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, FmcPriv_Get_SdramDivider( FMC_SDRAM_CLOCK_DIV_2, &divider ) );
    TEST_ASSERT_EQUAL_UINT32( UT_FMC_SDCLK_DIV2_FIELD, divider );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, FmcPriv_Get_SdramDivider( FMC_SDRAM_CLOCK_DIV_3, &divider ) );
    TEST_ASSERT_EQUAL_UINT32( UT_FMC_SDCLK_DIV3_FIELD, divider );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, FmcPriv_Get_SdramDivider( FMC_SDRAM_CLOCK_DIV_1, &divider ) );
    TEST_ASSERT_EQUAL_UINT32( FMC_NO_DIVIDER, divider );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_SdramDivider( FMC_SDRAM_CLOCK_DIV_CNT, &divider ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_SdramDivider( FMC_SDRAM_CLOCK_DIV_2, FMC_NULL_PTR ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_SDRAM_MSG );
#endif /* FMC_Bank5_6_R */
}


/**
 * \brief   FmcPriv_Get_SdramCycles() converts the times of the specification example at an SDRAM clock of 100 MHz and
 *          raises tMRD to the smallest value and tWR to the two rules of the reference manual.
 *
 * \par Expected results
 * - tMRD 14 ns: 3 cycles (raised from 2), tXSR 7, tRAS 5, tRC 7, tRP 2, tRCD 2 cycles.
 * - tWR 14 ns (2 cycles) is raised to 3 cycles (tRAS - tRCD = 3); a tWR of 5 cycles stays 5.
 */
void Ut_Fmc_Get_SdramCycles_ExampleTimes_ConvertedAndTwrRaised( void )
{
#if defined(FMC_Bank5_6_R)
    fmc_SdramConfig_t config;
    fmc_SdramCycles_t cycles;

    UtFmc_Get_SdramConfig( &config );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, FmcPriv_Get_SdramCycles( &config, UT_FMC_HCLK_200MHZ, &cycles ) );
    TEST_ASSERT_EQUAL_UINT32( UT_FMC_TMRD_CYC, cycles.LoadToActive );
    TEST_ASSERT_EQUAL_UINT32( UT_FMC_TXSR_CYC, cycles.ExitSelfRefresh );
    TEST_ASSERT_EQUAL_UINT32( UT_FMC_TRAS_CYC, cycles.SelfRefresh );
    TEST_ASSERT_EQUAL_UINT32( UT_FMC_TRC_CYC, cycles.RowCycle );
    TEST_ASSERT_EQUAL_UINT32( UT_FMC_TWR_CYC, cycles.WriteRecovery );
    TEST_ASSERT_EQUAL_UINT32( UT_FMC_SDRAM_TIME_2CYC, cycles.RowPrecharge );
    TEST_ASSERT_EQUAL_UINT32( UT_FMC_SDRAM_TIME_2CYC, cycles.RowToColumn );

    config.WriteRecoveryTime_ns = UT_FMC_TWR_LONG_NS;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, FmcPriv_Get_SdramCycles( &config, UT_FMC_HCLK_200MHZ, &cycles ) );
    TEST_ASSERT_EQUAL_UINT32( UT_FMC_TWR_LONG_CYC, cycles.WriteRecovery );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_SDRAM_MSG );
#endif /* FMC_Bank5_6_R */
}


/**
 * \brief   FmcPriv_Get_SdramCycles() uses the SDRAM clock of the configured divider.
 *
 * \par Expected results
 * - tXSR of 70 ns: 5 cycles with the divider 3 (a period of 15 ns), 14 cycles with the divider 1 (a period of 5 ns).
 */
void Ut_Fmc_Get_SdramCycles_Divider_UsesSdramClock( void )
{
#if defined(FMC_Bank5_6_R)
    fmc_SdramConfig_t config;
    fmc_SdramCycles_t cycles;

    UtFmc_Get_SdramConfig( &config );
    config.ClockDivider = FMC_SDRAM_CLOCK_DIV_3;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, FmcPriv_Get_SdramCycles( &config, UT_FMC_HCLK_200MHZ, &cycles ) );
    TEST_ASSERT_EQUAL_UINT32( UT_FMC_TXSR_DIV3_CYC, cycles.ExitSelfRefresh );
    config.ClockDivider = FMC_SDRAM_CLOCK_DIV_1;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, FmcPriv_Get_SdramCycles( &config, UT_FMC_HCLK_200MHZ, &cycles ) );
    TEST_ASSERT_EQUAL_UINT32( UT_FMC_TXSR_DIV1_CYC, cycles.ExitSelfRefresh );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_SDRAM_MSG );
#endif /* FMC_Bank5_6_R */
}


/**
 * \brief   FmcPriv_Get_SdramCycles() rejects a time above the register field and an item out of its list.
 *
 * \par Expected results
 * - FMC_REQUEST_ERROR for tXSR of 18 cycles, tMRD of 19 cycles, a column, row, internal bank and CAS latency item that
 *   is in no list, a clock divider, a read pipe and a function state out of the list, a width that is in no list.
 * - FMC_REQUEST_ERROR for a null pointer and a bus clock of 0 Hz.
 */
void Ut_Fmc_Get_SdramCycles_InvalidConfig_ReturnsError( void )
{
#if defined(FMC_Bank5_6_R)
    fmc_SdramConfig_t config;
    fmc_SdramCycles_t cycles;

    UtFmc_Get_SdramConfig( &config );
    config.ExitSelfRefreshDelay_ns = UT_FMC_TXSR_TOO_LONG_NS;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_SdramCycles( &config, UT_FMC_HCLK_200MHZ, &cycles ) );

    UtFmc_Get_SdramConfig( &config );
    config.LoadToActiveDelay_ns = UT_FMC_TMRD_TOO_LONG_NS;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_SdramCycles( &config, UT_FMC_HCLK_200MHZ, &cycles ) );

    UtFmc_Get_SdramConfig( &config );
    config.ColumnBits = (fmc_SdramColumnBits_t)UT_FMC_COLUMNS_INVALID;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_SdramCycles( &config, UT_FMC_HCLK_200MHZ, &cycles ) );

    UtFmc_Get_SdramConfig( &config );
    config.RowBits = (fmc_SdramRowBits_t)UT_FMC_ROWS_INVALID;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_SdramCycles( &config, UT_FMC_HCLK_200MHZ, &cycles ) );

    UtFmc_Get_SdramConfig( &config );
    config.InternalBanks = (fmc_SdramInternalBanks_t)UT_FMC_BANKS_INVALID;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_SdramCycles( &config, UT_FMC_HCLK_200MHZ, &cycles ) );

    UtFmc_Get_SdramConfig( &config );
    config.CasLatency = (fmc_CasLatency_t)UT_FMC_CAS_INVALID;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_SdramCycles( &config, UT_FMC_HCLK_200MHZ, &cycles ) );

    UtFmc_Get_SdramConfig( &config );
    config.ClockDivider = FMC_SDRAM_CLOCK_DIV_CNT;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_SdramCycles( &config, UT_FMC_HCLK_200MHZ, &cycles ) );

    UtFmc_Get_SdramConfig( &config );
    config.ReadPipe = FMC_READ_PIPE_CNT;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_SdramCycles( &config, UT_FMC_HCLK_200MHZ, &cycles ) );

    UtFmc_Get_SdramConfig( &config );
    config.WriteProtect = (fmc_FunctionState_t)( FMC_FUNCTION_ACTIVE + 1u );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_SdramCycles( &config, UT_FMC_HCLK_200MHZ, &cycles ) );

    UtFmc_Get_SdramConfig( &config );
    config.DataWidth = (fmc_DataWidth_t)UT_FMC_WIDTH_INVALID;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_SdramCycles( &config, UT_FMC_HCLK_200MHZ, &cycles ) );

    UtFmc_Get_SdramConfig( &config );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_SdramCycles( FMC_NULL_PTR, UT_FMC_HCLK_200MHZ, &cycles ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_SdramCycles( &config, UT_FMC_HCLK_200MHZ, FMC_NULL_PTR ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_SdramCycles( &config, 0u, &cycles ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_SDRAM_MSG );
#endif /* FMC_Bank5_6_R */
}


/**
 * \brief   FmcPriv_Get_SdramRefreshCount() calculates COUNT = period / rows * SDRAM clock - 20, rounded down.
 *
 * \par Expected results
 * - 64 ms, 8192 rows, 200 MHz / 2: 781 cycles per row, COUNT 761.
 * - A sum of the timings that equals the count plus 4 cycles decreases the count by one (760).
 * - 32 ms: COUNT 370.
 */
void Ut_Fmc_Get_SdramRefreshCount_Example_MatchesFormula( void )
{
#if defined(FMC_Bank5_6_R)
    fmc_RegValue_t count = 0u;

    TEST_ASSERT_EQUAL( FMC_REQUEST_OK,
                       FmcPriv_Get_SdramRefreshCount( UT_FMC_SDRAM_PERIOD_MS,
                                                      UT_FMC_SDRAM_ROWS,
                                                      UT_FMC_HCLK_200MHZ,
                                                      UT_FMC_SDCLK_DIV2_FIELD,
                                                      0u,
                                                      &count ) );
    TEST_ASSERT_EQUAL_UINT32( UT_FMC_SDRAM_COUNT, count );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK,
                       FmcPriv_Get_SdramRefreshCount( UT_FMC_SDRAM_PERIOD_MS,
                                                      UT_FMC_SDRAM_ROWS,
                                                      UT_FMC_HCLK_200MHZ,
                                                      UT_FMC_SDCLK_DIV2_FIELD,
                                                      UT_FMC_SUM_COLLIDE,
                                                      &count ) );
    TEST_ASSERT_EQUAL_UINT32( UT_FMC_SDRAM_COUNT_COLLIDE, count );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK,
                       FmcPriv_Get_SdramRefreshCount( UT_FMC_PERIOD_32MS,
                                                      UT_FMC_SDRAM_ROWS,
                                                      UT_FMC_HCLK_200MHZ,
                                                      UT_FMC_SDCLK_DIV2_FIELD,
                                                      0u,
                                                      &count ) );
    TEST_ASSERT_EQUAL_UINT32( UT_FMC_COUNT_32MS, count );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_SDRAM_MSG );
#endif /* FMC_Bank5_6_R */
}


/**
 * \brief   FmcPriv_Get_SdramRefreshCount() rejects a count outside 41 .. 8191 and invalid parameters.
 *
 * \par Expected results
 * - 1 ms for 8192 rows (12 cycles per row) and 1000 ms for one row (100 million cycles): FMC_REQUEST_ERROR.
 * - A period, rows, clock or divider of 0 and a null pointer: FMC_REQUEST_ERROR.
 */
void Ut_Fmc_Get_SdramRefreshCount_OutOfRange_ReturnsError( void )
{
#if defined(FMC_Bank5_6_R)
    fmc_RegValue_t count = 0u;

    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR,
                       FmcPriv_Get_SdramRefreshCount( UT_FMC_PERIOD_SHORT_MS,
                                                      UT_FMC_SDRAM_ROWS,
                                                      UT_FMC_HCLK_200MHZ,
                                                      UT_FMC_SDCLK_DIV2_FIELD,
                                                      0u,
                                                      &count ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR,
                       FmcPriv_Get_SdramRefreshCount( UT_FMC_PERIOD_LONG_MS,
                                                      UT_FMC_ONE_ROW,
                                                      UT_FMC_HCLK_200MHZ,
                                                      UT_FMC_SDCLK_DIV2_FIELD,
                                                      0u,
                                                      &count ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR,
                       FmcPriv_Get_SdramRefreshCount( 0u,
                                                      UT_FMC_SDRAM_ROWS,
                                                      UT_FMC_HCLK_200MHZ,
                                                      UT_FMC_SDCLK_DIV2_FIELD,
                                                      0u,
                                                      &count ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR,
                       FmcPriv_Get_SdramRefreshCount( UT_FMC_SDRAM_PERIOD_MS,
                                                      0u,
                                                      UT_FMC_HCLK_200MHZ,
                                                      UT_FMC_SDCLK_DIV2_FIELD,
                                                      0u,
                                                      &count ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR,
                       FmcPriv_Get_SdramRefreshCount( UT_FMC_SDRAM_PERIOD_MS,
                                                      UT_FMC_SDRAM_ROWS,
                                                      0u,
                                                      UT_FMC_SDCLK_DIV2_FIELD,
                                                      0u,
                                                      &count ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR,
                       FmcPriv_Get_SdramRefreshCount( UT_FMC_SDRAM_PERIOD_MS,
                                                      UT_FMC_SDRAM_ROWS,
                                                      UT_FMC_HCLK_200MHZ,
                                                      0u,
                                                      0u,
                                                      &count ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR,
                       FmcPriv_Get_SdramRefreshCount( UT_FMC_SDRAM_PERIOD_MS,
                                                      UT_FMC_SDRAM_ROWS,
                                                      UT_FMC_HCLK_200MHZ,
                                                      UT_FMC_SDCLK_DIV2_FIELD,
                                                      0u,
                                                      FMC_NULL_PTR ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_SDRAM_MSG );
#endif /* FMC_Bank5_6_R */
}


/**
 * \brief   FmcPriv_Get_SdramModeRegister() builds the mode register: burst length 1, sequential, the CAS latency,
 *          standard operation, single location write.
 *
 * \par Expected results
 * - CAS latency 3 gives 0x0230, CAS latency 2 gives 0x0220.
 */
void Ut_Fmc_Get_SdramModeRegister_CasLatency_BuildsRegister( void )
{
#if defined(FMC_Bank5_6_R)
    fmc_SdramConfig_t config;

    UtFmc_Get_SdramConfig( &config );
    TEST_ASSERT_EQUAL_HEX16( UT_FMC_MODE_REG_CAS3, FmcPriv_Get_SdramModeRegister( &config ) );
    config.CasLatency = FMC_CAS_LATENCY_2;
    TEST_ASSERT_EQUAL_HEX16( UT_FMC_MODE_REG_CAS2, FmcPriv_Get_SdramModeRegister( &config ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_SDRAM_MSG );
#endif /* FMC_Bank5_6_R */
}

/* ============================= SDRAM BANKS ================================ */

/**
 * \brief   The SDRAM functions return an error on a line with the FMC but without the SDRAM controller (STM32H523 /
 *          H533).
 *
 * \par Expected results
 * - On STM32H523 / H533 every SDRAM function returns FMC_REQUEST_ERROR.
 * - On the other lines the test has nothing to check.
 */
void Ut_Fmc_SdramFunctions_NoSdramController_ReturnError( void )
{
#if defined(FMC_Bank1_R)
#if !defined(FMC_Bank5_6_R)
    fmc_SdramConfig_t  config;
    fmc_SdramCommand_t command;
    fmc_SdramState_t   state;

    UtFmc_Init_Default();
    UtFmc_Get_SdramConfig( &config );
    (void)memset( &command, 0, sizeof( command ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramConfig( (fmc_Bank_t)UT_FMC_BANK_IDX_1, &config ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramInit( (fmc_Bank_t)UT_FMC_BANK_IDX_1 ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramRefreshRate( UT_FMC_SDRAM_PERIOD_MS, UT_FMC_SDRAM_ROWS ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramSelfRefresh( (fmc_Bank_t)UT_FMC_BANK_IDX_1 ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramPowerDown( (fmc_Bank_t)UT_FMC_BANK_IDX_1 ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramCommand( (fmc_Bank_t)UT_FMC_BANK_IDX_1, &command ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Get_SdramState( (fmc_Bank_t)UT_FMC_BANK_IDX_1, &state ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_SDRAM_MSG );
#endif /* !FMC_Bank5_6_R */
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   Fmc_Set_SdramConfig() writes the control register of the bank, the common bits to FMC_SDCR1 and the timing
 *          registers; the timing register of the bank that is not used keeps its reset value.
 *
 * \par Expected results
 * - SDCR1 holds 9 columns, 13 rows, 16 bit, 4 internal banks, CAS latency 3 and the SDRAM clock HCLK / 2.
 * - SDTR1 holds TMRD 0, TXSR 5, TRAS 3, TRC 5, TWR 1, TRP 0, TRCD 0 (the cycles minus the offsets); SDTR2 the reset
 *   value.
 * - The controller is enabled after the call.
 */
void Ut_Fmc_Set_SdramConfig_Bank1_WritesRegisters( void )
{
#if defined(FMC_Bank5_6_R)
    fmc_SdramConfig_t config;

    UtFmc_Init_Default();
    UtFmc_Get_SdramConfig( &config );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_SdramConfig( FMC_BANK_SDRAM_1, &config ) );

    TEST_ASSERT_EQUAL_HEX32( ( ( UT_FMC_NC_9_FIELD << FMC_SDCRx_NC_Pos ) |
                               ( UT_FMC_NR_13_FIELD << FMC_SDCRx_NR_Pos ) |
                               ( UT_FMC_MWID_16_FIELD << FMC_SDCRx_MWID_Pos ) |
                               FMC_SDCRx_NB |
                               ( UT_FMC_CAS3_FIELD << FMC_SDCRx_CAS_Pos ) |
                               ( UT_FMC_SDCLK_DIV2_FIELD << FMC_SDCRx_SDCLK_Pos ) ),
                             FMC_Bank5_6_R->SDCR[0] );
    TEST_ASSERT_EQUAL_HEX32( UT_FMC_SDTR( 0u,
                                          UT_FMC_TXSR_FIELD,
                                          UT_FMC_TRAS_FIELD,
                                          UT_FMC_TRC_FIELD,
                                          UT_FMC_TWR_FIELD,
                                          0u,
                                          0u ),
                             FMC_Bank5_6_R->SDTR[0] );
    TEST_ASSERT_EQUAL_HEX32( UT_FMC_SDTR_RESET, FMC_Bank5_6_R->SDTR[1] );
    TEST_ASSERT_EQUAL_HEX32( FMC_BCR1_FMCEN, ( FMC_Bank1_R->BTCR[0] & FMC_BCR1_FMCEN ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_SDRAM_MSG );
#endif /* FMC_Bank5_6_R */
}


/**
 * \brief   Fmc_Set_SdramConfig() of the second bank: the common values must match the first bank and the timings the
 *          controller shares between the devices become the slowest of both.
 *
 * \par Expected results
 * - Bank 2 with another clock divider: FMC_REQUEST_ERROR and no register changes.
 * - Bank 2 with tXSR 100 ns and tRP 40 ns: SDTR1 and SDTR2 hold TXSR 8 and TRP 2 (the slower device), the rest of SDTR1
 *   is
 *   unchanged.
 */
void Ut_Fmc_Set_SdramConfig_TwoBanks_CommonCheckedSharedTimingsMerged( void )
{
#if defined(FMC_Bank5_6_R)
    fmc_SdramConfig_t first;
    fmc_SdramConfig_t second;

    UtFmc_Init_Default();
    UtFmc_Get_SdramConfig( &first );
    UtFmc_Get_SdramConfig( &second );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_SdramConfig( FMC_BANK_SDRAM_1, &first ) );

    second.ClockDivider = FMC_SDRAM_CLOCK_DIV_3;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramConfig( FMC_BANK_SDRAM_2, &second ) );
    TEST_ASSERT_EQUAL_HEX32( UT_FMC_SDTR_RESET, FMC_Bank5_6_R->SDTR[1] );

    second.ClockDivider            = FMC_SDRAM_CLOCK_DIV_2;
    second.ExitSelfRefreshDelay_ns = UT_FMC_TXSR_BANK2_NS;
    second.RpDelay_ns              = UT_FMC_TRP_BANK2_NS;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_SdramConfig( FMC_BANK_SDRAM_2, &second ) );
    TEST_ASSERT_EQUAL_HEX32( UT_FMC_SDTR( 0u,
                                          UT_FMC_TXSR_BANK2_FIELD,
                                          UT_FMC_TRAS_FIELD,
                                          UT_FMC_TRC_FIELD,
                                          UT_FMC_TWR_FIELD,
                                          UT_FMC_TRP_BANK2_FIELD,
                                          0u ),
                             FMC_Bank5_6_R->SDTR[0] );
    TEST_ASSERT_EQUAL_HEX32( UT_FMC_SDTR( 0u,
                                          UT_FMC_TXSR_BANK2_FIELD,
                                          UT_FMC_TRAS_FIELD,
                                          UT_FMC_TRC_FIELD,
                                          UT_FMC_TWR_FIELD,
                                          UT_FMC_TRP_BANK2_FIELD,
                                          0u ),
                             FMC_Bank5_6_R->SDTR[1] );
    TEST_ASSERT_EQUAL_HEX32( ( ( UT_FMC_NC_9_FIELD << FMC_SDCRx_NC_Pos ) | ( UT_FMC_NR_13_FIELD << FMC_SDCRx_NR_Pos ) |
                               ( UT_FMC_MWID_16_FIELD << FMC_SDCRx_MWID_Pos ) | FMC_SDCRx_NB |
                               ( UT_FMC_CAS3_FIELD << FMC_SDCRx_CAS_Pos ) ),
                             FMC_Bank5_6_R->SDCR[UT_FMC_BANK_IDX_2] );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_SDRAM_MSG );
#endif /* FMC_Bank5_6_R */
}


/**
 * \brief   Fmc_Set_SdramConfig() rejects an invalid parameter, a configuration the clock can not meet and a state that
 *          does not allow it, and writes no register.
 *
 * \par Expected results
 * - Before Init, a null configuration, a bank that is not an SDRAM bank, a bus clock that is not available, a refresh
 *   period the clock can not serve (1 ms for 8192 rows): FMC_REQUEST_ERROR, SDCR1 stays 0.
 */
void Ut_Fmc_Set_SdramConfig_InvalidParameterOrState_ReturnsErrorNoWrite( void )
{
#if defined(FMC_Bank5_6_R)
    fmc_SdramConfig_t config;

    UtFmc_Setup_Mocks();
    UtFmc_Get_SdramConfig( &config );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramConfig( FMC_BANK_SDRAM_1, &config ) );

    UtFmc_Init_Default();
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramConfig( FMC_BANK_SDRAM_1, FMC_NULL_PTR ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramConfig( FMC_BANK_NAND, &config ) );
    utFmc_BusClk = 0u;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramConfig( FMC_BANK_SDRAM_1, &config ) );
    utFmc_BusClk = UT_FMC_HCLK_200MHZ;
    config.RefreshPeriod_ms = UT_FMC_PERIOD_SHORT_MS;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramConfig( FMC_BANK_SDRAM_1, &config ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, FMC_Bank5_6_R->SDCR[0] );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_SDRAM_MSG );
#endif /* FMC_Bank5_6_R */
}


/**
 * \brief   Fmc_Set_SdramInit() runs the initialization sequence in the order of the reference manual.
 *
 * \par Expected results
 * - FMC_REQUEST_OK; the commands are: clock enable, precharge all, 8 auto-refresh cycles (NRFS 7), load mode register
 *   with 0x0230, each for SDRAM bank 1.
 * - SDRTR holds the refresh count 761, the refresh error interrupt is not enabled (events polled).
 * - Fmc_Set_BankActive() then sends the normal mode command to the bank.
 */
void Ut_Fmc_Set_SdramInit_Bank1_RunsSequenceInOrder( void )
{
#if defined(FMC_Bank5_6_R)
    fmc_SdramConfig_t config;

    UtFmc_Init_Default();
    UtFmc_Get_SdramConfig( &config );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_SdramConfig( FMC_BANK_SDRAM_1, &config ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_BankActive( FMC_BANK_SDRAM_1 ) );

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtFmc_SdramModel ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_SdramInit( FMC_BANK_SDRAM_1 ) );
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelInactive() );

    TEST_ASSERT_EQUAL_UINT32( UT_FMC_INIT_COMMANDS, utFmc_CmrCnt );
    TEST_ASSERT_EQUAL_HEX32( ( ( UT_FMC_MODE_CLK_ENABLE << FMC_SDCMR_MODE_Pos ) | FMC_SDCMR_CTB1 ), utFmc_CmrLog[0] );
    TEST_ASSERT_EQUAL_HEX32( ( ( UT_FMC_MODE_PALL << FMC_SDCMR_MODE_Pos ) | FMC_SDCMR_CTB1 ),
                             utFmc_CmrLog[UT_FMC_CMD_IDX_2] );
    TEST_ASSERT_EQUAL_HEX32( ( ( UT_FMC_MODE_AUTO_REFRESH << FMC_SDCMR_MODE_Pos ) |
                               ( UT_FMC_NRFS_INIT_FIELD << FMC_SDCMR_NRFS_Pos ) |
                               FMC_SDCMR_CTB1 ),
                             utFmc_CmrLog[UT_FMC_CMD_IDX_3] );
    TEST_ASSERT_EQUAL_HEX32( ( ( UT_FMC_MODE_LOAD_MODE << FMC_SDCMR_MODE_Pos ) |
                               ( UT_FMC_MODE_REG_CAS3 << FMC_SDCMR_MRD_Pos ) |
                               FMC_SDCMR_CTB1 ),
                             utFmc_CmrLog[UT_FMC_CMD_IDX_4] );
    TEST_ASSERT_EQUAL_HEX32( ( UT_FMC_SDRAM_COUNT << FMC_SDRTR_COUNT_Pos ),
                             ( FMC_Bank5_6_R->SDRTR & ( FMC_SDRTR_COUNT | FMC_SDRTR_REIE ) ) );

    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_BankActive( FMC_BANK_SDRAM_1 ) );
    TEST_ASSERT_EQUAL_HEX32( FMC_SDCMR_CTB1, FMC_Bank5_6_R->SDCMR );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_SDRAM_MSG );
#endif /* FMC_Bank5_6_R */
}


/**
 * \brief   Fmc_Set_SdramInit() of two configured banks initializes both at once and loads the mode register of each.
 *
 * \par Expected results
 * - The clock enable, the precharge all and the auto-refresh are issued to both banks (CTB1 and CTB2).
 * - The load mode register commands follow, bank 1 with 0x0230 (CAS 3) and bank 2 with 0x0220 (CAS 2).
 * - Initializing with the bank 2 as the parameter gives the same result.
 */
void Ut_Fmc_Set_SdramInit_TwoBanks_CommandsToBothBanks( void )
{
#if defined(FMC_Bank5_6_R)
    fmc_SdramConfig_t first;
    fmc_SdramConfig_t second;

    UtFmc_Init_Default();
    UtFmc_Get_SdramConfig( &first );
    UtFmc_Get_SdramConfig( &second );
    second.CasLatency = FMC_CAS_LATENCY_2;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_SdramConfig( FMC_BANK_SDRAM_1, &first ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_SdramConfig( FMC_BANK_SDRAM_2, &second ) );

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtFmc_SdramModel ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_SdramInit( FMC_BANK_SDRAM_2 ) );
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelInactive() );

    TEST_ASSERT_EQUAL_UINT32( ( UT_FMC_INIT_COMMANDS + 1u ), utFmc_CmrCnt );
    TEST_ASSERT_EQUAL_HEX32( ( ( UT_FMC_MODE_CLK_ENABLE << FMC_SDCMR_MODE_Pos ) | FMC_SDCMR_CTB1 | FMC_SDCMR_CTB2 ),
                             utFmc_CmrLog[0] );
    TEST_ASSERT_EQUAL_HEX32( ( ( UT_FMC_MODE_PALL << FMC_SDCMR_MODE_Pos ) | FMC_SDCMR_CTB1 | FMC_SDCMR_CTB2 ),
                             utFmc_CmrLog[UT_FMC_CMD_IDX_2] );
    TEST_ASSERT_EQUAL_HEX32( ( ( UT_FMC_MODE_AUTO_REFRESH << FMC_SDCMR_MODE_Pos ) |
                               ( UT_FMC_NRFS_INIT_FIELD << FMC_SDCMR_NRFS_Pos ) |
                               FMC_SDCMR_CTB1 |
                               FMC_SDCMR_CTB2 ),
                             utFmc_CmrLog[UT_FMC_CMD_IDX_3] );
    TEST_ASSERT_EQUAL_HEX32( ( ( UT_FMC_MODE_LOAD_MODE << FMC_SDCMR_MODE_Pos ) |
                               ( UT_FMC_MODE_REG_CAS3 << FMC_SDCMR_MRD_Pos ) |
                               FMC_SDCMR_CTB1 ),
                             utFmc_CmrLog[UT_FMC_CMD_IDX_4] );
    TEST_ASSERT_EQUAL_HEX32( ( ( UT_FMC_MODE_LOAD_MODE << FMC_SDCMR_MODE_Pos ) |
                               ( UT_FMC_MODE_REG_CAS2 << FMC_SDCMR_MRD_Pos ) |
                               FMC_SDCMR_CTB2 ),
                             utFmc_CmrLog[UT_FMC_INIT_COMMANDS] );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_SDRAM_MSG );
#endif /* FMC_Bank5_6_R */
}


/**
 * \brief   Fmc_Set_SdramInit() fails on a timeout of the controller and for a state that does not allow it.
 *
 * \par Expected results
 * - With BUSY set in FMC_SDSR the call returns FMC_REQUEST_ERROR and the bank is not initialized (activation fails).
 * - A bank that is not configured, an invalid bank and an uninitialized module: FMC_REQUEST_ERROR.
 */
void Ut_Fmc_Set_SdramInit_BusyTimeoutOrInvalidState_ReturnsError( void )
{
#if defined(FMC_Bank5_6_R)
    fmc_SdramConfig_t config;

    UtFmc_Setup_Mocks();
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramInit( FMC_BANK_SDRAM_1 ) );
    UtFmc_Init_Default();
    UtFmc_Get_SdramConfig( &config );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramInit( FMC_BANK_SDRAM_1 ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramInit( FMC_BANK_NAND ) );

    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_SdramConfig( FMC_BANK_SDRAM_1, &config ) );
    FMC_Bank5_6_R->SDSR = FMC_REG_SDSR_BUSY;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramInit( FMC_BANK_SDRAM_1 ) );
    FMC_Bank5_6_R->SDSR = 0u;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_BankActive( FMC_BANK_SDRAM_1 ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_SDRAM_MSG );
#endif /* FMC_Bank5_6_R */
}


/**
 * \brief   The SDRAM bank has no enable bit: Fmc_Set_BankInactive() rejects it, and the configuration of an active
 *          SDRAM
 *          bank is rejected.
 *
 * \par Expected results
 * - After the initialization and the activation Fmc_Set_BankInactive( FMC_BANK_SDRAM_1 ) and Fmc_Set_SdramConfig() and
 *   Fmc_Set_SdramInit() return FMC_REQUEST_ERROR.
 */
void Ut_Fmc_Set_BankInactive_ActiveSdramBank_RejectedAndConfigLocked( void )
{
#if defined(FMC_Bank5_6_R)
    fmc_SdramConfig_t config;

    UtFmc_Init_Default();
    UtFmc_Get_SdramConfig( &config );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_SdramConfig( FMC_BANK_SDRAM_1, &config ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_SdramInit( FMC_BANK_SDRAM_1 ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_BankActive( FMC_BANK_SDRAM_1 ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_BankInactive( FMC_BANK_SDRAM_1 ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramConfig( FMC_BANK_SDRAM_1, &config ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramInit( FMC_BANK_SDRAM_1 ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_SDRAM_MSG );
#endif /* FMC_Bank5_6_R */
}


/**
 * \brief   Fmc_Set_SdramInit() enables the refresh error interrupt when the events are served by the interrupt and a
 *          callback exists; Fmc_Set_SdramRefreshRate() changes the count and keeps that bit.
 *
 * \par Expected results
 * - ISR mode with a refresh error callback: SDRTR holds the count 761 and REIE.
 * - Fmc_Set_SdramRefreshRate( 32 ms, 8192 rows ): the count 370 and REIE; a period the clock can not serve and a call
 *   before the initialization: FMC_REQUEST_ERROR and the register is unchanged.
 */
void Ut_Fmc_Set_SdramRefreshRate_AfterInit_ChangesCountKeepsInterruptEnable( void )
{
#if defined(FMC_Bank5_6_R)
    fmc_Config_t      fmcConfig;
    fmc_SdramConfig_t config;

    UtFmc_Setup_Mocks();
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_DefaultConfig( &fmcConfig ) );
    fmcConfig.EventMode            = FMC_EVENT_MODE_ISR;
    fmcConfig.RefreshErrorCallback = UtFmc_RefreshErrorCb;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Init( &fmcConfig ) );
    UtFmc_Get_SdramConfig( &config );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_SdramConfig( FMC_BANK_SDRAM_1, &config ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramRefreshRate( UT_FMC_PERIOD_32MS, UT_FMC_SDRAM_ROWS ) );

    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_SdramInit( FMC_BANK_SDRAM_1 ) );
    TEST_ASSERT_EQUAL_HEX32( ( ( UT_FMC_SDRAM_COUNT << FMC_SDRTR_COUNT_Pos ) | FMC_SDRTR_REIE ),
                             ( FMC_Bank5_6_R->SDRTR & ( FMC_SDRTR_COUNT | FMC_SDRTR_REIE ) ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_SdramRefreshRate( UT_FMC_PERIOD_32MS, UT_FMC_SDRAM_ROWS ) );
    TEST_ASSERT_EQUAL_HEX32( ( ( UT_FMC_COUNT_32MS << FMC_SDRTR_COUNT_Pos ) | FMC_SDRTR_REIE ),
                             ( FMC_Bank5_6_R->SDRTR & ( FMC_SDRTR_COUNT | FMC_SDRTR_REIE ) ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramRefreshRate( UT_FMC_PERIOD_SHORT_MS, UT_FMC_SDRAM_ROWS ) );
    TEST_ASSERT_EQUAL_HEX32( ( ( UT_FMC_COUNT_32MS << FMC_SDRTR_COUNT_Pos ) | FMC_SDRTR_REIE ),
                             ( FMC_Bank5_6_R->SDRTR & ( FMC_SDRTR_COUNT | FMC_SDRTR_REIE ) ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_SDRAM_MSG );
#endif /* FMC_Bank5_6_R */
}


/**
 * \brief   Fmc_Set_SdramSelfRefresh() and Fmc_Set_SdramPowerDown() send the command and check the mode status of the
 *          bank.
 *
 * \par Expected results
 * - With the mode status of the controller preset to self-refresh / power-down the calls return FMC_REQUEST_OK and the
 *   command register holds the command and the target bank.
 * - With the mode status normal the calls return FMC_REQUEST_ERROR.
 * - A bank that is not initialized and an invalid bank: FMC_REQUEST_ERROR.
 */
void Ut_Fmc_Set_SdramSelfRefreshPowerDown_ModeStatus_CheckedAgainstController( void )
{
#if defined(FMC_Bank5_6_R)
    fmc_SdramConfig_t config;

    UtFmc_Init_Default();
    UtFmc_Get_SdramConfig( &config );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_SdramConfig( FMC_BANK_SDRAM_1, &config ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramSelfRefresh( FMC_BANK_SDRAM_1 ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_SdramInit( FMC_BANK_SDRAM_1 ) );

    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramSelfRefresh( FMC_BANK_SDRAM_1 ) );
    FMC_Bank5_6_R->SDSR = ( UT_FMC_MODES_SELF << FMC_SDSR_MODES1_Pos );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_SdramSelfRefresh( FMC_BANK_SDRAM_1 ) );
    TEST_ASSERT_EQUAL_HEX32( ( ( UT_FMC_MODE_SELF_REFRESH << FMC_SDCMR_MODE_Pos ) | FMC_SDCMR_CTB1 ),
                             FMC_Bank5_6_R->SDCMR );

    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramPowerDown( FMC_BANK_SDRAM_1 ) );
    FMC_Bank5_6_R->SDSR = ( UT_FMC_MODES_POWER_DOWN << FMC_SDSR_MODES1_Pos );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_SdramPowerDown( FMC_BANK_SDRAM_1 ) );
    TEST_ASSERT_EQUAL_HEX32( ( ( UT_FMC_MODE_POWER_DOWN << FMC_SDCMR_MODE_Pos ) | FMC_SDCMR_CTB1 ),
                             FMC_Bank5_6_R->SDCMR );

    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramSelfRefresh( FMC_BANK_SDRAM_2 ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramPowerDown( FMC_BANK_NAND ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_SDRAM_MSG );
#endif /* FMC_Bank5_6_R */
}


/**
 * \brief   Fmc_Set_SdramCommand() sends a raw command to one bank and checks the parameters of the command.
 *
 * \par Expected results
 * - A load mode register command with 0x0123: SDCMR holds the mode, the mode register and the target bank.
 * - An auto-refresh command with 4 cycles: NRFS 3.
 * - 0 and 17 auto-refresh cycles, a mode register of 14 bits, a command out of the list, a bank that is not configured,
 *   a null command: FMC_REQUEST_ERROR and SDCMR is unchanged.
 */
void Ut_Fmc_Set_SdramCommand_RawCommands_SentOrRejected( void )
{
#if defined(FMC_Bank5_6_R)
    fmc_SdramConfig_t  config;
    fmc_SdramCommand_t command;

    UtFmc_Init_Default();
    UtFmc_Get_SdramConfig( &config );
    (void)memset( &command, 0, sizeof( command ) );
    command.Mode = FMC_SDRAM_COMMAND_PRECHARGE_ALL;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramCommand( FMC_BANK_SDRAM_1, &command ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_SdramConfig( FMC_BANK_SDRAM_1, &config ) );

    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_SdramCommand( FMC_BANK_SDRAM_1, &command ) );
    TEST_ASSERT_EQUAL_HEX32( ( ( UT_FMC_MODE_PALL << FMC_SDCMR_MODE_Pos ) | FMC_SDCMR_CTB1 ), FMC_Bank5_6_R->SDCMR );

    command.Mode          = FMC_SDRAM_COMMAND_LOAD_MODE_REGISTER;
    command.ModeRegister  = UT_FMC_MODE_REG_RAW;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_SdramCommand( FMC_BANK_SDRAM_1, &command ) );
    TEST_ASSERT_EQUAL_HEX32( ( ( UT_FMC_MODE_LOAD_MODE << FMC_SDCMR_MODE_Pos ) |
                               ( UT_FMC_MODE_REG_RAW << FMC_SDCMR_MRD_Pos ) |
                               FMC_SDCMR_CTB1 ),
                             FMC_Bank5_6_R->SDCMR );

    command.Mode           = FMC_SDRAM_COMMAND_AUTO_REFRESH;
    command.AutoRefreshCnt = UT_FMC_REFRESH_CNT_RAW;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_SdramCommand( FMC_BANK_SDRAM_1, &command ) );
    TEST_ASSERT_EQUAL_HEX32( ( ( UT_FMC_MODE_AUTO_REFRESH << FMC_SDCMR_MODE_Pos ) |
                               ( UT_FMC_NRFS_RAW_FIELD << FMC_SDCMR_NRFS_Pos ) |
                               FMC_SDCMR_CTB1 ),
                             FMC_Bank5_6_R->SDCMR );

    FMC_Bank5_6_R->SDCMR = UT_FMC_SENTINEL;
    command.AutoRefreshCnt = 0u;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramCommand( FMC_BANK_SDRAM_1, &command ) );
    command.AutoRefreshCnt = UT_FMC_REFRESH_CNT_TOO_MANY;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramCommand( FMC_BANK_SDRAM_1, &command ) );
    command.Mode         = FMC_SDRAM_COMMAND_LOAD_MODE_REGISTER;
    command.ModeRegister = UT_FMC_MODE_REG_TOO_LARGE;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramCommand( FMC_BANK_SDRAM_1, &command ) );
    command.Mode = FMC_SDRAM_COMMAND_CNT;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramCommand( FMC_BANK_SDRAM_1, &command ) );
    command.Mode = FMC_SDRAM_COMMAND_NORMAL;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramCommand( FMC_BANK_SDRAM_2, &command ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_SdramCommand( FMC_BANK_SDRAM_1, FMC_NULL_PTR ) );
    TEST_ASSERT_EQUAL_HEX32( UT_FMC_SENTINEL, FMC_Bank5_6_R->SDCMR );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_SDRAM_MSG );
#endif /* FMC_Bank5_6_R */
}


/**
 * \brief   Fmc_Get_SdramState() reports the busy flag and the mode status of the bank without a side effect.
 *
 * \par Expected results
 * - BUSY set: FMC_SDRAM_BUSY. Mode status 0, 1, 2 of bank 1: normal, self-refresh, power-down. The mode status of bank
 *   2
 *   is read from its own field.
 * - A mode status of 3, a bank that is not configured, a null pointer and an uninitialized module: FMC_REQUEST_ERROR.
 */
void Ut_Fmc_Get_SdramState_StatusRegister_MapsStates( void )
{
#if defined(FMC_Bank5_6_R)
    fmc_SdramConfig_t config;
    fmc_SdramState_t  state = FMC_SDRAM_STATE_CNT;

    UtFmc_Setup_Mocks();
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Get_SdramState( FMC_BANK_SDRAM_1, &state ) );
    UtFmc_Init_Default();
    UtFmc_Get_SdramConfig( &config );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Get_SdramState( FMC_BANK_SDRAM_1, &state ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_SdramConfig( FMC_BANK_SDRAM_1, &config ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_SdramConfig( FMC_BANK_SDRAM_2, &config ) );

    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_SdramState( FMC_BANK_SDRAM_1, &state ) );
    TEST_ASSERT_EQUAL( FMC_SDRAM_NORMAL, state );
    FMC_Bank5_6_R->SDSR = ( UT_FMC_MODES_SELF << FMC_SDSR_MODES1_Pos );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_SdramState( FMC_BANK_SDRAM_1, &state ) );
    TEST_ASSERT_EQUAL( FMC_SDRAM_SELF_REFRESH, state );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_SdramState( FMC_BANK_SDRAM_2, &state ) );
    TEST_ASSERT_EQUAL( FMC_SDRAM_NORMAL, state );
    FMC_Bank5_6_R->SDSR = ( UT_FMC_MODES_POWER_DOWN << FMC_SDSR_MODES2_Pos );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_SdramState( FMC_BANK_SDRAM_2, &state ) );
    TEST_ASSERT_EQUAL( FMC_SDRAM_POWER_DOWN, state );
    FMC_Bank5_6_R->SDSR = FMC_REG_SDSR_BUSY;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_SdramState( FMC_BANK_SDRAM_1, &state ) );
    TEST_ASSERT_EQUAL( FMC_SDRAM_BUSY, state );

    FMC_Bank5_6_R->SDSR = ( UT_FMC_MODES_RESERVED << FMC_SDSR_MODES1_Pos );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Get_SdramState( FMC_BANK_SDRAM_1, &state ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Get_SdramState( FMC_BANK_SDRAM_1, FMC_NULL_PTR ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_SDRAM_MSG );
#endif /* FMC_Bank5_6_R */
}


/**
 * \brief   Fmc_Set_BankWriteProtection() sets and clears the WP bit of an SDRAM bank with the controller disabled and
 *          enables the controller again.
 *
 * \par Expected results
 * - Protection active: WP of SDCR1 set; inactive: cleared; FMCEN is set after both calls.
 */
void Ut_Fmc_Set_BankWriteProtection_SdramBank_TogglesWp( void )
{
#if defined(FMC_Bank5_6_R)
    fmc_SdramConfig_t config;

    UtFmc_Init_Default();
    UtFmc_Get_SdramConfig( &config );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_SdramConfig( FMC_BANK_SDRAM_1, &config ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_BankWriteProtection( FMC_BANK_SDRAM_1, FMC_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL_HEX32( FMC_SDCRx_WP, ( FMC_Bank5_6_R->SDCR[0] & FMC_SDCRx_WP ) );
    TEST_ASSERT_EQUAL_HEX32( FMC_BCR1_FMCEN, ( FMC_Bank1_R->BTCR[0] & FMC_BCR1_FMCEN ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_BankWriteProtection( FMC_BANK_SDRAM_1, FMC_FUNCTION_INACTIVE ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, ( FMC_Bank5_6_R->SDCR[0] & FMC_SDCRx_WP ) );
    TEST_ASSERT_EQUAL_HEX32( FMC_BCR1_FMCEN, ( FMC_Bank1_R->BTCR[0] & FMC_BCR1_FMCEN ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_SDRAM_MSG );
#endif /* FMC_Bank5_6_R */
}


/* ============================ NAND REGISTER VALUES ========================= */

/**
 * \brief   FmcPriv_Get_NandRegs() converts the NAND configuration of the tests: the times of both spaces, the delay
 *          of ALE and the width.
 *
 * \par Expected results
 * - PMEM holds MEMSET 1, MEMWAIT 5, MEMHOLD 4, MEMHIZ 2; PATT holds ATTSET 0, ATTWAIT 1, ATTHOLD 1, ATTHIZ 0.
 * - PCR holds the NAND type, the 16-bit width, the 512 bytes page size and TAR 6 (8 cycles - ( smaller setup 0 + 2 )),
 *   TCLR 0, no wait feature and no ECC enable; SR holds no detection enable bit.
 */
void Ut_Fmc_Get_NandRegs_Example_MapsTimes( void )
{
#if defined(FMC_Bank1_R)
    fmc_NandConfig_t config;
    fmc_NandRegs_t   regs;

    UtFmc_Get_NandConfig( &config );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, FmcPriv_Get_NandRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );
    TEST_ASSERT_EQUAL_HEX32( UT_FMC_NAND_SPACE( UT_FMC_NAND_C_SETUP_FIELD,
                                                UT_FMC_NAND_C_WAIT_FIELD,
                                                UT_FMC_NAND_C_HOLD_FIELD,
                                                UT_FMC_NAND_C_HIZ_FIELD ),
                             regs.Pmem );
    TEST_ASSERT_EQUAL_HEX32( UT_FMC_NAND_SPACE( UT_FMC_NAND_A_SETUP_FIELD,
                                                UT_FMC_NAND_A_WAIT_FIELD,
                                                UT_FMC_NAND_A_HOLD_FIELD,
                                                UT_FMC_NAND_A_HIZ_FIELD ),
                             regs.Patt );
    TEST_ASSERT_EQUAL_HEX32( ( ( (uint32_t)FMC_ECC_PAGE_SIZE_512 << FMC_PCR_ECCPS_Pos ) |
                               ( UT_FMC_NAND_TAR_FIELD << FMC_PCR_TAR_Pos ) |
                               ( UT_FMC_NAND_PWID_16 << FMC_PCR_PWID_Pos ) |
                               FMC_PCR_PTYP ),
                             regs.Pcr );
    TEST_ASSERT_EQUAL_HEX32( 0u, regs.Sr );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   FmcPriv_Get_NandRegs() maps the 8-bit width, the wait feature, the three ready events, the CLE delay and a
 *          delay that the setup time already covers.
 *
 * \par Expected results
 * - 8-bit width: PWID 0; wait feature: PWAITEN; the events: IREN, IFEN and ILEN of SR.
 * - CLE delay 40 ns: TCLR 6; ALE delay 5 ns: TAR 0 (covered by the setup time).
 */
void Ut_Fmc_Get_NandRegs_OptionsAndDelays_MapsBits( void )
{
#if defined(FMC_Bank1_R)
    fmc_NandConfig_t config;
    fmc_NandRegs_t   regs;

    UtFmc_Get_NandConfig( &config );
    config.DataWidth       = FMC_DATA_WIDTH_8;
    config.WaitFeature     = FMC_FUNCTION_ACTIVE;
    config.ReadyRisingIrq  = FMC_FUNCTION_ACTIVE;
    config.ReadyFallingIrq = FMC_FUNCTION_ACTIVE;
    config.ReadyHighIrq    = FMC_FUNCTION_ACTIVE;
    config.CleToReDelay_ns = UT_FMC_NAND_ALE_NS;
    config.AleToReDelay_ns = UT_FMC_ONE_CYCLE_NS;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, FmcPriv_Get_NandRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );
    TEST_ASSERT_EQUAL_HEX32( ( ( (uint32_t)FMC_ECC_PAGE_SIZE_512 << FMC_PCR_ECCPS_Pos ) |
                               ( UT_FMC_NAND_TAR_FIELD << FMC_PCR_TCLR_Pos ) |
                               FMC_PCR_PWAITEN |
                               FMC_PCR_PTYP ),
                             regs.Pcr );
    TEST_ASSERT_EQUAL_HEX32( ( FMC_SR_IREN | FMC_SR_IFEN | FMC_SR_ILEN ), regs.Sr );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   FmcPriv_Get_NandRegs() rejects a null pointer, a zero clock, an item out of its list and a time or a delay
 *          that needs more cycles than its field holds.
 *
 * \par Expected results
 * - Every case returns FMC_REQUEST_ERROR.
 * - The largest setup time (255 cycles) is accepted, one cycle more is rejected.
 */
void Ut_Fmc_Get_NandRegs_InvalidInput_ReturnsError( void )
{
#if defined(FMC_Bank1_R)
    fmc_NandConfig_t config;
    fmc_NandRegs_t   regs;

    UtFmc_Get_NandConfig( &config );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NandRegs( FMC_NULL_PTR, UT_FMC_HCLK_200MHZ, &regs ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NandRegs( &config, UT_FMC_HCLK_200MHZ, FMC_NULL_PTR ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NandRegs( &config, 0u, &regs ) );

    config.DataWidth = (fmc_DataWidth_t)UT_FMC_WIDTH_INVALID;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NandRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );
    UtFmc_Get_NandConfig( &config );
    config.EccPageSize = FMC_ECC_PAGE_SIZE_CNT;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NandRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );
    UtFmc_Get_NandConfig( &config );
    config.ReadyHighIrq = (fmc_FunctionState_t)UT_FMC_STATE_INVALID;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NandRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );

    UtFmc_Get_NandConfig( &config );
    config.CommonTiming.Setup_ns = UT_FMC_NAND_SETUP_TOO_LONG_NS;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NandRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );
    config.CommonTiming.Setup_ns = ( UT_FMC_NAND_SETUP_TOO_LONG_NS - UT_FMC_ONE_CYCLE_NS );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, FmcPriv_Get_NandRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );

    UtFmc_Get_NandConfig( &config );
    config.AttributeTiming.Hold_ns = UT_FMC_NAND_HOLD_TOO_LONG_NS;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NandRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );

    UtFmc_Get_NandConfig( &config );
    config.AleToReDelay_ns = UT_FMC_NAND_ALE_TOO_LONG_NS;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NandRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );
    UtFmc_Get_NandConfig( &config );
    config.CleToReDelay_ns = UT_FMC_NAND_ALE_TOO_LONG_NS;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, FmcPriv_Get_NandRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}

/* ============================== NAND BANK ================================== */

/**
 * \brief   Fmc_Set_NandConfig() writes PMEM, PATT, PCR and SR with the controller kept enabled afterwards and leaves
 *          the bank disabled.
 *
 * \par Expected results
 * - The registers hold the converted values (see FmcPriv_Get_NandRegs), PBKEN and ECCEN are clear, FMCEN is set.
 * - The same call is accepted again while the bank is inactive.
 */
void Ut_Fmc_Set_NandConfig_Valid_WritesRegistersBankStaysDisabled( void )
{
#if defined(FMC_Bank1_R)
    fmc_NandConfig_t config;
    fmc_NandRegs_t   regs;

    UtFmc_Init_Default();
    UtFmc_Get_NandConfig( &config );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, FmcPriv_Get_NandRegs( &config, UT_FMC_HCLK_200MHZ, &regs ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_NandConfig( &config ) );
    TEST_ASSERT_EQUAL_HEX32( regs.Pmem, FMC_Bank3_R->PMEM );
    TEST_ASSERT_EQUAL_HEX32( regs.Patt, FMC_Bank3_R->PATT );
    TEST_ASSERT_EQUAL_HEX32( regs.Pcr, FMC_Bank3_R->PCR );
    TEST_ASSERT_EQUAL_HEX32( 0u, ( FMC_Bank3_R->PCR & ( FMC_PCR_PBKEN | FMC_PCR_ECCEN ) ) );
    TEST_ASSERT_EQUAL_HEX32( FMC_BCR1_FMCEN, ( FMC_Bank1_R->BTCR[0] & FMC_BCR1_FMCEN ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_NandConfig( &config ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   Fmc_Set_NandConfig() writes the ready event enable bits to SR.
 *
 * \par Expected results
 * - With the three events enabled, SR holds IREN, IFEN and ILEN.
 */
void Ut_Fmc_Set_NandConfig_ReadyEvents_WritesEnableBits( void )
{
#if defined(FMC_Bank1_R)
    fmc_NandConfig_t config;

    UtFmc_Init_Default();
    UtFmc_Get_NandConfig( &config );
    config.ReadyRisingIrq  = FMC_FUNCTION_ACTIVE;
    config.ReadyFallingIrq = FMC_FUNCTION_ACTIVE;
    config.ReadyHighIrq    = FMC_FUNCTION_ACTIVE;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_NandConfig( &config ) );
    TEST_ASSERT_EQUAL_HEX32( ( FMC_SR_IREN | FMC_SR_IFEN | FMC_SR_ILEN ),
                             ( FMC_Bank3_R->SR & ( FMC_SR_IREN | FMC_SR_IFEN | FMC_SR_ILEN ) ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   Fmc_Set_NandConfig() rejects a null pointer, an uninitialized module, an invalid configuration, a time the
 *          clock can not meet and an active bank, without any register write.
 *
 * \par Expected results
 * - Every case returns FMC_REQUEST_ERROR and PMEM, PATT and PCR keep their reset value (0).
 */
void Ut_Fmc_Set_NandConfig_InvalidParameterOrState_ReturnsErrorNoWrite( void )
{
#if defined(FMC_Bank1_R)
    fmc_NandConfig_t config;

    UtFmc_Setup_Mocks();
    UtFmc_Get_NandConfig( &config );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_NandConfig( &config ) );

    UtFmc_Init_Default();
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_NandConfig( FMC_NULL_PTR ) );
    config.DataWidth = (fmc_DataWidth_t)UT_FMC_WIDTH_INVALID;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_NandConfig( &config ) );
    UtFmc_Get_NandConfig( &config );
    config.CommonTiming.Setup_ns = UT_FMC_NAND_SETUP_TOO_LONG_NS;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_NandConfig( &config ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, FMC_Bank3_R->PMEM );
    TEST_ASSERT_EQUAL_HEX32( 0u, FMC_Bank3_R->PATT );
    TEST_ASSERT_EQUAL_HEX32( 0u, FMC_Bank3_R->PCR );

    UtFmc_Get_NandConfig( &config );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_NandConfig( &config ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_BankActive( FMC_BANK_NAND ) );
    FMC_Bank3_R->PMEM = 0u;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_NandConfig( &config ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, FMC_Bank3_R->PMEM );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}

/* ================================== NAND ECC =============================== */

/**
 * \brief   Fmc_Set_NandEccActive() and Fmc_Get_NandEcc() start the ECC calculation, wait for the empty write FIFO,
 *          read the result limited to the valid bits of the page size and stop the calculation.
 *
 * \par Expected results
 * - After Fmc_Set_NandEccActive() ECCEN is set; Fmc_Get_NandEcc() returns the result with the 24 valid bits of 512
 *   bytes pages and clears ECCEN.
 * - A second read without a new start returns FMC_REQUEST_ERROR.
 * - A not empty write FIFO: FMC_REQUEST_ERROR and ECCEN stays set.
 */
void Ut_Fmc_NandEcc_StartRead_ResultMaskedAndStopped( void )
{
#if defined(FMC_Bank1_R)
    fmc_NandConfig_t config;
    fmc_Ecc_t        ecc = 0u;

    UtFmc_Init_Default();
    UtFmc_Get_NandConfig( &config );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_NandConfig( &config ) );

    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Get_NandEcc( &ecc ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_NandEccActive() );
    TEST_ASSERT_EQUAL_HEX32( FMC_PCR_ECCEN, ( FMC_Bank3_R->PCR & FMC_PCR_ECCEN ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Get_NandEcc( FMC_NULL_PTR ) );

    FMC_Bank3_R->SR   = 0u;
    FMC_Bank3_R->ECCR = UT_FMC_ECC_ALL_SET;
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Get_NandEcc( &ecc ) );
    TEST_ASSERT_EQUAL_HEX32( FMC_PCR_ECCEN, ( FMC_Bank3_R->PCR & FMC_PCR_ECCEN ) );

    FMC_Bank3_R->SR = FMC_SR_FEMPT;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_NandEcc( &ecc ) );
    TEST_ASSERT_EQUAL_HEX32( UT_FMC_ECC_MASK_512, ecc );
    TEST_ASSERT_EQUAL_HEX32( 0u, ( FMC_Bank3_R->PCR & FMC_PCR_ECCEN ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Get_NandEcc( &ecc ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   Fmc_Get_NandEcc() limits the result to the valid bits of the configured page size.
 *
 * \par Expected results
 * - 256 bytes: 22 valid bits; 8192 bytes: all 32 bits.
 */
void Ut_Fmc_NandEcc_PageSize_MasksValidBits( void )
{
#if defined(FMC_Bank1_R)
    fmc_NandConfig_t config;
    fmc_Ecc_t        ecc = 0u;

    UtFmc_Init_Default();
    UtFmc_Get_NandConfig( &config );
    config.EccPageSize = FMC_ECC_PAGE_SIZE_256;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_NandConfig( &config ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_NandEccActive() );
    FMC_Bank3_R->SR   = FMC_SR_FEMPT;
    FMC_Bank3_R->ECCR = UT_FMC_ECC_ALL_SET;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_NandEcc( &ecc ) );
    TEST_ASSERT_EQUAL_HEX32( UT_FMC_ECC_MASK_256, ecc );

    config.EccPageSize = FMC_ECC_PAGE_SIZE_8192;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_NandConfig( &config ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_NandEccActive() );
    FMC_Bank3_R->SR   = FMC_SR_FEMPT;
    FMC_Bank3_R->ECCR = UT_FMC_ECC_ALL_SET;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_NandEcc( &ecc ) );
    TEST_ASSERT_EQUAL_HEX32( UT_FMC_ECC_MASK_8192, ecc );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   The ECC functions return an error before the initialization, with an unconfigured NAND bank and with the
 *          ECC disabled in the configuration.
 *
 * \par Expected results
 * - Every case returns FMC_REQUEST_ERROR and ECCEN stays clear.
 */
void Ut_Fmc_NandEcc_NotAvailable_ReturnsError( void )
{
#if defined(FMC_Bank1_R)
    fmc_NandConfig_t config;
    fmc_Ecc_t        ecc = 0u;

    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_NandEccActive() );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Get_NandEcc( &ecc ) );
    UtFmc_Init_Default();
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_NandEccActive() );

    UtFmc_Get_NandConfig( &config );
    config.EccEnable = FMC_FUNCTION_INACTIVE;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_NandConfig( &config ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_NandEccActive() );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Get_NandEcc( &ecc ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, ( FMC_Bank3_R->PCR & FMC_PCR_ECCEN ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}

/* ============================ NAND READY SIGNAL ============================ */

/**
 * \brief   Fmc_Get_NandReadyState() reports the ready level through the high level flag and restores the detection.
 *
 * \par Expected results
 * - The ready model sets the flag while the detection is on: FMC_NAND_READY, ILEN is clear again (the model sets the
 *   flag after every access, so ILS is not checked).
 * - Without the model the flag stays clear: FMC_NAND_BUSY.
 * - With the detection enabled by the configuration ILEN stays set.
 * - A null pointer, an uninitialized module and an unconfigured bank: FMC_REQUEST_ERROR.
 */
void Ut_Fmc_Get_NandReadyState_Signal_ReadyOrBusy( void )
{
#if defined(FMC_Bank1_R)
    fmc_NandConfig_t     config;
    fmc_NandReadyState_t readyState = FMC_NAND_STATE_CNT;

    UtFmc_Setup_Mocks();
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Get_NandReadyState( &readyState ) );
    UtFmc_Init_Default();
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Get_NandReadyState( &readyState ) );
    UtFmc_Get_NandConfig( &config );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_NandConfig( &config ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Get_NandReadyState( FMC_NULL_PTR ) );

    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_NandReadyState( &readyState ) );
    TEST_ASSERT_EQUAL( FMC_NAND_BUSY, readyState );
    TEST_ASSERT_EQUAL_HEX32( 0u, ( FMC_Bank3_R->SR & ( FMC_SR_ILEN | FMC_SR_ILS ) ) );

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtFmc_NandReadyModel ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_NandReadyState( &readyState ) );
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelInactive() );
    TEST_ASSERT_EQUAL( FMC_NAND_READY, readyState );
    TEST_ASSERT_EQUAL_HEX32( 0u, ( FMC_Bank3_R->SR & FMC_SR_ILEN ) );

    config.ReadyHighIrq = FMC_FUNCTION_ACTIVE;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_NandConfig( &config ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_NandReadyState( &readyState ) );
    TEST_ASSERT_EQUAL( FMC_NAND_BUSY, readyState );
    TEST_ASSERT_EQUAL_HEX32( FMC_SR_ILEN, ( FMC_Bank3_R->SR & ( FMC_SR_ILEN | FMC_SR_ILS ) ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}

/* ============================ NAND READY EVENTS ============================ */

/**
 * \brief   Fmc_Task() reports every enabled and set ready event by the callback and clears its flag; an event that
 *          is set but not enabled is not reported.
 *
 * \par Expected results
 * - Rising edge and high level enabled, all three flags set: two callbacks, the last event is the high level, the
 *   rising edge and the high level flags are cleared, the falling edge flag stays set.
 * - No flag set: no callback.
 */
void Ut_Fmc_Task_NandEvents_ReportedAndCleared( void )
{
#if defined(FMC_Bank1_R)
    fmc_Config_t     fmcConfig;
    fmc_NandConfig_t config;

    UtFmc_Setup_Mocks();
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_DefaultConfig( &fmcConfig ) );
    fmcConfig.NandReadyCallback = UtFmc_NandReadyCb;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Init( &fmcConfig ) );
    UtFmc_Get_NandConfig( &config );
    config.ReadyRisingIrq = FMC_FUNCTION_ACTIVE;
    config.ReadyHighIrq   = FMC_FUNCTION_ACTIVE;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_NandConfig( &config ) );

    Fmc_Task();
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmc_NandEventCnt );

    FMC_Bank3_R->SR |= ( FMC_SR_IRS | FMC_SR_IFS | FMC_SR_ILS );
    Fmc_Task();
    TEST_ASSERT_EQUAL_UINT32( UT_FMC_NAND_EVENT_ENABLED_CNT, utFmc_NandEventCnt );
    TEST_ASSERT_EQUAL( FMC_NAND_EVENT_HIGH_LEVEL, utFmc_NandEvent );
    TEST_ASSERT_EQUAL_HEX32( FMC_SR_IFS, ( FMC_Bank3_R->SR & ( FMC_SR_IRS | FMC_SR_IFS | FMC_SR_ILS ) ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief   The events of the NAND ready signal are served without a callback (only cleared) and not by Fmc_Task()
 *          while the interrupt serves them.
 *
 * \par Expected results
 * - No callback configured: the flag is cleared and nothing else happens.
 * - ISR mode: Fmc_Task() leaves the flag set, the interrupt handler of the module clears it and calls the callback.
 */
void Ut_Fmc_NandEvents_NoCallbackOrIsrMode_ServedAccordingly( void )
{
#if defined(FMC_Bank1_R)
    fmc_Config_t     fmcConfig;
    fmc_NandConfig_t config;

    UtFmc_Setup_Mocks();
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_DefaultConfig( &fmcConfig ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Init( &fmcConfig ) );
    UtFmc_Get_NandConfig( &config );
    config.ReadyFallingIrq = FMC_FUNCTION_ACTIVE;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_NandConfig( &config ) );
    FMC_Bank3_R->SR |= FMC_SR_IFS;
    Fmc_Task();
    TEST_ASSERT_EQUAL_HEX32( 0u, ( FMC_Bank3_R->SR & FMC_SR_IFS ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmc_NandEventCnt );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Deinit() );

    fmcConfig.EventMode         = FMC_EVENT_MODE_ISR;
    fmcConfig.NandReadyCallback = UtFmc_NandReadyCb;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Init( &fmcConfig ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_NandConfig( &config ) );
    TEST_ASSERT_NOT_NULL( (void *)utFmc_Isr );
    FMC_Bank3_R->SR |= FMC_SR_IFS;
    Fmc_Task();
    TEST_ASSERT_EQUAL_UINT32( 0u, utFmc_NandEventCnt );
    TEST_ASSERT_EQUAL_HEX32( FMC_SR_IFS, ( FMC_Bank3_R->SR & FMC_SR_IFS ) );
    utFmc_Isr();
    TEST_ASSERT_EQUAL_UINT32( 1u, utFmc_NandEventCnt );
    TEST_ASSERT_EQUAL( FMC_NAND_EVENT_FALLING_EDGE, utFmc_NandEvent );
    TEST_ASSERT_EQUAL_HEX32( 0u, ( FMC_Bank3_R->SR & FMC_SR_IFS ) );
#else
    TEST_PASS_MESSAGE( UT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}
