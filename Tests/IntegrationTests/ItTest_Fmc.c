/**
 * \author Mr.Nobody
 * \file ItTest_Fmc.c
 * \ingroup Fmc
 * \brief Integration tests of the Fmc module on target.
 *
 * The Fmc module runs on the MCU together with the real FMC / GPIO / RCC / NVIC hardware. A bare board carries no
 * external memory, so the tests without a fixture prove what the controller does alone: the module initializes and
 * deinitializes, a NOR / PSRAM / SRAM bank and the NAND bank are configured, enabled and disabled without a fault
 * (no access to the window is made), the window base addresses are reported and the NAND ECC calculation is
 * started and read with an empty write FIFO.
 *
 * Tests that need an external part check a fixture feature: `#if defined(IT_FEATURE_<NAME>) || !defined(IT_FIXTURE)`
 * runs the test, otherwise the test is ignored with a message naming the missing fixture.
 * - IT_FEATURE_FMC_SRAM  : a 16-bit asynchronous SRAM (at least 128 bytes) on the bank 1 (NE1, NOE, NWE, NBL0, NBL1,
 *                          A0 = PF0 .. A5 = PF5, D0 .. D15 = PD14, PD15, PD0, PD1, PE7 .. PE15, PD8 .. PD10).
 *                          NBL1 is PE1, on STM32H523 / H533 / H543 / H553 (no NBL1 on PE1) PA15.
 * - IT_FEATURE_FMC_SDRAM : a 16-bit SDRAM (9 column bits, 13 row bits, 4 internal banks) on the SDRAM bank 1; the
 *                          SDRAM pins of the board are configured by the board startup.
 * - IT_FEATURE_FMC_NAND  : a NAND flash on the NAND bank with its ready / busy signal on the FMC_INT / NWAIT pin.
 *
 * Used resources:
 * - FMC (kernel clock = bus clock)
 * - GPIO PD0, PD1, PD4, PD5, PD7 .. PD10, PD14, PD15, PE0, PE1 (PA15 on STM32H523 / H533 / H543 / H553), PE7 .. PE15,
 *   PF0 .. PF5 (SRAM fixture only)
 *
 * \note On STM32H503 (no FMC) every test is ignored (`TEST_IGNORE_MESSAGE`), on STM32H523 / H533 (no SDRAM
 *       controller) the SDRAM test is ignored, see the module README availability table.
 * \note Compiled for the target but not run on hardware, see the module report.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "Fmc_Port.h"                       /* Module under test              */
#include "Fmc_Pins.h"                       /* Pin items                      */
#include <string.h>                         /* memset                         */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

#if defined(FMC_Bank1_R)
static void ItFmc_Get_SramConfig( fmc_NorSramConfig_t * const config );
static void ItFmc_Get_NandConfig( fmc_NandConfig_t * const config );
static void ItFmc_Get_SramPins ( fmc_Config_t * const fmcConfig );
#endif /* FMC_Bank1_R */

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Message of a test on a line without the FMC */
#define IT_FMC_NO_FMC_MSG                     "No FMC on this line (STM32H503)."

/** Message of a test on a line without the SDRAM controller */
#define IT_FMC_NO_SDRAM_MSG                   "No SDRAM controller on this line (STM32H523 / STM32H533)."

/** Message of a test that needs the SRAM fixture */
#define IT_FMC_NO_SRAM_MSG                    "Board without the fixture FMC_SRAM (16-bit SRAM on the bank 1)."

/** Message of a test that needs the SDRAM fixture */
#define IT_FMC_NO_SDRAM_FIXTURE_MSG           "Board without the fixture FMC_SDRAM (16-bit SDRAM on the SDRAM bank 1)."

/** Message of a test that needs the NAND fixture */
#define IT_FMC_NO_NAND_MSG                    "Board without the fixture FMC_NAND (NAND flash with its ready signal)."

/** Interrupt priority used by the tests */
#define IT_FMC_IRQ_PRIO                       ( 5u )

/** Address setup time of the SRAM (ns) */
#define IT_FMC_SRAM_ADDSET_NS                 ( 20u )

/** Data setup time of the SRAM (ns) */
#define IT_FMC_SRAM_DATAST_NS                 ( 40u )

/** Common space setup time of the NAND (ns) */
#define IT_FMC_NAND_SETUP_NS                  ( 20u )

/** Common space wait time of the NAND (ns) */
#define IT_FMC_NAND_WAIT_NS                   ( 50u )

/** Common space hold time of the NAND (ns) */
#define IT_FMC_NAND_HOLD_NS                   ( 20u )

/** Common space high-Z time of the NAND (ns) */
#define IT_FMC_NAND_HIZ_NS                    ( 20u )

/** Delay of ALE and CLE to the read strobe of the NAND (ns) */
#define IT_FMC_NAND_DELAY_NS                  ( 40u )

/** Number of the 16-bit words of the SRAM pattern test */
#define IT_FMC_SRAM_WORD_CNT                  ( 64u )

/** Start value of the SRAM pattern */
#define IT_FMC_SRAM_PATTERN_START             ( 0xA5A5u )

/** Step of the SRAM pattern */
#define IT_FMC_SRAM_PATTERN_STEP              ( 0x0101u )

/** Window address of the SRAM bank 1 (RM0481 table of the FMC address map) */
#define IT_FMC_SRAM_BANK1_ADDRESS             ( 0x60000000u )

/** Window address of the SDRAM bank 1 */
#define IT_FMC_SDRAM_BANK1_ADDRESS            ( 0xC0000000u )

/** Window address of the NAND bank */
#define IT_FMC_NAND_BANK_ADDRESS              ( 0x80000000u )

/** Bits of the ECC result above the 24 valid bits of 512 bytes pages */
#define IT_FMC_ECC_INVALID_MASK               ( 0xFF000000u )

/** Refresh period of the SDRAM (ms) */
#define IT_FMC_SDRAM_PERIOD_MS                ( 64u )

/** Rows refreshed in the refresh period */
#define IT_FMC_SDRAM_ROWS                     ( 8192u )

/** Time of the SDRAM timing parameters (ns) */
#define IT_FMC_SDRAM_TIME_NS                  ( 20u )

/** Time of the SDRAM row cycle (ns) */
#define IT_FMC_SDRAM_ROW_CYCLE_NS             ( 60u )

/** Time of the SDRAM self-refresh (ns) */
#define IT_FMC_SDRAM_SELF_REFRESH_NS          ( 42u )

/** Number of the address pins of the SRAM fixture */
#define IT_FMC_SRAM_ADDRESS_PIN_CNT           ( 6u )

/** Number of the data pins of the SRAM fixture */
#define IT_FMC_SRAM_DATA_PIN_CNT              ( 16u )

/** Number of the control pins of the SRAM fixture */
#define IT_FMC_SRAM_CONTROL_PIN_CNT           ( 5u )

/* ========================== LOCAL VARIABLES =============================== */

#if defined(FMC_Bank1_R)

/** Address pins of the SRAM fixture (A0 .. A5) */
static const fmc_AddressPin_t itFmc_SramAddressPin[ IT_FMC_SRAM_ADDRESS_PIN_CNT ] =
{
    FMC_A0_PIN_PF0,
    FMC_A1_PIN_PF1,
    FMC_A2_PIN_PF2,
    FMC_A3_PIN_PF3,
    FMC_A4_PIN_PF4,
    FMC_A5_PIN_PF5,
};

/** Data pins of the SRAM fixture (D0 .. D15) */
static const fmc_DataPin_t itFmc_SramDataPin[ IT_FMC_SRAM_DATA_PIN_CNT ] =
{
    FMC_D0_PIN_PD14,
    FMC_D1_PIN_PD15,
    FMC_D2_PIN_PD0,
    FMC_D3_PIN_PD1,
    FMC_D4_PIN_PE7,
    FMC_D5_PIN_PE8,
    FMC_D6_PIN_PE9,
    FMC_D7_PIN_PE10,
    FMC_D8_PIN_PE11,
    FMC_D9_PIN_PE12,
    FMC_D10_PIN_PE13,
    FMC_D11_PIN_PE14,
    FMC_D12_PIN_PE15,
    FMC_D13_PIN_PD8,
    FMC_D14_PIN_PD9,
    FMC_D15_PIN_PD10,
};

/** Control pins of the SRAM fixture (NE1, NOE, NWE, NBL0, NBL1) */
static const fmc_ControlPin_t itFmc_SramControlPin[ IT_FMC_SRAM_CONTROL_PIN_CNT ] =
{
    FMC_NE1_PIN_PD7,
    FMC_NOE_PIN_PD4,
    FMC_NWE_PIN_PD5,
    FMC_NBL0_PIN_PE0,
#if defined(STM32H523xx) || \
    defined(STM32H533xx) || \
    defined(STM32H543xx) || \
    defined(STM32H553xx)
    FMC_NBL1_PIN_PA15,
#else
    FMC_NBL1_PIN_PE1,
#endif
};

#endif /* FMC_Bank1_R */

/* ============================= TEST SETUP ================================= */

void setUp( void )
{
    /* No per-test state */
}


void tearDown( void )
{
    /* Peripheral state is cleared by system reset after the test case */
}

/* ========================== LOCAL FUNCTIONS ================================ */

#if defined(FMC_Bank1_R)

/**
 * \brief Fills the configuration of a 16-bit asynchronous SRAM.
 *
 * \param config [out]: Pointer to the configuration filled in place.
 */
static void ItFmc_Get_SramConfig( fmc_NorSramConfig_t * const config )
{
    (void)memset( config, 0, sizeof( fmc_NorSramConfig_t ) );
    config->MemoryType      = FMC_MEMORY_SRAM;
    config->DataWidth       = FMC_DATA_WIDTH_16;
    config->AddressSetup_ns = IT_FMC_SRAM_ADDSET_NS;
    config->DataSetup_ns    = IT_FMC_SRAM_DATAST_NS;
}


/**
 * \brief Fills the configuration of a 16-bit NAND with the ECC of 512 bytes pages.
 *
 * \param config [out]: Pointer to the configuration filled in place.
 */
static void ItFmc_Get_NandConfig( fmc_NandConfig_t * const config )
{
    (void)memset( config, 0, sizeof( fmc_NandConfig_t ) );
    config->DataWidth             = FMC_DATA_WIDTH_16;
    config->EccEnable             = FMC_FUNCTION_ACTIVE;
    config->EccPageSize           = FMC_ECC_PAGE_SIZE_512;
    config->CommonTiming.Setup_ns = IT_FMC_NAND_SETUP_NS;
    config->CommonTiming.Wait_ns  = IT_FMC_NAND_WAIT_NS;
    config->CommonTiming.Hold_ns  = IT_FMC_NAND_HOLD_NS;
    config->CommonTiming.HiZ_ns   = IT_FMC_NAND_HIZ_NS;
    config->AttributeTiming       = config->CommonTiming;
    config->AleToReDelay_ns       = IT_FMC_NAND_DELAY_NS;
    config->CleToReDelay_ns       = IT_FMC_NAND_DELAY_NS;
}


/**
 * \brief Sets the pins of the 16-bit SRAM fixture in the module configuration.
 *
 * \param fmcConfig [in,out]: Pointer to the module configuration (default configuration, pins unused).
 */
static void ItFmc_Get_SramPins( fmc_Config_t * const fmcConfig )
{
    for( uint32_t idx = 0u; ( IT_FMC_SRAM_ADDRESS_PIN_CNT > idx ); idx++ )
    {
        fmcConfig->AddressPin[idx] = itFmc_SramAddressPin[idx];
    }
    for( uint32_t idx = 0u; ( IT_FMC_SRAM_DATA_PIN_CNT > idx ); idx++ )
    {
        fmcConfig->DataPin[idx] = itFmc_SramDataPin[idx];
    }
    for( uint32_t idx = 0u; ( IT_FMC_SRAM_CONTROL_PIN_CNT > idx ); idx++ )
    {
        fmcConfig->ControlPin[idx] = itFmc_SramControlPin[idx];
    }
}

#endif /* FMC_Bank1_R */

/* =============================== TESTS ==================================== */

/**
 * \brief The module version is reported without a fault.
 * \par Expected results
 * \ref Fmc_Get_ModuleVersion returns major version 1.
 */
void It_Fmc_Get_ModuleVersion_ReturnsVersion( void )
{
    fmc_ModuleVersion_t version = Fmc_Get_ModuleVersion();

    TEST_ASSERT_EQUAL_UINT8( 1u, version.Major );
}


/**
 * \brief The module initializes with the default configuration (no bank, no pin) and deinitializes.
 * \par Expected results
 * \ref Fmc_Init returns \ref FMC_REQUEST_OK, a second init returns \ref FMC_REQUEST_ERROR, \ref Fmc_Deinit returns
 * \ref FMC_REQUEST_OK, a second deinit returns \ref FMC_REQUEST_ERROR.
 */
void It_Fmc_Init_DefaultConfig_InitDeinitOk( void )
{
#if defined(FMC_Bank1_R)
    fmc_Config_t config;

    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_DefaultConfig( &config ) );
    config.IrqPriority = IT_FMC_IRQ_PRIO;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Init( &config ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Init( &config ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Deinit() );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Deinit() );
#else
    TEST_IGNORE_MESSAGE( IT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief A SRAM bank is configured, enabled and disabled and its window address is reported; the window is not
 *        accessed.
 * \par Expected results
 * Configuration, enable and disable return \ref FMC_REQUEST_OK, the base address of the bank 1 is 0x60000000, a
 * second configuration of the enabled bank returns \ref FMC_REQUEST_ERROR.
 */
void It_Fmc_Set_NorSramConfig_Sram_EnableDisableWithoutAccess( void )
{
#if defined(FMC_Bank1_R)
    fmc_Config_t        config;
    fmc_NorSramConfig_t sram;
    fmc_Address_t       address = 0u;

    ItFmc_Get_SramConfig( &sram );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_DefaultConfig( &config ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Init( &config ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_NorSramConfig( FMC_BANK_NOR_PSRAM_1, &sram ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_BankActive( FMC_BANK_NOR_PSRAM_1 ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_NorSramConfig( FMC_BANK_NOR_PSRAM_1, &sram ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_BankBaseAddress( FMC_BANK_NOR_PSRAM_1, &address ) );
    TEST_ASSERT_EQUAL_HEX32( IT_FMC_SRAM_BANK1_ADDRESS, address );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_BankInactive( FMC_BANK_NOR_PSRAM_1 ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Deinit() );
#else
    TEST_IGNORE_MESSAGE( IT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief A time that the bus clock can not meet is rejected by the configuration without a fault.
 * \par Expected results
 * \ref Fmc_Set_NorSramConfig returns \ref FMC_REQUEST_ERROR for a data setup time above the field.
 */
void It_Fmc_Set_NorSramConfig_TimeTooLong_ReturnsError( void )
{
#if defined(FMC_Bank1_R)
    fmc_Config_t        config;
    fmc_NorSramConfig_t sram;

    ItFmc_Get_SramConfig( &sram );
    sram.DataSetup_ns = UINT32_MAX;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_DefaultConfig( &config ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Init( &config ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Set_NorSramConfig( FMC_BANK_NOR_PSRAM_1, &sram ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Deinit() );
#else
    TEST_IGNORE_MESSAGE( IT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief The NAND bank is configured and enabled and the ECC calculation is started and read without a memory (the
 *        write FIFO is empty, no access to the window is made).
 * \par Expected results
 * Configuration, enable, ECC start, ECC read and disable return \ref FMC_REQUEST_OK, the ECC is within its 24 valid
 * bits, a second read without a new start returns \ref FMC_REQUEST_ERROR.
 */
void It_Fmc_Nand_EccStartRead_EmptyFifoWithoutMemory( void )
{
#if defined(FMC_Bank1_R)
    fmc_Config_t     config;
    fmc_NandConfig_t nand;
    fmc_Ecc_t        ecc = 0u;

    ItFmc_Get_NandConfig( &nand );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_DefaultConfig( &config ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Init( &config ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_NandConfig( &nand ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_BankActive( FMC_BANK_NAND ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_NandEccActive() );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_NandEcc( &ecc ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, ( ecc & IT_FMC_ECC_INVALID_MASK ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_ERROR, Fmc_Get_NandEcc( &ecc ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_BankInactive( FMC_BANK_NAND ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Deinit() );
#else
    TEST_IGNORE_MESSAGE( IT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief A pattern is written to the SRAM and read back.
 * \par Expected results
 * Every word of the pattern of 64 half words reads back as written.
 */
void It_Fmc_Sram_PatternWriteRead_ReadsBack( void )
{
#if defined(FMC_Bank1_R)
#if defined(IT_FEATURE_FMC_SRAM) || \
    !defined(IT_FIXTURE)
    fmc_Config_t        config;
    fmc_NorSramConfig_t sram;
    fmc_Address_t       address = 0u;
    volatile uint16_t * window  = (volatile uint16_t *)IT_FMC_SRAM_BANK1_ADDRESS;

    ItFmc_Get_SramConfig( &sram );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_DefaultConfig( &config ) );
    ItFmc_Get_SramPins( &config );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Init( &config ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_NorSramConfig( FMC_BANK_NOR_PSRAM_1, &sram ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_BankActive( FMC_BANK_NOR_PSRAM_1 ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_BankBaseAddress( FMC_BANK_NOR_PSRAM_1, &address ) );
    window = (volatile uint16_t *)address;

    for( uint32_t idx = 0u; ( IT_FMC_SRAM_WORD_CNT > idx ); idx++ )
    {
        window[idx] = (uint16_t)( IT_FMC_SRAM_PATTERN_START + ( idx * IT_FMC_SRAM_PATTERN_STEP ) );
    }
    for( uint32_t idx = 0u; ( IT_FMC_SRAM_WORD_CNT > idx ); idx++ )
    {
        const uint16_t expected = (uint16_t)( IT_FMC_SRAM_PATTERN_START + ( idx * IT_FMC_SRAM_PATTERN_STEP ) );
        TEST_ASSERT_EQUAL_HEX16( expected, window[idx] );
    }
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Deinit() );
#else
    TEST_IGNORE_MESSAGE( IT_FMC_NO_SRAM_MSG );
#endif /* IT_FEATURE_FMC_SRAM */
#else
    TEST_IGNORE_MESSAGE( IT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}


/**
 * \brief The SDRAM bank is configured and initialized (clock, precharge, auto-refresh, mode register, refresh rate)
 *        and reports the normal state.
 * \par Expected results
 * Configuration and initialization return \ref FMC_REQUEST_OK, the state is \ref FMC_SDRAM_NORMAL and the base
 * address of the SDRAM bank 1 is 0xC0000000.
 */
void It_Fmc_Sdram_Init_ReportsNormalState( void )
{
#if defined(FMC_Bank5_6_R)
#if defined(IT_FEATURE_FMC_SDRAM) || \
    !defined(IT_FIXTURE)
    fmc_Config_t      config;
    fmc_SdramConfig_t sdram;
    fmc_SdramState_t  state   = FMC_SDRAM_STATE_CNT;
    fmc_Address_t     address = 0u;

    (void)memset( &sdram, 0, sizeof( sdram ) );
    sdram.ColumnBits              = FMC_SDRAM_COLUMN_BITS_9;
    sdram.RowBits                 = FMC_SDRAM_ROW_BITS_13;
    sdram.DataWidth               = FMC_DATA_WIDTH_16;
    sdram.InternalBanks           = FMC_SDRAM_INTERNAL_BANKS_4;
    sdram.CasLatency              = FMC_CAS_LATENCY_3;
    sdram.LoadToActiveDelay_ns    = IT_FMC_SDRAM_TIME_NS;
    sdram.ExitSelfRefreshDelay_ns = IT_FMC_SDRAM_ROW_CYCLE_NS;
    sdram.SelfRefreshTime_ns      = IT_FMC_SDRAM_SELF_REFRESH_NS;
    sdram.RowCycleDelay_ns        = IT_FMC_SDRAM_ROW_CYCLE_NS;
    sdram.WriteRecoveryTime_ns    = IT_FMC_SDRAM_TIME_NS;
    sdram.RpDelay_ns              = IT_FMC_SDRAM_TIME_NS;
    sdram.RcdDelay_ns             = IT_FMC_SDRAM_TIME_NS;
    sdram.RefreshPeriod_ms        = IT_FMC_SDRAM_PERIOD_MS;
    sdram.RefreshRows             = IT_FMC_SDRAM_ROWS;
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_DefaultConfig( &config ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Init( &config ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_SdramConfig( FMC_BANK_SDRAM_1, &sdram ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_SdramInit( FMC_BANK_SDRAM_1 ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_SdramState( FMC_BANK_SDRAM_1, &state ) );
    TEST_ASSERT_EQUAL( FMC_SDRAM_NORMAL, state );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_BankBaseAddress( FMC_BANK_SDRAM_1, &address ) );
    TEST_ASSERT_EQUAL_HEX32( IT_FMC_SDRAM_BANK1_ADDRESS, address );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Deinit() );
#else
    TEST_IGNORE_MESSAGE( IT_FMC_NO_SDRAM_FIXTURE_MSG );
#endif /* IT_FEATURE_FMC_SDRAM */
#else
    TEST_IGNORE_MESSAGE( IT_FMC_NO_SDRAM_MSG );
#endif /* FMC_Bank5_6_R */
}


/**
 * \brief The ready / busy signal of the NAND flash is read.
 * \par Expected results
 * \ref Fmc_Get_NandReadyState returns \ref FMC_REQUEST_OK and an idle NAND flash reports \ref FMC_NAND_READY; the
 * base address of the NAND bank is 0x80000000.
 */
void It_Fmc_Nand_ReadyState_IdleFlashIsReady( void )
{
#if defined(FMC_Bank1_R)
#if defined(IT_FEATURE_FMC_NAND) || \
    !defined(IT_FIXTURE)
    fmc_Config_t         config;
    fmc_NandConfig_t     nand;
    fmc_NandReadyState_t readyState = FMC_NAND_STATE_CNT;
    fmc_Address_t        address    = 0u;

    ItFmc_Get_NandConfig( &nand );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_DefaultConfig( &config ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Init( &config ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_NandConfig( &nand ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Set_BankActive( FMC_BANK_NAND ) );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_NandReadyState( &readyState ) );
    TEST_ASSERT_EQUAL( FMC_NAND_READY, readyState );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Get_BankBaseAddress( FMC_BANK_NAND, &address ) );
    TEST_ASSERT_EQUAL_HEX32( IT_FMC_NAND_BANK_ADDRESS, address );
    TEST_ASSERT_EQUAL( FMC_REQUEST_OK, Fmc_Deinit() );
#else
    TEST_IGNORE_MESSAGE( IT_FMC_NO_NAND_MSG );
#endif /* IT_FEATURE_FMC_NAND */
#else
    TEST_IGNORE_MESSAGE( IT_FMC_NO_FMC_MSG );
#endif /* FMC_Bank1_R */
}
