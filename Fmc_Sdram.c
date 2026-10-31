/**
 * \author Mr.Nobody
 * \file Fmc_Sdram.c
 * \ingroup Fmc
 * \brief Fmc module source file (SDRAM banks: configuration, initialization, refresh and power modes)
 *
 * The SDRAM controller of the flexible memory controller (STM32H543 and later, not on STM32H523 / H533
 * and not on STM32H503). The configuration converts every timing in nanoseconds to cycles of the SDRAM
 * clock (the bus clock divided by the clock divider of the configuration, rounded up, raised to the
 * smallest value of the controller), applies the rules of RM0481 23.8 (the write recovery time is raised
 * to the two minimum rules, the shared timings of two banks are the slowest of both devices, the refresh
 * count keeps the safety margin and differs from the sum of the timings) and rejects a combination the
 * clocks can not meet before a register is written.
 *
 * Fmc_Set_SdramInit runs the sequence of the reference manual (clock enable, power-up delay, precharge all,
 * auto-refresh cycles, load of the mode register, refresh rate) for every configured SDRAM bank at once
 * (the commands to two devices must be issued simultaneously). The power-up delay is a time requirement of
 * the memory, it is derived from the bus clock (it is never shorter than requested); every wait for the
 * controller is a bounded loop on the BUSY flag. On the lines without the SDRAM controller every exported
 * function checks the CMSIS macro (defined(FMC_Bank5_6_R)) and only returns FMC_REQUEST_ERROR.
 *
 */
/* ============================== INCLUDES ================================== */
#include "Fmc.h"                            /* Own private declarations       */
#include "Fmc_Reg.h"                        /* Module Reg component header    */
#include "Stm32.h"                          /* Device registers definition    */

/* ========================== PRIVATE DECLARATIONS =========================== */

#if defined(FMC_Bank5_6_R)

/** \brief Smallest number of cycles of the SDRAM timings tRCD, tRP, tWR, tRC, tRAS and tXSR (register value 0) */
#define FMC_SDRAM_CYCLES_MIN                ( 2u )

/** \brief Largest number of cycles of the SDRAM timings tRCD, tRP, tWR, tRC, tRAS and tXSR (register value 15) */
#define FMC_SDRAM_CYCLES_MAX                ( 17u )

/** \brief Smallest number of cycles of the SDRAM timing tMRD (register value 0) */
#define FMC_SDRAM_TMRD_CYCLES_MIN           ( 3u )

/** \brief Largest number of cycles of the SDRAM timing tMRD (register value 15) */
#define FMC_SDRAM_TMRD_CYCLES_MAX           ( 18u )

/** \brief Reset value of FMC_SDTRx (also the value that must stay in the register of a bank that is not used) */
#define FMC_SDTR_RESET_VALUE                ( 0x0FFFFFFFu )

/** \brief Smallest value of the refresh timer count (SDRAM clock cycles) */
#define FMC_SDRAM_REFRESH_COUNT_MIN         ( 41u )

/** \brief Largest value of the refresh timer count (13-bit field) */
#define FMC_SDRAM_REFRESH_COUNT_MAX         ( 0x1FFFu )

/** \brief Safety margin subtracted from the refresh timer count (SDRAM clock cycles, RM0481 23.8.5) */
#define FMC_SDRAM_REFRESH_MARGIN            ( 20u )

/** \brief The refresh count must not equal the sum of the SDRAM timings and this number of cycles */
#define FMC_SDRAM_REFRESH_SUM_OFFSET        ( 4u )

/** \brief Number of the column address bits of the register value 0 */
#define FMC_SDRAM_COLUMN_BITS_BASE          ( 8u )

/** \brief Number of the row address bits of the register value 0 */
#define FMC_SDRAM_ROW_BITS_BASE             ( 11u )

/** \brief Divider of the SDRAM clock: the SDRAM clock is the bus clock (also the value of the SDCLK field) */
#define FMC_SDRAM_DIVIDER_1                 ( 1u )

/** \brief Divider of the SDRAM clock: the SDRAM clock is the bus clock divided by two (also the SDCLK field) */
#define FMC_SDRAM_DIVIDER_2                 ( 2u )

/** \brief Divider of the SDRAM clock: the SDRAM clock is the bus clock divided by three (also the SDCLK field) */
#define FMC_SDRAM_DIVIDER_3                 ( 3u )

/** \brief Position of the CAS latency in the SDRAM mode register */
#define FMC_SDRAM_MODE_CAS_POS              ( 4u )

/** \brief Mode register: burst length 1, sequential, standard operation, single location write access */
#define FMC_SDRAM_MODE_BASE                 ( 0x0200u )

/** \brief FMC_SDSR MODESx field value: normal mode */
#define FMC_SDSR_MODES_NORMAL               ( 0u )

/** \brief FMC_SDSR MODESx field value: self-refresh mode */
#define FMC_SDSR_MODES_SELF_REFRESH         ( 1u )

/** \brief FMC_SDSR MODESx field value: power-down mode */
#define FMC_SDSR_MODES_POWER_DOWN           ( 2u )

/** \brief Bits of FMC_SDCRx written by the configuration of a bank (the bits of the bank, not the common ones) */
#define FMC_SDCR_BANK_MASK                  ( FMC_SDCRx_NC  | FMC_SDCRx_NR  | FMC_SDCRx_MWID | \
                                              FMC_SDCRx_NB  | FMC_SDCRx_CAS | FMC_SDCRx_WP )

/** \brief Bits of FMC_SDCR1 common to both banks */
#define FMC_SDCR_COMMON_MASK                ( FMC_SDCRx_SDCLK | FMC_SDCRx_RBURST | FMC_SDCRx_RPIPE )

/* =========================== LOCAL VARIABLES ============================== */

/** \brief Control register of every SDRAM bank */
static const fmc_RegId_t fmc_SdramCrReg[ FMC_SDRAM_BANK_CNT ] =
{
    FMC_REG_SDCR1, /**< SDRAM bank 1 */
    FMC_REG_SDCR2, /**< SDRAM bank 2 */
};

_Static_assert( ( sizeof( fmc_SdramCrReg ) / sizeof( fmc_RegId_t ) ) == FMC_SDRAM_BANK_CNT,
                 "Fmc_Sdram: fmc_SdramCrReg has incorrect size." );

/** \brief Timing register of every SDRAM bank */
static const fmc_RegId_t fmc_SdramTrReg[ FMC_SDRAM_BANK_CNT ] =
{
    FMC_REG_SDTR1, /**< SDRAM bank 1 */
    FMC_REG_SDTR2, /**< SDRAM bank 2 */
};

_Static_assert( ( sizeof( fmc_SdramTrReg ) / sizeof( fmc_RegId_t ) ) == FMC_SDRAM_BANK_CNT,
                 "Fmc_Sdram: fmc_SdramTrReg has incorrect size." );

/** \brief Command target bit of FMC_SDCMR of every SDRAM bank */
static const fmc_RegValue_t fmc_SdramTargetBit[ FMC_SDRAM_BANK_CNT ] =
{
    FMC_SDCMR_CTB1, /**< SDRAM bank 1 */
    FMC_SDCMR_CTB2, /**< SDRAM bank 2 */
};

_Static_assert( ( sizeof( fmc_SdramTargetBit ) / sizeof( fmc_RegValue_t ) ) == FMC_SDRAM_BANK_CNT,
                 "Fmc_Sdram: fmc_SdramTargetBit has incorrect size." );

/** \brief Mode status field mask of FMC_SDSR of every SDRAM bank */
static const fmc_RegValue_t fmc_SdramModesMask[ FMC_SDRAM_BANK_CNT ] =
{
    FMC_SDSR_MODES1, /**< SDRAM bank 1 */
    FMC_SDSR_MODES2, /**< SDRAM bank 2 */
};

_Static_assert( ( sizeof( fmc_SdramModesMask ) / sizeof( fmc_RegValue_t ) ) == FMC_SDRAM_BANK_CNT,
                 "Fmc_Sdram: fmc_SdramModesMask has incorrect size." );

/** \brief Mode status field position of FMC_SDSR of every SDRAM bank */
static const fmc_RegValue_t fmc_SdramModesPos[ FMC_SDRAM_BANK_CNT ] =
{
    FMC_SDSR_MODES1_Pos, /**< SDRAM bank 1 */
    FMC_SDSR_MODES2_Pos, /**< SDRAM bank 2 */
};

_Static_assert( ( sizeof( fmc_SdramModesPos ) / sizeof( fmc_RegValue_t ) ) == FMC_SDRAM_BANK_CNT,
                 "Fmc_Sdram: fmc_SdramModesPos has incorrect size." );

/** \brief Limits of the timings tRCD, tRP, tWR, tRC, tRAS and tXSR in cycles of the SDRAM clock */
static const fmc_PhaseLimit_t fmc_LimSdramCycle = { FMC_SDRAM_CYCLES_MIN, FMC_SDRAM_CYCLES_MAX };

/** \brief Limits of the timing tMRD in cycles of the SDRAM clock */
static const fmc_PhaseLimit_t fmc_LimSdramTmrd = { FMC_SDRAM_TMRD_CYCLES_MIN, FMC_SDRAM_TMRD_CYCLES_MAX };

/** \brief Timing of an SDRAM bank that is not used: the reset value of the registers */
static const fmc_SdramCycles_t fmc_SdramCyclesUnused =
{
    .LoadToActive    = FMC_SDRAM_TMRD_CYCLES_MAX,
    .ExitSelfRefresh = FMC_SDRAM_CYCLES_MAX,
    .SelfRefresh     = FMC_SDRAM_CYCLES_MAX,
    .RowCycle        = FMC_SDRAM_CYCLES_MAX,
    .WriteRecovery   = FMC_SDRAM_CYCLES_MAX,
    .RowPrecharge    = FMC_SDRAM_CYCLES_MAX,
    .RowToColumn     = FMC_SDRAM_CYCLES_MAX,
};

/* ========================== PRIVATE FUNCTIONS ============================== */

/**
 * \brief Returns the larger of two cycle counts.
 *
 * \param first  [in] : First count.
 * \param second [in] : Second count.
 *
 * \return The larger count.
 */
static fmc_CycleCnt_t FmcPriv_Max_Cycles( fmc_CycleCnt_t first, fmc_CycleCnt_t second )
{
    fmc_CycleCnt_t retValue = first;

    if( second > first )
    {
        retValue = second;
    }
    else
    {
        /* The first count is the larger one */
        retValue = first;
    }

    return ( retValue );
}


/**
 * \brief Checks the enumerated items of an SDRAM configuration.
 *
 * \param config [in] : Configuration to check, not null.
 *
 * \return FMC_REQUEST_OK for a valid configuration, FMC_REQUEST_ERROR otherwise.
 */
static fmc_RequestState_t FmcPriv_Check_SdramConfig( const fmc_SdramConfig_t * const config )
{
    fmc_RequestState_t       retState   = FMC_REQUEST_ERROR;
    fmc_RegValue_t           widthField = 0u;
    fmc_FlagState_t          banksValid = FMC_FLAG_INACTIVE;
    const fmc_RequestState_t widthState = FmcPriv_Get_WidthField( config->DataWidth, &widthField );

    if( ( FMC_SDRAM_INTERNAL_BANKS_2 == config->InternalBanks ) ||
        ( FMC_SDRAM_INTERNAL_BANKS_4 == config->InternalBanks )    )
    {
        banksValid = FMC_FLAG_ACTIVE;
    }
    else
    {
        /* The number of the internal banks is not in the list */
        banksValid = FMC_FLAG_INACTIVE;
    }

    if( ( FMC_REQUEST_OK              == widthState             ) &&
        ( FMC_FLAG_ACTIVE             == banksValid             ) &&
        ( FMC_SDRAM_COLUMN_BITS_8     <= config->ColumnBits     ) &&
        ( FMC_SDRAM_COLUMN_BITS_11    >= config->ColumnBits     ) &&
        ( FMC_SDRAM_ROW_BITS_11       <= config->RowBits        ) &&
        ( FMC_SDRAM_ROW_BITS_13       >= config->RowBits        ) &&
        ( FMC_CAS_LATENCY_1           <= config->CasLatency     ) &&
        ( FMC_CAS_LATENCY_3           >= config->CasLatency     ) &&
        ( FMC_FUNCTION_ACTIVE         >= config->ReadBurst      ) &&
        ( FMC_READ_PIPE_CNT            > config->ReadPipe       ) &&
        ( FMC_SDRAM_CLOCK_DIV_CNT      > config->ClockDivider   ) &&
        ( FMC_FUNCTION_ACTIVE         >= config->WriteProtect   )    )
    {
        retState = FMC_REQUEST_OK;
    }
    else
    {
        /* An item is out of its list */
        retState = FMC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Converts the control items of an SDRAM configuration to the control register value of its bank and to
 *        the control bits common to both banks.
 *
 * \param config [in]  : Checked configuration.
 * \param bankCr [out] : Pointer receiving the bits of the bank (FMC_SDCR_BANK_MASK).
 * \param common [out] : Pointer receiving the bits common to both banks (FMC_SDCR_COMMON_MASK).
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for an item the controller does not know.
 */
static fmc_RequestState_t FmcPriv_Get_SdramControl( const fmc_SdramConfig_t * const config,
                                                    fmc_RegValue_t * const bankCr,
                                                    fmc_RegValue_t * const common )
{
    fmc_RequestState_t retState   = FMC_REQUEST_ERROR;
    fmc_RegValue_t     widthField = 0u;
    fmc_CycleCnt_t     divider    = 0u;
    const fmc_RequestState_t widthState   = FmcPriv_Get_WidthField( config->DataWidth, &widthField );
    const fmc_RequestState_t dividerState = FmcPriv_Get_SdramDivider( config->ClockDivider, &divider );

    if( ( FMC_REQUEST_OK == widthState   ) &&
        ( FMC_REQUEST_OK == dividerState )    )
    {
        *bankCr = ( ( ( (fmc_RegValue_t)config->ColumnBits - FMC_SDRAM_COLUMN_BITS_BASE ) << FMC_SDCRx_NC_Pos   ) |
                    ( ( (fmc_RegValue_t)config->RowBits    - FMC_SDRAM_ROW_BITS_BASE    ) << FMC_SDCRx_NR_Pos   ) |
                    ( widthField                                                          << FMC_SDCRx_MWID_Pos ) |
                    ( (fmc_RegValue_t)config->CasLatency                                  << FMC_SDCRx_CAS_Pos  ) |
                    FmcPriv_Get_StateMask( config->WriteProtect, FMC_FUNCTION_ACTIVE, FMC_SDCRx_WP ) );

        if( FMC_SDRAM_INTERNAL_BANKS_4 == config->InternalBanks )
        {
            *bankCr |= FMC_SDCRx_NB;
        }
        else
        {
            /* Two internal banks, the NB bit stays cleared */
        }

        *common = ( ( divider                              << FMC_SDCRx_SDCLK_Pos  ) |
                    ( (fmc_RegValue_t)config->ReadPipe     << FMC_SDCRx_RPIPE_Pos  ) |
                    FmcPriv_Get_StateMask( config->ReadBurst, FMC_FUNCTION_ACTIVE, FMC_SDCRx_RBURST ) );
        retState = FMC_REQUEST_OK;
    }
    else
    {
        /* The data width or the clock divider is not valid */
        retState = FMC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Converts the timing of an SDRAM bank in cycles to the timing register value.
 *
 * \param cycles [in] : Timing in cycles (inside the limits of the fields).
 *
 * \return Timing register value.
 */
static fmc_RegValue_t FmcPriv_Get_SdramTimingValue( const fmc_SdramCycles_t * const cycles )
{
    const fmc_RegValue_t retValue =
        ( ( ( cycles->LoadToActive    - FMC_SDRAM_TMRD_CYCLES_MIN ) << FMC_SDTRx_TMRD_Pos ) |
          ( ( cycles->ExitSelfRefresh - FMC_SDRAM_CYCLES_MIN      ) << FMC_SDTRx_TXSR_Pos ) |
          ( ( cycles->SelfRefresh     - FMC_SDRAM_CYCLES_MIN      ) << FMC_SDTRx_TRAS_Pos ) |
          ( ( cycles->RowCycle        - FMC_SDRAM_CYCLES_MIN      ) << FMC_SDTRx_TRC_Pos  ) |
          ( ( cycles->WriteRecovery   - FMC_SDRAM_CYCLES_MIN      ) << FMC_SDTRx_TWR_Pos  ) |
          ( ( cycles->RowPrecharge    - FMC_SDRAM_CYCLES_MIN      ) << FMC_SDTRx_TRP_Pos  ) |
          ( ( cycles->RowToColumn     - FMC_SDRAM_CYCLES_MIN      ) << FMC_SDTRx_TRCD_Pos ) );

    return ( retValue );
}


/**
 * \brief Calculates the timing in cycles that is programmed in the timing register of both SDRAM banks. The
 *        timings shared by two devices are the slowest of both (RM0481 23.8.5): tRP and tRC in the register of
 *        bank 1, tWR and tXSR in both registers, the load mode register command to both banks uses tMRD and
 *        tRAS of bank 1. A bank that is not used keeps the reset value (only tRP and tRC of bank 1 are needed by
 *        a device on bank 2).
 *
 * \param instance [in]  : Run-time state of the module.
 * \param cycles   [out] : Timing in cycles of every bank.
 */
static void FmcPriv_Get_SdramProgrammed( const fmc_InstanceData_t * const instance,
                                         fmc_SdramCycles_t cycles[ FMC_SDRAM_BANK_CNT ] )
{
    fmc_SdramCycles_t merged = { 0u };

    for( uint32_t idx = 0u; ( FMC_SDRAM_BANK_CNT > idx ); idx++ )
    {
        if( FMC_FUNCTION_ACTIVE == instance->Configured[ (uint32_t)FMC_BANK_SDRAM_1 + idx ] )
        {
            const fmc_SdramCycles_t * const own = &( instance->SdramCycles[idx] );

            merged.LoadToActive    = FmcPriv_Max_Cycles( merged.LoadToActive,    own->LoadToActive    );
            merged.ExitSelfRefresh = FmcPriv_Max_Cycles( merged.ExitSelfRefresh, own->ExitSelfRefresh );
            merged.SelfRefresh     = FmcPriv_Max_Cycles( merged.SelfRefresh,     own->SelfRefresh     );
            merged.RowCycle        = FmcPriv_Max_Cycles( merged.RowCycle,        own->RowCycle        );
            merged.WriteRecovery   = FmcPriv_Max_Cycles( merged.WriteRecovery,   own->WriteRecovery   );
            merged.RowPrecharge    = FmcPriv_Max_Cycles( merged.RowPrecharge,    own->RowPrecharge    );
            merged.RowToColumn     = FmcPriv_Max_Cycles( merged.RowToColumn,     own->RowToColumn     );
        }
        else
        {
            /* The bank is not configured, it does not slow the shared timings down */
        }
    }

    for( uint32_t idx = 0u; ( FMC_SDRAM_BANK_CNT > idx ); idx++ )
    {
        if( FMC_FUNCTION_ACTIVE == instance->Configured[ (uint32_t)FMC_BANK_SDRAM_1 + idx ] )
        {
            /* The shared timings are the merged ones, the timings of the bank stay its own */
            cycles[idx] = merged;
            cycles[idx].RowToColumn = instance->SdramCycles[idx].RowToColumn;
            if( 0u != idx )
            {
                cycles[idx].LoadToActive = instance->SdramCycles[idx].LoadToActive;
                cycles[idx].SelfRefresh  = instance->SdramCycles[idx].SelfRefresh;
            }
            else
            {
                /* The load mode register command to both banks uses tMRD and tRAS of bank 1 (the slowest) */
            }
        }
        else
        {
            /* The bank is not used: reset value, only tRP and tRC of bank 1 serve the device on bank 2 */
            cycles[idx] = fmc_SdramCyclesUnused;
            if( 0u == idx )
            {
                cycles[idx].RowCycle     = merged.RowCycle;
                cycles[idx].RowPrecharge = merged.RowPrecharge;
            }
            else
            {
                /* The register of the second bank keeps the reset value */
            }
        }
    }
}


/**
 * \brief Writes the control registers of an SDRAM bank and the timing registers of both banks with the controller
 *        disabled and verifies them by read-back.
 *
 * \param instance [in] : Run-time state of the module (the configuration of the bank is stored already).
 * \param bankIdx  [in] : Index of the bank (0 or 1).
 * \param bankCr   [in] : Bits of the bank for its control register.
 * \param common   [in] : Control bits common to both banks.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR when the controller or a register does not follow.
 */
static fmc_RequestState_t FmcPriv_Write_SdramRegs( const fmc_InstanceData_t * const instance,
                                                   uint32_t bankIdx,
                                                   fmc_RegValue_t bankCr,
                                                   fmc_RegValue_t common )
{
    fmc_RequestState_t  retState  = FMC_REQUEST_ERROR;
    fmc_FunctionState_t wasActive = FMC_FUNCTION_INACTIVE;
    fmc_SdramCycles_t   cycles[ FMC_SDRAM_BANK_CNT ];
    fmc_RegValue_t      timing[ FMC_SDRAM_BANK_CNT ];
    const fmc_RequestState_t suspendState = FmcPriv_Controller_Suspend( &wasActive );

    FmcPriv_Get_SdramProgrammed( instance, cycles );
    for( uint32_t idx = 0u; ( FMC_SDRAM_BANK_CNT > idx ); idx++ )
    {
        timing[idx] = FmcPriv_Get_SdramTimingValue( &cycles[idx] );
    }

    if( FMC_REQUEST_OK == suspendState )
    {
        Fmc_Set_RegVal( FMC_REG_SDCR1, FMC_SDCR_COMMON_MASK, common );
        Fmc_Set_RegVal( fmc_SdramCrReg[bankIdx], FMC_SDCR_BANK_MASK, bankCr );
        for( uint32_t idx = 0u; ( FMC_SDRAM_BANK_CNT > idx ); idx++ )
        {
            Fmc_Set_RegVal( fmc_SdramTrReg[idx], FMC_REG_MASK_ALL, timing[idx] );
        }

        const fmc_RegValue_t commonBack = Fmc_Get_RegVal( FMC_REG_SDCR1, FMC_SDCR_COMMON_MASK );
        const fmc_RegValue_t bankBack   = Fmc_Get_RegVal( fmc_SdramCrReg[bankIdx], FMC_SDCR_BANK_MASK );
        const fmc_RegValue_t firstBack  = Fmc_Get_RegVal( FMC_REG_SDTR1, FMC_REG_MASK_ALL );
        const fmc_RegValue_t secondBack = Fmc_Get_RegVal( FMC_REG_SDTR2, FMC_REG_MASK_ALL );
        const fmc_RequestState_t resumeState = FmcPriv_Controller_Resume( wasActive );

        if( ( FMC_REQUEST_OK == resumeState   ) &&
            ( common         == commonBack    ) &&
            ( bankCr         == bankBack      ) &&
            ( timing[0]      == firstBack     ) &&
            ( timing[1]      == secondBack    )    )
        {
            retState = FMC_REQUEST_OK;
        }
        else
        {
            /* A register does not read back the written value */
            retState = FMC_REQUEST_ERROR;
        }
    }
    else
    {
        /* The controller did not follow the disable request */
        retState = FMC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Issues a command to the SDRAM controller and waits (bounded) until it is ready again.
 *
 * \param commandMode [in] : Command (the MODE field value).
 * \param targetBits  [in] : Command target bits (CTB1 / CTB2) of the banks that execute the command.
 * \param refreshCnt  [in] : Number of the auto-refresh cycles (1 .. 16, auto-refresh only).
 * \param modeReg     [in] : Content of the mode register (load mode register only).
 *
 * \return FMC_REQUEST_OK when the controller was ready again, FMC_REQUEST_ERROR on a timeout.
 */
static fmc_RequestState_t FmcPriv_Sdram_Send( fmc_SdramCommandMode_t commandMode,
                                              fmc_RegValue_t targetBits,
                                              fmc_RefreshCnt_t refreshCnt,
                                              fmc_ModeRegister_t modeReg )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;
    const fmc_RegValue_t commandValue = ( ( (fmc_RegValue_t)commandMode                  << FMC_SDCMR_MODE_Pos ) |
                                          ( ( (fmc_RegValue_t)refreshCnt - 1u )          << FMC_SDCMR_NRFS_Pos ) |
                                          ( (fmc_RegValue_t)modeReg                      << FMC_SDCMR_MRD_Pos  ) |
                                          targetBits );

    Fmc_Set_RegVal( FMC_REG_SDCMR, FMC_REG_MASK_ALL, commandValue );
    retState = Fmc_Reg_Wait_RegBitClr( FMC_REG_SDSR, FMC_REG_SDSR_BUSY );

    return ( retState );
}


/**
 * \brief Returns the command target bits of every configured SDRAM bank.
 *
 * \param instance [in] : Run-time state of the module.
 *
 * \return Command target bits.
 */
static fmc_RegValue_t FmcPriv_Get_SdramTargets( const fmc_InstanceData_t * const instance )
{
    fmc_RegValue_t targets = FMC_REG_BITS_CLEAR;

    for( uint32_t idx = 0u; ( FMC_SDRAM_BANK_CNT > idx ); idx++ )
    {
        targets |= FmcPriv_Get_StateMask( instance->Configured[ (uint32_t)FMC_BANK_SDRAM_1 + idx ],
                                          FMC_FUNCTION_ACTIVE,
                                          fmc_SdramTargetBit[idx] );
    }

    return ( targets );
}


/**
 * \brief Tells whether one of the SDRAM banks is activated.
 *
 * \param instance [in] : Run-time state of the module.
 *
 * \return FMC_FLAG_ACTIVE when a bank is active, FMC_FLAG_INACTIVE otherwise.
 */
static fmc_FlagState_t FmcPriv_Get_SdramAnyActive( const fmc_InstanceData_t * const instance )
{
    fmc_FlagState_t retState = FMC_FLAG_INACTIVE;

    if( ( FMC_FUNCTION_ACTIVE == instance->Active[FMC_BANK_SDRAM_1] ) ||
        ( FMC_FUNCTION_ACTIVE == instance->Active[FMC_BANK_SDRAM_2] )    )
    {
        retState = FMC_FLAG_ACTIVE;
    }
    else
    {
        /* No SDRAM bank is active */
        retState = FMC_FLAG_INACTIVE;
    }

    return ( retState );
}


/**
 * \brief Calculates the refresh count for a refresh period and a number of rows with the timings programmed in the
 *        registers of both banks: the count differs from the sum of the timings plus four cycles of either register.
 *
 * \param instance     [in]  : Run-time state of the module.
 * \param busClk       [in]  : Bus clock in Hz.
 * \param clockDivider [in]  : Divider item of the SDRAM clock.
 * \param period       [in]  : Refresh period in milliseconds.
 * \param rows         [in]  : Number of the rows refreshed in that period.
 * \param count        [out] : Pointer receiving the refresh count.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR when the clock can not meet the refresh period.
 */
static fmc_RequestState_t FmcPriv_Get_SdramBankCount( const fmc_InstanceData_t * const instance,
                                                      fmc_Frequency_Hz_t busClk,
                                                      fmc_SdramClockDiv_t clockDivider,
                                                      fmc_Time_ms_t period,
                                                      fmc_RowCnt_t rows,
                                                      fmc_RegValue_t * const count )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;
    fmc_RegValue_t     value    = 0u;
    fmc_CycleCnt_t     divider  = 0u;
    fmc_SdramCycles_t  cycles[ FMC_SDRAM_BANK_CNT ];

    FmcPriv_Get_SdramProgrammed( instance, cycles );

    const fmc_CycleCnt_t     firstSum     = ( cycles[0].WriteRecovery + cycles[0].RowPrecharge +
                                              cycles[0].RowCycle + cycles[0].RowToColumn );
    const fmc_CycleCnt_t     secondSum    = ( cycles[1].WriteRecovery + cycles[1].RowPrecharge +
                                              cycles[1].RowCycle + cycles[1].RowToColumn );
    const fmc_RequestState_t dividerState = FmcPriv_Get_SdramDivider( clockDivider, &divider );
    fmc_RequestState_t       valueState   = FMC_REQUEST_ERROR;

    if( FMC_REQUEST_OK == dividerState )
    {
        valueState = FmcPriv_Get_SdramRefreshCount( period, rows, busClk, divider, firstSum, &value );
    }
    else
    {
        /* The clock divider is not valid */
        valueState = FMC_REQUEST_ERROR;
    }

    if( FMC_REQUEST_OK == valueState )
    {
        /* The count differs from the sum of the timings of the register of the second bank too */
        for( uint32_t pass = 0u; ( FMC_SDRAM_BANK_CNT > pass ); pass++ )
        {
            if( value == ( secondSum + FMC_SDRAM_REFRESH_SUM_OFFSET ) )
            {
                value = ( value - 1u );
            }
            else
            {
                /* The count differs from the sum of the second register */
            }

            if( value == ( firstSum + FMC_SDRAM_REFRESH_SUM_OFFSET ) )
            {
                value = ( value - 1u );
            }
            else
            {
                /* The count differs from the sum of the first register */
            }
        }

        if( FMC_SDRAM_REFRESH_COUNT_MIN <= value )
        {
            *count   = value;
            retState = FMC_REQUEST_OK;
        }
        else
        {
            /* The decreased count is below the smallest value the controller accepts */
            retState = FMC_REQUEST_ERROR;
        }
    }
    else
    {
        /* The clock can not meet the refresh period */
        retState = FMC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Calculates the refresh count of the configured SDRAM banks (the smallest count, so that every device is
 *        refreshed in time).
 *
 * \param instance [in]  : Run-time state of the module.
 * \param busClk   [in]  : Bus clock in Hz.
 * \param count    [out] : Pointer receiving the refresh count.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR when the clock can not meet a refresh period.
 */
static fmc_RequestState_t FmcPriv_Get_SdramInitCount( const fmc_InstanceData_t * const instance,
                                                      fmc_Frequency_Hz_t busClk,
                                                      fmc_RegValue_t * const count )
{
    fmc_RequestState_t retState  = FMC_REQUEST_ERROR;
    fmc_RegValue_t     bankCount = 0u;
    fmc_RegValue_t     minCount  = FMC_SDRAM_REFRESH_COUNT_MAX;
    fmc_FlagState_t    failed    = FMC_FLAG_INACTIVE;
    fmc_FlagState_t    anyBank   = FMC_FLAG_INACTIVE;

    for( uint32_t idx = 0u; ( FMC_SDRAM_BANK_CNT > idx ); idx++ )
    {
        if( FMC_FUNCTION_ACTIVE == instance->Configured[ (uint32_t)FMC_BANK_SDRAM_1 + idx ] )
        {
            const fmc_SdramConfig_t * const config = &( instance->Sdram[idx] );
            const fmc_RequestState_t countState = FmcPriv_Get_SdramBankCount( instance,
                                                                              busClk,
                                                                              config->ClockDivider,
                                                                              config->RefreshPeriod_ms,
                                                                              config->RefreshRows,
                                                                              &bankCount );

            if( FMC_REQUEST_OK == countState )
            {
                if( bankCount < minCount )
                {
                    minCount = bankCount;
                }
                else
                {
                    /* The refresh count of another bank is the smaller one */
                }
                anyBank = FMC_FLAG_ACTIVE;
            }
            else
            {
                /* The clock can not meet the refresh period of the bank */
                failed = FMC_FLAG_ACTIVE;
            }
        }
        else
        {
            /* The bank is not configured */
        }
    }

    if( ( FMC_FLAG_ACTIVE   == anyBank ) &&
        ( FMC_FLAG_INACTIVE == failed  )    )
    {
        *count   = minCount;
        retState = FMC_REQUEST_OK;
    }
    else
    {
        /* No bank, or a refresh period the clock can not meet */
        retState = FMC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Moves an SDRAM bank to a low-power mode (self-refresh or power-down) and checks the mode status.
 *
 * \param bank        [in] : SDRAM bank that was initialized.
 * \param commandMode [in] : Command that enters the mode.
 * \param modesValue  [in] : Value of the mode status field of the bank that confirms the mode.
 *
 * \return FMC_REQUEST_OK when the controller reports the mode, FMC_REQUEST_ERROR otherwise.
 */
static fmc_RequestState_t FmcPriv_Sdram_Enter( fmc_Bank_t bank,
                                               fmc_SdramCommandMode_t commandMode,
                                               fmc_RegValue_t modesValue )
{
    fmc_RequestState_t   retState      = FMC_REQUEST_ERROR;
    fmc_InstanceData_t * instance      = FMC_NULL_PTR;
    fmc_RequestState_t   instanceState = FmcPriv_Get_Instance( &instance );

    if( ( FMC_REQUEST_OK       == instanceState         ) &&
        ( FMC_BANK_SDRAM_1     <= bank                  ) &&
        ( FMC_BANK_SDRAM_2     >= bank                  ) &&
        ( FMC_FUNCTION_ACTIVE  == instance->Initialized )    )
    {
        const uint32_t bankIdx = ( (uint32_t)bank - (uint32_t)FMC_BANK_SDRAM_1 );

        if( FMC_FUNCTION_ACTIVE == instance->SdramReady[bankIdx] )
        {
            const fmc_RequestState_t sendState = FmcPriv_Sdram_Send( commandMode, fmc_SdramTargetBit[bankIdx], 1u, 0u );
            const fmc_RegValue_t     modesRead = ( Fmc_Get_RegVal( FMC_REG_SDSR, fmc_SdramModesMask[bankIdx] ) >>
                                                   fmc_SdramModesPos[bankIdx] );

            if( ( FMC_REQUEST_OK == sendState ) &&
                ( modesValue     == modesRead )    )
            {
                retState = FMC_REQUEST_OK;
            }
            else
            {
                /* The command timed out or the controller does not report the mode */
                retState = FMC_REQUEST_ERROR;
            }
        }
        else
        {
            /* The SDRAM is not initialized */
            retState = FMC_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid parameter or state, no action required */
        retState = FMC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Writes the refresh timer count and the refresh error interrupt enable and verifies them by read-back.
 *
 * \param instance [in] : Run-time state of the module.
 * \param count    [in] : Refresh count.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR when the register does not read back.
 */
static fmc_RequestState_t FmcPriv_Write_SdramRefresh( const fmc_InstanceData_t * const instance,
                                                      fmc_RegValue_t count )
{
    fmc_RequestState_t retState  = FMC_REQUEST_ERROR;
    fmc_RegValue_t     reieValue = FMC_REG_BITS_CLEAR;
    fmc_RegValue_t     regValue  = FMC_REG_BITS_CLEAR;

    if( ( FMC_EVENT_MODE_ISR == instance->Config.EventMode ) &&
        ( ( FMC_NULL_PTR != instance->Config.RefreshErrorCallback ) ||
          ( FMC_NULL_PTR != instance->Config.ErrorCallback        )    ) )
    {
        reieValue = FMC_SDRTR_REIE;
    }
    else
    {
        /* The refresh error is polled by Fmc_Task or nobody wants it, the interrupt stays disabled */
        reieValue = FMC_REG_BITS_CLEAR;
    }

    regValue = ( ( count << FMC_SDRTR_COUNT_Pos ) | reieValue );
    retState = Fmc_Reg_Set_RegValVerified( FMC_REG_SDRTR, ( FMC_SDRTR_COUNT | FMC_SDRTR_REIE ), regValue );

    return ( retState );
}

#endif /* FMC_Bank5_6_R */

/* ========================= EXPORTED FUNCTIONS =============================== */

#if defined(FMC_Bank5_6_R)

/**
 * \brief Converts the divider item of the SDRAM clock to the divider of the bus clock.
 *
 * \param clockDivider [in]  : Divider item.
 * \param divider      [out] : Pointer receiving the divider (1, 2 or 3).
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for an item out of the list or a null pointer.
 */
fmc_RequestState_t FmcPriv_Get_SdramDivider( fmc_SdramClockDiv_t clockDivider, fmc_CycleCnt_t * const divider )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;

    if( FMC_NULL_PTR == divider )
    {
        /* Invalid parameter, retState stays FMC_REQUEST_ERROR */
        retState = FMC_REQUEST_ERROR;
    }
    else if( FMC_SDRAM_CLOCK_DIV_2 == clockDivider )
    {
        *divider = FMC_SDRAM_DIVIDER_2;
        retState = FMC_REQUEST_OK;
    }
    else if( FMC_SDRAM_CLOCK_DIV_3 == clockDivider )
    {
        *divider = FMC_SDRAM_DIVIDER_3;
        retState = FMC_REQUEST_OK;
    }
    else if( FMC_SDRAM_CLOCK_DIV_1 == clockDivider )
    {
        *divider = FMC_SDRAM_DIVIDER_1;
        retState = FMC_REQUEST_OK;
    }
    else
    {
        /* The item is not in the list */
        retState = FMC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Converts the timing of an SDRAM configuration to cycles of the SDRAM clock (the bus clock divided by the
 *        clock divider of the configuration): every time is rounded up and raised to the smallest value of the
 *        controller; tWR is raised so that tWR >= tRAS - tRCD and tWR >= tRC - tRCD - tRP (RM0481 23.8.5).
 *
 * \param config [in]  : Configuration of the bank.
 * \param busClk [in]  : Bus clock in Hz.
 * \param cycles [out] : Pointer receiving the timing in cycles.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for an invalid item or a time the clock can not meet.
 */
fmc_RequestState_t FmcPriv_Get_SdramCycles( const fmc_SdramConfig_t * const config,
                                            fmc_Frequency_Hz_t busClk,
                                            fmc_SdramCycles_t * const cycles )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;
    fmc_CycleCnt_t     divider  = 0u;

    if( ( FMC_NULL_PTR != config ) &&
        ( FMC_NULL_PTR != cycles ) &&
        ( 0u           != busClk )    )
    {
        const fmc_RequestState_t checkState   = FmcPriv_Check_SdramConfig( config );
        const fmc_RequestState_t dividerState = FmcPriv_Get_SdramDivider( config->ClockDivider, &divider );

        if( ( FMC_REQUEST_OK == checkState   ) &&
            ( FMC_REQUEST_OK == dividerState )    )
        {
            const fmc_Clock_t clock = { .Clk = busClk, .Divider = divider };
            const fmc_Phase_t phases[] =
            {
                { config->LoadToActiveDelay_ns,    &fmc_LimSdramTmrd,  &( cycles->LoadToActive    ) }, /**< tMRD */
                { config->ExitSelfRefreshDelay_ns, &fmc_LimSdramCycle, &( cycles->ExitSelfRefresh ) }, /**< tXSR */
                { config->SelfRefreshTime_ns,      &fmc_LimSdramCycle, &( cycles->SelfRefresh     ) }, /**< tRAS */
                { config->RowCycleDelay_ns,        &fmc_LimSdramCycle, &( cycles->RowCycle        ) }, /**< tRC  */
                { config->WriteRecoveryTime_ns,    &fmc_LimSdramCycle, &( cycles->WriteRecovery   ) }, /**< tWR  */
                { config->RpDelay_ns,              &fmc_LimSdramCycle, &( cycles->RowPrecharge    ) }, /**< tRP  */
                { config->RcdDelay_ns,             &fmc_LimSdramCycle, &( cycles->RowToColumn     ) }, /**< tRCD */
            };
            const fmc_RequestState_t phaseState = FmcPriv_Get_Phases( &clock, phases, FMC_PHASE_CNT( phases ) );

            if( FMC_REQUEST_OK == phaseState )
            {
                /* tWR >= tRAS - tRCD and tWR >= tRC - tRCD - tRP */
                if( cycles->SelfRefresh > cycles->RowToColumn )
                {
                    cycles->WriteRecovery = FmcPriv_Max_Cycles( cycles->WriteRecovery,
                                                                ( cycles->SelfRefresh - cycles->RowToColumn ) );
                }
                else
                {
                    /* tRAS does not exceed tRCD, no minimum of tWR from this rule */
                }

                if( cycles->RowCycle > ( cycles->RowToColumn + cycles->RowPrecharge ) )
                {
                    cycles->WriteRecovery = FmcPriv_Max_Cycles( cycles->WriteRecovery,
                                                                ( cycles->RowCycle - cycles->RowToColumn -
                                                                  cycles->RowPrecharge ) );
                }
                else
                {
                    /* tRC does not exceed tRCD + tRP, no minimum of tWR from this rule */
                }

                if( FMC_SDRAM_CYCLES_MAX >= cycles->WriteRecovery )
                {
                    retState = FMC_REQUEST_OK;
                }
                else
                {
                    /* The recovery time the rules need does not fit the field */
                    retState = FMC_REQUEST_ERROR;
                }
            }
            else
            {
                /* A time does not fit its field */
                retState = FMC_REQUEST_ERROR;
            }
        }
        else
        {
            /* An item is out of its list */
            retState = FMC_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid parameter, no action required */
        retState = FMC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Calculates the refresh timer count: COUNT = floor( period / rows * SDRAM clock ) - 20 (rounded down, so the
 *        refresh is never later than the memory needs, with the safety margin of the reference manual); a count that
 *        equals the sum of the timings plus four cycles is decreased by one.
 *
 * \param refreshPeriod [in]  : Time in which the whole array must be refreshed in milliseconds.
 * \param refreshRows   [in]  : Number of the rows refreshed in that time.
 * \param busClk        [in]  : Bus clock in Hz.
 * \param divider       [in]  : Divider of the SDRAM clock (the SDRAM clock is busClk / divider).
 * \param cycleSum      [in]  : Sum of tWR, tRP, tRC and tRCD in cycles of the SDRAM clock.
 * \param count         [out] : Pointer receiving the COUNT value.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for an invalid parameter or a count outside 41 .. 8191.
 */
fmc_RequestState_t FmcPriv_Get_SdramRefreshCount( fmc_Time_ms_t refreshPeriod,
                                                  fmc_RowCnt_t refreshRows,
                                                  fmc_Frequency_Hz_t busClk,
                                                  fmc_CycleCnt_t divider,
                                                  fmc_CycleCnt_t cycleSum,
                                                  fmc_RegValue_t * const count )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;

    if( ( FMC_NULL_PTR != count          ) &&
        ( 0u           != refreshPeriod  ) &&
        ( 0u           != refreshRows    ) &&
        ( 0u           != busClk         ) &&
        ( 0u           != divider        )    )
    {
        const uint64_t numerator   = ( (uint64_t)refreshPeriod * (uint64_t)busClk );
        const uint64_t denominator = ( (uint64_t)divider * (uint64_t)FMC_MS_PER_S * (uint64_t)refreshRows );
        const uint64_t rowCycles   = ( numerator / denominator );

        if( ( (uint64_t)FMC_SDRAM_REFRESH_MARGIN + (uint64_t)FMC_SDRAM_REFRESH_COUNT_MIN ) <= rowCycles )
        {
            uint64_t refreshCount = ( rowCycles - FMC_SDRAM_REFRESH_MARGIN );

            if( ( (uint64_t)cycleSum + FMC_SDRAM_REFRESH_SUM_OFFSET ) == refreshCount )
            {
                refreshCount = ( refreshCount - 1u );
            }
            else
            {
                /* The count differs from the sum of the timings */
            }

            if( ( (uint64_t)FMC_SDRAM_REFRESH_COUNT_MIN <= refreshCount ) &&
                ( (uint64_t)FMC_SDRAM_REFRESH_COUNT_MAX >= refreshCount )    )
            {
                *count   = (fmc_RegValue_t)refreshCount;
                retState = FMC_REQUEST_OK;
            }
            else
            {
                /* The count does not fit the field */
                retState = FMC_REQUEST_ERROR;
            }
        }
        else
        {
            /* The refresh interval is shorter than the controller can serve */
            retState = FMC_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid parameter, no action required */
        retState = FMC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Builds the content of the SDRAM mode register of a bank: burst length 1, sequential burst, the CAS latency
 *        of the configuration, standard operation, single location write access (RM0481 23.8.3 step 7).
 *
 * \param config [in] : Configuration of the bank, not null.
 *
 * \return Content of the mode register.
 */
fmc_ModeRegister_t FmcPriv_Get_SdramModeRegister( const fmc_SdramConfig_t * const config )
{
    const fmc_ModeRegister_t modeRegister =
        (fmc_ModeRegister_t)( FMC_SDRAM_MODE_BASE | ( (fmc_RegValue_t)config->CasLatency << FMC_SDRAM_MODE_CAS_POS ) );

    return ( modeRegister );
}


/**
 * \brief Brings an SDRAM bank to the normal mode (the exit of the self-refresh and the power-down too) after its
 *        initialization.
 *
 * \param bank [in] : SDRAM bank.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for a bank that is not initialized or a timeout.
 */
fmc_RequestState_t FmcPriv_Sdram_Activate( fmc_Bank_t bank )
{
    fmc_RequestState_t   retState      = FMC_REQUEST_ERROR;
    fmc_InstanceData_t * instance      = FMC_NULL_PTR;
    fmc_RequestState_t   instanceState = FmcPriv_Get_Instance( &instance );

    if( ( FMC_REQUEST_OK    == instanceState ) &&
        ( FMC_BANK_SDRAM_1  <= bank          ) &&
        ( FMC_BANK_SDRAM_2  >= bank          )    )
    {
        const uint32_t bankIdx = ( (uint32_t)bank - (uint32_t)FMC_BANK_SDRAM_1 );

        if( FMC_FUNCTION_ACTIVE == instance->SdramReady[bankIdx] )
        {
            retState = FmcPriv_Sdram_Send( FMC_SDRAM_COMMAND_NORMAL, fmc_SdramTargetBit[bankIdx], 1u, 0u );
        }
        else
        {
            /* The SDRAM is not initialized */
            retState = FMC_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid parameter, no action required */
        retState = FMC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Sets or clears the write protection (WP) of an SDRAM bank with the controller disabled and verifies it.
 *
 * \param bank            [in] : SDRAM bank.
 * \param protectionState [in] : FMC_FUNCTION_ACTIVE protects the bank.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR when the controller or the register does not follow.
 */
fmc_RequestState_t FmcPriv_Sdram_Set_WriteProtection( fmc_Bank_t bank, fmc_FunctionState_t protectionState )
{
    fmc_RequestState_t   retState      = FMC_REQUEST_ERROR;
    fmc_InstanceData_t * instance      = FMC_NULL_PTR;
    fmc_RequestState_t   instanceState = FmcPriv_Get_Instance( &instance );

    if( ( FMC_REQUEST_OK    == instanceState ) &&
        ( FMC_BANK_SDRAM_1  <= bank          ) &&
        ( FMC_BANK_SDRAM_2  >= bank          )    )
    {
        const uint32_t      bankIdx   = ( (uint32_t)bank - (uint32_t)FMC_BANK_SDRAM_1 );
        const fmc_RegValue_t regValue = FmcPriv_Get_StateMask( protectionState, FMC_FUNCTION_ACTIVE, FMC_SDCRx_WP );
        fmc_FunctionState_t wasActive = FMC_FUNCTION_INACTIVE;
        const fmc_RequestState_t suspendState = FmcPriv_Controller_Suspend( &wasActive );

        if( FMC_REQUEST_OK == suspendState )
        {
            const fmc_RequestState_t writeState  = Fmc_Reg_Set_RegValVerified( fmc_SdramCrReg[bankIdx],
                                                                               FMC_SDCRx_WP,
                                                                               regValue );
            const fmc_RequestState_t resumeState = FmcPriv_Controller_Resume( wasActive );

            if( ( FMC_REQUEST_OK == writeState  ) &&
                ( FMC_REQUEST_OK == resumeState )    )
            {
                instance->Sdram[bankIdx].WriteProtect = protectionState;
                retState = FMC_REQUEST_OK;
            }
            else
            {
                /* The register or the controller did not follow */
                retState = FMC_REQUEST_ERROR;
            }
        }
        else
        {
            /* The controller did not follow the disable request */
            retState = FMC_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid parameter, no action required */
        retState = FMC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Serves the refresh error event: the flag is read and cleared and reported by the callbacks.
 */
void FmcPriv_Sdram_ServeEvents( void )
{
    fmc_InstanceData_t * instance      = FMC_NULL_PTR;
    fmc_RequestState_t   instanceState = FmcPriv_Get_Instance( &instance );

    if( ( FMC_REQUEST_OK      == instanceState         ) &&
        ( FMC_FUNCTION_ACTIVE == instance->Initialized )    )
    {
        const fmc_RegValue_t refreshError = Fmc_Get_RegBit( FMC_REG_SDSR, FMC_SDSR_RE );

        if( FMC_REG_BITS_CLEAR != refreshError )
        {
            Fmc_Set_RegBit( FMC_REG_SDRTR, FMC_SDRTR_CRE );

            if( FMC_NULL_PTR != instance->Config.RefreshErrorCallback )
            {
                instance->Config.RefreshErrorCallback();
            }
            else
            {
                /* No dedicated callback */
            }

            if( FMC_NULL_PTR != instance->Config.ErrorCallback )
            {
                instance->Config.ErrorCallback( FMC_ERROR_SDRAM_REFRESH );
            }
            else
            {
                /* No error callback */
            }
        }
        else
        {
            /* No refresh error */
        }
    }
    else
    {
        /* The module is not initialized, no action required */
    }
}

#endif /* FMC_Bank5_6_R */


/**
 * \brief Configures an SDRAM bank. The timing in nanoseconds is converted to cycles of the SDRAM clock first (a
 *        time the clock can not meet, or a refresh period it can not serve, rejects the whole call), then the
 *        registers are written with the controller disabled and verified by read-back. The SDRAM clock divider, the
 *        read burst and the read pipe are common to both banks: the second bank must carry the same values. The
 *        timings shared by both banks become the slowest of both devices. Both banks are configured before one of
 *        them is activated. The SDRAM is not usable before \ref Fmc_Set_SdramInit and \ref Fmc_Set_BankActive.
 *
 * \param bank         [in] : SDRAM bank, FMC_BANK_SDRAM_1 or FMC_BANK_SDRAM_2.
 * \param sdramConfig [in] : Pointer to the configuration of the bank.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for an invalid parameter, an uninitialized module, an SDRAM
 *         bank that is active, a common value that differs from the other bank, a time the clock can not meet or a
 *         register that does not read back.
 */
fmc_RequestState_t Fmc_Set_SdramConfig( fmc_Bank_t bank, const fmc_SdramConfig_t * const sdramConfig )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;

#if defined(FMC_Bank5_6_R)
    fmc_InstanceData_t * instance      = FMC_NULL_PTR;
    fmc_RequestState_t   instanceState = FmcPriv_Get_Instance( &instance );

    if( ( FMC_REQUEST_OK       == instanceState         ) &&
        ( FMC_NULL_PTR         != sdramConfig           ) &&
        ( FMC_BANK_SDRAM_1     <= bank                  ) &&
        ( FMC_BANK_SDRAM_2     >= bank                  ) &&
        ( FMC_FUNCTION_ACTIVE  == instance->Initialized )    )
    {
        const uint32_t         bankIdx     = ( (uint32_t)bank - (uint32_t)FMC_BANK_SDRAM_1 );
        const uint32_t         otherIdx    = ( ( FMC_SDRAM_BANK_CNT - 1u ) - bankIdx );
        const fmc_Bank_t       otherBank   = (fmc_Bank_t)( (uint32_t)FMC_BANK_SDRAM_1 + otherIdx );
        const fmc_FlagState_t  anyActive   = FmcPriv_Get_SdramAnyActive( instance );
        fmc_Frequency_Hz_t     busClk      = 0u;
        fmc_SdramCycles_t      cycles      = { 0u };
        fmc_RegValue_t         bankCr      = 0u;
        fmc_RegValue_t         common      = 0u;
        fmc_RegValue_t         count       = 0u;
        fmc_CycleCnt_t         divider     = 0u;
        const fmc_RequestState_t clkState     = FmcPriv_Get_BusClock( &busClk );
        const fmc_RequestState_t checkState   = FmcPriv_Check_SdramConfig( sdramConfig );
        const fmc_RequestState_t dividerState = FmcPriv_Get_SdramDivider( sdramConfig->ClockDivider, &divider );
        fmc_RequestState_t       convState    = FMC_REQUEST_ERROR;
        fmc_RequestState_t       countState   = FMC_REQUEST_ERROR;
        fmc_FlagState_t          commonDiffers = FMC_FLAG_INACTIVE;

        if( ( FMC_REQUEST_OK    == clkState     ) &&
            ( FMC_REQUEST_OK    == checkState   ) &&
            ( FMC_REQUEST_OK    == dividerState ) &&
            ( FMC_FLAG_INACTIVE == anyActive    )    )
        {
            convState = FmcPriv_Get_SdramCycles( sdramConfig, busClk, &cycles );
            if( FMC_REQUEST_OK == convState )
            {
                countState = FmcPriv_Get_SdramRefreshCount( sdramConfig->RefreshPeriod_ms,
                                                            sdramConfig->RefreshRows,
                                                            busClk,
                                                            divider,
                                                            ( cycles.WriteRecovery + cycles.RowPrecharge +
                                                              cycles.RowCycle + cycles.RowToColumn ),
                                                            &count );
            }
            else
            {
                /* A time does not fit, the refresh count is not calculated */
                countState = FMC_REQUEST_ERROR;
            }
        }
        else
        {
            /* Invalid configuration, bad clock or an SDRAM bank is active */
            convState  = FMC_REQUEST_ERROR;
            countState = FMC_REQUEST_ERROR;
        }

        if( ( FMC_FUNCTION_ACTIVE == instance->Configured[otherBank] ) &&
            ( ( instance->Sdram[otherIdx].ClockDivider != sdramConfig->ClockDivider ) ||
              ( instance->Sdram[otherIdx].ReadBurst    != sdramConfig->ReadBurst    ) ||
              ( instance->Sdram[otherIdx].ReadPipe     != sdramConfig->ReadPipe     )    ) )
        {
            /* The clock divider, the read burst and the read pipe are common to both banks */
            commonDiffers = FMC_FLAG_ACTIVE;
        }
        else
        {
            /* The other bank is not configured or has the same common values */
            commonDiffers = FMC_FLAG_INACTIVE;
        }

        const fmc_RequestState_t controlState = FmcPriv_Get_SdramControl( sdramConfig, &bankCr, &common );

        if( ( FMC_REQUEST_OK    == convState      ) &&
            ( FMC_REQUEST_OK    == countState     ) &&
            ( FMC_REQUEST_OK    == controlState   ) &&
            ( FMC_FLAG_INACTIVE == commonDiffers  )    )
        {
            instance->Sdram[bankIdx]       = *sdramConfig;
            instance->SdramCycles[bankIdx] = cycles;
            instance->Configured[bank]     = FMC_FUNCTION_ACTIVE;
            for( uint32_t idx = 0u; ( FMC_SDRAM_BANK_CNT > idx ); idx++ )
            {
                /* The timing registers changed, the SDRAM has to be initialized again */
                instance->SdramReady[idx] = FMC_FUNCTION_INACTIVE;
            }

            retState = FmcPriv_Write_SdramRegs( instance, bankIdx, bankCr, common );

            if( FMC_REQUEST_OK != retState )
            {
                /* The registers were not written as required */
                instance->Configured[bank] = FMC_FUNCTION_INACTIVE;
            }
            else
            {
                /* The bank is configured */
            }
        }
        else
        {
            /* The configuration is invalid or the clocks can not meet it, no register was written */
            retState = FMC_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid parameter or state, no action required */
        retState = FMC_REQUEST_ERROR;
    }
#else
    (void)bank;
    (void)sdramConfig;
#endif /* FMC_Bank5_6_R */

    return ( retState );
}


/**
 * \brief Runs the SDRAM initialization sequence of the reference manual for every configured SDRAM bank at once:
 *        clock enable, power-up delay (at least 100 us), precharge all, auto-refresh cycles, load of the mode register
 *        of each bank, refresh rate. It returns when the controller is ready (bounded wait on BUSY). The bank is
 *        activated afterwards by \ref Fmc_Set_BankActive.
 *
 * \param bank [in] : SDRAM bank that was configured.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for an invalid or not configured bank, an uninitialized module,
 *         an SDRAM bank that is active, a refresh period the clock can not meet or a timeout.
 */
fmc_RequestState_t Fmc_Set_SdramInit( fmc_Bank_t bank )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;

#if defined(FMC_Bank5_6_R)
    fmc_InstanceData_t * instance      = FMC_NULL_PTR;
    fmc_RequestState_t   instanceState = FmcPriv_Get_Instance( &instance );

    if( ( FMC_REQUEST_OK       == instanceState         ) &&
        ( FMC_BANK_SDRAM_1     <= bank                  ) &&
        ( FMC_BANK_SDRAM_2     >= bank                  ) &&
        ( FMC_FUNCTION_ACTIVE  == instance->Initialized ) &&
        ( FMC_FUNCTION_ACTIVE  == instance->Configured[bank] )    )
    {
        const fmc_FlagState_t anyActive = FmcPriv_Get_SdramAnyActive( instance );
        const fmc_RegValue_t  targets   = FmcPriv_Get_SdramTargets( instance );
        fmc_Frequency_Hz_t    busClk    = 0u;
        fmc_RegValue_t        count     = 0u;
        fmc_Time_us_t         delay     = FMC_SDRAM_POWER_UP_DELAY_MIN_US;
        const fmc_RequestState_t clkState   = FmcPriv_Get_BusClock( &busClk );
        fmc_RequestState_t       countState = FMC_REQUEST_ERROR;

        if( ( FMC_REQUEST_OK    == clkState  ) &&
            ( FMC_FLAG_INACTIVE == anyActive )    )
        {
            countState = FmcPriv_Get_SdramInitCount( instance, busClk, &count );
        }
        else
        {
            /* The bus clock is not available or an SDRAM bank is active */
            countState = FMC_REQUEST_ERROR;
        }

        for( uint32_t idx = 0u; ( FMC_SDRAM_BANK_CNT > idx ); idx++ )
        {
            if( ( FMC_FUNCTION_ACTIVE == instance->Configured[ (uint32_t)FMC_BANK_SDRAM_1 + idx ] ) &&
                ( delay                < instance->Sdram[idx].PowerUpDelay_us                    )    )
            {
                delay = instance->Sdram[idx].PowerUpDelay_us;
            }
            else
            {
                /* The delay of the bank does not extend the minimum */
            }
        }

        if( FMC_REQUEST_OK == countState )
        {
            fmc_RequestState_t stepState = FmcPriv_Sdram_Send( FMC_SDRAM_COMMAND_CLOCK_ENABLE, targets, 1u, 0u );

            if( FMC_REQUEST_OK == stepState )
            {
                stepState = FmcPriv_Set_Delay( delay );
            }
            else
            {
                /* The clock enable command was not executed */
            }

            if( FMC_REQUEST_OK == stepState )
            {
                stepState = FmcPriv_Sdram_Send( FMC_SDRAM_COMMAND_PRECHARGE_ALL, targets, 1u, 0u );
            }
            else
            {
                /* An earlier step failed */
            }

            if( FMC_REQUEST_OK == stepState )
            {
                stepState = FmcPriv_Sdram_Send( FMC_SDRAM_COMMAND_AUTO_REFRESH,
                                                targets,
                                                FMC_SDRAM_INIT_AUTO_REFRESH_CNT,
                                                0u );
            }
            else
            {
                /* An earlier step failed */
            }

            for( uint32_t idx = 0u; ( FMC_SDRAM_BANK_CNT > idx ); idx++ )
            {
                if( ( FMC_REQUEST_OK      == stepState                                              ) &&
                    ( FMC_FUNCTION_ACTIVE == instance->Configured[ (uint32_t)FMC_BANK_SDRAM_1 + idx ] )    )
                {
                    stepState = FmcPriv_Sdram_Send( FMC_SDRAM_COMMAND_LOAD_MODE_REGISTER,
                                                    fmc_SdramTargetBit[idx],
                                                    1u,
                                                    FmcPriv_Get_SdramModeRegister( &( instance->Sdram[idx] ) ) );
                }
                else
                {
                    /* An earlier step failed or the bank is not configured */
                }
            }

            if( FMC_REQUEST_OK == stepState )
            {
                stepState = FmcPriv_Write_SdramRefresh( instance, count );
            }
            else
            {
                /* An earlier step failed */
            }

            if( FMC_REQUEST_OK == stepState )
            {
                for( uint32_t idx = 0u; ( FMC_SDRAM_BANK_CNT > idx ); idx++ )
                {
                    instance->SdramReady[idx] = instance->Configured[ (uint32_t)FMC_BANK_SDRAM_1 + idx ];
                }
            }
            else
            {
                /* The sequence failed, no bank is initialized */
                for( uint32_t idx = 0u; ( FMC_SDRAM_BANK_CNT > idx ); idx++ )
                {
                    instance->SdramReady[idx] = FMC_FUNCTION_INACTIVE;
                }
            }
            retState = stepState;
        }
        else
        {
            /* The clock can not meet a refresh period, no command was issued */
            retState = FMC_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid parameter or state, no action required */
        retState = FMC_REQUEST_ERROR;
    }
#else
    (void)bank;
#endif /* FMC_Bank5_6_R */

    return ( retState );
}


/**
 * \brief Changes the refresh rate of the SDRAM (the count of the refresh timer) for the clock divider and the shared
 *        timings of the initialized banks. The refresh period must be met by the clock. The refresh error interrupt
 *        enable stays as \ref Fmc_Set_SdramInit set it.
 *
 * \param refreshPeriod [in] : Time in which the whole array must be refreshed in milliseconds.
 * \param refreshRows   [in] : Number of the rows refreshed in that time.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for an uninitialized module or SDRAM, a refresh period the clock
 *         can not meet or a register that does not read back.
 */
fmc_RequestState_t Fmc_Set_SdramRefreshRate( fmc_Time_ms_t refreshPeriod, fmc_RowCnt_t refreshRows )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;

#if defined(FMC_Bank5_6_R)
    fmc_InstanceData_t * instance      = FMC_NULL_PTR;
    fmc_RequestState_t   instanceState = FmcPriv_Get_Instance( &instance );

    if( ( FMC_REQUEST_OK      == instanceState         ) &&
        ( FMC_FUNCTION_ACTIVE == instance->Initialized )    )
    {
        fmc_Frequency_Hz_t       busClk     = 0u;
        fmc_RegValue_t           count      = 0u;
        fmc_FlagState_t          anyReady   = FMC_FLAG_INACTIVE;
        fmc_SdramClockDiv_t      divider    = FMC_SDRAM_CLOCK_DIV_2;
        const fmc_RequestState_t clkState   = FmcPriv_Get_BusClock( &busClk );

        for( uint32_t idx = 0u; ( FMC_SDRAM_BANK_CNT > idx ); idx++ )
        {
            if( FMC_FUNCTION_ACTIVE == instance->SdramReady[idx] )
            {
                /* The clock divider is common to both banks */
                divider  = instance->Sdram[idx].ClockDivider;
                anyReady = FMC_FLAG_ACTIVE;
            }
            else
            {
                /* The bank is not initialized */
            }
        }

        if( ( FMC_REQUEST_OK == clkState ) &&
            ( FMC_FLAG_ACTIVE == anyReady )    )
        {
            const fmc_RequestState_t countState = FmcPriv_Get_SdramBankCount( instance,
                                                                              busClk,
                                                                              divider,
                                                                              refreshPeriod,
                                                                              refreshRows,
                                                                              &count );

            if( FMC_REQUEST_OK == countState )
            {
                retState = FmcPriv_Write_SdramRefresh( instance, count );
            }
            else
            {
                /* The clock can not meet the refresh period, no register was written */
                retState = FMC_REQUEST_ERROR;
            }
        }
        else
        {
            /* No SDRAM is initialized or the bus clock is not available */
            retState = FMC_REQUEST_ERROR;
        }
    }
    else
    {
        /* The module is not initialized, no action required */
        retState = FMC_REQUEST_ERROR;
    }
#else
    (void)refreshPeriod;
    (void)refreshRows;
#endif /* FMC_Bank5_6_R */

    return ( retState );
}


/**
 * \brief Puts an SDRAM bank in the self-refresh mode: the auto-refresh cycles are performed by the memory itself (the
 *        SDRAM clock stops after tRAS). The controller precharges all banks and sends the pending writes first. The
 *        call waits (bounded) until the controller reports the mode. \ref Fmc_Set_BankActive ends the mode.
 *
 * \param bank [in] : SDRAM bank that was initialized.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for an invalid bank, a bank that is not initialized, an
 *         uninitialized module or a controller that does not report the mode.
 */
fmc_RequestState_t Fmc_Set_SdramSelfRefresh( fmc_Bank_t bank )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;

#if defined(FMC_Bank5_6_R)
    retState = FmcPriv_Sdram_Enter( bank, FMC_SDRAM_COMMAND_SELF_REFRESH, FMC_SDSR_MODES_SELF_REFRESH );
#else
    (void)bank;
#endif /* FMC_Bank5_6_R */

    return ( retState );
}


/**
 * \brief Puts an SDRAM bank in the power-down mode: the controller refreshes the memory (exits the mode, auto-refresh,
 *        returns to it). The call waits (bounded) until the controller reports the mode.
 *        \ref Fmc_Set_BankActive ends the mode.
 *
 * \param bank [in] : SDRAM bank that was initialized.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for an invalid bank, a bank that is not initialized, an
 *         uninitialized module or a controller that does not report the mode.
 */
fmc_RequestState_t Fmc_Set_SdramPowerDown( fmc_Bank_t bank )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;

#if defined(FMC_Bank5_6_R)
    retState = FmcPriv_Sdram_Enter( bank, FMC_SDRAM_COMMAND_POWER_DOWN, FMC_SDSR_MODES_POWER_DOWN );
#else
    (void)bank;
#endif /* FMC_Bank5_6_R */

    return ( retState );
}


/**
 * \brief Sends a raw command to an SDRAM bank and waits (bounded) until the controller is ready. The command is
 *        executed for the bank only (the commands the reference manual requires for both banks simultaneously, the
 *        precharge all and the auto-refresh, are issued by \ref Fmc_Set_SdramInit). The bank must be configured.
 *
 * \param bank         [in] : SDRAM bank.
 * \param sdramCommand [in] : Pointer to the command: the auto-refresh count is used by the auto-refresh command and the
 *                            mode register by the load mode register command only.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for an invalid parameter, a bank that is not configured, an
 *         uninitialized module or a timeout.
 */
fmc_RequestState_t Fmc_Set_SdramCommand( fmc_Bank_t bank, const fmc_SdramCommand_t * const sdramCommand )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;

#if defined(FMC_Bank5_6_R)
    fmc_InstanceData_t * instance      = FMC_NULL_PTR;
    fmc_RequestState_t   instanceState = FmcPriv_Get_Instance( &instance );

    if( ( FMC_REQUEST_OK        == instanceState              ) &&
        ( FMC_NULL_PTR          != sdramCommand               ) &&
        ( FMC_BANK_SDRAM_1      <= bank                       ) &&
        ( FMC_BANK_SDRAM_2      >= bank                       ) &&
        ( FMC_FUNCTION_ACTIVE   == instance->Initialized      ) &&
        ( FMC_FUNCTION_ACTIVE   == instance->Configured[bank] ) &&
        ( FMC_SDRAM_COMMAND_CNT  > sdramCommand->Mode         )    )
    {
        const uint32_t     bankIdx    = ( (uint32_t)bank - (uint32_t)FMC_BANK_SDRAM_1 );
        fmc_RefreshCnt_t   refreshCnt = 1u;
        fmc_ModeRegister_t modeReg    = 0u;
        fmc_FlagState_t    valid      = FMC_FLAG_ACTIVE;

        if( FMC_SDRAM_COMMAND_AUTO_REFRESH == sdramCommand->Mode )
        {
            if( ( 1u                            <= sdramCommand->AutoRefreshCnt ) &&
                ( FMC_SDRAM_AUTO_REFRESH_CNT_MAX >= sdramCommand->AutoRefreshCnt )    )
            {
                refreshCnt = sdramCommand->AutoRefreshCnt;
            }
            else
            {
                /* The number of the auto-refresh cycles is not 1 .. 16 */
                valid = FMC_FLAG_INACTIVE;
            }
        }
        else
        {
            /* The auto-refresh count is not used by this command */
        }

        if( FMC_SDRAM_COMMAND_LOAD_MODE_REGISTER == sdramCommand->Mode )
        {
            if( FMC_SDRAM_MODE_REGISTER_MAX >= sdramCommand->ModeRegister )
            {
                modeReg = sdramCommand->ModeRegister;
            }
            else
            {
                /* The mode register has 13 bits */
                valid = FMC_FLAG_INACTIVE;
            }
        }
        else
        {
            /* The mode register is not used by this command */
        }

        if( FMC_FLAG_ACTIVE == valid )
        {
            retState = FmcPriv_Sdram_Send( sdramCommand->Mode, fmc_SdramTargetBit[bankIdx], refreshCnt, modeReg );
        }
        else
        {
            /* A parameter of the command is invalid, no command was issued */
            retState = FMC_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid parameter or state, no action required */
        retState = FMC_REQUEST_ERROR;
    }
#else
    (void)bank;
    (void)sdramCommand;
#endif /* FMC_Bank5_6_R */

    return ( retState );
}


/**
 * \brief Reads the state of an SDRAM bank: the controller is busy executing a command, or the bank is in the normal,
 *        self-refresh or power-down mode. The function has no side effect.
 *
 * \param bank        [in]  : SDRAM bank that was configured.
 * \param sdramState [out] : Pointer receiving the state.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for an invalid parameter, a bank that is not configured, an
 *         uninitialized module or a mode the module does not know.
 */
fmc_RequestState_t Fmc_Get_SdramState( fmc_Bank_t bank, fmc_SdramState_t * const sdramState )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;

#if defined(FMC_Bank5_6_R)
    fmc_InstanceData_t * instance      = FMC_NULL_PTR;
    fmc_RequestState_t   instanceState = FmcPriv_Get_Instance( &instance );

    if( ( FMC_REQUEST_OK       == instanceState              ) &&
        ( FMC_NULL_PTR         != sdramState                 ) &&
        ( FMC_BANK_SDRAM_1     <= bank                       ) &&
        ( FMC_BANK_SDRAM_2     >= bank                       ) &&
        ( FMC_FUNCTION_ACTIVE  == instance->Initialized      ) &&
        ( FMC_FUNCTION_ACTIVE  == instance->Configured[bank] )    )
    {
        const uint32_t       bankIdx   = ( (uint32_t)bank - (uint32_t)FMC_BANK_SDRAM_1 );
        const fmc_RegValue_t busyFlag  = Fmc_Get_RegBit( FMC_REG_SDSR, FMC_REG_SDSR_BUSY );
        const fmc_RegValue_t modesRead = ( Fmc_Get_RegVal( FMC_REG_SDSR, fmc_SdramModesMask[bankIdx] ) >>
                                           fmc_SdramModesPos[bankIdx] );

        if( FMC_REG_BITS_CLEAR != busyFlag )
        {
            *sdramState = FMC_SDRAM_BUSY;
            retState    = FMC_REQUEST_OK;
        }
        else if( FMC_SDSR_MODES_NORMAL == modesRead )
        {
            *sdramState = FMC_SDRAM_NORMAL;
            retState    = FMC_REQUEST_OK;
        }
        else if( FMC_SDSR_MODES_SELF_REFRESH == modesRead )
        {
            *sdramState = FMC_SDRAM_SELF_REFRESH;
            retState    = FMC_REQUEST_OK;
        }
        else if( FMC_SDSR_MODES_POWER_DOWN == modesRead )
        {
            *sdramState = FMC_SDRAM_POWER_DOWN;
            retState    = FMC_REQUEST_OK;
        }
        else
        {
            /* The controller reports a mode the module does not know */
            retState = FMC_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid parameter or state, no action required */
        retState = FMC_REQUEST_ERROR;
    }
#else
    (void)bank;
    (void)sdramState;
#endif /* FMC_Bank5_6_R */

    return ( retState );
}
