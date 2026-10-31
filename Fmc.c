/**
 * \author Mr.Nobody
 * \file Fmc.c
 * \ingroup Fmc
 * \brief Fmc module source file (module management, NOR / PSRAM / SRAM banks, interrupt, pins)
 *
 * The flexible memory controller of STM32H5 (STM32H523 and later, no FMC on STM32H503). The module
 * owns the controller: the clock, the interrupt, the pins and the registers. The banks are configured
 * by their functions (this file: the NOR / PSRAM / SRAM banks, Fmc_Sdram.c: the SDRAM banks, Fmc_Nand.c:
 * the NAND bank) and enabled by Fmc_Set_BankActive. Every timing is given in nanoseconds and converted
 * to the cycles of the bus clock (HCLK read from the Rcc module, rounded up, never approximated to a
 * shorter time); a combination the clock cannot meet is rejected before a register is written.
 *
 * Every register of the controller except the on-the-fly bits (see Fmc_Reg.c) is changed with the
 * controller disabled (RM0481 23.2): a configuration function disables the controller for its register
 * writes and enables it again, so no bank may be accessed while a configuration function runs.
 * On STM32H503, which has no FMC, every exported function checks the CMSIS macro (defined(FMC_Bank1_R))
 * and only returns FMC_REQUEST_ERROR (Fmc_Get_ModuleVersion returns the version).
 *
 */
/* ============================== INCLUDES ================================== */
#include "Fmc.h"                            /* Own private declarations       */
#include "Fmc_Reg.h"                        /* Module Reg component header    */
#include "Stm32.h"                          /* Device registers definition    */
#include "Rcc_Port.h"                       /* RCC Port module definitions    */
#include "Nvic_Port.h"                      /* NVIC Port module definitions   */
#include "Gpio_Port.h"                      /* GPIO Port module definitions   */

/* ========================== PRIVATE DECLARATIONS =========================== */

/** \brief Module version, major number */
#define FMC_VERSION_MAJOR                ( 1u )

/** \brief Module version, minor number */
#define FMC_VERSION_MINOR                ( 0u )

/** \brief Module version, patch number */
#define FMC_VERSION_PATCH                ( 0u )

#if defined(FMC_Bank1_R)

/** \brief FMC_BCRx MWID field value: 8-bit data bus */
#define FMC_MWID_8BIT                    ( 0u )

/** \brief FMC_BCRx MWID field value: 16-bit data bus */
#define FMC_MWID_16BIT                   ( 1u )

/** \brief FMC_BCRx MWID field value: 32-bit data bus (STM32H5E / H5F) */
#define FMC_MWID_32BIT                   ( 2u )

/** \brief FMC_BCRx MTYP field value: SRAM / FRAM */
#define FMC_MTYP_SRAM                    ( 0u )

/** \brief FMC_BCRx MTYP field value: PSRAM / FRAM */
#define FMC_MTYP_PSRAM                   ( 1u )

/** \brief FMC_BCRx MTYP field value: NOR flash */
#define FMC_MTYP_NOR                     ( 2u )

/** \brief Smallest ADDSET of a bank in cycles (SRAM, asynchronous) */
#define FMC_ADDSET_CYCLES_MIN            ( 0u )

/** \brief Smallest ADDSET of a multiplexed bank, a PSRAM and of the access mode D in cycles */
#define FMC_ADDSET_CYCLES_MIN_MUXED      ( 1u )

/** \brief Largest ADDSET in cycles */
#define FMC_ADDSET_CYCLES_MAX            ( 15u )

/** \brief Smallest ADDHLD in cycles */
#define FMC_ADDHLD_CYCLES_MIN            ( 1u )

/** \brief Largest ADDHLD in cycles */
#define FMC_ADDHLD_CYCLES_MAX            ( 15u )

/** \brief Smallest DATAST in cycles */
#define FMC_DATAST_CYCLES_MIN            ( 1u )

/** \brief Largest DATAST in cycles */
#define FMC_DATAST_CYCLES_MAX            ( 255u )

/** \brief Smallest BUSTURN in cycles (the register holds the cycles minus this value) */
#define FMC_BUSTURN_CYCLES_MIN           ( 1u )

/** \brief Largest BUSTURN in cycles */
#define FMC_BUSTURN_CYCLES_MAX           ( 16u )

/** \brief Largest DATAHLD of a read access in cycles (the register holds the cycles) */
#define FMC_DATAHLD_READ_CYCLES_MAX      ( 3u )

/** \brief Smallest DATAHLD of a write access in cycles (the register holds the cycles minus this value) */
#define FMC_DATAHLD_WRITE_CYCLES_MIN     ( 1u )

/** \brief Largest DATAHLD of a write access in cycles */
#define FMC_DATAHLD_WRITE_CYCLES_MAX     ( 4u )

/** \brief Largest NBLSET in cycles */
#define FMC_NBLSET_CYCLES_MAX            ( 3u )

/** \brief Smallest CLKDIV in cycles of the bus clock (the register holds the cycles minus this value) */
#define FMC_CLKDIV_CYCLES_MIN            ( 1u )

/** \brief Largest CLKDIV in cycles of the bus clock */
#define FMC_CLKDIV_CYCLES_MAX            ( 16u )

/** \brief Smallest DATLAT in cycles of FMC_CLK (the register holds the cycles minus this value) */
#define FMC_DATLAT_CYCLES_MIN            ( 2u )

/** \brief Largest DATLAT in cycles of FMC_CLK */
#define FMC_DATLAT_CYCLES_MAX            ( 17u )

/** \brief Largest value of the chip select counter */
#define FMC_CSCOUNT_MAX                  ( 0xFFFFu )

/** \brief Reset value of FMC_BWTRx (written when the extended mode is not used) */
#define FMC_BWTR_RESET_VALUE             ( 0x0FFFFFFFu )

/** \brief Bits of FMC_BCRx written by the configuration of a bank (not the bits common to the banks and not MBKEN) */
#define FMC_BCR_CONFIG_MASK              ( FMC_BCRx_MUXEN     | FMC_BCRx_MTYP      | FMC_BCRx_MWID      | \
                                           FMC_BCRx_FACCEN    | FMC_BCRx_BURSTEN   | FMC_BCRx_WAITPOL   | \
                                           FMC_BCRx_WAITCFG   | FMC_BCRx_WREN      | FMC_BCRx_WAITEN    | \
                                           FMC_BCRx_EXTMOD    | FMC_BCRx_ASYNCWAIT | FMC_BCRx_CPSIZE   | \
                                           FMC_BCRx_CBURSTRW  | FMC_BCRx_NBLSET )

/** \brief Bits of FMC_BCR1 common to the banks, written by Fmc_Init */
#define FMC_BCR1_COMMON_MASK             ( FMC_BCR1_WFDIS | FMC_BCR1_CCLKEN )

/** \brief All the chip select counter enable bits of FMC_PCSCNTR */
#define FMC_PCSCNTR_CNTBEN_MASK          ( FMC_PCSCNTR_CNTB1EN | FMC_PCSCNTR_CNTB2EN | \
                                           FMC_PCSCNTR_CNTB3EN | FMC_PCSCNTR_CNTB4EN )

#endif /* FMC_Bank1_R */

/* =========================== LOCAL VARIABLES ============================== */

#if defined(FMC_Bank1_R)

/** \brief Run-time state of the module */
static fmc_InstanceData_t fmc_Instance;

/** \brief Control register of every NOR / PSRAM / SRAM bank */
static const fmc_RegId_t fmc_NorBcrReg[ FMC_NOR_BANK_CNT ] =
{
    FMC_REG_BCR1, /**< NOR / PSRAM / SRAM bank 1 */
    FMC_REG_BCR2, /**< NOR / PSRAM / SRAM bank 2 */
    FMC_REG_BCR3, /**< NOR / PSRAM / SRAM bank 3 */
    FMC_REG_BCR4, /**< NOR / PSRAM / SRAM bank 4 */
};

_Static_assert( ( sizeof( fmc_NorBcrReg ) / sizeof( fmc_RegId_t ) ) == FMC_NOR_BANK_CNT,
                 "Fmc: fmc_NorBcrReg has incorrect size." );

/** \brief Timing register of every NOR / PSRAM / SRAM bank */
static const fmc_RegId_t fmc_NorBtrReg[ FMC_NOR_BANK_CNT ] =
{
    FMC_REG_BTR1, /**< NOR / PSRAM / SRAM bank 1 */
    FMC_REG_BTR2, /**< NOR / PSRAM / SRAM bank 2 */
    FMC_REG_BTR3, /**< NOR / PSRAM / SRAM bank 3 */
    FMC_REG_BTR4, /**< NOR / PSRAM / SRAM bank 4 */
};

_Static_assert( ( sizeof( fmc_NorBtrReg ) / sizeof( fmc_RegId_t ) ) == FMC_NOR_BANK_CNT,
                 "Fmc: fmc_NorBtrReg has incorrect size." );

/** \brief Write timing register of every NOR / PSRAM / SRAM bank */
static const fmc_RegId_t fmc_NorBwtrReg[ FMC_NOR_BANK_CNT ] =
{
    FMC_REG_BWTR1, /**< NOR / PSRAM / SRAM bank 1 */
    FMC_REG_BWTR2, /**< NOR / PSRAM / SRAM bank 2 */
    FMC_REG_BWTR3, /**< NOR / PSRAM / SRAM bank 3 */
    FMC_REG_BWTR4, /**< NOR / PSRAM / SRAM bank 4 */
};

_Static_assert( ( sizeof( fmc_NorBwtrReg ) / sizeof( fmc_RegId_t ) ) == FMC_NOR_BANK_CNT,
                 "Fmc: fmc_NorBwtrReg has incorrect size." );

/** \brief Chip select counter enable bit of every NOR / PSRAM / SRAM bank */
static const fmc_RegValue_t fmc_NorCsCounterBit[ FMC_NOR_BANK_CNT ] =
{
    FMC_PCSCNTR_CNTB1EN, /**< NOR / PSRAM / SRAM bank 1 */
    FMC_PCSCNTR_CNTB2EN, /**< NOR / PSRAM / SRAM bank 2 */
    FMC_PCSCNTR_CNTB3EN, /**< NOR / PSRAM / SRAM bank 3 */
    FMC_PCSCNTR_CNTB4EN, /**< NOR / PSRAM / SRAM bank 4 */
};

_Static_assert( ( sizeof( fmc_NorCsCounterBit ) / sizeof( fmc_RegValue_t ) ) == FMC_NOR_BANK_CNT,
                 "Fmc: fmc_NorCsCounterBit has incorrect size." );

/** \brief Base address of the memory window of every bank of the device line, indexed by fmc_Bank_t */
static const fmc_Address_t fmc_BankBase[ FMC_BANK_CNT ] =
{
    FMC_BANK1_1,        /**< FMC_BANK_NOR_PSRAM_1 */
    FMC_BANK1_2,        /**< FMC_BANK_NOR_PSRAM_2 */
    FMC_BANK1_3,        /**< FMC_BANK_NOR_PSRAM_3 */
    FMC_BANK1_4,        /**< FMC_BANK_NOR_PSRAM_4 */
    FMC_BANK3,          /**< FMC_BANK_NAND        */
#if defined(FMC_Bank5_6_R)
    FMC_SDRAM_BANK_1,   /**< FMC_BANK_SDRAM_1     */
    FMC_SDRAM_BANK_2,   /**< FMC_BANK_SDRAM_2     */
#endif
};

_Static_assert( ( sizeof( fmc_BankBase ) / sizeof( fmc_Address_t ) ) == FMC_BANK_CNT,
                 "Fmc: fmc_BankBase has incorrect size." );

/** \brief Limits of the ADDSET phase of an asynchronous bank (cycles of the bus clock) */
static const fmc_PhaseLimit_t fmc_LimAddSet = { FMC_ADDSET_CYCLES_MIN, FMC_ADDSET_CYCLES_MAX };

/** \brief Limits of the ADDSET phase of a multiplexed bank, a PSRAM and of the access mode D */
static const fmc_PhaseLimit_t fmc_LimAddSetMux = { FMC_ADDSET_CYCLES_MIN_MUXED, FMC_ADDSET_CYCLES_MAX };

/** \brief Limits of the ADDHLD phase */
static const fmc_PhaseLimit_t fmc_LimAddHld = { FMC_ADDHLD_CYCLES_MIN, FMC_ADDHLD_CYCLES_MAX };

/** \brief Limits of the DATAST phase */
static const fmc_PhaseLimit_t fmc_LimDataSt = { FMC_DATAST_CYCLES_MIN, FMC_DATAST_CYCLES_MAX };

/** \brief Limits of the BUSTURN phase */
static const fmc_PhaseLimit_t fmc_LimBusTurn = { FMC_BUSTURN_CYCLES_MIN, FMC_BUSTURN_CYCLES_MAX };

/** \brief Limits of the DATAHLD phase of a read access */
static const fmc_PhaseLimit_t fmc_LimDataHldRd = { 0u, FMC_DATAHLD_READ_CYCLES_MAX };

/** \brief Limits of the DATAHLD phase of a write access */
static const fmc_PhaseLimit_t fmc_LimDataHldWr = { FMC_DATAHLD_WRITE_CYCLES_MIN, FMC_DATAHLD_WRITE_CYCLES_MAX };

/** \brief Limits of the NBLSET phase */
static const fmc_PhaseLimit_t fmc_LimNblSet = { 0u, FMC_NBLSET_CYCLES_MAX };

/** \brief Limits of the FMC_CLK period in cycles of the bus clock (CLKDIV) */
static const fmc_PhaseLimit_t fmc_LimClkDiv = { FMC_CLKDIV_CYCLES_MIN, FMC_CLKDIV_CYCLES_MAX };

/** \brief Limits of the chip select counter in cycles of the bus clock */
static const fmc_PhaseLimit_t fmc_LimCsCount = { 1u, FMC_CSCOUNT_MAX };

#endif /* FMC_Bank1_R */

/* ========================== PRIVATE FUNCTIONS ============================== */

#if defined(FMC_Bank1_R)

/**
 * \brief Returns a bit mask when a function state has the requested value, otherwise no bit.
 *
 * \param state      [in] : State to check.
 * \param matchState [in] : State for which the mask is returned.
 * \param bitMask    [in] : Mask returned for the matching state.
 *
 * \return bitMask when state equals matchState, otherwise 0.
 */
fmc_RegValue_t FmcPriv_Get_StateMask( fmc_FunctionState_t state,
                                      fmc_FunctionState_t matchState,
                                      fmc_RegValue_t bitMask )
{
    fmc_RegValue_t retValue = FMC_REG_BITS_CLEAR;

    if( state == matchState )
    {
        retValue = bitMask;
    }
    else
    {
        /* The state does not match, no bit is set */
        retValue = FMC_REG_BITS_CLEAR;
    }

    return ( retValue );
}


/**
 * \brief Converts a data width item to the MWID register field value.
 *
 * \param dataWidth [in]  : Data width item.
 * \param widthField [out] : Pointer receiving the MWID field value.
 *
 * \return FMC_REQUEST_OK for a width the device line supports, FMC_REQUEST_ERROR otherwise.
 */
fmc_RequestState_t FmcPriv_Get_WidthField( fmc_DataWidth_t dataWidth, fmc_RegValue_t * const widthField )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;

    if( FMC_NULL_PTR == widthField )
    {
        /* Invalid parameter, retState stays FMC_REQUEST_ERROR */
        retState = FMC_REQUEST_ERROR;
    }
    else if( FMC_DATA_WIDTH_8 == dataWidth )
    {
        *widthField = FMC_MWID_8BIT;
        retState    = FMC_REQUEST_OK;
    }
    else if( FMC_DATA_WIDTH_16 == dataWidth )
    {
        *widthField = FMC_MWID_16BIT;
        retState    = FMC_REQUEST_OK;
    }
    else
    {
#if defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
        if( FMC_DATA_WIDTH_32 == dataWidth )
        {
            *widthField = FMC_MWID_32BIT;
            retState    = FMC_REQUEST_OK;
        }
        else
        {
            /* Width not supported by the device line */
            retState = FMC_REQUEST_ERROR;
        }
#else
        /* Width not supported by the device line */
        retState = FMC_REQUEST_ERROR;
#endif
    }

    return ( retState );
}


/**
 * \brief Checks the enumerated items and the combinations of a NOR / PSRAM / SRAM configuration.
 *
 * \param config [in] : Configuration to check, not null.
 *
 * \return FMC_REQUEST_OK for a valid configuration, FMC_REQUEST_ERROR otherwise.
 */
static fmc_RequestState_t FmcPriv_Check_NorSramConfig( const fmc_NorSramConfig_t * const config )
{
    fmc_RequestState_t       retState   = FMC_REQUEST_ERROR;
    fmc_RegValue_t           widthField = 0u;
    fmc_FlagState_t          violation  = FMC_FLAG_INACTIVE;
    fmc_FlagState_t          busCapable = FMC_FLAG_INACTIVE;
    const fmc_RequestState_t widthState = FmcPriv_Get_WidthField( config->DataWidth, &widthField );

    if( ( FMC_REQUEST_OK       == widthState             ) &&
        ( FMC_MEMORY_CNT        > config->MemoryType     ) &&
        ( FMC_FUNCTION_ACTIVE  >= config->AddressDataMux ) &&
        ( FMC_FUNCTION_ACTIVE  >= config->BurstAccess    ) &&
        ( FMC_FUNCTION_ACTIVE  >= config->WriteBurst     ) &&
        ( FMC_FUNCTION_ACTIVE  >= config->WriteProtect   ) &&
        ( FMC_FUNCTION_ACTIVE  >= config->WaitSignal     ) &&
        ( FMC_FUNCTION_ACTIVE  >= config->AsyncWait      ) &&
        ( FMC_WAIT_POLARITY_CNT > config->WaitPolarity   ) &&
        ( FMC_WAIT_TIMING_CNT   > config->WaitTiming     ) &&
        ( FMC_FUNCTION_ACTIVE  >= config->ExtendedMode   ) &&
        ( FMC_ACCESS_MODE_CNT   > config->AccessMode     ) &&
        ( FMC_PAGE_SIZE_CNT     > config->PageSize       )    )
    {
        if( ( FMC_MEMORY_PSRAM == config->MemoryType ) ||
            ( FMC_MEMORY_NOR   == config->MemoryType )    )
        {
            busCapable = FMC_FLAG_ACTIVE;
        }
        else
        {
            /* SRAM and FRAM have neither the multiplexed bus nor the synchronous access */
            busCapable = FMC_FLAG_INACTIVE;
        }

        if( ( FMC_FUNCTION_ACTIVE == config->AddressDataMux ) &&
            ( FMC_FLAG_INACTIVE   == busCapable             )    )
        {
            violation = FMC_FLAG_ACTIVE;
        }
        else
        {
            /* Multiplexing is possible for this memory type */
        }

        if( ( FMC_FUNCTION_ACTIVE == config->BurstAccess ) &&
            ( FMC_FLAG_INACTIVE   == busCapable          )    )
        {
            violation = FMC_FLAG_ACTIVE;
        }
        else
        {
            /* The burst access is possible for this memory type */
        }

        if( ( ( FMC_FUNCTION_ACTIVE == config->WriteBurst   ) ||
              ( FMC_PAGE_SIZE_NONE  != config->PageSize     ) ||
              ( 0u                  != config->ChipSelectMaxLow_ns ) ) &&
            ( FMC_MEMORY_PSRAM      != config->MemoryType   )              )
        {
            violation = FMC_FLAG_ACTIVE;
        }
        else
        {
            /* The write burst, the page size and the chip select counter belong to the PSRAM only */
        }

        if( FMC_FLAG_INACTIVE == violation )
        {
            retState = FMC_REQUEST_OK;
        }
        else
        {
            /* The combination is not possible */
            retState = FMC_REQUEST_ERROR;
        }
    }
    else
    {
        /* An item is out of its list */
        retState = FMC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Converts the NOR / PSRAM / SRAM configuration to the control register value.
 *
 * \param config [in]  : Checked configuration.
 * \param clock  [in]  : Clock of the controller (the bus clock).
 * \param bcr    [out] : Pointer receiving the control register value.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR when the byte lane setup time does not fit.
 */
static fmc_RequestState_t FmcPriv_Get_NorSramControl( const fmc_NorSramConfig_t * const config,
                                                      const fmc_Clock_t * const clock,
                                                      fmc_RegValue_t * const bcr )
{
    fmc_RequestState_t retState   = FMC_REQUEST_ERROR;
    fmc_RegValue_t     widthField = 0u;
    fmc_RegValue_t     typeField  = FMC_MTYP_SRAM;
    fmc_CycleCnt_t     nblSet     = 0u;
    const fmc_RequestState_t widthState = FmcPriv_Get_WidthField( config->DataWidth, &widthField );
    const fmc_RequestState_t nblState   = FmcPriv_Get_PhaseCycles( config->ByteLaneSetup_ns,
                                                                    clock,
                                                                    &fmc_LimNblSet,
                                                                    &nblSet );

    if( ( FMC_REQUEST_OK == widthState ) &&
        ( FMC_REQUEST_OK == nblState   )    )
    {
        if( FMC_MEMORY_PSRAM == config->MemoryType )
        {
            typeField = FMC_MTYP_PSRAM;
        }
        else if( FMC_MEMORY_NOR == config->MemoryType )
        {
            typeField = FMC_MTYP_NOR;
        }
        else
        {
            /* SRAM and FRAM use the SRAM type */
            typeField = FMC_MTYP_SRAM;
        }

        *bcr = ( ( widthField                       << FMC_BCRx_MWID_Pos   ) |
                 ( typeField                        << FMC_BCRx_MTYP_Pos   ) |
                 ( (fmc_RegValue_t)config->PageSize << FMC_BCRx_CPSIZE_Pos ) |
                 ( nblSet                           << FMC_BCRx_NBLSET_Pos ) |
                 FmcPriv_Get_StateMask( config->AddressDataMux, FMC_FUNCTION_ACTIVE,   FMC_BCRx_MUXEN     ) |
                 FmcPriv_Get_StateMask( config->BurstAccess,    FMC_FUNCTION_ACTIVE,   FMC_BCRx_BURSTEN   ) |
                 FmcPriv_Get_StateMask( config->WriteBurst,     FMC_FUNCTION_ACTIVE,   FMC_BCRx_CBURSTRW  ) |
                 FmcPriv_Get_StateMask( config->WriteProtect,   FMC_FUNCTION_INACTIVE, FMC_BCRx_WREN      ) |
                 FmcPriv_Get_StateMask( config->WaitSignal,     FMC_FUNCTION_ACTIVE,   FMC_BCRx_WAITEN    ) |
                 FmcPriv_Get_StateMask( config->AsyncWait,      FMC_FUNCTION_ACTIVE,   FMC_BCRx_ASYNCWAIT ) |
                 FmcPriv_Get_StateMask( config->ExtendedMode,   FMC_FUNCTION_ACTIVE,   FMC_BCRx_EXTMOD    ) );

        if( FMC_WAIT_POLARITY_HIGH == config->WaitPolarity )
        {
            *bcr |= FMC_BCRx_WAITPOL;
        }
        else
        {
            /* NWAIT is active low */
        }

        if( FMC_WAIT_TIMING_DURING_STATE == config->WaitTiming )
        {
            *bcr |= FMC_BCRx_WAITCFG;
        }
        else
        {
            /* NWAIT is active one data cycle before the wait state */
        }

        if( FMC_MEMORY_NOR == config->MemoryType )
        {
            *bcr |= FMC_BCRx_FACCEN;
        }
        else
        {
            /* The NOR flash access enable belongs to the NOR flash only */
        }

        retState = FMC_REQUEST_OK;
    }
    else
    {
        /* The byte lane setup time or the data width is not valid */
        retState = FMC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Converts the NOR / PSRAM / SRAM configuration to the timing register value (read timing).
 *
 * \param config [in]  : Checked configuration.
 * \param clock  [in]  : Clock of the controller (the bus clock).
 * \param btr    [out] : Pointer receiving the timing register value.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR when a time or a count does not fit its field.
 */
static fmc_RequestState_t FmcPriv_Get_NorSramReadTiming( const fmc_NorSramConfig_t * const config,
                                                         const fmc_Clock_t * const clock,
                                                         fmc_RegValue_t * const btr )
{
    fmc_RequestState_t       retState    = FMC_REQUEST_ERROR;
    const fmc_PhaseLimit_t * addSetLimit = &fmc_LimAddSet;
    fmc_CycleCnt_t           addSet      = 0u;
    fmc_CycleCnt_t           addHld      = 0u;
    fmc_CycleCnt_t           dataSt      = 0u;
    fmc_CycleCnt_t           busTurn     = 0u;
    fmc_CycleCnt_t           dataHld     = 0u;
    fmc_CycleCnt_t           clkDiv      = FMC_CLKDIV_CYCLES_MIN;
    fmc_CycleCnt_t           datLat      = FMC_DATLAT_CYCLES_MIN;
    fmc_RequestState_t       clkState    = FMC_REQUEST_OK;
    fmc_RegValue_t           accMode     = FMC_REG_BITS_CLEAR;

    if( ( FMC_FUNCTION_ACTIVE == config->AddressDataMux ) ||
        ( FMC_MEMORY_PSRAM    == config->MemoryType     ) ||
        ( ( FMC_FUNCTION_ACTIVE == config->ExtendedMode ) &&
          ( FMC_ACCESS_MODE_D   == config->AccessMode   )    ) )
    {
        addSetLimit = &fmc_LimAddSetMux;
    }
    else
    {
        /* The address setup time may be zero cycles */
        addSetLimit = &fmc_LimAddSet;
    }

    const fmc_Phase_t phases[] =
    {
        { config->AddressSetup_ns,  addSetLimit,       &addSet  }, /**< ADDSET  */
        { config->AddressHold_ns,   &fmc_LimAddHld,    &addHld  }, /**< ADDHLD  */
        { config->DataSetup_ns,     &fmc_LimDataSt,    &dataSt  }, /**< DATAST  */
        { config->BusTurnaround_ns, &fmc_LimBusTurn,   &busTurn }, /**< BUSTURN */
        { config->DataHold_ns,      &fmc_LimDataHldRd, &dataHld }, /**< DATAHLD */
    };
    const fmc_RequestState_t phaseState = FmcPriv_Get_Phases( clock, phases, FMC_PHASE_CNT( phases ) );

    if( ( FMC_FUNCTION_ACTIVE == config->BurstAccess ) ||
        ( FMC_FUNCTION_ACTIVE == config->WriteBurst  )    )
    {
        clkState = FmcPriv_Get_PhaseCycles( config->ClockPeriod_ns, clock, &fmc_LimClkDiv, &clkDiv );
        datLat   = config->DataLatency;
    }
    else
    {
        /* Asynchronous access: the clock divider and the data latency are not used */
        clkState = FMC_REQUEST_OK;
    }

    if( ( FMC_REQUEST_OK        == phaseState ) &&
        ( FMC_REQUEST_OK        == clkState   ) &&
        ( FMC_DATLAT_CYCLES_MIN <= datLat     ) &&
        ( FMC_DATLAT_CYCLES_MAX >= datLat     )    )
    {
        accMode = FmcPriv_Get_StateMask( config->ExtendedMode,
                                         FMC_FUNCTION_ACTIVE,
                                         ( (fmc_RegValue_t)config->AccessMode << FMC_BTRx_ACCMOD_Pos ) );

        *btr = ( ( addSet                               << FMC_BTRx_ADDSET_Pos  ) |
                 ( addHld                               << FMC_BTRx_ADDHLD_Pos  ) |
                 ( dataSt                               << FMC_BTRx_DATAST_Pos  ) |
                 ( ( busTurn - FMC_BUSTURN_CYCLES_MIN ) << FMC_BTRx_BUSTURN_Pos ) |
                 ( ( clkDiv  - FMC_CLKDIV_CYCLES_MIN  ) << FMC_BTRx_CLKDIV_Pos  ) |
                 ( ( datLat  - FMC_DATLAT_CYCLES_MIN  ) << FMC_BTRx_DATLAT_Pos  ) |
                 ( dataHld                              << FMC_BTRx_DATAHLD_Pos ) |
                 accMode );
        retState = FMC_REQUEST_OK;
    }
    else
    {
        /* A time does not fit its field */
        retState = FMC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Converts the NOR / PSRAM / SRAM configuration to the write timing register value.
 *
 * \param config [in]  : Checked configuration.
 * \param clock  [in]  : Clock of the controller (the bus clock).
 * \param bwtr   [out] : Pointer receiving the write timing register value (the reset value without extended mode).
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR when a time does not fit its field.
 */
static fmc_RequestState_t FmcPriv_Get_NorSramWriteTiming( const fmc_NorSramConfig_t * const config,
                                                          const fmc_Clock_t * const clock,
                                                          fmc_RegValue_t * const bwtr )
{
    fmc_RequestState_t       retState    = FMC_REQUEST_ERROR;
    const fmc_PhaseLimit_t * addSetLimit = &fmc_LimAddSet;
    fmc_CycleCnt_t           addSet      = 0u;
    fmc_CycleCnt_t           addHld      = 0u;
    fmc_CycleCnt_t           dataSt      = 0u;
    fmc_CycleCnt_t           busTurn     = 0u;
    fmc_CycleCnt_t           dataHld     = 0u;

    if( FMC_FUNCTION_ACTIVE == config->ExtendedMode )
    {
        const fmc_NorSramWriteTiming_t * const write = &( config->WriteTiming );

        if( ( FMC_FUNCTION_ACTIVE == config->AddressDataMux ) ||
            ( FMC_MEMORY_PSRAM    == config->MemoryType     ) ||
            ( FMC_ACCESS_MODE_D   == config->AccessMode     )    )
        {
            addSetLimit = &fmc_LimAddSetMux;
        }
        else
        {
            /* The address setup time may be zero cycles */
            addSetLimit = &fmc_LimAddSet;
        }

        const fmc_Phase_t phases[] =
        {
            { write->AddressSetup_ns,  addSetLimit,       &addSet  }, /**< ADDSET  */
            { write->AddressHold_ns,   &fmc_LimAddHld,    &addHld  }, /**< ADDHLD  */
            { write->DataSetup_ns,     &fmc_LimDataSt,    &dataSt  }, /**< DATAST  */
            { write->BusTurnaround_ns, &fmc_LimBusTurn,   &busTurn }, /**< BUSTURN */
            { write->DataHold_ns,      &fmc_LimDataHldWr, &dataHld }, /**< DATAHLD */
        };
        const fmc_RequestState_t phaseState = FmcPriv_Get_Phases( clock, phases, FMC_PHASE_CNT( phases ) );

        if( FMC_REQUEST_OK == phaseState )
        {
            *bwtr = ( ( addSet                                    << FMC_BWTRx_ADDSET_Pos  ) |
                      ( addHld                                    << FMC_BWTRx_ADDHLD_Pos  ) |
                      ( dataSt                                    << FMC_BWTRx_DATAST_Pos  ) |
                      ( ( busTurn - FMC_BUSTURN_CYCLES_MIN )      << FMC_BWTRx_BUSTURN_Pos ) |
                      ( ( dataHld - FMC_DATAHLD_WRITE_CYCLES_MIN ) << FMC_BWTRx_DATAHLD_Pos ) |
                      ( (fmc_RegValue_t)config->AccessMode        << FMC_BWTRx_ACCMOD_Pos  ) );
            retState = FMC_REQUEST_OK;
        }
        else
        {
            /* A write time does not fit its field */
            retState = FMC_REQUEST_ERROR;
        }
    }
    else
    {
        /* No extended mode: the write timing register keeps its reset value */
        *bwtr    = FMC_BWTR_RESET_VALUE;
        retState = FMC_REQUEST_OK;
    }

    return ( retState );
}


/**
 * \brief Writes the converted registers of a NOR / PSRAM / SRAM bank with the controller disabled and verifies them.
 *
 * \param bank [in] : NOR / PSRAM / SRAM bank, valid.
 * \param regs [in] : Register values of the bank.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR when the controller or a register does not follow.
 */
static fmc_RequestState_t FmcPriv_Write_NorSramRegs( fmc_Bank_t bank, const fmc_NorSramRegs_t * const regs )
{
    fmc_RequestState_t  retState     = FMC_REQUEST_ERROR;
    fmc_FunctionState_t wasActive    = FMC_FUNCTION_INACTIVE;
    fmc_RegValue_t      csCountMask  = ( FMC_PCSCNTR_CSCOUNT_Msk );
    fmc_RegValue_t      csCountValue = FMC_REG_BITS_CLEAR;
    const fmc_RegValue_t csCountBit  = fmc_NorCsCounterBit[bank];
    const fmc_RegValue_t csOtherBits = Fmc_Get_RegBit( FMC_REG_PCSCNTR, ( FMC_PCSCNTR_CNTBEN_MASK & ~csCountBit ) );
    const fmc_RegValue_t csCurrent   = ( Fmc_Get_RegVal( FMC_REG_PCSCNTR, FMC_PCSCNTR_CSCOUNT_Msk ) >>
                                         FMC_PCSCNTR_CSCOUNT_Pos );

    if( ( FMC_FUNCTION_ACTIVE == regs->CsCounter ) &&
        ( FMC_REG_BITS_CLEAR  != csOtherBits     ) &&
        ( regs->CsCount       != csCurrent       )    )
    {
        /* The chip select count is common to the banks and another bank uses a different one */
        retState = FMC_REQUEST_ERROR;
    }
    else
    {
        const fmc_RequestState_t suspendState = FmcPriv_Controller_Suspend( &wasActive );

        if( FMC_REQUEST_OK == suspendState )
        {
            Fmc_Set_RegVal( fmc_NorBtrReg[bank], FMC_REG_MASK_ALL, regs->Btr );
            Fmc_Set_RegVal( fmc_NorBwtrReg[bank], FMC_REG_MASK_ALL, regs->Bwtr );
            Fmc_Set_RegVal( fmc_NorBcrReg[bank], FMC_BCR_CONFIG_MASK, regs->Bcr );

            if( FMC_FUNCTION_ACTIVE == regs->CsCounter )
            {
                csCountValue = ( ( regs->CsCount << FMC_PCSCNTR_CSCOUNT_Pos ) | csCountBit );
                csCountMask  = ( csCountMask | csCountBit );
            }
            else
            {
                /* The counter is not used by this bank, its enable bit is cleared */
                csCountValue = FMC_REG_BITS_CLEAR;
                csCountMask  = csCountBit;
            }
            Fmc_Set_RegVal( FMC_REG_PCSCNTR, csCountMask, csCountValue );

            const fmc_RegValue_t btrBack  = Fmc_Get_RegVal( fmc_NorBtrReg[bank], FMC_REG_MASK_ALL );
            const fmc_RegValue_t bwtrBack = Fmc_Get_RegVal( fmc_NorBwtrReg[bank], FMC_REG_MASK_ALL );
            const fmc_RegValue_t bcrBack  = Fmc_Get_RegVal( fmc_NorBcrReg[bank], FMC_BCR_CONFIG_MASK );
            const fmc_RequestState_t resumeState = FmcPriv_Controller_Resume( wasActive );

            if( ( FMC_REQUEST_OK == resumeState ) &&
                ( regs->Btr      == btrBack     ) &&
                ( regs->Bwtr     == bwtrBack    ) &&
                ( regs->Bcr      == bcrBack     )    )
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
    }

    return ( retState );
}


/**
 * \brief Configures one pin as the FMC alternate function decoded from an encoded pin value.
 *
 * \param pinCode     [in] : Encoded pin value of a pin item.
 * \param firstSignal [in] : First signal of the list the pin item belongs to.
 * \param lastSignal  [in] : Last signal of the list the pin item belongs to.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for a pin outside the list or a Gpio rejection.
 */
static fmc_RequestState_t FmcPriv_Init_AltFuncGpio( uint32_t pinCode,
                                                    fmc_Signal_t firstSignal,
                                                    fmc_Signal_t lastSignal )
{
    fmc_RequestState_t  retState = FMC_REQUEST_ERROR;
    gpio_RequestState_t gpioState = GPIO_REQUEST_ERROR;
    gpio_Config_t       gpioConfig;
    const uint32_t      signalId = FMC_PIN_BIT_MASK_DECODE_SIGNAL( pinCode );
    const uint32_t      portId   = FMC_PIN_BIT_MASK_DECODE_PORT( pinCode );
    const uint32_t      pinId    = FMC_PIN_BIT_MASK_DECODE_PIN( pinCode );
    const uint32_t      altFunc  = FMC_PIN_BIT_MASK_DECODE_AF( pinCode );

    if( ( (uint32_t)firstSignal <= signalId       ) &&
        ( (uint32_t)lastSignal  >= signalId       ) &&
        ( GPIO_PORT_CNT          > portId         ) &&
        ( GPIO_PIN_ID_CNT        > pinId          ) &&
        ( GPIO_ALT_FUNC_CNT      > altFunc        )    )
    {
        gpioConfig.PortId         = (gpio_PortId_t)portId;
        gpioConfig.PinId          = (gpio_PinId_t)pinId;
        gpioConfig.PinMode        = GPIO_PIN_MODE_ALTERNATE;
        gpioConfig.PinPull        = GPIO_PIN_PULL_NONE;
        gpioConfig.PinSpeed       = GPIO_PIN_SPEED_VERY_HIGH;
        gpioConfig.PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
        gpioConfig.PinAltFunction = (gpio_AltFunction_t)altFunc;
        gpioConfig.PinActiveLevel = GPIO_PIN_LEVEL_HIGH;

        if( ( (uint32_t)FMC_SIGNAL_NWAIT == signalId ) ||
            ( (uint32_t)FMC_SIGNAL_INT   == signalId )    )
        {
            /* The wait / ready signals of the memories are active low, the idle level is high */
            gpioConfig.PinPull = GPIO_PIN_PULL_UP;
        }
        else
        {
            /* No pull resistor on the address, data and strobe signals */
            gpioConfig.PinPull = GPIO_PIN_PULL_NONE;
        }

        gpioState = Gpio_Init( &gpioConfig );

        if( GPIO_REQUEST_OK == gpioState )
        {
            retState = FMC_REQUEST_OK;
        }
        else
        {
            /* Gpio rejected the configuration */
            retState = FMC_REQUEST_ERROR;
        }
    }
    else
    {
        /* The value is not an item of the list (or the unused item) */
        retState = FMC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Serves the events of the controller (NAND ready signal and SDRAM refresh error), called by the interrupt
 *        and by Fmc_Task.
 */
static void FmcPriv_ServeEvents( void )
{
    FmcPriv_Nand_ServeEvents();
#if defined(FMC_Bank5_6_R)
    FmcPriv_Sdram_ServeEvents();
#endif
}


/**
 * \brief Interrupt service routine of the module.
 */
static void FmcPriv_Isr( void )
{
    FmcPriv_ServeEvents();
}


/**
 * \brief Resets the run-time state of the module (the module is not initialized afterwards).
 *
 * \param instance [in] : Run-time state to reset.
 */
static void FmcPriv_Clear_Instance( fmc_InstanceData_t * const instance )
{
    instance->Initialized    = FMC_FUNCTION_INACTIVE;
    instance->NandEccStarted = FMC_FUNCTION_INACTIVE;

    for( uint32_t idx = 0u; ( FMC_BANK_CNT > idx ); idx++ )
    {
        instance->Configured[idx] = FMC_FUNCTION_INACTIVE;
        instance->Active[idx]     = FMC_FUNCTION_INACTIVE;
    }
#if defined(FMC_Bank5_6_R)
    for( uint32_t idx = 0u; ( FMC_SDRAM_BANK_CNT > idx ); idx++ )
    {
        instance->SdramReady[idx] = FMC_FUNCTION_INACTIVE;
    }
#endif
}


/**
 * \brief Checks the items of the module configuration.
 *
 * \param fmcConfig [in] : Configuration to check, not null.
 *
 * \return FMC_REQUEST_OK for a valid configuration, FMC_REQUEST_ERROR otherwise.
 */
static fmc_RequestState_t FmcPriv_Check_Config( const fmc_Config_t * const fmcConfig )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;

    if( ( FMC_FUNCTION_ACTIVE >= fmcConfig->WriteFifo       ) &&
        ( FMC_FUNCTION_ACTIVE >= fmcConfig->ContinuousClock ) &&
        ( FMC_EVENT_MODE_CNT   > fmcConfig->EventMode       )    )
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
 * \brief Configures the pins of the lists of the module configuration.
 *
 * \param fmcConfig [in] : Module configuration.
 *
 * \return FMC_REQUEST_OK when every listed pin was configured, FMC_REQUEST_ERROR otherwise.
 */
static fmc_RequestState_t FmcPriv_Init_ConfigPins( const fmc_Config_t * const fmcConfig )
{
    fmc_RequestState_t retState = FMC_REQUEST_OK;
    fmc_RequestState_t pinState = FMC_REQUEST_OK;

    for( uint32_t idx = 0u; ( FMC_ADDRESS_LINE_CNT > idx ); idx++ )
    {
        if( FMC_ADDRESS_PIN_UNUSED != fmcConfig->AddressPin[idx] )
        {
            pinState = Fmc_InitAddressGpio( fmcConfig->AddressPin[idx] );
            if( FMC_REQUEST_OK != pinState )
            {
                retState = FMC_REQUEST_ERROR;
            }
            else
            {
                /* The pin is configured */
            }
        }
        else
        {
            /* Address line not used by the board, no action required */
        }
    }

    for( uint32_t idx = 0u; ( FMC_DATA_LINE_CNT > idx ); idx++ )
    {
        if( FMC_DATA_PIN_UNUSED != fmcConfig->DataPin[idx] )
        {
            pinState = Fmc_InitDataGpio( fmcConfig->DataPin[idx] );
            if( FMC_REQUEST_OK != pinState )
            {
                retState = FMC_REQUEST_ERROR;
            }
            else
            {
                /* The pin is configured */
            }
        }
        else
        {
            /* Data line not used by the board, no action required */
        }
    }

    for( uint32_t idx = 0u; ( FMC_CONTROL_PIN_CNT > idx ); idx++ )
    {
        if( FMC_CONTROL_PIN_UNUSED != fmcConfig->ControlPin[idx] )
        {
            pinState = Fmc_InitControlGpio( fmcConfig->ControlPin[idx] );
            if( FMC_REQUEST_OK != pinState )
            {
                retState = FMC_REQUEST_ERROR;
            }
            else
            {
                /* The pin is configured */
            }
        }
        else
        {
            /* Control signal not used by the board, no action required */
        }
    }

    for( uint32_t idx = 0u; ( FMC_SDRAM_PIN_CNT > idx ); idx++ )
    {
        if( FMC_SDRAM_PIN_UNUSED != fmcConfig->SdramPin[idx] )
        {
            pinState = Fmc_InitSdramGpio( fmcConfig->SdramPin[idx] );
            if( FMC_REQUEST_OK != pinState )
            {
                retState = FMC_REQUEST_ERROR;
            }
            else
            {
                /* The pin is configured */
            }
        }
        else
        {
            /* SDRAM signal not used by the board, no action required */
        }
    }

    return ( retState );
}


/**
 * \brief Configures the banks of the module configuration (the banks without a configuration are skipped).
 *
 * \param fmcConfig [in] : Module configuration (the original, with the bank configuration pointers).
 *
 * \return FMC_REQUEST_OK when every given bank was configured, FMC_REQUEST_ERROR otherwise.
 */
static fmc_RequestState_t FmcPriv_Init_ConfigBanks( const fmc_Config_t * const fmcConfig )
{
    fmc_RequestState_t retState  = FMC_REQUEST_OK;
    fmc_RequestState_t bankState = FMC_REQUEST_OK;

    for( uint32_t idx = 0u; ( FMC_NOR_BANK_CNT > idx ); idx++ )
    {
        if( FMC_NULL_PTR != fmcConfig->NorSramConfig[idx] )
        {
            bankState = Fmc_Set_NorSramConfig( (fmc_Bank_t)idx, fmcConfig->NorSramConfig[idx] );
            if( FMC_REQUEST_OK != bankState )
            {
                retState = FMC_REQUEST_ERROR;
            }
            else
            {
                /* The bank is configured */
            }
        }
        else
        {
            /* Bank not configured at the start, no action required */
        }
    }

    if( FMC_NULL_PTR != fmcConfig->NandConfig )
    {
        bankState = Fmc_Set_NandConfig( fmcConfig->NandConfig );
        if( FMC_REQUEST_OK != bankState )
        {
            retState = FMC_REQUEST_ERROR;
        }
        else
        {
            /* The bank is configured */
        }
    }
    else
    {
        /* NAND bank not configured at the start, no action required */
    }

#if defined(FMC_Bank5_6_R)
    for( uint32_t idx = 0u; ( FMC_SDRAM_BANK_CNT > idx ); idx++ )
    {
        if( FMC_NULL_PTR != fmcConfig->SdramConfig[idx] )
        {
            bankState = Fmc_Set_SdramConfig( (fmc_Bank_t)( (uint32_t)FMC_BANK_SDRAM_1 + idx ),
                                             fmcConfig->SdramConfig[idx] );
            if( FMC_REQUEST_OK != bankState )
            {
                retState = FMC_REQUEST_ERROR;
            }
            else
            {
                /* The bank is configured */
            }
        }
        else
        {
            /* Bank not configured at the start, no action required */
        }
    }
#endif /* FMC_Bank5_6_R */

    return ( retState );
}

#endif /* FMC_Bank1_R */

/* ========================= EXPORTED FUNCTIONS =============================== */

#if defined(FMC_Bank1_R)

/**
 * \brief Returns the run-time state of the module (private, shared by the files of the module).
 *
 * \param instance [out] : Pointer receiving the address of the run-time state.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for a null pointer.
 */
fmc_RequestState_t FmcPriv_Get_Instance( fmc_InstanceData_t ** const instance )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;

    if( FMC_NULL_PTR != instance )
    {
        *instance = &fmc_Instance;
        retState  = FMC_REQUEST_OK;
    }
    else
    {
        /* Invalid parameter, no action required */
        retState = FMC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads the bus clock (HCLK) of the controller from the Rcc module.
 *
 * \param busClk [out] : Pointer receiving the bus clock in Hz.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for a null pointer or when the clock is not available.
 */
fmc_RequestState_t FmcPriv_Get_BusClock( fmc_Frequency_Hz_t * const busClk )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;
    rcc_FreqHz_t       clk      = 0u;
    rcc_RequestState_t rccState = RCC_REQUEST_ERROR;

    if( FMC_NULL_PTR != busClk )
    {
        rccState = Rcc_Get_PeriphClk( RCC_PERIPH_FMC, &clk );

        if( ( RCC_REQUEST_OK == rccState ) &&
            ( 0u             != clk      )    )
        {
            *busClk  = clk;
            retState = FMC_REQUEST_OK;
        }
        else
        {
            /* The clock is not available */
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
 * \brief Converts a time to cycles of a clock, rounded up: cycles = ceil( time * clk / ( divider * 10^9 ) ).
 *
 * \param time    [in]  : Time in nanoseconds.
 * \param clk     [in]  : Clock in Hz.
 * \param divider [in]  : Divider of the clock (the memory clock is clk / divider), at least 1.
 * \param cycles  [out] : Pointer receiving the number of cycles.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for an invalid parameter or a result that does not fit.
 */
fmc_RequestState_t FmcPriv_Get_Cycles( fmc_Time_ns_t time,
                                       fmc_Frequency_Hz_t clk,
                                       fmc_CycleCnt_t divider,
                                       fmc_CycleCnt_t * const cycles )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;

    if( ( FMC_NULL_PTR != cycles ) &&
        ( 0u           != clk    ) &&
        ( 0u           != divider )    )
    {
        const uint64_t numerator   = ( (uint64_t)time * (uint64_t)clk );
        const uint64_t denominator = ( (uint64_t)divider * (uint64_t)FMC_NS_PER_S );
        const uint64_t result      = ( ( numerator + denominator - 1u ) / denominator );

        if( (uint64_t)UINT32_MAX >= result )
        {
            *cycles  = (fmc_CycleCnt_t)result;
            retState = FMC_REQUEST_OK;
        }
        else
        {
            /* The result does not fit */
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
 * \brief Converts a time to the cycles of a phase of an access: rounded up, raised to the smallest value the
 *        controller can generate, rejected above the largest value of the register field.
 *
 * \param time   [in]  : Time in nanoseconds.
 * \param clock  [in]  : Clock of the memory (the bus clock and its divider).
 * \param limit  [in]  : Smallest and largest number of cycles of the phase.
 * \param cycles [out] : Pointer receiving the number of cycles.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR when the time needs more cycles than the limit.
 */
fmc_RequestState_t FmcPriv_Get_PhaseCycles( fmc_Time_ns_t time,
                                            const fmc_Clock_t * const clock,
                                            const fmc_PhaseLimit_t * const limit,
                                            fmc_CycleCnt_t * const cycles )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;
    fmc_CycleCnt_t     calc     = 0u;

    if( ( FMC_NULL_PTR != clock  ) &&
        ( FMC_NULL_PTR != limit  ) &&
        ( FMC_NULL_PTR != cycles )    )
    {
        const fmc_RequestState_t calcState = FmcPriv_Get_Cycles( time, clock->Clk, clock->Divider, &calc );

        if( ( FMC_REQUEST_OK == calcState  ) &&
            ( limit->Max     >= calc       )    )
        {
            if( limit->Min > calc )
            {
                *cycles = limit->Min;
            }
            else
            {
                *cycles = calc;
            }
            retState = FMC_REQUEST_OK;
        }
        else
        {
            /* The time needs more cycles than the register field holds */
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
 * \brief Converts a list of phase times to cycles (see \ref FmcPriv_Get_PhaseCycles), every phase is converted.
 *
 * \param clock    [in] : Clock of the memory (the bus clock and its divider).
 * \param phases   [in] : List of the phases, every entry holds the time, the limits and the result pointer.
 * \param phaseCnt [in] : Number of the entries of the list.
 *
 * \return FMC_REQUEST_OK when every phase was converted, FMC_REQUEST_ERROR otherwise.
 */
fmc_RequestState_t FmcPriv_Get_Phases( const fmc_Clock_t * const clock,
                                       const fmc_Phase_t * const phases,
                                       fmc_CycleCnt_t phaseCnt )
{
    fmc_RequestState_t retState   = FMC_REQUEST_ERROR;
    fmc_RequestState_t phaseState = FMC_REQUEST_ERROR;

    if( FMC_NULL_PTR != phases )
    {
        retState = FMC_REQUEST_OK;

        for( fmc_CycleCnt_t idx = 0u; ( phaseCnt > idx ); idx++ )
        {
            phaseState = FmcPriv_Get_PhaseCycles( phases[idx].Time, clock, phases[idx].Limit, phases[idx].Cycles );
            if( FMC_REQUEST_OK != phaseState )
            {
                retState = FMC_REQUEST_ERROR;
            }
            else
            {
                /* The phase is converted */
            }
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
 * \brief Busy-waits a time in microseconds derived from the bus clock (the iteration count is the delay in
 *        microseconds times the bus clock cycles per microsecond, so the delay is never shorter than requested).
 *
 * \param delay [in] : Delay in microseconds.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR when the bus clock is not available.
 */
fmc_RequestState_t FmcPriv_Set_Delay( fmc_Time_us_t delay )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;
    fmc_Frequency_Hz_t busClk   = 0u;
    const fmc_RequestState_t clkState = FmcPriv_Get_BusClock( &busClk );

    if( FMC_REQUEST_OK == clkState )
    {
        const uint64_t cyclesPerUs = ( ( (uint64_t)busClk + FMC_US_PER_S - 1u ) / FMC_US_PER_S );
        uint64_t       loopCnt     = ( (uint64_t)delay * cyclesPerUs );

        if( (uint64_t)UINT32_MAX < loopCnt )
        {
            loopCnt = (uint64_t)UINT32_MAX;
        }
        else
        {
            /* The iteration count fits */
        }

        for( volatile fmc_CycleCnt_t delayCnt = 0u; (fmc_CycleCnt_t)loopCnt > delayCnt; delayCnt++ )
        {
            /* Busy-wait: intentionally empty */
        }
        retState = FMC_REQUEST_OK;
    }
    else
    {
        /* The bus clock is not available, the delay can not be derived */
        retState = FMC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Disables the controller for a register update (RM0481 23.2) and reports whether it was enabled.
 *
 * \param wasActive [out] : Pointer receiving the state of the controller before the call.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for a null pointer or when the controller stays enabled.
 */
fmc_RequestState_t FmcPriv_Controller_Suspend( fmc_FunctionState_t * const wasActive )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;

    if( FMC_NULL_PTR != wasActive )
    {
        *wasActive = Fmc_Reg_Get_ControllerState();

        if( FMC_FUNCTION_ACTIVE == *wasActive )
        {
            retState = Fmc_Reg_Set_ControllerState( FMC_FUNCTION_INACTIVE );
        }
        else
        {
            /* The controller is disabled already */
            retState = FMC_REQUEST_OK;
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
 * \brief Enables the controller again after a register update when it was enabled before.
 *
 * \param wasActive [in] : State of the controller before \ref FmcPriv_Controller_Suspend.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR when the controller does not follow.
 */
fmc_RequestState_t FmcPriv_Controller_Resume( fmc_FunctionState_t wasActive )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;

    if( FMC_FUNCTION_ACTIVE == wasActive )
    {
        retState = Fmc_Reg_Set_ControllerState( FMC_FUNCTION_ACTIVE );
    }
    else
    {
        /* The controller was disabled before, it stays disabled */
        retState = FMC_REQUEST_OK;
    }

    return ( retState );
}


/**
 * \brief Converts a NOR / PSRAM / SRAM configuration to the register values (no register is touched).
 *
 * \param config [in]  : Configuration of the bank.
 * \param busClk [in]  : Bus clock in Hz.
 * \param regs   [out] : Pointer receiving the register values.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for an invalid item or a time the clock can not meet.
 */
fmc_RequestState_t FmcPriv_Get_NorSramRegs( const fmc_NorSramConfig_t * const config,
                                            fmc_Frequency_Hz_t busClk,
                                            fmc_NorSramRegs_t * const regs )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;
    fmc_CycleCnt_t     csCount  = 0u;

    if( ( FMC_NULL_PTR != config ) &&
        ( FMC_NULL_PTR != regs   ) &&
        ( 0u           != busClk )    )
    {
        const fmc_Clock_t        clock        = { .Clk = busClk, .Divider = FMC_NO_DIVIDER };
        const fmc_RequestState_t checkState   = FmcPriv_Check_NorSramConfig( config );
        const fmc_RequestState_t controlState = FmcPriv_Get_NorSramControl( config, &clock, &( regs->Bcr ) );
        const fmc_RequestState_t readState    = FmcPriv_Get_NorSramReadTiming( config, &clock, &( regs->Btr ) );
        const fmc_RequestState_t writeState   = FmcPriv_Get_NorSramWriteTiming( config, &clock, &( regs->Bwtr ) );
        fmc_RequestState_t       csState      = FMC_REQUEST_OK;

        if( 0u != config->ChipSelectMaxLow_ns )
        {
            csState         = FmcPriv_Get_PhaseCycles( config->ChipSelectMaxLow_ns, &clock, &fmc_LimCsCount, &csCount );
            regs->CsCounter = FMC_FUNCTION_ACTIVE;
            regs->CsCount   = csCount;
        }
        else
        {
            /* No limit of the chip select low time */
            regs->CsCounter = FMC_FUNCTION_INACTIVE;
            regs->CsCount   = 0u;
        }

        if( ( FMC_REQUEST_OK == checkState   ) &&
            ( FMC_REQUEST_OK == controlState ) &&
            ( FMC_REQUEST_OK == readState    ) &&
            ( FMC_REQUEST_OK == writeState   ) &&
            ( FMC_REQUEST_OK == csState      )    )
        {
            retState = FMC_REQUEST_OK;
        }
        else
        {
            /* The configuration is not valid */
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

#endif /* FMC_Bank1_R */


/**
 * \brief Returns the module version.
 *
 * \return Module version.
 */
fmc_ModuleVersion_t Fmc_Get_ModuleVersion( void )
{
    fmc_ModuleVersion_t version =
    {
        .Major = FMC_VERSION_MAJOR,
        .Minor = FMC_VERSION_MINOR,
        .Patch = FMC_VERSION_PATCH,
    };

    return ( version );
}


/**
 * \brief Fills a configuration structure with safe default values: the write FIFO enabled, no continuous
 *        clock, the events served by Fmc_Task, no bank configuration, every pin unused, no callback.
 *
 * \param fmcConfig [out] : Pointer to the configuration structure.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for a null pointer.
 */
fmc_RequestState_t Fmc_Get_DefaultConfig( fmc_Config_t * const fmcConfig )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;

#if defined(FMC_Bank1_R)
    if( FMC_NULL_PTR != fmcConfig )
    {
        fmcConfig->WriteFifo       = FMC_FUNCTION_ACTIVE;
        fmcConfig->ContinuousClock = FMC_FUNCTION_INACTIVE;
        fmcConfig->EventMode       = FMC_EVENT_MODE_POLL;
        fmcConfig->IrqPriority     = 0u;
        fmcConfig->NandConfig      = FMC_NULL_PTR;

        for( uint32_t idx = 0u; ( FMC_NOR_BANK_CNT > idx ); idx++ )
        {
            fmcConfig->NorSramConfig[idx] = FMC_NULL_PTR;
        }
        for( uint32_t idx = 0u; ( FMC_SDRAM_BANK_CNT > idx ); idx++ )
        {
            fmcConfig->SdramConfig[idx] = FMC_NULL_PTR;
        }
        for( uint32_t idx = 0u; ( FMC_ADDRESS_LINE_CNT > idx ); idx++ )
        {
            fmcConfig->AddressPin[idx] = FMC_ADDRESS_PIN_UNUSED;
        }
        for( uint32_t idx = 0u; ( FMC_DATA_LINE_CNT > idx ); idx++ )
        {
            fmcConfig->DataPin[idx] = FMC_DATA_PIN_UNUSED;
        }
        for( uint32_t idx = 0u; ( FMC_CONTROL_PIN_CNT > idx ); idx++ )
        {
            fmcConfig->ControlPin[idx] = FMC_CONTROL_PIN_UNUSED;
        }
        for( uint32_t idx = 0u; ( FMC_SDRAM_PIN_CNT > idx ); idx++ )
        {
            fmcConfig->SdramPin[idx] = FMC_SDRAM_PIN_UNUSED;
        }

        fmcConfig->NandReadyCallback    = FMC_NULL_PTR;
        fmcConfig->RefreshErrorCallback = FMC_NULL_PTR;
        fmcConfig->ErrorCallback        = FMC_NULL_PTR;
        retState = FMC_REQUEST_OK;
    }
    else
    {
        /* Invalid parameter, no action required */
        retState = FMC_REQUEST_ERROR;
    }
#else
    (void)fmcConfig;
#endif /* FMC_Bank1_R */

    return ( retState );
}


/**
 * \brief Initializes the controller: the clock, the interrupt, the pins of the lists, the write FIFO and the
 *        continuous clock, all banks disabled, the controller enabled; then the banks of the configuration
 *        are configured (not activated, see \ref Fmc_Set_BankActive). The result is verified by read-back;
 *        on a failure the module is deinitialized.
 *
 * \note  The bank configuration pointers and the pin lists are read here only, they are not kept.
 *
 * \param fmcConfig [in] : Pointer to the configuration structure.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR otherwise (a module that is initialized already is
 *         not touched).
 */
fmc_RequestState_t Fmc_Init( const fmc_Config_t * const fmcConfig )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;

#if defined(FMC_Bank1_R)
    fmc_InstanceData_t * instance      = FMC_NULL_PTR;
    fmc_RequestState_t   instanceState = FmcPriv_Get_Instance( &instance );
    fmc_RequestState_t   checkState    = FMC_REQUEST_ERROR;

    if( ( FMC_NULL_PTR   != fmcConfig    ) &&
        ( FMC_REQUEST_OK == instanceState )    )
    {
        checkState = FmcPriv_Check_Config( fmcConfig );
    }
    else
    {
        /* Invalid parameter, checkState stays FMC_REQUEST_ERROR */
        checkState = FMC_REQUEST_ERROR;
    }

    if( ( FMC_REQUEST_OK        == checkState             ) &&
        ( FMC_FUNCTION_INACTIVE == instance->Initialized  )    )
    {
        rcc_RequestState_t  rccState    = RCC_REQUEST_ERROR;
        nvic_RequestState_t handlerState = NVIC_REQUEST_ERROR;
        nvic_RequestState_t prioState    = NVIC_REQUEST_ERROR;
        nvic_RequestState_t activeState  = NVIC_REQUEST_OK;
        fmc_RequestState_t  pinState     = FMC_REQUEST_ERROR;
        fmc_RequestState_t  bankState    = FMC_REQUEST_ERROR;
        fmc_RequestState_t  enableState  = FMC_REQUEST_ERROR;
        fmc_RegValue_t      bcr1Value    = FMC_REG_BITS_CLEAR;

        FmcPriv_Clear_Instance( instance );
        instance->Config      = *fmcConfig;
        instance->Initialized = FMC_FUNCTION_ACTIVE;

        for( uint32_t idx = 0u; ( FMC_NOR_BANK_CNT > idx ); idx++ )
        {
            instance->Config.NorSramConfig[idx] = FMC_NULL_PTR;
        }
        for( uint32_t idx = 0u; ( FMC_SDRAM_BANK_CNT > idx ); idx++ )
        {
            instance->Config.SdramConfig[idx] = FMC_NULL_PTR;
        }
        instance->Config.NandConfig = FMC_NULL_PTR;

        rccState     = Rcc_Set_PeriphActive( RCC_PERIPH_FMC );
        handlerState = Nvic_Set_PeriphIrq_Handler( NVIC_PERIPH_IRQ_FMC, FmcPriv_Isr );
        prioState    = Nvic_Set_PeriphIrq_Prio( NVIC_PERIPH_IRQ_FMC, fmcConfig->IrqPriority );

        if( FMC_EVENT_MODE_ISR == fmcConfig->EventMode )
        {
            activeState = Nvic_Set_PeriphIrq_Active( NVIC_PERIPH_IRQ_FMC );
        }
        else
        {
            /* The events are served by Fmc_Task, the interrupt is not enabled */
            activeState = NVIC_REQUEST_OK;
        }

        pinState = FmcPriv_Init_ConfigPins( fmcConfig );

        /* The controller is disabled for the register update, all the banks are disabled (bank 1 is enabled
         * after the reset), the common bits are set, the controller is enabled */
        (void)Fmc_Reg_Set_ControllerState( FMC_FUNCTION_INACTIVE );
        for( uint32_t idx = 0u; ( FMC_NOR_BANK_CNT > idx ); idx++ )
        {
            Fmc_Reset_RegBit( fmc_NorBcrReg[idx], FMC_BCRx_MBKEN );
        }
        Fmc_Reset_RegBit( FMC_REG_PCR, FMC_PCR_PBKEN );
        Fmc_Reset_RegBit( FMC_REG_PCR, FMC_PCR_ECCEN );

        bcr1Value = ( FmcPriv_Get_StateMask( fmcConfig->WriteFifo, FMC_FUNCTION_INACTIVE, FMC_BCR1_WFDIS ) |
                      FmcPriv_Get_StateMask( fmcConfig->ContinuousClock, FMC_FUNCTION_ACTIVE, FMC_BCR1_CCLKEN ) );
        Fmc_Set_RegVal( FMC_REG_BCR1, FMC_BCR1_COMMON_MASK, bcr1Value );
        enableState = Fmc_Reg_Set_ControllerState( FMC_FUNCTION_ACTIVE );

        bankState = FmcPriv_Init_ConfigBanks( fmcConfig );

        const fmc_RegValue_t bcr1Back = Fmc_Get_RegVal( FMC_REG_BCR1, ( FMC_BCR1_COMMON_MASK | FMC_BCR1_FMCEN ) );

        if( ( RCC_REQUEST_OK  == rccState     ) &&
            ( NVIC_REQUEST_OK == handlerState ) &&
            ( NVIC_REQUEST_OK == prioState    ) &&
            ( NVIC_REQUEST_OK == activeState  ) &&
            ( FMC_REQUEST_OK  == pinState     ) &&
            ( FMC_REQUEST_OK  == enableState  ) &&
            ( FMC_REQUEST_OK  == bankState    ) &&
            ( bcr1Back        == ( bcr1Value | FMC_BCR1_FMCEN ) )    )
        {
            retState = FMC_REQUEST_OK;
        }
        else
        {
            /* The initialization is not as required, the module is deinitialized */
            (void)Fmc_Deinit();
            retState = FMC_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid configuration or the module is initialized already, no action required */
        retState = FMC_REQUEST_ERROR;
    }
#else
    (void)fmcConfig;
#endif /* FMC_Bank1_R */

    return ( retState );
}


/**
 * \brief Deinitializes the controller: every bank is disabled, the controller is disabled, the interrupt and
 *        the peripheral clock are switched off. The data of an SDRAM is not retained.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR when the module is not initialized.
 */
fmc_RequestState_t Fmc_Deinit( void )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;

#if defined(FMC_Bank1_R)
    fmc_InstanceData_t * instance      = FMC_NULL_PTR;
    fmc_RequestState_t   instanceState = FmcPriv_Get_Instance( &instance );

    if( ( FMC_REQUEST_OK       == instanceState         ) &&
        ( FMC_FUNCTION_ACTIVE  == instance->Initialized )    )
    {
        for( uint32_t idx = 0u; ( FMC_NOR_BANK_CNT > idx ); idx++ )
        {
            Fmc_Reset_RegBit( fmc_NorBcrReg[idx], FMC_BCRx_MBKEN );
        }
        Fmc_Reset_RegBit( FMC_REG_PCR, FMC_PCR_PBKEN );
        Fmc_Reset_RegBit( FMC_REG_PCR, FMC_PCR_ECCEN );
        Fmc_Reset_RegBit( FMC_REG_SR, ( FMC_SR_IREN | FMC_SR_ILEN | FMC_SR_IFEN ) );
#if defined(FMC_Bank5_6_R)
        Fmc_Reset_RegBit( FMC_REG_SDRTR, FMC_SDRTR_REIE );
#endif
        (void)Fmc_Reg_Set_ControllerState( FMC_FUNCTION_INACTIVE );

        (void)Nvic_Set_PeriphIrq_Inactive( NVIC_PERIPH_IRQ_FMC );
        (void)Rcc_Set_PeriphInactive( RCC_PERIPH_FMC );

        FmcPriv_Clear_Instance( instance );
        retState = FMC_REQUEST_OK;
    }
    else
    {
        /* The module is not initialized, no action required */
        retState = FMC_REQUEST_ERROR;
    }
#endif /* FMC_Bank1_R */

    return ( retState );
}


/**
 * \brief Serves the events of the controller (the NAND ready signal and the SDRAM refresh error) in the
 *        FMC_EVENT_MODE_POLL mode: the flags are read, cleared and reported by the callbacks. Without effect in
 *        the FMC_EVENT_MODE_ISR mode and while the module is not initialized.
 */
void Fmc_Task( void )
{
#if defined(FMC_Bank1_R)
    fmc_InstanceData_t * instance      = FMC_NULL_PTR;
    fmc_RequestState_t   instanceState = FmcPriv_Get_Instance( &instance );

    if( ( FMC_REQUEST_OK      == instanceState              ) &&
        ( FMC_FUNCTION_ACTIVE == instance->Initialized      ) &&
        ( FMC_EVENT_MODE_POLL == instance->Config.EventMode )    )
    {
        FmcPriv_ServeEvents();
    }
    else
    {
        /* Not initialized or served by the interrupt, no action required */
    }
#endif /* FMC_Bank1_R */

    return;
}


/**
 * \brief Configures a NOR / PSRAM / SRAM bank. The timing in nanoseconds is converted to the bus clock cycles
 *        first (a time the clock can not meet rejects the whole call), then the registers are written with
 *        the controller disabled and verified by read-back. The bank stays disabled (see
 *        \ref Fmc_Set_BankActive).
 *
 * \param bank           [in] : NOR / PSRAM / SRAM bank, FMC_BANK_NOR_PSRAM_1 .. FMC_BANK_NOR_PSRAM_4.
 * \param norSramConfig [in] : Pointer to the configuration of the bank.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for an invalid parameter, an uninitialized module, a bank
 *         that is active, a time the clock can not meet or a register that does not read back.
 */
fmc_RequestState_t Fmc_Set_NorSramConfig( fmc_Bank_t bank, const fmc_NorSramConfig_t * const norSramConfig )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;

#if defined(FMC_Bank1_R)
    fmc_InstanceData_t * instance      = FMC_NULL_PTR;
    fmc_RequestState_t   instanceState = FmcPriv_Get_Instance( &instance );

    if( ( FMC_REQUEST_OK        == instanceState         ) &&
        ( FMC_NULL_PTR          != norSramConfig         ) &&
        ( FMC_BANK_NOR_PSRAM_4  >= bank                  ) &&
        ( FMC_FUNCTION_ACTIVE   == instance->Initialized ) &&
        ( FMC_FUNCTION_INACTIVE == instance->Active[bank] )   )
    {
        fmc_Frequency_Hz_t busClk = 0u;
        fmc_NorSramRegs_t  regs;
        const fmc_RequestState_t clkState = FmcPriv_Get_BusClock( &busClk );

        if( FMC_REQUEST_OK == clkState )
        {
            const fmc_RequestState_t regsState = FmcPriv_Get_NorSramRegs( norSramConfig, busClk, &regs );

            if( FMC_REQUEST_OK == regsState )
            {
                retState = FmcPriv_Write_NorSramRegs( bank, &regs );

                if( FMC_REQUEST_OK == retState )
                {
                    instance->Configured[bank] = FMC_FUNCTION_ACTIVE;
                }
                else
                {
                    /* The registers were not written as required */
                    instance->Configured[bank] = FMC_FUNCTION_INACTIVE;
                }
            }
            else
            {
                /* The configuration is invalid or the clock can not meet a time, no register was written */
                retState = FMC_REQUEST_ERROR;
            }
        }
        else
        {
            /* The bus clock is not available */
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
    (void)norSramConfig;
#endif /* FMC_Bank1_R */

    return ( retState );
}


/**
 * \brief Enables a bank: the NOR / PSRAM / SRAM bank (MBKEN) or the NAND bank (PBKEN) after its configuration, the
 *        SDRAM bank (normal mode, also the exit of the self-refresh and the power-down) after
 *        \ref Fmc_Set_SdramInit. The memory is visible in the address space after this call.
 *
 * \param bank [in] : Bank to enable.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for an invalid or not configured bank, an uninitialized
 *         module or a register that does not read back.
 */
fmc_RequestState_t Fmc_Set_BankActive( fmc_Bank_t bank )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;

#if defined(FMC_Bank1_R)
    fmc_InstanceData_t * instance      = FMC_NULL_PTR;
    fmc_RequestState_t   instanceState = FmcPriv_Get_Instance( &instance );

    if( ( FMC_REQUEST_OK      == instanceState         ) &&
        ( FMC_BANK_CNT         > bank                  ) &&
        ( FMC_FUNCTION_ACTIVE == instance->Initialized )    )
    {
        if( FMC_FUNCTION_ACTIVE == instance->Configured[bank] )
        {
            if( FMC_BANK_NOR_PSRAM_4 >= bank )
            {
                retState = Fmc_Reg_Set_RegValVerified( fmc_NorBcrReg[bank], FMC_BCRx_MBKEN, FMC_BCRx_MBKEN );
            }
            else if( FMC_BANK_NAND == bank )
            {
                retState = Fmc_Reg_Set_RegValVerified( FMC_REG_PCR, FMC_PCR_PBKEN, FMC_PCR_PBKEN );
            }
            else
            {
#if defined(FMC_Bank5_6_R)
                retState = FmcPriv_Sdram_Activate( bank );
#else
                retState = FMC_REQUEST_ERROR;
#endif /* FMC_Bank5_6_R */
            }

            if( FMC_REQUEST_OK == retState )
            {
                instance->Active[bank] = FMC_FUNCTION_ACTIVE;
            }
            else
            {
                /* The bank is not enabled */
                retState = FMC_REQUEST_ERROR;
            }
        }
        else
        {
            /* The bank is not configured */
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
#endif /* FMC_Bank1_R */

    return ( retState );
}


/**
 * \brief Disables a NOR / PSRAM / SRAM bank (MBKEN) or the NAND bank (PBKEN). An access to a disabled bank is a
 *        bus error. An SDRAM bank has no enable bit and is not accepted (see \ref Fmc_Set_SdramSelfRefresh).
 *
 * \param bank [in] : Bank to disable.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for an invalid or SDRAM bank, an uninitialized module or a
 *         register that does not read back.
 */
fmc_RequestState_t Fmc_Set_BankInactive( fmc_Bank_t bank )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;

#if defined(FMC_Bank1_R)
    fmc_InstanceData_t * instance      = FMC_NULL_PTR;
    fmc_RequestState_t   instanceState = FmcPriv_Get_Instance( &instance );

    if( ( FMC_REQUEST_OK      == instanceState         ) &&
        ( FMC_BANK_NAND       >= bank                  ) &&
        ( FMC_FUNCTION_ACTIVE == instance->Initialized )    )
    {
        if( FMC_BANK_NOR_PSRAM_4 >= bank )
        {
            retState = Fmc_Reg_Set_RegValVerified( fmc_NorBcrReg[bank], FMC_BCRx_MBKEN, FMC_REG_BITS_CLEAR );
        }
        else
        {
            retState = Fmc_Reg_Set_RegValVerified( FMC_REG_PCR, FMC_PCR_PBKEN, FMC_REG_BITS_CLEAR );
        }

        if( FMC_REQUEST_OK == retState )
        {
            instance->Active[bank] = FMC_FUNCTION_INACTIVE;
        }
        else
        {
            /* The bank is not disabled */
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
#endif /* FMC_Bank1_R */

    return ( retState );
}


/**
 * \brief Returns the base address of the memory window of a bank in the address space of the CPU.
 *
 * \param bank        [in]  : Bank.
 * \param baseAddress [out] : Pointer receiving the base address.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for an invalid bank or pointer or an uninitialized module.
 */
fmc_RequestState_t Fmc_Get_BankBaseAddress( fmc_Bank_t bank, fmc_Address_t * const baseAddress )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;

#if defined(FMC_Bank1_R)
    fmc_InstanceData_t * instance      = FMC_NULL_PTR;
    fmc_RequestState_t   instanceState = FmcPriv_Get_Instance( &instance );

    if( ( FMC_REQUEST_OK      == instanceState         ) &&
        ( FMC_NULL_PTR        != baseAddress           ) &&
        ( FMC_BANK_CNT         > bank                  ) &&
        ( FMC_FUNCTION_ACTIVE == instance->Initialized )    )
    {
        *baseAddress = fmc_BankBase[bank];
        retState     = FMC_REQUEST_OK;
    }
    else
    {
        /* Invalid parameter or state, no action required */
        retState = FMC_REQUEST_ERROR;
    }
#else
    (void)bank;
    (void)baseAddress;
#endif /* FMC_Bank1_R */

    return ( retState );
}


/**
 * \brief Sets or clears the write protection of a bank: a NOR / PSRAM / SRAM bank rejects the writes (bus error)
 *        while WREN is cleared, an SDRAM bank ignores them (WP). The NAND bank has no write protection. The
 *        bank must be configured.
 *
 * \param bank            [in] : Bank.
 * \param protectionState [in] : FMC_FUNCTION_ACTIVE protects the bank, FMC_FUNCTION_INACTIVE allows the writes.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for an invalid or NAND bank, a bank that is not configured,
 *         an uninitialized module or a register that does not read back.
 */
fmc_RequestState_t Fmc_Set_BankWriteProtection( fmc_Bank_t bank, fmc_FunctionState_t protectionState )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;

#if defined(FMC_Bank1_R)
    fmc_InstanceData_t * instance      = FMC_NULL_PTR;
    fmc_RequestState_t   instanceState = FmcPriv_Get_Instance( &instance );

    if( ( FMC_REQUEST_OK      == instanceState         ) &&
        ( FMC_BANK_CNT         > bank                  ) &&
        ( FMC_BANK_NAND       != bank                  ) &&
        ( FMC_FUNCTION_ACTIVE >= protectionState       ) &&
        ( FMC_FUNCTION_ACTIVE == instance->Initialized )    )
    {
        if( FMC_FUNCTION_ACTIVE == instance->Configured[bank] )
        {
            if( FMC_BANK_NOR_PSRAM_4 >= bank )
            {
                retState = Fmc_Reg_Set_RegValVerified( fmc_NorBcrReg[bank],
                                                       FMC_BCRx_WREN,
                                                       FmcPriv_Get_StateMask( protectionState,
                                                                              FMC_FUNCTION_INACTIVE,
                                                                              FMC_BCRx_WREN ) );
            }
            else
            {
#if defined(FMC_Bank5_6_R)
                retState = FmcPriv_Sdram_Set_WriteProtection( bank, protectionState );
#else
                retState = FMC_REQUEST_ERROR;
#endif /* FMC_Bank5_6_R */
            }
        }
        else
        {
            /* The bank is not configured */
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
    (void)protectionState;
#endif /* FMC_Bank1_R */

    return ( retState );
}


/**
 * \brief Sets the NVIC priority of the module interrupt.
 *
 * \param irqPriority [in] : NVIC priority.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for an uninitialized module or when Nvic rejects it.
 */
fmc_RequestState_t Fmc_Set_IrqPriority( fmc_IrqPrio_t irqPriority )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;

#if defined(FMC_Bank1_R)
    fmc_InstanceData_t * instance      = FMC_NULL_PTR;
    fmc_RequestState_t   instanceState = FmcPriv_Get_Instance( &instance );

    if( ( FMC_REQUEST_OK      == instanceState         ) &&
        ( FMC_FUNCTION_ACTIVE == instance->Initialized )    )
    {
        const nvic_RequestState_t nvicState = Nvic_Set_PeriphIrq_Prio( NVIC_PERIPH_IRQ_FMC, irqPriority );

        if( NVIC_REQUEST_OK == nvicState )
        {
            instance->Config.IrqPriority = irqPriority;
            retState = FMC_REQUEST_OK;
        }
        else
        {
            /* Nvic rejected the priority */
            retState = FMC_REQUEST_ERROR;
        }
    }
    else
    {
        /* The module is not initialized, no action required */
        retState = FMC_REQUEST_ERROR;
    }
#else
    (void)irqPriority;
#endif /* FMC_Bank1_R */

    return ( retState );
}


/**
 * \brief Reads the NVIC priority of the module interrupt.
 *
 * \param irqPriority [out] : Pointer receiving the NVIC priority.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for a null pointer, an uninitialized module or when Nvic
 *         rejects it.
 */
fmc_RequestState_t Fmc_Get_IrqPriority( fmc_IrqPrio_t * const irqPriority )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;

#if defined(FMC_Bank1_R)
    fmc_InstanceData_t * instance      = FMC_NULL_PTR;
    fmc_RequestState_t   instanceState = FmcPriv_Get_Instance( &instance );

    if( ( FMC_REQUEST_OK      == instanceState         ) &&
        ( FMC_NULL_PTR        != irqPriority           ) &&
        ( FMC_FUNCTION_ACTIVE == instance->Initialized )    )
    {
        const nvic_RequestState_t nvicState = Nvic_Get_PeriphIrq_Prio( NVIC_PERIPH_IRQ_FMC, irqPriority );

        if( NVIC_REQUEST_OK == nvicState )
        {
            retState = FMC_REQUEST_OK;
        }
        else
        {
            /* Nvic rejected the request */
            retState = FMC_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid parameter or state, no action required */
        retState = FMC_REQUEST_ERROR;
    }
#else
    (void)irqPriority;
#endif /* FMC_Bank1_R */

    return ( retState );
}


/**
 * \brief Configures an address pin (alternate function, very high speed, no pull resistor).
 *
 * \param addressPin [in] : Pin item of the address list.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for the unused item or when Gpio rejects the pin.
 */
fmc_RequestState_t Fmc_InitAddressGpio( fmc_AddressPin_t addressPin )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;

#if defined(FMC_Bank1_R)
    retState = FmcPriv_Init_AltFuncGpio( (uint32_t)addressPin, FMC_SIGNAL_A0, FMC_SIGNAL_A25 );
#else
    (void)addressPin;
#endif /* FMC_Bank1_R */

    return ( retState );
}


/**
 * \brief Configures a data pin (alternate function, very high speed, no pull resistor). The pin is the same in the
 *        multiplexed address / data mode.
 *
 * \param dataPin [in] : Pin item of the data list.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for the unused item or when Gpio rejects the pin.
 */
fmc_RequestState_t Fmc_InitDataGpio( fmc_DataPin_t dataPin )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;

#if defined(FMC_Bank1_R)
    retState = FmcPriv_Init_AltFuncGpio( (uint32_t)dataPin, FMC_SIGNAL_D0, FMC_SIGNAL_D31 );
#else
    (void)dataPin;
#endif /* FMC_Bank1_R */

    return ( retState );
}


/**
 * \brief Configures a control pin (alternate function, very high speed, pull-up on the wait and ready signals).
 *
 * \param controlPin [in] : Pin item of the control list.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for the unused item or when Gpio rejects the pin.
 */
fmc_RequestState_t Fmc_InitControlGpio( fmc_ControlPin_t controlPin )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;

#if defined(FMC_Bank1_R)
    retState = FmcPriv_Init_AltFuncGpio( (uint32_t)controlPin, FMC_SIGNAL_CLK, FMC_SIGNAL_INT );
#else
    (void)controlPin;
#endif /* FMC_Bank1_R */

    return ( retState );
}


/**
 * \brief Configures an SDRAM pin (alternate function, very high speed, no pull resistor).
 *
 * \param sdramPin [in] : Pin item of the SDRAM list.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for the unused item or when Gpio rejects the pin.
 */
fmc_RequestState_t Fmc_InitSdramGpio( fmc_SdramPin_t sdramPin )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;

#if defined(FMC_Bank1_R)
    retState = FmcPriv_Init_AltFuncGpio( (uint32_t)sdramPin, FMC_SIGNAL_SDCLK, FMC_SIGNAL_BA1 );
#else
    (void)sdramPin;
#endif /* FMC_Bank1_R */

    return ( retState );
}
