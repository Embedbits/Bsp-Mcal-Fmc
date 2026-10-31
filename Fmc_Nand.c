/**
 * \author Mr.Nobody
 * \file Fmc_Nand.c
 * \ingroup Fmc
 * \brief Fmc module source file (NAND bank: configuration, ECC, ready / busy signal)
 *
 * The NAND flash controller of the flexible memory controller (every STM32H5 line with the FMC). The
 * configuration converts every timing in nanoseconds to cycles of the bus clock (rounded up, raised to the
 * smallest value of the controller, rejected above the largest value of the register field) and writes the
 * registers with the controller disabled. The ECC is calculated by the controller while a page is
 * transferred (up to 8 KB, one code per page): Fmc_Set_NandEccActive starts it, Fmc_Get_NandEcc waits
 * until the write FIFO is empty, returns the code and stops the calculation (RM0481 23.7.6). The module
 * does not know the commands of a NAND flash and does not correct the data. On STM32H503, which has no
 * FMC, every exported function checks the CMSIS macro (defined(FMC_Bank1_R)) and only returns
 * FMC_REQUEST_ERROR.
 *
 */
/* ============================== INCLUDES ================================== */
#include "Fmc.h"                            /* Own private declarations       */
#include "Fmc_Reg.h"                        /* Module Reg component header    */
#include "Stm32.h"                          /* Device registers definition    */

/* ========================== PRIVATE DECLARATIONS =========================== */

#if defined(FMC_Bank1_R)

/** \brief Smallest setup time (MEMSET / ATTSET) in cycles, the register holds the cycles minus this value */
#define FMC_NAND_SET_CYCLES_MIN             ( 1u )

/** \brief Largest setup time (MEMSET / ATTSET) in cycles */
#define FMC_NAND_SET_CYCLES_MAX             ( 255u )

/** \brief Smallest wait time (MEMWAIT / ATTWAIT) in cycles, the register holds the cycles minus one */
#define FMC_NAND_WAIT_CYCLES_MIN            ( 2u )

/** \brief Largest wait time (MEMWAIT / ATTWAIT) in cycles */
#define FMC_NAND_WAIT_CYCLES_MAX            ( 255u )

/** \brief Smallest hold time (MEMHOLD / ATTHOLD) in cycles, the register holds the cycles */
#define FMC_NAND_HOLD_CYCLES_MIN            ( 1u )

/** \brief Largest hold time (MEMHOLD / ATTHOLD) in cycles */
#define FMC_NAND_HOLD_CYCLES_MAX            ( 254u )

/** \brief Smallest data bus high-Z time (MEMHIZ / ATTHIZ) in cycles, the register holds the cycles minus this value */
#define FMC_NAND_HIZ_CYCLES_MIN             ( 1u )

/** \brief Largest data bus high-Z time (MEMHIZ / ATTHIZ) in cycles */
#define FMC_NAND_HIZ_CYCLES_MAX             ( 255u )

/** \brief Largest value of the TAR and TCLR fields */
#define FMC_NAND_DELAY_FIELD_MAX            ( 15u )

/** \brief Cycles added to TAR and TCLR and to the setup time in the delay formula: t = ( TAR + SET + 2 ) cycles */
#define FMC_NAND_DELAY_OFFSET               ( 2u )

/** \brief Difference between the cycles and the register field of the wait time (MEMWAIT / ATTWAIT) */
#define FMC_NAND_WAIT_FIELD_OFFSET          ( 1u )

/** \brief Valid bits of the ECC result for the page size of 256 bytes (ECC[21:0]) */
#define FMC_ECC_MASK_256                    ( 0x003FFFFFu )

/** \brief Valid bits of the ECC result for the page size of 512 bytes (ECC[23:0]) */
#define FMC_ECC_MASK_512                    ( 0x00FFFFFFu )

/** \brief Valid bits of the ECC result for the page size of 1024 bytes (ECC[25:0]) */
#define FMC_ECC_MASK_1024                   ( 0x03FFFFFFu )

/** \brief Valid bits of the ECC result for the page size of 2048 bytes (ECC[27:0]) */
#define FMC_ECC_MASK_2048                   ( 0x0FFFFFFFu )

/** \brief Valid bits of the ECC result for the page size of 4096 bytes (ECC[29:0]) */
#define FMC_ECC_MASK_4096                   ( 0x3FFFFFFFu )

/** \brief Valid bits of the ECC result for the page size of 8192 bytes (ECC[31:0]) */
#define FMC_ECC_MASK_8192                   ( 0xFFFFFFFFu )

/** \brief FMC_PCR PWID field value: 8-bit data bus */
#define FMC_PWID_8BIT                       ( 0u )

/** \brief FMC_PCR PWID field value: 16-bit data bus */
#define FMC_PWID_16BIT                      ( 1u )

/** \brief Number of the polls of the high level flag of the ready signal before the NAND is reported busy */
#define FMC_NAND_READY_POLL_CNT             ( 64u )

/** \brief Bits of FMC_PCR written by the configuration (the bank enable bit is not written) */
#define FMC_PCR_CONFIG_MASK                 ( FMC_PCR_PWAITEN | FMC_PCR_PTYP  | FMC_PCR_PWID  | FMC_PCR_ECCEN | \
                                              FMC_PCR_TCLR    | FMC_PCR_TAR   | FMC_PCR_ECCPS )

/** \brief Interrupt detection enable bits of FMC_SR */
#define FMC_SR_ENABLE_MASK                  ( FMC_SR_IREN | FMC_SR_ILEN | FMC_SR_IFEN )

/* ============================== TYPEDEFS ================================== */

/** \brief Register field values of the timing of one memory space of the NAND bank */
typedef struct
{
    fmc_RegValue_t Set;  /**< Setup time field                   */
    fmc_RegValue_t Wait; /**< Wait time field                    */
    fmc_RegValue_t Hold; /**< Hold time field                    */
    fmc_RegValue_t HiZ;  /**< Data bus high-Z time field         */
}   fmc_NandSpace_t;

/** \brief Flag, detection enable bit and event of one NAND ready signal event */
typedef struct
{
    fmc_RegValue_t  FlagBit;   /**< Flag of the event in FMC_SR                    */
    fmc_RegValue_t  EnableBit; /**< Detection enable bit of the event in FMC_SR    */
    fmc_NandEvent_t Event;     /**< Event reported by the callback                 */
}   fmc_NandEventMap_t;

/* =========================== LOCAL VARIABLES ============================== */

/** \brief Limits of the setup time in cycles of the bus clock */
static const fmc_PhaseLimit_t fmc_LimNandSet = { FMC_NAND_SET_CYCLES_MIN, FMC_NAND_SET_CYCLES_MAX };

/** \brief Limits of the wait time in cycles of the bus clock */
static const fmc_PhaseLimit_t fmc_LimNandWait = { FMC_NAND_WAIT_CYCLES_MIN, FMC_NAND_WAIT_CYCLES_MAX };

/** \brief Limits of the hold time in cycles of the bus clock */
static const fmc_PhaseLimit_t fmc_LimNandHold = { FMC_NAND_HOLD_CYCLES_MIN, FMC_NAND_HOLD_CYCLES_MAX };

/** \brief Limits of the data bus high-Z time in cycles of the bus clock */
static const fmc_PhaseLimit_t fmc_LimNandHiz = { FMC_NAND_HIZ_CYCLES_MIN, FMC_NAND_HIZ_CYCLES_MAX };

/** \brief Valid bits of the ECC result register for every page size (RM0481 Table 229: 22 bits for 256 bytes, 2 more
 *         bits for every doubling of the page size) */
static const fmc_Ecc_t fmc_EccMask[ FMC_ECC_PAGE_SIZE_CNT ] =
{
    FMC_ECC_MASK_256,  /**< FMC_ECC_PAGE_SIZE_256  */
    FMC_ECC_MASK_512,  /**< FMC_ECC_PAGE_SIZE_512  */
    FMC_ECC_MASK_1024, /**< FMC_ECC_PAGE_SIZE_1024 */
    FMC_ECC_MASK_2048, /**< FMC_ECC_PAGE_SIZE_2048 */
    FMC_ECC_MASK_4096, /**< FMC_ECC_PAGE_SIZE_4096 */
    FMC_ECC_MASK_8192, /**< FMC_ECC_PAGE_SIZE_8192 */
};

_Static_assert( ( sizeof( fmc_EccMask ) / sizeof( fmc_Ecc_t ) ) == FMC_ECC_PAGE_SIZE_CNT,
                 "Fmc_Nand: fmc_EccMask has incorrect size." );

/** \brief Events of the NAND ready signal, indexed by fmc_NandEvent_t */
static const fmc_NandEventMap_t fmc_NandEventMap[ FMC_NAND_EVENT_CNT ] =
{
    { FMC_SR_IRS, FMC_SR_IREN, FMC_NAND_EVENT_RISING_EDGE  }, /**< Rising edge of the ready signal  */
    { FMC_SR_IFS, FMC_SR_IFEN, FMC_NAND_EVENT_FALLING_EDGE }, /**< Falling edge of the ready signal */
    { FMC_SR_ILS, FMC_SR_ILEN, FMC_NAND_EVENT_HIGH_LEVEL   }, /**< High level of the ready signal   */
};

_Static_assert( ( sizeof( fmc_NandEventMap ) / sizeof( fmc_NandEventMap_t ) ) == FMC_NAND_EVENT_CNT,
                 "Fmc_Nand: fmc_NandEventMap has incorrect size." );

/* ========================== PRIVATE FUNCTIONS ============================== */

/**
 * \brief Checks the enumerated items of a NAND configuration.
 *
 * \param config [in] : Configuration to check, not null.
 *
 * \return FMC_REQUEST_OK for a valid configuration, FMC_REQUEST_ERROR otherwise.
 */
static fmc_RequestState_t FmcPriv_Check_NandConfig( const fmc_NandConfig_t * const config )
{
    fmc_RequestState_t retState   = FMC_REQUEST_ERROR;
    fmc_FlagState_t    widthValid = FMC_FLAG_INACTIVE;

    if( ( FMC_DATA_WIDTH_8  == config->DataWidth ) ||
        ( FMC_DATA_WIDTH_16 == config->DataWidth )    )
    {
        widthValid = FMC_FLAG_ACTIVE;
    }
    else
    {
        /* The NAND bus is 8 or 16 bits wide, never 32 bits */
        widthValid = FMC_FLAG_INACTIVE;
    }

    if( ( FMC_FLAG_ACTIVE        == widthValid              ) &&
        ( FMC_FUNCTION_ACTIVE   >= config->WaitFeature      ) &&
        ( FMC_FUNCTION_ACTIVE   >= config->EccEnable        ) &&
        ( FMC_ECC_PAGE_SIZE_CNT  > config->EccPageSize      ) &&
        ( FMC_FUNCTION_ACTIVE   >= config->ReadyRisingIrq   ) &&
        ( FMC_FUNCTION_ACTIVE   >= config->ReadyFallingIrq  ) &&
        ( FMC_FUNCTION_ACTIVE   >= config->ReadyHighIrq     )    )
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
 * \brief Converts the timing of one memory space to the register field values.
 *
 * \param timing [in]  : Timing in nanoseconds.
 * \param clock  [in]  : Clock of the controller (the bus clock).
 * \param space  [out] : Pointer receiving the register field values.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR when a time needs more cycles than its field holds.
 */
static fmc_RequestState_t FmcPriv_Get_NandSpace( const fmc_NandTiming_t * const timing,
                                                 const fmc_Clock_t * const clock,
                                                 fmc_NandSpace_t * const space )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;
    fmc_CycleCnt_t     setCycles  = 0u;
    fmc_CycleCnt_t     waitCycles = 0u;
    fmc_CycleCnt_t     holdCycles = 0u;
    fmc_CycleCnt_t     hizCycles  = 0u;
    const fmc_Phase_t  phases[] =
    {
        { timing->Setup_ns, &fmc_LimNandSet,  &setCycles  }, /**< MEMSET / ATTSET   */
        { timing->Wait_ns,  &fmc_LimNandWait, &waitCycles }, /**< MEMWAIT / ATTWAIT */
        { timing->Hold_ns,  &fmc_LimNandHold, &holdCycles }, /**< MEMHOLD / ATTHOLD */
        { timing->HiZ_ns,   &fmc_LimNandHiz,  &hizCycles  }, /**< MEMHIZ / ATTHIZ   */
    };
    const fmc_RequestState_t phaseState = FmcPriv_Get_Phases( clock, phases, FMC_PHASE_CNT( phases ) );

    if( FMC_REQUEST_OK == phaseState )
    {
        space->Set  = ( setCycles  - FMC_NAND_SET_CYCLES_MIN );
        space->Wait = ( waitCycles - FMC_NAND_WAIT_FIELD_OFFSET );
        space->Hold = holdCycles;
        space->HiZ  = ( hizCycles  - FMC_NAND_HIZ_CYCLES_MIN );
        retState    = FMC_REQUEST_OK;
    }
    else
    {
        /* A time does not fit its field */
        retState = FMC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Converts a delay (ALE or CLE to the read strobe) to the TAR / TCLR field value: the delay is
 *        ( field + setup field + 2 ) cycles (RM0481 23.7.7), the setup field is the smaller one of both spaces.
 *
 * \param delay    [in]  : Delay in nanoseconds.
 * \param clock    [in]  : Clock of the controller (the bus clock).
 * \param setField [in]  : Smaller setup field of the common and the attribute space.
 * \param field    [out] : Pointer receiving the field value.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR when the delay needs a field above 15.
 */
static fmc_RequestState_t FmcPriv_Get_NandDelay( fmc_Time_ns_t delay,
                                                 const fmc_Clock_t * const clock,
                                                 fmc_RegValue_t setField,
                                                 fmc_RegValue_t * const field )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;
    fmc_CycleCnt_t     cycles   = 0u;
    const fmc_RequestState_t cycleState = FmcPriv_Get_Cycles( delay, clock->Clk, clock->Divider, &cycles );
    const fmc_CycleCnt_t     base       = ( setField + FMC_NAND_DELAY_OFFSET );

    if( FMC_REQUEST_OK == cycleState )
    {
        if( cycles > base )
        {
            *field = ( cycles - base );
        }
        else
        {
            /* The setup time already covers the delay */
            *field = 0u;
        }

        if( FMC_NAND_DELAY_FIELD_MAX >= *field )
        {
            retState = FMC_REQUEST_OK;
        }
        else
        {
            /* The delay needs more cycles than the field holds */
            retState = FMC_REQUEST_ERROR;
        }
    }
    else
    {
        /* The delay does not fit */
        retState = FMC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Writes the converted registers of the NAND bank with the controller disabled and verifies them.
 *
 * \param regs [in] : Register values of the bank.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR when the controller or a register does not follow.
 */
static fmc_RequestState_t FmcPriv_Write_NandRegs( const fmc_NandRegs_t * const regs )
{
    fmc_RequestState_t  retState  = FMC_REQUEST_ERROR;
    fmc_FunctionState_t wasActive = FMC_FUNCTION_INACTIVE;
    const fmc_RequestState_t suspendState = FmcPriv_Controller_Suspend( &wasActive );

    if( FMC_REQUEST_OK == suspendState )
    {
        Fmc_Set_RegVal( FMC_REG_PMEM, FMC_REG_MASK_ALL, regs->Pmem );
        Fmc_Set_RegVal( FMC_REG_PATT, FMC_REG_MASK_ALL, regs->Patt );
        Fmc_Set_RegVal( FMC_REG_SR, FMC_SR_ENABLE_MASK, regs->Sr );
        Fmc_Set_RegVal( FMC_REG_PCR, FMC_PCR_CONFIG_MASK, regs->Pcr );

        const fmc_RegValue_t pmemBack = Fmc_Get_RegVal( FMC_REG_PMEM, FMC_REG_MASK_ALL );
        const fmc_RegValue_t pattBack = Fmc_Get_RegVal( FMC_REG_PATT, FMC_REG_MASK_ALL );
        const fmc_RegValue_t srBack   = Fmc_Get_RegVal( FMC_REG_SR, FMC_SR_ENABLE_MASK );
        const fmc_RegValue_t pcrBack  = Fmc_Get_RegVal( FMC_REG_PCR, FMC_PCR_CONFIG_MASK );
        const fmc_RequestState_t resumeState = FmcPriv_Controller_Resume( wasActive );

        if( ( FMC_REQUEST_OK == resumeState ) &&
            ( regs->Pmem     == pmemBack    ) &&
            ( regs->Patt     == pattBack    ) &&
            ( regs->Sr       == srBack      ) &&
            ( regs->Pcr      == pcrBack     )    )
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
 * \brief Checks the state of the module for the ECC functions: the module is initialized, the NAND bank is
 *        configured and its ECC is enabled.
 *
 * \param instance [out] : Pointer receiving the run-time state when the ECC can be used.
 *
 * \return FMC_REQUEST_OK when the ECC can be used, FMC_REQUEST_ERROR otherwise.
 */
static fmc_RequestState_t FmcPriv_Get_EccInstance( fmc_InstanceData_t ** const instance )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;
    fmc_InstanceData_t * data   = FMC_NULL_PTR;
    const fmc_RequestState_t instanceState = FmcPriv_Get_Instance( &data );

    if( ( FMC_REQUEST_OK      == instanceState                    ) &&
        ( FMC_FUNCTION_ACTIVE == data->Initialized                ) &&
        ( FMC_FUNCTION_ACTIVE == data->Configured[FMC_BANK_NAND]  ) &&
        ( FMC_FUNCTION_ACTIVE == data->Nand.EccEnable             )    )
    {
        *instance = data;
        retState  = FMC_REQUEST_OK;
    }
    else
    {
        /* The module is not initialized, the NAND is not configured or its ECC is not enabled */
        retState = FMC_REQUEST_ERROR;
    }

    return ( retState );
}

#endif /* FMC_Bank1_R */

/* ========================= EXPORTED FUNCTIONS =============================== */

#if defined(FMC_Bank1_R)

/**
 * \brief Converts a NAND configuration to the register values (no register is touched).
 *
 * \param config [in]  : Configuration of the NAND bank.
 * \param busClk [in]  : Bus clock in Hz.
 * \param regs   [out] : Pointer receiving the register values.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for an invalid item or a time the clock can not meet.
 */
fmc_RequestState_t FmcPriv_Get_NandRegs( const fmc_NandConfig_t * const config,
                                         fmc_Frequency_Hz_t busClk,
                                         fmc_NandRegs_t * const regs )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;

    if( ( FMC_NULL_PTR != config ) &&
        ( FMC_NULL_PTR != regs   ) &&
        ( 0u           != busClk )    )
    {
        const fmc_Clock_t        clock      = { .Clk = busClk, .Divider = FMC_NO_DIVIDER };
        fmc_NandSpace_t          common     = { 0u };
        fmc_NandSpace_t          attribute  = { 0u };
        fmc_RegValue_t           aleField   = 0u;
        fmc_RegValue_t           cleField   = 0u;
        fmc_RegValue_t           widthField = 0u;
        const fmc_RequestState_t checkState = FmcPriv_Check_NandConfig( config );
        const fmc_RequestState_t commonState = FmcPriv_Get_NandSpace( &( config->CommonTiming ),
                                                                      &clock,
                                                                      &common );
        const fmc_RequestState_t attrState   = FmcPriv_Get_NandSpace( &( config->AttributeTiming ),
                                                                      &clock,
                                                                      &attribute );
        fmc_RegValue_t           minSet      = common.Set;
        fmc_RequestState_t       aleState    = FMC_REQUEST_ERROR;
        fmc_RequestState_t       cleState    = FMC_REQUEST_ERROR;

        if( attribute.Set < common.Set )
        {
            minSet = attribute.Set;
        }
        else
        {
            /* The setup field of the common space is the smaller one */
            minSet = common.Set;
        }

        aleState = FmcPriv_Get_NandDelay( config->AleToReDelay_ns, &clock, minSet, &aleField );
        cleState = FmcPriv_Get_NandDelay( config->CleToReDelay_ns, &clock, minSet, &cleField );

        if( FMC_DATA_WIDTH_16 == config->DataWidth )
        {
            widthField = FMC_PWID_16BIT;
        }
        else
        {
            widthField = FMC_PWID_8BIT;
        }

        if( ( FMC_REQUEST_OK == checkState  ) &&
            ( FMC_REQUEST_OK == commonState ) &&
            ( FMC_REQUEST_OK == attrState   ) &&
            ( FMC_REQUEST_OK == aleState    ) &&
            ( FMC_REQUEST_OK == cleState    )    )
        {
            regs->Pcr = ( ( (fmc_RegValue_t)config->EccPageSize << FMC_PCR_ECCPS_Pos ) |
                          ( aleField                            << FMC_PCR_TAR_Pos   ) |
                          ( cleField                            << FMC_PCR_TCLR_Pos  ) |
                          ( widthField                          << FMC_PCR_PWID_Pos  ) |
                          FMC_PCR_PTYP |
                          FmcPriv_Get_StateMask( config->WaitFeature, FMC_FUNCTION_ACTIVE, FMC_PCR_PWAITEN ) );
            regs->Sr  = ( FmcPriv_Get_StateMask( config->ReadyRisingIrq,  FMC_FUNCTION_ACTIVE, FMC_SR_IREN ) |
                          FmcPriv_Get_StateMask( config->ReadyFallingIrq, FMC_FUNCTION_ACTIVE, FMC_SR_IFEN ) |
                          FmcPriv_Get_StateMask( config->ReadyHighIrq,    FMC_FUNCTION_ACTIVE, FMC_SR_ILEN ) );
            regs->Pmem = ( ( common.HiZ  << FMC_PMEM_MEMHIZ_Pos  ) |
                           ( common.Hold << FMC_PMEM_MEMHOLD_Pos ) |
                           ( common.Wait << FMC_PMEM_MEMWAIT_Pos ) |
                           ( common.Set  << FMC_PMEM_MEMSET_Pos  ) );
            regs->Patt = ( ( attribute.HiZ  << FMC_PATT_ATTHIZ_Pos  ) |
                           ( attribute.Hold << FMC_PATT_ATTHOLD_Pos ) |
                           ( attribute.Wait << FMC_PATT_ATTWAIT_Pos ) |
                           ( attribute.Set  << FMC_PATT_ATTSET_Pos  ) );
            retState = FMC_REQUEST_OK;
        }
        else
        {
            /* The configuration is not valid or the clock can not meet a time */
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
 * \brief Serves the events of the NAND ready signal: every flag that is set and enabled is cleared and reported by
 *        the callback.
 */
void FmcPriv_Nand_ServeEvents( void )
{
    fmc_InstanceData_t * instance      = FMC_NULL_PTR;
    fmc_RequestState_t   instanceState = FmcPriv_Get_Instance( &instance );

    if( ( FMC_REQUEST_OK      == instanceState                   ) &&
        ( FMC_FUNCTION_ACTIVE == instance->Initialized           ) &&
        ( FMC_FUNCTION_ACTIVE == instance->Configured[FMC_BANK_NAND] )    )
    {
        for( uint32_t idx = 0u; ( FMC_NAND_EVENT_CNT > idx ); idx++ )
        {
            const fmc_RegValue_t flagState   = Fmc_Get_RegBit( FMC_REG_SR, fmc_NandEventMap[idx].FlagBit );
            const fmc_RegValue_t enableState = Fmc_Get_RegBit( FMC_REG_SR, fmc_NandEventMap[idx].EnableBit );

            if( ( FMC_REG_BITS_CLEAR != flagState   ) &&
                ( FMC_REG_BITS_CLEAR != enableState )    )
            {
                Fmc_Reset_RegBit( FMC_REG_SR, fmc_NandEventMap[idx].FlagBit );

                if( FMC_NULL_PTR != instance->Config.NandReadyCallback )
                {
                    instance->Config.NandReadyCallback( fmc_NandEventMap[idx].Event );
                }
                else
                {
                    /* No callback, the event is only cleared */
                }
            }
            else
            {
                /* The event did not occur or is not enabled */
            }
        }
    }
    else
    {
        /* The module is not initialized or the NAND is not configured, no action required */
    }
}

#endif /* FMC_Bank1_R */


/**
 * \brief Configures the NAND bank: the data width, the wait feature, the ECC page size, the timing of the common and
 *        the attribute memory space and the delays of ALE and CLE in nanoseconds (converted to bus clock cycles
 *        first, a time the clock can not meet rejects the whole call), the events of the ready signal. The registers
 *        are written with the controller disabled and verified by read-back. The bank stays disabled (see
 *        \ref Fmc_Set_BankActive).
 *
 * \param nandConfig [in] : Pointer to the configuration of the NAND bank.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for an invalid parameter, an uninitialized module, a bank that
 *         is active, a time the clock can not meet or a register that does not read back.
 */
fmc_RequestState_t Fmc_Set_NandConfig( const fmc_NandConfig_t * const nandConfig )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;

#if defined(FMC_Bank1_R)
    fmc_InstanceData_t * instance      = FMC_NULL_PTR;
    fmc_RequestState_t   instanceState = FmcPriv_Get_Instance( &instance );

    if( ( FMC_REQUEST_OK        == instanceState                    ) &&
        ( FMC_NULL_PTR          != nandConfig                       ) &&
        ( FMC_FUNCTION_ACTIVE   == instance->Initialized            ) &&
        ( FMC_FUNCTION_INACTIVE == instance->Active[FMC_BANK_NAND]  )    )
    {
        fmc_Frequency_Hz_t       busClk   = 0u;
        fmc_NandRegs_t           regs;
        const fmc_RequestState_t clkState = FmcPriv_Get_BusClock( &busClk );

        if( FMC_REQUEST_OK == clkState )
        {
            const fmc_RequestState_t regsState = FmcPriv_Get_NandRegs( nandConfig, busClk, &regs );

            if( FMC_REQUEST_OK == regsState )
            {
                retState = FmcPriv_Write_NandRegs( &regs );

                if( FMC_REQUEST_OK == retState )
                {
                    instance->Nand                      = *nandConfig;
                    instance->Configured[FMC_BANK_NAND] = FMC_FUNCTION_ACTIVE;
                    instance->NandEccStarted            = FMC_FUNCTION_INACTIVE;
                }
                else
                {
                    /* The registers were not written as required */
                    instance->Configured[FMC_BANK_NAND] = FMC_FUNCTION_INACTIVE;
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
    (void)nandConfig;
#endif /* FMC_Bank1_R */

    return ( retState );
}


/**
 * \brief Starts the ECC calculation for the next page transfer: the ECC logic is reset (ECCEN cleared) and enabled
 *        again. The NAND bank has to be configured with EccEnable.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for an uninitialized module, a NAND bank without the ECC or a
 *         register that does not read back.
 */
fmc_RequestState_t Fmc_Set_NandEccActive( void )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;

#if defined(FMC_Bank1_R)
    fmc_InstanceData_t * instance      = FMC_NULL_PTR;
    fmc_RequestState_t   instanceState = FmcPriv_Get_EccInstance( &instance );

    if( FMC_REQUEST_OK == instanceState )
    {
        Fmc_Reset_RegBit( FMC_REG_PCR, FMC_PCR_ECCEN );
        retState = Fmc_Reg_Set_RegValVerified( FMC_REG_PCR, FMC_PCR_ECCEN, FMC_PCR_ECCEN );

        if( FMC_REQUEST_OK == retState )
        {
            instance->NandEccStarted = FMC_FUNCTION_ACTIVE;
        }
        else
        {
            /* The ECC logic did not start */
            instance->NandEccStarted = FMC_FUNCTION_INACTIVE;
        }
    }
    else
    {
        /* The module is not initialized or the NAND ECC is not available */
        retState = FMC_REQUEST_ERROR;
    }
#endif /* FMC_Bank1_R */

    return ( retState );
}


/**
 * \brief Returns the ECC of the page that was transferred after \ref Fmc_Set_NandEccActive. The call waits (bounded)
 *        until the write FIFO is empty (the ECC is calculated while the data is written to the memory), reads the code,
 *        keeps the bits valid for the page size and stops the calculation (ECCEN cleared, the code is read once).
 *
 * \param ecc [out] : Pointer receiving the code.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for a null pointer, a calculation that was not started, an
 *         uninitialized module, a NAND bank without the ECC or a FIFO that does not become empty.
 */
fmc_RequestState_t Fmc_Get_NandEcc( fmc_Ecc_t * const ecc )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;

#if defined(FMC_Bank1_R)
    fmc_InstanceData_t * instance      = FMC_NULL_PTR;
    fmc_RequestState_t   instanceState = FmcPriv_Get_EccInstance( &instance );

    if( ( FMC_REQUEST_OK == instanceState ) &&
        ( FMC_NULL_PTR   != ecc           )    )
    {
        if( FMC_FUNCTION_ACTIVE == instance->NandEccStarted )
        {
            const fmc_RequestState_t waitState = Fmc_Reg_Wait_RegBitSet( FMC_REG_SR, FMC_SR_FEMPT );

            if( FMC_REQUEST_OK == waitState )
            {
                const fmc_RegValue_t eccValue = Fmc_Get_RegVal( FMC_REG_ECCR, FMC_REG_MASK_ALL );

                *ecc = ( eccValue & fmc_EccMask[instance->Nand.EccPageSize] );
                Fmc_Reset_RegBit( FMC_REG_PCR, FMC_PCR_ECCEN );
                instance->NandEccStarted = FMC_FUNCTION_INACTIVE;
                retState = FMC_REQUEST_OK;
            }
            else
            {
                /* The write FIFO is not empty, the code is not final */
                retState = FMC_REQUEST_ERROR;
            }
        }
        else
        {
            /* The ECC calculation was not started */
            retState = FMC_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid parameter or state, no action required */
        retState = FMC_REQUEST_ERROR;
    }
#else
    (void)ecc;
#endif /* FMC_Bank1_R */

    return ( retState );
}


/**
 * \brief Reads the ready / busy signal of the NAND flash through the high level detection of the controller: the
 *        detection is enabled for the call, the flag is cleared and polled (bounded); the flag is set again by the
 *        hardware while the signal is high. The flag of the high level event is cleared by the call.
 *
 * \param readyState [out] : Pointer receiving the state.
 *
 * \return FMC_REQUEST_OK on success, FMC_REQUEST_ERROR for a null pointer, an uninitialized module or a NAND bank that
 *         is not configured.
 */
fmc_RequestState_t Fmc_Get_NandReadyState( fmc_NandReadyState_t * const readyState )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;

#if defined(FMC_Bank1_R)
    fmc_InstanceData_t * instance      = FMC_NULL_PTR;
    fmc_RequestState_t   instanceState = FmcPriv_Get_Instance( &instance );

    if( ( FMC_REQUEST_OK      == instanceState                         ) &&
        ( FMC_NULL_PTR        != readyState                            ) &&
        ( FMC_FUNCTION_ACTIVE == instance->Initialized                 ) &&
        ( FMC_FUNCTION_ACTIVE == instance->Configured[FMC_BANK_NAND]   )    )
    {
        const fmc_RegValue_t detectBit = Fmc_Get_RegBit( FMC_REG_SR, FMC_SR_ILEN );
        fmc_RegValue_t       flagValue = FMC_REG_BITS_CLEAR;

        Fmc_Set_RegBit( FMC_REG_SR, FMC_SR_ILEN );
        Fmc_Reset_RegBit( FMC_REG_SR, FMC_SR_ILS );

        for( uint32_t idx = 0u;
             ( FMC_NAND_READY_POLL_CNT > idx           ) &&
             ( FMC_REG_BITS_CLEAR      == flagValue    ) ;
             idx++ )
        {
            flagValue = Fmc_Get_RegBit( FMC_REG_SR, FMC_SR_ILS );
        }

        if( FMC_REG_BITS_CLEAR != flagValue )
        {
            *readyState = FMC_NAND_READY;
        }
        else
        {
            *readyState = FMC_NAND_BUSY;
        }

        Fmc_Reset_RegBit( FMC_REG_SR, FMC_SR_ILS );
        if( FMC_REG_BITS_CLEAR == detectBit )
        {
            /* The detection was off before the call, it is switched off again */
            Fmc_Reset_RegBit( FMC_REG_SR, FMC_SR_ILEN );
        }
        else
        {
            /* The detection is a part of the configuration, it stays enabled */
        }
        retState = FMC_REQUEST_OK;
    }
    else
    {
        /* Invalid parameter or state, no action required */
        retState = FMC_REQUEST_ERROR;
    }
#else
    (void)readyState;
#endif /* FMC_Bank1_R */

    return ( retState );
}
