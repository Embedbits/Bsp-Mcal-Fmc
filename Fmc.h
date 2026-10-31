/**
 * \author Mr.Nobody
 * \file Fmc.h
 * \ingroup Fmc
 * \brief Fmc module private declarations
 *
 * Internal state and helpers of the module, not part of the public interface
 * (Fmc_Port.h). Shared between Fmc.c, Fmc_Sdram.c, Fmc_Nand.c and Fmc_Reg.c. The
 * conversions of the timing in nanoseconds to the register values are pure functions
 * (no register is touched), so that they can be checked without hardware.
 *
 */

#ifndef FMC_FMC_H
#define FMC_FMC_H
/* ============================== INCLUDES ================================== */
#include "Fmc_Port.h"                       /* Own port file include          */
/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Number of the loop iterations of a bounded wait on a register flag before a timeout is declared */
#define FMC_REG_WAIT_LOOP_CNT                 ( 100000u )

/** Shortest power-up delay of an SDRAM after the clock start (microseconds) */
#define FMC_SDRAM_POWER_UP_DELAY_MIN_US       ( 100u )

/** Number of the auto-refresh cycles of the SDRAM initialization sequence */
#define FMC_SDRAM_INIT_AUTO_REFRESH_CNT       ( 8u )

/** Divider of a time conversion when the clock of the memory is the bus clock itself */
#define FMC_NO_DIVIDER                        ( 1u )

/** Number of the nanoseconds in a second */
#define FMC_NS_PER_S                          ( 1000000000u )

/** Number of the microseconds in a second */
#define FMC_US_PER_S                          ( 1000000u )

/** Number of the milliseconds in a second */
#define FMC_MS_PER_S                          ( 1000u )

/** Number of the entries of a list of phases */
#define FMC_PHASE_CNT( PHASES )               ( (fmc_CycleCnt_t)( sizeof( PHASES ) / sizeof( ( PHASES )[0] ) ) )

/* ============================== TYPEDEFS ================================== */

/** Raw value of a register (or of a register field) */
typedef uint32_t fmc_RegValue_t;

/** Count of the clock cycles in a timing calculation */
typedef uint32_t fmc_CycleCnt_t;

/** Clock of a memory: the bus clock and the divider of the memory clock (the memory clock is Clk / Divider) */
typedef struct
{
    fmc_Frequency_Hz_t Clk;     /**< Bus clock in Hz                             */
    fmc_CycleCnt_t     Divider; /**< Divider of the memory clock, at least 1     */
}   fmc_Clock_t;

/** Smallest and largest number of the cycles of a phase of an access */
typedef struct
{
    fmc_CycleCnt_t Min; /**< Smallest number of the cycles the controller can generate */
    fmc_CycleCnt_t Max; /**< Largest number of the cycles the register field holds     */
}   fmc_PhaseLimit_t;

/** A phase of an access to convert: the time, the limits and the result of the conversion */
typedef struct
{
    fmc_Time_ns_t            Time;   /**< Time of the phase in nanoseconds                  */
    const fmc_PhaseLimit_t * Limit;  /**< Limits of the phase                               */
    fmc_CycleCnt_t *         Cycles; /**< Pointer receiving the number of the cycles        */
}   fmc_Phase_t;

/** Register values of a NOR / PSRAM / SRAM bank calculated from a configuration */
typedef struct
{
    fmc_RegValue_t      Bcr;       /**< Control register value (without the bits common to the banks and MBKEN) */
    fmc_RegValue_t      Btr;       /**< Timing register value (read timing)                                     */
    fmc_RegValue_t      Bwtr;      /**< Write timing register value (reset value when no extended mode)         */
    fmc_FunctionState_t CsCounter; /**< The chip select counter of the bank is used                             */
    fmc_RegValue_t      CsCount;   /**< Chip select counter value in bus clock cycles                           */
}   fmc_NorSramRegs_t;

/** Register values of the NAND bank calculated from a configuration */
typedef struct
{
    fmc_RegValue_t Pcr;  /**< Control register value (ECC logic and bank disabled)         */
    fmc_RegValue_t Sr;   /**< Interrupt enable bits of the status register                 */
    fmc_RegValue_t Pmem; /**< Common memory space timing register value                    */
    fmc_RegValue_t Patt; /**< Attribute memory space timing register value                 */
}   fmc_NandRegs_t;

/** Timing of an SDRAM bank in cycles of the SDRAM clock (the number of the cycles, not the register values) */
typedef struct
{
    fmc_CycleCnt_t LoadToActive;    /**< tMRD in cycles                 */
    fmc_CycleCnt_t ExitSelfRefresh; /**< tXSR in cycles                 */
    fmc_CycleCnt_t SelfRefresh;     /**< tRAS in cycles                 */
    fmc_CycleCnt_t RowCycle;        /**< tRC in cycles                  */
    fmc_CycleCnt_t WriteRecovery;   /**< tWR in cycles                  */
    fmc_CycleCnt_t RowPrecharge;    /**< tRP in cycles                  */
    fmc_CycleCnt_t RowToColumn;     /**< tRCD in cycles                 */
}   fmc_SdramCycles_t;

#if defined(FMC_Bank1_R)

/** Run-time state of the module */
typedef struct
{
    fmc_FunctionState_t Initialized;                       /**< Module initialized by Fmc_Init               */
    fmc_Config_t        Config;                            /**< Copy of the configuration (no pointers)      */
    fmc_FunctionState_t Configured[ FMC_BANK_CNT ];        /**< Bank configured by its configuration function */
    fmc_FunctionState_t Active[ FMC_BANK_CNT ];            /**< Bank activated by Fmc_Set_BankActive         */
    fmc_NandConfig_t    Nand;                              /**< Configuration of the NAND bank               */
    fmc_FunctionState_t NandEccStarted;                    /**< ECC calculation started and not read yet     */
#if defined(FMC_Bank5_6_R)
    fmc_SdramConfig_t   Sdram[ FMC_SDRAM_BANK_CNT ];       /**< Configuration of the SDRAM banks             */
    fmc_SdramCycles_t   SdramCycles[ FMC_SDRAM_BANK_CNT ]; /**< Timing of the SDRAM banks in cycles          */
    fmc_FunctionState_t SdramReady[ FMC_SDRAM_BANK_CNT ];  /**< SDRAM bank initialized by Fmc_Set_SdramInit  */
#endif
}   fmc_InstanceData_t;

#endif /* FMC_Bank1_R */

/* ========================= EXPORTED VARIABLES ============================= */
/* ======================== EXPORTED FUNCTIONS ============================== */

#if defined(FMC_Bank1_R)

/* --------------------------- Module state and clock ------------------------ */

fmc_RequestState_t      FmcPriv_Get_Instance            ( fmc_InstanceData_t ** const instance );
fmc_RequestState_t      FmcPriv_Get_BusClock            ( fmc_Frequency_Hz_t * const busClk );
fmc_RequestState_t      FmcPriv_Get_Cycles              ( fmc_Time_ns_t time,
                                                          fmc_Frequency_Hz_t clk,
                                                          fmc_CycleCnt_t divider,
                                                          fmc_CycleCnt_t * const cycles );
fmc_RequestState_t      FmcPriv_Get_PhaseCycles         ( fmc_Time_ns_t time,
                                                          const fmc_Clock_t * const clock,
                                                          const fmc_PhaseLimit_t * const limit,
                                                          fmc_CycleCnt_t * const cycles );
fmc_RequestState_t      FmcPriv_Get_Phases              ( const fmc_Clock_t * const clock,
                                                          const fmc_Phase_t * const phases,
                                                          fmc_CycleCnt_t phaseCnt );
fmc_RegValue_t          FmcPriv_Get_StateMask           ( fmc_FunctionState_t state,
                                                          fmc_FunctionState_t matchState,
                                                          fmc_RegValue_t bitMask );
fmc_RequestState_t      FmcPriv_Get_WidthField          ( fmc_DataWidth_t dataWidth,
                                                          fmc_RegValue_t * const widthField );
fmc_RequestState_t      FmcPriv_Set_Delay               ( fmc_Time_us_t delay );
fmc_RequestState_t      FmcPriv_Controller_Suspend      ( fmc_FunctionState_t * const wasActive );
fmc_RequestState_t      FmcPriv_Controller_Resume       ( fmc_FunctionState_t wasActive );

/* ------------------------ Register values of the banks --------------------- */

fmc_RequestState_t      FmcPriv_Get_NorSramRegs         ( const fmc_NorSramConfig_t * const config,
                                                          fmc_Frequency_Hz_t busClk,
                                                          fmc_NorSramRegs_t * const regs );
fmc_RequestState_t      FmcPriv_Get_NandRegs            ( const fmc_NandConfig_t * const config,
                                                          fmc_Frequency_Hz_t busClk,
                                                          fmc_NandRegs_t * const regs );

/* ----------------------------- Events of the banks ------------------------- */

void                    FmcPriv_Nand_ServeEvents        ( void );

#if defined(FMC_Bank5_6_R)

fmc_RequestState_t      FmcPriv_Get_SdramCycles         ( const fmc_SdramConfig_t * const config,
                                                          fmc_Frequency_Hz_t busClk,
                                                          fmc_SdramCycles_t * const cycles );
fmc_RequestState_t      FmcPriv_Get_SdramRefreshCount   ( fmc_Time_ms_t refreshPeriod,
                                                          fmc_RowCnt_t refreshRows,
                                                          fmc_Frequency_Hz_t busClk,
                                                          fmc_CycleCnt_t divider,
                                                          fmc_CycleCnt_t cycleSum,
                                                          fmc_RegValue_t * const count );
fmc_RequestState_t      FmcPriv_Get_SdramDivider        ( fmc_SdramClockDiv_t clockDivider,
                                                          fmc_CycleCnt_t * const divider );
fmc_ModeRegister_t      FmcPriv_Get_SdramModeRegister   ( const fmc_SdramConfig_t * const config );
fmc_RequestState_t      FmcPriv_Sdram_Activate          ( fmc_Bank_t bank );
fmc_RequestState_t      FmcPriv_Sdram_Set_WriteProtection ( fmc_Bank_t bank, fmc_FunctionState_t protectionState );
void                    FmcPriv_Sdram_ServeEvents       ( void );

#endif /* FMC_Bank5_6_R */

#endif /* FMC_Bank1_R */

#endif /* FMC_FMC_H */
