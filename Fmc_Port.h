/**
 * \author Mr.Nobody
 * \file Fmc_Port.h
 * \ingroup Fmc
 * \brief Fmc module public interface header file
 *
 * This file contains the public interface of the module, identical for all
 * supported families. Include this file to use the module from an application
 * or from another module. Every function is documented at its definition in
 * Fmc.c, Fmc_Sdram.c and Fmc_Nand.c (on STM32H503, which has no FMC, every
 * function returns an error, on STM32H523 / H533 the SDRAM functions do).
 *
 */

#ifndef FMC_FMC_PORT_H
#define FMC_FMC_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "Fmc_Types.h"                      /* Module types definition        */
/* ============================= TYPEDEFS =================================== */
/* ========================= SYMBOLIC CONSTANTS ============================= */
/* ========================= EXPORTED MACROS ================================ */
/* ========================= EXPORTED VARIABLES ============================= */
/* ======================== EXPORTED FUNCTIONS ============================== */

/* --------------------------- Module management ----------------------------- */

fmc_ModuleVersion_t     Fmc_Get_ModuleVersion          ( void );

fmc_RequestState_t      Fmc_Init                       ( const fmc_Config_t * const fmcConfig );
fmc_RequestState_t      Fmc_Deinit                     ( void );
void                    Fmc_Task                       ( void );

fmc_RequestState_t      Fmc_Get_DefaultConfig          ( fmc_Config_t * const fmcConfig );

/* --------------------------------- Banks ----------------------------------- */

fmc_RequestState_t      Fmc_Set_NorSramConfig          ( fmc_Bank_t bank,
                                                         const fmc_NorSramConfig_t * const norSramConfig );
fmc_RequestState_t      Fmc_Set_NandConfig             ( const fmc_NandConfig_t * const nandConfig );
fmc_RequestState_t      Fmc_Set_SdramConfig            ( fmc_Bank_t bank,
                                                         const fmc_SdramConfig_t * const sdramConfig );

fmc_RequestState_t      Fmc_Set_BankActive             ( fmc_Bank_t bank );
fmc_RequestState_t      Fmc_Set_BankInactive           ( fmc_Bank_t bank );
fmc_RequestState_t      Fmc_Get_BankBaseAddress        ( fmc_Bank_t bank, fmc_Address_t * const baseAddress );
fmc_RequestState_t      Fmc_Set_BankWriteProtection    ( fmc_Bank_t bank, fmc_FunctionState_t protectionState );

/* --------------------------------- SDRAM ----------------------------------- */

fmc_RequestState_t      Fmc_Set_SdramInit              ( fmc_Bank_t bank );
fmc_RequestState_t      Fmc_Set_SdramRefreshRate       ( fmc_Time_ms_t refreshPeriod, fmc_RowCnt_t refreshRows );
fmc_RequestState_t      Fmc_Set_SdramSelfRefresh       ( fmc_Bank_t bank );
fmc_RequestState_t      Fmc_Set_SdramPowerDown         ( fmc_Bank_t bank );
fmc_RequestState_t      Fmc_Set_SdramCommand           ( fmc_Bank_t bank,
                                                         const fmc_SdramCommand_t * const sdramCommand );
fmc_RequestState_t      Fmc_Get_SdramState             ( fmc_Bank_t bank, fmc_SdramState_t * const sdramState );

/* ---------------------------------- NAND ----------------------------------- */

fmc_RequestState_t      Fmc_Set_NandEccActive          ( void );
fmc_RequestState_t      Fmc_Get_NandEcc                ( fmc_Ecc_t * const ecc );
fmc_RequestState_t      Fmc_Get_NandReadyState         ( fmc_NandReadyState_t * const readyState );

/* -------------------------------- Interrupts -------------------------------- */

fmc_RequestState_t      Fmc_Set_IrqPriority            ( fmc_IrqPrio_t irqPriority );
fmc_RequestState_t      Fmc_Get_IrqPriority            ( fmc_IrqPrio_t * const irqPriority );

/* ------------------------------------ Pins ----------------------------------- */

fmc_RequestState_t      Fmc_InitAddressGpio            ( fmc_AddressPin_t addressPin );
fmc_RequestState_t      Fmc_InitDataGpio               ( fmc_DataPin_t dataPin );
fmc_RequestState_t      Fmc_InitControlGpio            ( fmc_ControlPin_t controlPin );
fmc_RequestState_t      Fmc_InitSdramGpio              ( fmc_SdramPin_t sdramPin );

#ifdef __cplusplus
}
#endif

#endif /* FMC_FMC_PORT_H */
