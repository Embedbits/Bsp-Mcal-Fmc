/**
 * \author Mr.Nobody
 * \file Fmc_Reg.h
 * \ingroup Fmc
 * \brief Fmc module Reg component header file
 *
 * This component contains the functionality for register operations. It is
 * private to the module (the register access is not shared with other
 * modules). The registers of the controller are spread over four CMSIS
 * structures (NOR / PSRAM, write timing, NAND, SDRAM), they are addressed by one
 * register identification; a bank number is a part of the identification.
 *
 */

#ifndef FMC_REG_FMC_REG_H
#define FMC_REG_FMC_REG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "Fmc.h"                            /* Module private declarations    */
/* ============================= TYPEDEFS =================================== */

/** \brief List of the FMC registers used by the module */
typedef enum
{
#if defined(FMC_Bank1_R)
    FMC_REG_BCR1 = 0u, /**< NOR / PSRAM chip select control register of bank 1 (also the controller enable) */
    FMC_REG_BTR1,      /**< NOR / PSRAM chip select timing register of bank 1                               */
    FMC_REG_BCR2,      /**< NOR / PSRAM chip select control register of bank 2                              */
    FMC_REG_BTR2,      /**< NOR / PSRAM chip select timing register of bank 2                               */
    FMC_REG_BCR3,      /**< NOR / PSRAM chip select control register of bank 3                              */
    FMC_REG_BTR3,      /**< NOR / PSRAM chip select timing register of bank 3                               */
    FMC_REG_BCR4,      /**< NOR / PSRAM chip select control register of bank 4                              */
    FMC_REG_BTR4,      /**< NOR / PSRAM chip select timing register of bank 4                               */
    FMC_REG_BWTR1,     /**< NOR / PSRAM write timing register of bank 1                                     */
    FMC_REG_BWTR2,     /**< NOR / PSRAM write timing register of bank 2                                     */
    FMC_REG_BWTR3,     /**< NOR / PSRAM write timing register of bank 3                                     */
    FMC_REG_BWTR4,     /**< NOR / PSRAM write timing register of bank 4                                     */
    FMC_REG_PCSCNTR,   /**< PSRAM chip select counter register                                              */
    FMC_REG_PCR,       /**< NAND control register                                                           */
    FMC_REG_SR,        /**< NAND FIFO status and interrupt register                                         */
    FMC_REG_PMEM,      /**< NAND common memory space timing register                                        */
    FMC_REG_PATT,      /**< NAND attribute memory space timing register                                     */
    FMC_REG_ECCR,      /**< NAND ECC result register                                                        */
#endif
#if defined(FMC_Bank5_6_R)
    FMC_REG_SDCR1,     /**< SDRAM control register of bank 1                                                */
    FMC_REG_SDCR2,     /**< SDRAM control register of bank 2                                                */
    FMC_REG_SDTR1,     /**< SDRAM timing register of bank 1                                                 */
    FMC_REG_SDTR2,     /**< SDRAM timing register of bank 2                                                 */
    FMC_REG_SDCMR,     /**< SDRAM command mode register                                                     */
    FMC_REG_SDRTR,     /**< SDRAM refresh timer register                                                    */
    FMC_REG_SDSR,      /**< SDRAM status register                                                           */
#endif
    FMC_REG_CNT        /**< Count of available registers                                                    */
}   fmc_RegId_t;

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** \brief Position of the BUSY bit of FMC_SDSR (the bit is not defined in the CMSIS device header) */
#define FMC_REG_SDSR_BUSY_Pos       ( 5u )

/** \brief Mask of the BUSY bit of FMC_SDSR */
#define FMC_REG_SDSR_BUSY           ( 1u << FMC_REG_SDSR_BUSY_Pos )

/** \brief Register value without any bit set */
#define FMC_REG_BITS_CLEAR          ( 0u )

/** \brief Register mask with every bit set */
#define FMC_REG_MASK_ALL            ( 0xFFFFFFFFu )

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

#if defined(FMC_Bank1_R)

/* ---------------------------- Generic accessors --------------------------- */

void                    Fmc_Set_RegBit                  ( fmc_RegId_t regId, fmc_RegValue_t bitMask );
void                    Fmc_Reset_RegBit                ( fmc_RegId_t regId, fmc_RegValue_t bitMask );
fmc_RegValue_t          Fmc_Get_RegBit                  ( fmc_RegId_t regId, fmc_RegValue_t bitMask );

void                    Fmc_Set_RegVal                  ( fmc_RegId_t regId,
                                                          fmc_RegValue_t regMask,
                                                          fmc_RegValue_t regValue );
fmc_RegValue_t          Fmc_Get_RegVal                  ( fmc_RegId_t regId, fmc_RegValue_t regMask );

/* ----------------------------- Register protocol -------------------------- */

fmc_RequestState_t      Fmc_Reg_Wait_RegBitSet          ( fmc_RegId_t regId, fmc_RegValue_t bitMask );
fmc_RequestState_t      Fmc_Reg_Wait_RegBitClr          ( fmc_RegId_t regId, fmc_RegValue_t bitMask );
fmc_RequestState_t      Fmc_Reg_Set_RegValVerified      ( fmc_RegId_t regId,
                                                          fmc_RegValue_t regMask,
                                                          fmc_RegValue_t regValue );

fmc_FunctionState_t     Fmc_Reg_Get_ControllerState     ( void );
fmc_RequestState_t      Fmc_Reg_Set_ControllerState     ( fmc_FunctionState_t controllerState );

#endif /* FMC_Bank1_R */

#ifdef __cplusplus
}
#endif

#endif /* FMC_REG_FMC_REG_H */
