/**
 * \author Mr.Nobody
 * \file Fmc_Reg.c
 * \ingroup Fmc
 * \brief Fmc module Reg component source file
 *
 * This component contains the functionality for register operations of the
 * flexible memory controller. The registers are written only here and in the
 * module files through these accessors, the masks come from the CMSIS device
 * header. Properties of the registers, which the protocol respects (RM0481 /
 * RM0517, FMC chapter):
 * - FMC_BCR1.FMCEN enables the whole controller (NOR / PSRAM, NAND and SDRAM);
 *   FMC_BCRx.MBKEN, FMC_BCRx.WREN, FMC_PCR.ECCEN, FMC_PCR.PBKEN and the flags of
 *   FMC_SR may change while it is enabled, every other register is changed
 *   with the controller disabled.
 * - FMC_SDSR.BUSY is set while the SDRAM controller executes a command.
 *
 */

/* ============================= INCLUDES =================================== */
#include "Fmc_Reg.h"                        /* Module Reg component header    */
#include "Stm32.h"                          /* Device registers definition    */

#if defined(FMC_Bank1_R)

/* ============================== TYPEDEFS ================================== */

/** \brief Structure type used to store the physical address of an FMC register.
 * The registers are referenced through the enumeration \ref fmc_RegId_t.
 */
typedef struct
{
    fmc_RegId_t         RegId;   /**< Peripheral register ID      */
    volatile uint32_t * RegAddr; /**< Peripheral register address */
}   fmc_RegList_t;

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** \brief Index of the NOR / PSRAM bank 1 */
#define FMC_NOR_IDX_1               ( 0u )

/** \brief Index of the NOR / PSRAM bank 2 */
#define FMC_NOR_IDX_2               ( 1u )

/** \brief Index of the NOR / PSRAM bank 3 */
#define FMC_NOR_IDX_3               ( 2u )

/** \brief Index of the NOR / PSRAM bank 4 */
#define FMC_NOR_IDX_4               ( 3u )

/** \brief Number of the registers of a bank in the CMSIS arrays BTCR and BWTR (control and timing register) */
#define FMC_NOR_REG_PER_BANK        ( 2u )

/** \brief Index of the control register of a NOR / PSRAM bank in the CMSIS array BTCR */
#define FMC_BTCR_BCR( NOR_IDX )     ( ( NOR_IDX ) * FMC_NOR_REG_PER_BANK )

/** \brief Index of the timing register of a NOR / PSRAM bank in the CMSIS array BTCR */
#define FMC_BTCR_BTR( NOR_IDX )     ( ( ( NOR_IDX ) * FMC_NOR_REG_PER_BANK ) + 1u )

/** \brief Index of the write timing register of a NOR / PSRAM bank in the CMSIS array BWTR */
#define FMC_BWTR_IDX( NOR_IDX )     ( ( NOR_IDX ) * FMC_NOR_REG_PER_BANK )

/* =========================== LOCAL VARIABLES ============================== */

/** \brief FMC registers configuration structure, the address of every register of \ref fmc_RegId_t */
static const fmc_RegList_t fmc_RegList[] =
{
    { .RegId = FMC_REG_BCR1, .RegAddr = &( FMC_Bank1_R->BTCR[ FMC_BTCR_BCR( FMC_NOR_IDX_1 ) ] ) }, /**< Control 1 */
    { .RegId = FMC_REG_BTR1, .RegAddr = &( FMC_Bank1_R->BTCR[ FMC_BTCR_BTR( FMC_NOR_IDX_1 ) ] ) }, /**< Timing 1  */
    { .RegId = FMC_REG_BCR2, .RegAddr = &( FMC_Bank1_R->BTCR[ FMC_BTCR_BCR( FMC_NOR_IDX_2 ) ] ) }, /**< Control 2 */
    { .RegId = FMC_REG_BTR2, .RegAddr = &( FMC_Bank1_R->BTCR[ FMC_BTCR_BTR( FMC_NOR_IDX_2 ) ] ) }, /**< Timing 2  */
    { .RegId = FMC_REG_BCR3, .RegAddr = &( FMC_Bank1_R->BTCR[ FMC_BTCR_BCR( FMC_NOR_IDX_3 ) ] ) }, /**< Control 3 */
    { .RegId = FMC_REG_BTR3, .RegAddr = &( FMC_Bank1_R->BTCR[ FMC_BTCR_BTR( FMC_NOR_IDX_3 ) ] ) }, /**< Timing 3  */
    { .RegId = FMC_REG_BCR4, .RegAddr = &( FMC_Bank1_R->BTCR[ FMC_BTCR_BCR( FMC_NOR_IDX_4 ) ] ) }, /**< Control 4 */
    { .RegId = FMC_REG_BTR4, .RegAddr = &( FMC_Bank1_R->BTCR[ FMC_BTCR_BTR( FMC_NOR_IDX_4 ) ] ) }, /**< Timing 4  */
    { .RegId = FMC_REG_BWTR1, .RegAddr = &( FMC_Bank1E_R->BWTR[ FMC_BWTR_IDX( FMC_NOR_IDX_1 ) ] ) }, /**< Write 1 */
    { .RegId = FMC_REG_BWTR2, .RegAddr = &( FMC_Bank1E_R->BWTR[ FMC_BWTR_IDX( FMC_NOR_IDX_2 ) ] ) }, /**< Write 2 */
    { .RegId = FMC_REG_BWTR3, .RegAddr = &( FMC_Bank1E_R->BWTR[ FMC_BWTR_IDX( FMC_NOR_IDX_3 ) ] ) }, /**< Write 3 */
    { .RegId = FMC_REG_BWTR4, .RegAddr = &( FMC_Bank1E_R->BWTR[ FMC_BWTR_IDX( FMC_NOR_IDX_4 ) ] ) }, /**< Write 4 */
    { .RegId = FMC_REG_PCSCNTR, .RegAddr = &( FMC_Bank1_R->PCSCNTR ) }, /**< PSRAM chip select counter   */
    { .RegId = FMC_REG_PCR,     .RegAddr = &( FMC_Bank3_R->PCR     ) }, /**< NAND control                */
    { .RegId = FMC_REG_SR,      .RegAddr = &( FMC_Bank3_R->SR      ) }, /**< NAND status and interrupt   */
    { .RegId = FMC_REG_PMEM,    .RegAddr = &( FMC_Bank3_R->PMEM    ) }, /**< NAND common space timing    */
    { .RegId = FMC_REG_PATT,    .RegAddr = &( FMC_Bank3_R->PATT    ) }, /**< NAND attribute space timing */
    { .RegId = FMC_REG_ECCR,    .RegAddr = &( FMC_Bank3_R->ECCR    ) }, /**< NAND ECC result             */
#if defined(FMC_Bank5_6_R)
    { .RegId = FMC_REG_SDCR1, .RegAddr = &( FMC_Bank5_6_R->SDCR[0] ) }, /**< SDRAM control, bank 1 */
    { .RegId = FMC_REG_SDCR2, .RegAddr = &( FMC_Bank5_6_R->SDCR[1] ) }, /**< SDRAM control, bank 2 */
    { .RegId = FMC_REG_SDTR1, .RegAddr = &( FMC_Bank5_6_R->SDTR[0] ) }, /**< SDRAM timing, bank 1  */
    { .RegId = FMC_REG_SDTR2, .RegAddr = &( FMC_Bank5_6_R->SDTR[1] ) }, /**< SDRAM timing, bank 2  */
    { .RegId = FMC_REG_SDCMR, .RegAddr = &( FMC_Bank5_6_R->SDCMR   ) }, /**< SDRAM command mode    */
    { .RegId = FMC_REG_SDRTR, .RegAddr = &( FMC_Bank5_6_R->SDRTR   ) }, /**< SDRAM refresh timer   */
    { .RegId = FMC_REG_SDSR,  .RegAddr = &( FMC_Bank5_6_R->SDSR    ) }, /**< SDRAM status          */
#endif
};

_Static_assert( ( sizeof( fmc_RegList ) / sizeof( fmc_RegList_t ) ) == FMC_REG_CNT,
                 "Fmc_Reg: fmc_RegList has incorrect size." );

/* ========================== PRIVATE FUNCTIONS ============================== */

/**
 * \brief Returns the address of a register.
 *
 * \param regId [in] : Register identification.
 *
 * \return Address of the register, or FMC_NULL_PTR for an invalid register identification.
 */
static volatile uint32_t * Fmc_Reg_GetAddr( fmc_RegId_t regId )
{
    volatile uint32_t * regAddr = FMC_NULL_PTR;

    if( FMC_REG_CNT > regId )
    {
        regAddr = fmc_RegList[regId].RegAddr;
    }
    else
    {
        /* Invalid register ID, no address is returned */
        regAddr = FMC_NULL_PTR;
    }

    return ( regAddr );
}

/* ========================= EXPORTED FUNCTIONS =============================== */

/**
 * \brief Sets the register bit(s) with the given mask.
 *
 * \param regId   [in] : FMC register ID
 * \param bitMask [in] : Mask to apply to the register
 */
void Fmc_Set_RegBit( fmc_RegId_t regId, fmc_RegValue_t bitMask )
{
    volatile uint32_t * regAddr = Fmc_Reg_GetAddr( regId );

    if( FMC_NULL_PTR != regAddr )
    {
        *regAddr |= bitMask;
    }
    else
    {
        /* Invalid register ID, no action required */
    }
}


/**
 * \brief Resets the register bit(s) with the given mask.
 *
 * \param regId   [in] : FMC register ID
 * \param bitMask [in] : Mask to apply to the register
 */
void Fmc_Reset_RegBit( fmc_RegId_t regId, fmc_RegValue_t bitMask )
{
    volatile uint32_t * regAddr = Fmc_Reg_GetAddr( regId );

    if( FMC_NULL_PTR != regAddr )
    {
        *regAddr &= ~bitMask;
    }
    else
    {
        /* Invalid register ID, no action required */
    }
}


/**
 * \brief Gets the register bit(s) value with the given mask.
 *
 * \param regId   [in] : FMC register ID
 * \param bitMask [in] : Mask to apply to the register
 *
 * \return Register bit value with applied mask, FMC_REG_BITS_CLEAR for an invalid register ID.
 */
fmc_RegValue_t Fmc_Get_RegBit( fmc_RegId_t regId, fmc_RegValue_t bitMask )
{
    volatile uint32_t * regAddr  = Fmc_Reg_GetAddr( regId );
    fmc_RegValue_t      retValue = FMC_REG_BITS_CLEAR;

    if( FMC_NULL_PTR != regAddr )
    {
        retValue = ( *regAddr & bitMask );
    }
    else
    {
        /* Invalid register ID, no bit is set */
        retValue = FMC_REG_BITS_CLEAR;
    }

    return ( retValue );
}


/**
 * \brief Updates the register value with the given mask and value.
 *
 * \param regId    [in] : FMC register ID
 * \param regMask  [in] : Mask to apply to the register
 * \param regValue [in] : Value to set in the register
 */
void Fmc_Set_RegVal( fmc_RegId_t regId, fmc_RegValue_t regMask, fmc_RegValue_t regValue )
{
    volatile uint32_t * regAddr = Fmc_Reg_GetAddr( regId );

    if( FMC_NULL_PTR != regAddr )
    {
        *regAddr = ( ( *regAddr & ~regMask ) | ( regValue & regMask ) );
    }
    else
    {
        /* Invalid register ID, no action required */
    }
}


/**
 * \brief Gets the register value with the given mask.
 *
 * \param regId   [in] : FMC register ID
 * \param regMask [in] : Mask to apply to the register
 *
 * \return Register value with applied mask, FMC_REG_BITS_CLEAR for an invalid register ID.
 */
fmc_RegValue_t Fmc_Get_RegVal( fmc_RegId_t regId, fmc_RegValue_t regMask )
{
    volatile uint32_t * regAddr  = Fmc_Reg_GetAddr( regId );
    fmc_RegValue_t      retValue = FMC_REG_BITS_CLEAR;

    if( FMC_NULL_PTR != regAddr )
    {
        retValue = ( *regAddr & regMask );
    }
    else
    {
        /* Invalid register ID, no bit is set */
        retValue = FMC_REG_BITS_CLEAR;
    }

    return ( retValue );
}


/**
 * \brief Waits (bounded) until every bit of the mask is set in the register.
 *
 * \param regId   [in] : FMC register ID
 * \param bitMask [in] : Mask of the bit(s) to wait for
 *
 * \return FMC_REQUEST_OK once every bit of the mask is set, FMC_REQUEST_ERROR on timeout.
 */
fmc_RequestState_t Fmc_Reg_Wait_RegBitSet( fmc_RegId_t regId, fmc_RegValue_t bitMask )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;
    fmc_RegValue_t     bitValue = FMC_REG_BITS_CLEAR;

    for( uint32_t idx = 0u;
         ( FMC_REG_WAIT_LOOP_CNT > idx ) &&
         ( bitMask              != bitValue ) ;
         idx++ )
    {
        bitValue = Fmc_Get_RegBit( regId, bitMask );
    }

    if( bitMask == bitValue )
    {
        retState = FMC_REQUEST_OK;
    }
    else
    {
        retState = FMC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Waits (bounded) until every bit of the mask is cleared in the register.
 *
 * \param regId   [in] : FMC register ID
 * \param bitMask [in] : Mask of the bit(s) to wait for
 *
 * \return FMC_REQUEST_OK once every bit of the mask is cleared, FMC_REQUEST_ERROR on timeout.
 */
fmc_RequestState_t Fmc_Reg_Wait_RegBitClr( fmc_RegId_t regId, fmc_RegValue_t bitMask )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;
    fmc_RegValue_t     bitValue = bitMask;

    for( uint32_t idx = 0u;
         ( FMC_REG_WAIT_LOOP_CNT > idx ) &&
         ( FMC_REG_BITS_CLEAR   != bitValue ) ;
         idx++ )
    {
        bitValue = Fmc_Get_RegBit( regId, bitMask );
    }

    if( FMC_REG_BITS_CLEAR == bitValue )
    {
        retState = FMC_REQUEST_OK;
    }
    else
    {
        retState = FMC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Updates the register value with the given mask and value and verifies it by read-back.
 *
 * \param regId    [in] : FMC register ID
 * \param regMask  [in] : Mask to apply to the register
 * \param regValue [in] : Value to set in the register
 *
 * \return FMC_REQUEST_OK when the masked register reads back the requested value, FMC_REQUEST_ERROR otherwise.
 */
fmc_RequestState_t Fmc_Reg_Set_RegValVerified( fmc_RegId_t regId, fmc_RegValue_t regMask, fmc_RegValue_t regValue )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;
    fmc_RegValue_t     readBack = FMC_REG_BITS_CLEAR;

    Fmc_Set_RegVal( regId, regMask, regValue );
    readBack = Fmc_Get_RegVal( regId, regMask );

    if( ( regValue & regMask ) == readBack )
    {
        retState = FMC_REQUEST_OK;
    }
    else
    {
        retState = FMC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns the state of the controller (FMC_BCR1.FMCEN).
 *
 * \return FMC_FUNCTION_ACTIVE if the controller is enabled, otherwise FMC_FUNCTION_INACTIVE.
 */
fmc_FunctionState_t Fmc_Reg_Get_ControllerState( void )
{
    fmc_FunctionState_t retState = FMC_FUNCTION_INACTIVE;
    const fmc_RegValue_t bitValue = Fmc_Get_RegBit( FMC_REG_BCR1, FMC_BCR1_FMCEN );

    if( FMC_REG_BITS_CLEAR != bitValue )
    {
        retState = FMC_FUNCTION_ACTIVE;
    }
    else
    {
        retState = FMC_FUNCTION_INACTIVE;
    }

    return ( retState );
}


/**
 * \brief Enables or disables the controller (FMC_BCR1.FMCEN) and verifies it by read-back.
 *
 * \param controllerState [in] : FMC_FUNCTION_ACTIVE enables, FMC_FUNCTION_INACTIVE disables the controller.
 *
 * \return FMC_REQUEST_OK if the bit reads back as requested, FMC_REQUEST_ERROR otherwise.
 */
fmc_RequestState_t Fmc_Reg_Set_ControllerState( fmc_FunctionState_t controllerState )
{
    fmc_RequestState_t retState = FMC_REQUEST_ERROR;
    fmc_RegValue_t     regValue = FMC_REG_BITS_CLEAR;

    if( FMC_FUNCTION_ACTIVE == controllerState )
    {
        regValue = FMC_BCR1_FMCEN;
    }
    else
    {
        regValue = FMC_REG_BITS_CLEAR;
    }

    retState = Fmc_Reg_Set_RegValVerified( FMC_REG_BCR1, FMC_BCR1_FMCEN, regValue );

    return ( retState );
}

#endif /* FMC_Bank1_R */
