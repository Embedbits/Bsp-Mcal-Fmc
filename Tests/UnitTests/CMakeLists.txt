#======================== Unit tests configuration ============================#
# Unit tests of Fmc module. Registered in module CMakeLists.txt by
# UnitTesting_AddPath() and created by UnitTesting_Generate() in "UnitTest"
# build (see EmBi_Platform/CMake/UnitTesting/README.md).
#
# The test set is independent - it lists everything it needs: code under test,
# mocked headers, host emulation of the target and libraries built for host.

get_filename_component(FMC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
get_filename_component(MCAL_DIR "${FMC_DIR}/.." ABSOLUTE)

UnitTesting_Add_Test(
    NAME            Fmc
    TEST_SOURCE     Test_Fmc.c
    UUT_SOURCES     ${FMC_DIR}/Fmc.c
                    ${FMC_DIR}/Fmc_Sdram.c
                    ${FMC_DIR}/Fmc_Nand.c
                    ${FMC_DIR}/Fmc_Reg.c
    MOCK_HEADERS    ${MCAL_DIR}/Rcc/Rcc_Port.h
                    ${MCAL_DIR}/Nvic/Nvic_Port.h
                    ${MCAL_DIR}/Gpio/Gpio_Port.h
    INCLUDE_DIRS    ${FMC_DIR}
                    ${MCAL_DIR}/Rcc
                    ${MCAL_DIR}/Nvic
                    ${MCAL_DIR}/Gpio
    LINK_LIBS       ut_RegMem                           # Registers in host memory
                    ut_CmsisHost                        # CMSIS core intrinsics on host
    FORCE_INCLUDES  ${UNIT_TESTING_CMSIS_HOST_HEADER}   # Replaces cmsis_gcc.h
    HOST_LIBS       Ral_Lib                             # ST LL compiled for host
    COVERAGE_EXCLUDE ".*/Ral/.*"
)
