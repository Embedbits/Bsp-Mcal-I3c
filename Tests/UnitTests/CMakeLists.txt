#======================== Unit tests configuration ============================#
# Unit tests of I3c module. Registered in module CMakeLists.txt by
# UnitTesting_AddPath() and created by UnitTesting_Generate() in "UnitTest"
# build (see EmBi_Platform/CMake/UnitTesting/README.md).

get_filename_component(I3C_DIR  "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
get_filename_component(MCAL_DIR "${I3C_DIR}/.." ABSOLUTE)

UnitTesting_Add_Test(
    NAME            I3c
    TEST_SOURCE     Test_I3c.c
    UUT_SOURCES     ${I3C_DIR}/I3c.c
                    ${I3C_DIR}/I3c_Reg.c
                    ${I3C_DIR}/I3c_Ctrl.c
                    ${I3C_DIR}/I3c_Xfer.c
                    ${I3C_DIR}/I3c_Tgt.c
                    ${I3C_DIR}/I3c_Dma.c
    MOCK_HEADERS    ${MCAL_DIR}/Rcc/Rcc_Port.h
                    ${MCAL_DIR}/Nvic/Nvic_Port.h
                    ${MCAL_DIR}/Gpio/Gpio_Port.h
                    ${MCAL_DIR}/Gpdma/Gpdma_Port.h
    INCLUDE_DIRS    ${I3C_DIR}
                    ${MCAL_DIR}/Rcc
                    ${MCAL_DIR}/Nvic
                    ${MCAL_DIR}/Gpio
                    ${MCAL_DIR}/Gpdma
    LINK_LIBS       ut_RegMem                           # Registers in host memory
                    ut_CmsisHost                        # CMSIS core intrinsics on host
    FORCE_INCLUDES  ${UNIT_TESTING_CMSIS_HOST_HEADER}   # Replaces cmsis_gcc.h
    HOST_LIBS       Ral_Lib                             # ST LL compiled for host
    COVERAGE_EXCLUDE ".*/Ral/.*"
)
