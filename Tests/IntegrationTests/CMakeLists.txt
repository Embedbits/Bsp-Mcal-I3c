#===================== Integration tests configuration ========================#
# Integration tests of I3c module on target (Nucleo boards). Registered in module
# CMakeLists.txt by IntegrationTesting_AddPath() and created by
# IntegrationTesting_Generate() in "IntegrationTest" build
# (see EmBi_Platform/CMake/IntegrationTesting/README.md).
#
# The test set is independent - target interface (entry point, initialization,
# fault handlers, system reset) is implemented by ItTarget_I3c.c and
# all libraries of the firmware are listed here.

# StartUp calls BspMain() - the entry point of the test firmware is provided by
# the test set (BspMain.h, ItTarget_I3c.c)
if(NOT TARGET BspMain_Lib)
    add_library(BspMain_Lib INTERFACE)
    target_include_directories(BspMain_Lib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
endif()

IntegrationTesting_Add_Test(
    NAME            I3c
    TEST_SOURCE     ItTest_I3c.c
    SOURCES         ItTarget_I3c.c  # Target interface of the test set
    LINK_LIBS       StartUp_Lib         # Startup code (calls BspMain)
                    Nvic_Lib            # Fault handlers, system reset (ItTarget)
                    Gpio_Lib
                    Gpdma_Lib
                    Rcc_Lib
                    I3c_Lib
    TIMEOUT         60
)
