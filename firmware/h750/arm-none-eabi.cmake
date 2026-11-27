# Bare-metal ARM toolchain. Set ARM_TOOLCHAIN_PATH (dir containing
# arm-none-eabi-gcc) or have it on PATH. Common install dirs are tried too.
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

find_program(ARM_GCC arm-none-eabi-gcc
  HINTS $ENV{ARM_TOOLCHAIN_PATH} ${ARM_TOOLCHAIN_PATH}
        $ENV{HOME}/.pico-sdk/toolchain/14_2_Rel1/bin
        /opt/st/stm32cubeclt_1.16.0/GNU-tools-for-STM32/bin
  REQUIRED)
get_filename_component(ARM_BIN ${ARM_GCC} DIRECTORY)

set(CMAKE_C_COMPILER   ${ARM_BIN}/arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER ${ARM_BIN}/arm-none-eabi-gcc)
set(CMAKE_OBJCOPY      ${ARM_BIN}/arm-none-eabi-objcopy CACHE FILEPATH "")
set(CMAKE_SIZE         ${ARM_BIN}/arm-none-eabi-size CACHE FILEPATH "")
