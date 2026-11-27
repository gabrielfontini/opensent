# OpenSENT firmware for the WeAct STM32H750VBT6 core board.
#
#   cmake -S firmware/h750 -B build-h750 -G Ninja && cmake --build build-h750
#
# Dependencies (found automatically, or fetched):
#   CMSIS_H7_PATH   cmsis_device_h7 checkout (headers + startup + system file)
#   CMSIS_CORE_PATH cmsis_core checkout
#   TINYUSB_PATH    TinyUSB source tree (the Pico SDK's copy works)
cmake_minimum_required(VERSION 3.20)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_LIST_DIR}/arm-none-eabi.cmake)
project(opensent_h750 C ASM)
set(CMAKE_C_STANDARD 11)
include(FetchContent)

set(CMSIS_H7_PATH "" CACHE PATH "cmsis_device_h7")
set(CMSIS_CORE_PATH "" CACHE PATH "cmsis_core")
if(NOT CMSIS_H7_PATH)
  FetchContent_Declare(cmsis_device_h7
    GIT_REPOSITORY https://github.com/STMicroelectronics/cmsis_device_h7.git
    GIT_TAG master GIT_SHALLOW TRUE SOURCE_SUBDIR none)
  FetchContent_MakeAvailable(cmsis_device_h7)
  set(CMSIS_H7_PATH ${cmsis_device_h7_SOURCE_DIR})
endif()
if(NOT CMSIS_CORE_PATH)
  FetchContent_Declare(cmsis_core
    GIT_REPOSITORY https://github.com/STMicroelectronics/cmsis_core.git
    GIT_TAG master GIT_SHALLOW TRUE SOURCE_SUBDIR none)
  FetchContent_MakeAvailable(cmsis_core)
  set(CMSIS_CORE_PATH ${cmsis_core_SOURCE_DIR})
endif()

set(TINYUSB_PATH "" CACHE PATH "TinyUSB source tree")
if(NOT TINYUSB_PATH)
  if(EXISTS "$ENV{HOME}/.pico-sdk/sdk/2.2.0/lib/tinyusb/src/tusb.h")
    set(TINYUSB_PATH "$ENV{HOME}/.pico-sdk/sdk/2.2.0/lib/tinyusb")
  else()
    FetchContent_Declare(tinyusb
      GIT_REPOSITORY https://github.com/hathach/tinyusb.git
      GIT_TAG 0.18.0 GIT_SHALLOW TRUE SOURCE_SUBDIR none)
    FetchContent_MakeAvailable(tinyusb)
    set(TINYUSB_PATH ${tinyusb_SOURCE_DIR})
  endif()
endif()
message(STATUS "CMSIS H7: ${CMSIS_H7_PATH}  TinyUSB: ${TINYUSB_PATH}")

set(ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)
set(TUSB ${TINYUSB_PATH}/src)

add_executable(opensent
  main.c clock.c hw_h750.c qspi_flash.c la.c usb_descriptors.c syscalls.c
  ${ROOT}/firmware/app/app.c
  ${ROOT}/core/sent_common.c ${ROOT}/core/sent_rx.c ${ROOT}/core/sent_tx.c
  ${CMSIS_H7_PATH}/Source/Templates/gcc/startup_stm32h750xx.s
  ${CMSIS_H7_PATH}/Source/Templates/system_stm32h7xx.c
  ${TUSB}/tusb.c ${TUSB}/common/tusb_fifo.c
  ${TUSB}/device/usbd.c ${TUSB}/device/usbd_control.c
  ${TUSB}/class/cdc/cdc_device.c
  ${TUSB}/portable/synopsys/dwc2/dcd_dwc2.c
  ${TUSB}/portable/synopsys/dwc2/dwc2_common.c
)
set_target_properties(opensent PROPERTIES SUFFIX ".elf")
target_include_directories(opensent PRIVATE
  ${CMAKE_CURRENT_LIST_DIR} ${ROOT}/core ${ROOT}/firmware/app
  ${CMSIS_H7_PATH}/Include ${CMSIS_CORE_PATH}/CMSIS/Core/Include ${TUSB})
target_compile_definitions(opensent PRIVATE
  STM32H750xx CORE_CM7 HSE_VALUE=25000000U CFG_TUSB_MCU=OPT_MCU_STM32H7)

set(MCU_FLAGS -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard)
target_compile_options(opensent PRIVATE ${MCU_FLAGS} -O2 -g -ffunction-sections -fdata-sections
  $<$<COMPILE_LANGUAGE:C>:-Wall -Wextra>)
target_link_options(opensent PRIVATE ${MCU_FLAGS}
  -T${CMAKE_CURRENT_LIST_DIR}/STM32H750VB.ld
  --specs=nano.specs --specs=nosys.specs -u _printf_float
  -Wl,--gc-sections -Wl,-Map=opensent.map -Wl,--print-memory-usage -Wl,--no-warn-rwx-segments)
target_link_libraries(opensent PRIVATE m)
add_custom_command(TARGET opensent POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary opensent.elf opensent.bin
  COMMAND ${CMAKE_OBJCOPY} -O ihex opensent.elf opensent.hex
  COMMAND ${CMAKE_SIZE} opensent.elf)
