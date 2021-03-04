cmake_minimum_required(VERSION 3.10.0)

set(name MiniOctocopter)

include(toolchain.cmake)

# Add local sources

list(APPEND SOURCES
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.cpp
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.cpp
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.cpp
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.cpp
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.cpp
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.cpp
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.cpp
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.cpp
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.cpp
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.cpp
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.cpp
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.cpp
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.cpp
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.cpp
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.cpp
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.cpp
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.cpp
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.cpp
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.cpp
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.cpp
  Src/main.cpp
  Src/stm32f4xx_hal_msp.cpp
  Src/stm32f4xx_it.cpp
  Src/system_stm32f4xx.cpp
)

# Add sources from src and CommonDriver

include(../src/src.cmake)
prepend_list(LOCAL_INCLUDE_PATHS "../src/")
prepend_list(LOCAL_SOURCE_PATHS  "../src/")

include(../CommonDrivers/src/src.cmake)
prepend_list(INCLUDE_PATHS "../CommonDrivers/src/")
prepend_list(SOURCE_PATHS  "../CommonDrivers/src/")

# define target

add_library(${name}-asm STATIC "startup/startup_stm32f411xe.s")
add_executable(${name}.elf ${SOURCES} ${SOURCE_PATHS} ${LOCAL_SOURCE_PATHS})
target_link_libraries(${name}.elf LINK_PRIVATE ${name}-asm)
set_property(TARGET ${name}.elf PROPERTY CXX_STANDARD 14)

separate_arguments(ADDITIONAL_FLAGS)
target_compile_options(${name}.elf PRIVATE ${ADDITIONAL_FLAGS})
target_compile_options(${name}.elf PRIVATE ${COMMON_FLAGS})

# Add headers

target_include_directories(${name}.elf PRIVATE ${INCLUDE_PATHS} ${LOCAL_INCLUDE_PATHS})
target_include_directories(${name}.elf PRIVATE Inc)
target_include_directories(${name}.elf PRIVATE Src)
target_include_directories(${name}.elf PRIVATE ../CommonDrivers)
target_include_directories(${name}.elf PRIVATE Drivers)
target_include_directories(${name}.elf PRIVATE Drivers/STM32F4xx_HAL_Driver)
target_include_directories(${name}.elf PRIVATE Drivers/STM32F4xx_HAL_Driver/Inc)
target_include_directories(${name}.elf PRIVATE Drivers/STM32F4xx_HAL_Driver/Inc/Legacy)
target_include_directories(${name}.elf PRIVATE Drivers/STM32F4xx_HAL_Driver/Src)
target_include_directories(${name}.elf PRIVATE Drivers/CMSIS)
target_include_directories(${name}.elf PRIVATE Drivers/CMSIS/Device/ST/STM32F4xx/Include)
target_include_directories(${name}.elf PRIVATE Drivers/CMSIS/Include)

# Create a binary from the elf file
add_custom_command(TARGET ${name}.elf POST_BUILD
    COMMAND arm-none-eabi-objcopy -Oihex ${name}.elf ${name}.hex
    COMMAND arm-none-eabi-objcopy -O binary ${name}.elf ${name}.bin
    COMMAND arm-none-eabi-size ${name}.elf
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)
