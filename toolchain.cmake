set(COMMON_CXX_ASM_COMPILE_FLAGS "-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16")

set(CMAKE_CXX_FLAGS "${COMMON_CXX_ASM_COMPILE_FLAGS} -DSTM32F411xE -DUSE_HAL_DRIVER -specs=nosys.specs -specs=nano.specs -Wl,-Map=output.map -Wl,--gc-sections -fno-exceptions -fno-rtti -O0 -g3 -fmessage-length=0 -ffunction-sections -fno-exceptions -fno-rtti -MMD -MP")
set(CMAKE_ASM_FLAGS "${COMMON_CXX_ASM_COMPILE_FLAGS} -g")

set(OPENOCD_CFG "st_nucleo_f4.cfg")
set(CMAKE_CXX_COMPILER "arm-none-eabi-g++")
set(CMAKE_ASM_COMPILER "arm-none-eabi-as")
set(CMAKE_EXE_LINKER_FLAGS "-T ${CMAKE_CURRENT_LIST_DIR}/STM32F411RETx_FLASH.ld")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "" )
