cmake_minimum_required(VERSION 3.22)

set(CMAKE_SYSTEM_NAME      Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(GCC_PATH "" CACHE STRING "Path to arm-none-eabi-gcc bin dir (optional)")
if(GCC_PATH)
    set(CMAKE_C_COMPILER   "${GCC_PATH}/arm-none-eabi-gcc")
    set(CMAKE_CXX_COMPILER "${GCC_PATH}/arm-none-eabi-g++")
    set(CMAKE_ASM_COMPILER "${GCC_PATH}/arm-none-eabi-gcc")
    set(CMAKE_OBJCOPY      "${GCC_PATH}/arm-none-eabi-objcopy")
    set(CMAKE_SIZE         "${GCC_PATH}/arm-none-eabi-size")
else()
    set(CMAKE_C_COMPILER   arm-none-eabi-gcc)
    set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
    set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
    set(CMAKE_OBJCOPY      arm-none-eabi-objcopy)
    set(CMAKE_SIZE         arm-none-eabi-size)
endif()

project(SensiNerveX_Anomaly C ASM)

set(MCU_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard   )       

set(COMMON_FLAGS
    ${MCU_FLAGS}
    -Wall
    -Wextra
    -fdata-sections
    -ffunction-sections
    -fno-common
    -fmessage-length=0
)

set(CMAKE_C_FLAGS_DEBUG   "-Og -g3 -DDEBUG")
set(CMAKE_C_FLAGS_RELEASE "-O2")

add_compile_definitions(
    USE_HAL_DRIVER
    STM32F405xx
    SNX_NODE_ID=0x01u       # Change per node: 0x01 / 0x02 / 0x03
)

include_directories(
    Core/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/CMSIS/Include
    Middlewares/FatFs/src
    Middlewares/FatFs
)

file(GLOB_RECURSE APP_SOURCES
    "Core/Src/*.c"
    "Core/Src/*.s"
)

file(GLOB_RECURSE HAL_SOURCES
    "Drivers/STM32F4xx_HAL_Driver/Src/*.c"
)

list(FILTER HAL_SOURCES EXCLUDE REGEX ".*_template\\.c$")

set(FATFS_SOURCES
    Middlewares/FatFs/fatfs.c
    Middlewares/FatFs/src/ff.c
    Middlewares/FatFs/src/diskio.c
    Middlewares/FatFs/src/ff_gen_drv.c
)
  

set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/Linker/STM32F405RGT6_FLASH.ld")

set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} \
    -T${LINKER_SCRIPT} \
    -Wl,-u,_printf_float \
    -lc -lm \
    -specs=nano.specs \
    -Wl,--gc-sections \
    -Wl,-Map=${PROJECT_NAME}.map,--cref \
    -Wl,--print-memory-usage"
)

set(STARTUP_FILE startup_stm32f405xx.s)
add_executable(${PROJECT_NAME}.elf
    ${STARTUP_FILE}
    ${APP_SOURCES}
    ${HAL_SOURCES}
    ${FATFS_SOURCES}
)

target_compile_options(${PROJECT_NAME}.elf PRIVATE ${COMMON_FLAGS})
target_link_options(${PROJECT_NAME}.elf    PRIVATE ${MCU_FLAGS})

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${PROJECT_NAME}.elf>
            ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf>
            ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} --format=berkeley $<TARGET_FILE:${PROJECT_NAME}.elf>
    COMMENT "Generating HEX and BIN files"
)

add_custom_target(flash
    COMMAND openocd
            -f interface/stlink.cfg
            -f target/stm32f4x.cfg
            -c "program ${PROJECT_NAME}.bin verify reset exit 0x08000000"
    DEPENDS ${PROJECT_NAME}.elf
    COMMENT "Flashing via OpenOCD + ST-Link"
)
