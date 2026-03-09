cmake_minimum_required(VERSION 3.22)

project(CANopenApp LANGUAGES CXX C ASM)
set(CMAKE_PROJECT_NAME CANopenApp)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# --- Executable ---
add_executable(CANopenApp src/main.cpp)

# --- Sources ---
target_sources(CANopenApp 
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp

        ${CMAKE_CURRENT_SOURCE_DIR}/extern/CubeMX/Src/main.c
        ${CMAKE_CURRENT_SOURCE_DIR}/extern/CubeMX/Src/gpio.c
        ${CMAKE_CURRENT_SOURCE_DIR}/extern/CubeMX/Src/rng.c
        ${CMAKE_CURRENT_SOURCE_DIR}/extern/CubeMX/Src/rtc.c
        ${CMAKE_CURRENT_SOURCE_DIR}/extern/CubeMX/Src/tim.c
        ${CMAKE_CURRENT_SOURCE_DIR}/extern/CubeMX/Src/stm32h7xx_it.c
        ${CMAKE_CURRENT_SOURCE_DIR}/extern/CubeMX/Src/stm32h7xx_hal_msp.c
        ${CMAKE_CURRENT_SOURCE_DIR}/extern/CubeMX/Src/stm32h7xx_hal_timebase_tim.c
        ${CMAKE_CURRENT_SOURCE_DIR}/extern/CubeMX/Src/sysmem.c
        ${CMAKE_CURRENT_SOURCE_DIR}/extern/CubeMX/Src/syscalls.c
        ${CMAKE_CURRENT_SOURCE_DIR}/extern/CubeMX/startup_stm32h753xx.s
        ${CMAKE_CURRENT_SOURCE_DIR}/extern/CubeMX/Src/system_stm32h7xx.c
        ${CMAKE_CURRENT_SOURCE_DIR}/extern/CubeMX/Src/tx_initialize_low_level.S
)

# --- Includes ---
target_include_directories(CANopenApp 
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/extern/CubeMX/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/extern/CubeMX/NetXDuo/App
        ${CMAKE_CURRENT_SOURCE_DIR}/extern/CubeMX/NetXDuo/Target
)

set(TX_USER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/extern/CubeMX/Inc/tx_user.h)
include("STM32H753ZITX.cmake")

target_link_libraries(CANopenApp 
    PRIVATE
        Platform
)

target_compile_features(CANopenApp
    PUBLIC
        cxx_std_20
)

# --- Post-build steps ---
add_custom_command(TARGET CANopenApp POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:CANopenApp> CANopenApp.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:CANopenApp> CANopenApp.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:CANopenApp>
    COMMENT "Generating hex and bin files, and printing size"
)