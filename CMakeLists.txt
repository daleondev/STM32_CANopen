cmake_minimum_required(VERSION 3.22)

set(CMAKE_PROJECT_NAME CANopenApp)
project(${CMAKE_PROJECT_NAME} LANGUAGES CXX C ASM)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/extern/lely)

# --- Executable ---
add_executable(${CMAKE_PROJECT_NAME})

# --- Sources ---
target_sources(${CMAKE_PROJECT_NAME} 
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/canopen_cia402_motor_threadx.cpp

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
target_include_directories(${CMAKE_PROJECT_NAME} 
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/extern/CubeMX/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/extern/CubeMX/NetXDuo/App
        ${CMAKE_CURRENT_SOURCE_DIR}/extern/CubeMX/NetXDuo/Target
)

set(TX_USER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/extern/CubeMX/Inc/tx_user.h)
include("STM32H753ZITX.cmake")

target_link_libraries(${CMAKE_PROJECT_NAME} 
    PRIVATE
        Platform
        CANopen
)

target_compile_features(${CMAKE_PROJECT_NAME}
    PUBLIC
        cxx_std_20
)

# --- Post-build steps ---
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMENT "Generating hex and bin files, and printing size"
)