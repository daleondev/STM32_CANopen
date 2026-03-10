set(THREADX_ARCH cortex_m7)
set(THREADX_TOOLCHAIN gnu)
set(TX_ENABLE_STACK_CHECKING ON)
set(TX_USER_FILE ${CMAKE_SOURCE_DIR}/extern/CubeMX/Inc/tx_user.h)
add_subdirectory(${CMAKE_SOURCE_DIR}/extern/threadx)

add_library(Platform STATIC)

target_sources(Platform
    PRIVATE
        ${CMAKE_SOURCE_DIR}/extern/stm32h7xx-hal-driver/Src/stm32h7xx_hal_tim.c
        ${CMAKE_SOURCE_DIR}/extern/stm32h7xx-hal-driver/Src/stm32h7xx_hal_tim_ex.c
        ${CMAKE_SOURCE_DIR}/extern/stm32h7xx-hal-driver/Src/stm32h7xx_hal_cortex.c
        ${CMAKE_SOURCE_DIR}/extern/stm32h7xx-hal-driver/Src/stm32h7xx_hal_eth.c
        ${CMAKE_SOURCE_DIR}/extern/stm32h7xx-hal-driver/Src/stm32h7xx_hal_eth_ex.c
        ${CMAKE_SOURCE_DIR}/extern/stm32h7xx-hal-driver/Src/stm32h7xx_hal_rcc.c
        ${CMAKE_SOURCE_DIR}/extern/stm32h7xx-hal-driver/Src/stm32h7xx_hal_rcc_ex.c
        ${CMAKE_SOURCE_DIR}/extern/stm32h7xx-hal-driver/Src/stm32h7xx_hal_flash.c
        ${CMAKE_SOURCE_DIR}/extern/stm32h7xx-hal-driver/Src/stm32h7xx_hal_flash_ex.c
        ${CMAKE_SOURCE_DIR}/extern/stm32h7xx-hal-driver/Src/stm32h7xx_hal_gpio.c
        ${CMAKE_SOURCE_DIR}/extern/stm32h7xx-hal-driver/Src/stm32h7xx_hal_hsem.c
        ${CMAKE_SOURCE_DIR}/extern/stm32h7xx-hal-driver/Src/stm32h7xx_hal_dma.c
        ${CMAKE_SOURCE_DIR}/extern/stm32h7xx-hal-driver/Src/stm32h7xx_hal_dma_ex.c
        ${CMAKE_SOURCE_DIR}/extern/stm32h7xx-hal-driver/Src/stm32h7xx_hal_mdma.c
        ${CMAKE_SOURCE_DIR}/extern/stm32h7xx-hal-driver/Src/stm32h7xx_hal_pwr.c
        ${CMAKE_SOURCE_DIR}/extern/stm32h7xx-hal-driver/Src/stm32h7xx_hal_pwr_ex.c
        ${CMAKE_SOURCE_DIR}/extern/stm32h7xx-hal-driver/Src/stm32h7xx_hal.c
        ${CMAKE_SOURCE_DIR}/extern/stm32h7xx-hal-driver/Src/stm32h7xx_hal_i2c.c
        ${CMAKE_SOURCE_DIR}/extern/stm32h7xx-hal-driver/Src/stm32h7xx_hal_i2c_ex.c
        ${CMAKE_SOURCE_DIR}/extern/stm32h7xx-hal-driver/Src/stm32h7xx_hal_exti.c
        ${CMAKE_SOURCE_DIR}/extern/stm32h7xx-hal-driver/Src/stm32h7xx_hal_rng.c
        ${CMAKE_SOURCE_DIR}/extern/stm32h7xx-hal-driver/Src/stm32h7xx_hal_rng_ex.c
        ${CMAKE_SOURCE_DIR}/extern/stm32h7xx-hal-driver/Src/stm32h7xx_hal_rtc.c
        ${CMAKE_SOURCE_DIR}/extern/stm32h7xx-hal-driver/Src/stm32h7xx_hal_rtc_ex.c
        ${CMAKE_SOURCE_DIR}/extern/stm32h7xx-hal-driver/Src/stm32h7xx_hal_usart.c
        ${CMAKE_SOURCE_DIR}/extern/stm32h7xx-hal-driver/Src/stm32h7xx_hal_usart_ex.c
        ${CMAKE_SOURCE_DIR}/extern/stm32h7xx-hal-driver/Src/stm32h7xx_hal_uart_ex.c
        ${CMAKE_SOURCE_DIR}/extern/stm32h7xx-hal-driver/Src/stm32h7xx_hal_uart.c
        ${CMAKE_SOURCE_DIR}/extern/stm32h7xx-nucleo-bsp/stm32h7xx_nucleo.c
        ${CMAKE_SOURCE_DIR}/extern/CubeMX/Src/main.c
        ${CMAKE_SOURCE_DIR}/extern/CubeMX/Src/gpio.c
        ${CMAKE_SOURCE_DIR}/extern/CubeMX/Src/rng.c
        ${CMAKE_SOURCE_DIR}/extern/CubeMX/Src/rtc.c
        ${CMAKE_SOURCE_DIR}/extern/CubeMX/Src/tim.c
        ${CMAKE_SOURCE_DIR}/extern/CubeMX/Src/stm32h7xx_it.c
        ${CMAKE_SOURCE_DIR}/extern/CubeMX/Src/stm32h7xx_hal_msp.c
        ${CMAKE_SOURCE_DIR}/extern/CubeMX/Src/stm32h7xx_hal_timebase_tim.c
        ${CMAKE_SOURCE_DIR}/extern/CubeMX/Src/sysmem.c
        ${CMAKE_SOURCE_DIR}/extern/CubeMX/Src/syscalls.c
        ${CMAKE_SOURCE_DIR}/extern/CubeMX/startup_stm32h753xx.s
        ${CMAKE_SOURCE_DIR}/extern/CubeMX/Src/system_stm32h7xx.c
        ${CMAKE_SOURCE_DIR}/extern/CubeMX/Src/tx_initialize_low_level.S
)

target_include_directories(Platform 
    PUBLIC
        ${CMAKE_SOURCE_DIR}/extern/stm32h7xx-hal-driver/Inc
        ${CMAKE_SOURCE_DIR}/extern/stm32h7xx-hal-driver/Inc/Legacy
        ${CMAKE_SOURCE_DIR}/extern/stm32h7xx-nucleo-bsp
        ${CMAKE_SOURCE_DIR}/extern/cmsis-core/Include
        ${CMAKE_SOURCE_DIR}/extern/cmsis-device-h7/Include
        ${CMAKE_SOURCE_DIR}/extern/CubeMX/Inc
)

target_compile_definitions(Platform 
    PUBLIC
        USE_PWR_LDO_SUPPLY
        USE_NUCLEO_64
        USE_HAL_DRIVER
        STM32H753xx
        $<$<CONFIG:Debug>:DEBUG>

        TX_INCLUDE_USER_DEFINE_FILE
)

target_link_libraries(Platform
    PUBLIC
        threadx
)
