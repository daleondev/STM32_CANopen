include_guard(GLOBAL)

set(PLATFORM_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(PLATFORM_EXTERN_DIR "${PLATFORM_ROOT_DIR}/extern")
set(CUBEMX_DIR "${PLATFORM_EXTERN_DIR}/CubeMX")
set(HAL_DIR "${PLATFORM_EXTERN_DIR}/stm32h7xx-hal-driver")
set(CMSIS_CORE_DIR "${PLATFORM_EXTERN_DIR}/cmsis-core")
set(CMSIS_DEVICE_DIR "${PLATFORM_EXTERN_DIR}/cmsis-device-h7")
set(NUCLEO_BSP_DIR "${PLATFORM_EXTERN_DIR}/stm32h7xx-nucleo-bsp")
set(THREADX_DIR "${PLATFORM_EXTERN_DIR}/threadx")

function(platform_prepend out_var base_dir)
    set(result)
    foreach(file IN LISTS ARGN)
        list(APPEND result "${base_dir}/${file}")
    endforeach()
    set(${out_var} "${result}" PARENT_SCOPE)
endfunction()

set(THREADX_ARCH cortex_m7)
set(THREADX_TOOLCHAIN gnu)
set(TX_ENABLE_STACK_CHECKING ON)
set(TX_USER_FILE "${CUBEMX_DIR}/Inc/tx_user.h")

add_subdirectory("${THREADX_DIR}")

platform_prepend(PLATFORM_HAL_SOURCES "${HAL_DIR}/Src"
    stm32h7xx_hal_tim.c
    stm32h7xx_hal_tim_ex.c
    stm32h7xx_hal_cortex.c
    stm32h7xx_hal_eth.c
    stm32h7xx_hal_eth_ex.c
    stm32h7xx_hal_rcc.c
    stm32h7xx_hal_rcc_ex.c
    stm32h7xx_hal_flash.c
    stm32h7xx_hal_flash_ex.c
    stm32h7xx_hal_gpio.c
    stm32h7xx_hal_hsem.c
    stm32h7xx_hal_dma.c
    stm32h7xx_hal_dma_ex.c
    stm32h7xx_hal_mdma.c
    stm32h7xx_hal_pwr.c
    stm32h7xx_hal_pwr_ex.c
    stm32h7xx_hal.c
    stm32h7xx_hal_i2c.c
    stm32h7xx_hal_i2c_ex.c
    stm32h7xx_hal_exti.c
    stm32h7xx_hal_rng.c
    stm32h7xx_hal_rng_ex.c
    stm32h7xx_hal_rtc.c
    stm32h7xx_hal_rtc_ex.c
    stm32h7xx_hal_usart.c
    stm32h7xx_hal_usart_ex.c
    stm32h7xx_hal_uart_ex.c
    stm32h7xx_hal_uart.c
    stm32h7xx_hal_fdcan.c
)

platform_prepend(PLATFORM_CUBEMX_SOURCES "${CUBEMX_DIR}/Src"
    main.c
    gpio.c
    rng.c
    rtc.c
    tim.c
    fdcan.c
    stm32h7xx_it.c
    stm32h7xx_hal_msp.c
    stm32h7xx_hal_timebase_tim.c
    sysmem.c
    syscalls.c
    system_stm32h7xx.c
    tx_initialize_low_level.S
)

set(PLATFORM_STARTUP_SOURCES
    "${CUBEMX_DIR}/startup_stm32h753xx.s"
)

set(PLATFORM_BSP_SOURCES
    "${NUCLEO_BSP_DIR}/stm32h7xx_nucleo.c"
)

add_library(Platform STATIC)

target_sources(Platform
    PRIVATE
        ${PLATFORM_HAL_SOURCES}
        ${PLATFORM_BSP_SOURCES}
        ${PLATFORM_CUBEMX_SOURCES}
        ${PLATFORM_STARTUP_SOURCES}
)

target_include_directories(Platform
    PUBLIC
        ${HAL_DIR}/Inc
        ${HAL_DIR}/Inc/Legacy
        ${NUCLEO_BSP_DIR}
        ${CMSIS_CORE_DIR}/Include
        ${CMSIS_DEVICE_DIR}/Include
        ${CUBEMX_DIR}/Inc
)

target_compile_definitions(Platform
    PUBLIC
        USE_PWR_LDO_SUPPLY
        USE_NUCLEO_64
        USE_HAL_DRIVER
        STM32H753xx
        TX_INCLUDE_USER_DEFINE_FILE
        $<$<CONFIG:Debug>:DEBUG>
)

target_link_libraries(Platform
    PUBLIC
        threadx
)
