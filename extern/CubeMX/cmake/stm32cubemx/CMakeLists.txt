cmake_minimum_required(VERSION 3.22)
# Enable CMake support for ASM and C languages
enable_language(C ASM)
# STM32CubeMX generated symbols (macros)
set(MX_Defines_Syms 
	USE_PWR_LDO_SUPPLY 
	TX_INCLUDE_USER_DEFINE_FILE 
	USE_HAL_DRIVER 
	STM32H753xx
    $<$<CONFIG:Debug>:DEBUG>
)

# STM32CubeMX generated include paths
set(MX_Include_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Inc
    /home/david/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0/Drivers/STM32H7xx_HAL_Driver/Inc
    /home/david/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
    /home/david/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0/Drivers/BSP/STM32H7xx_Nucleo
    /home/david/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0/Drivers/CMSIS/Device/ST/STM32H7xx/Include
    /home/david/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0/Drivers/CMSIS/Include
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/inc/
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/ports/cortex_m7/gnu/inc/
)

# STM32CubeMX generated application sources
set(MX_Application_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/tx_initialize_low_level.S
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/app_threadx.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/app_azure_rtos.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/rng.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/rtc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/stm32h7xx_it.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/stm32h7xx_hal_msp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/stm32h7xx_hal_timebase_tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/sysmem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/syscalls.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../startup_stm32h753xx.s
)

# STM32 HAL/LL Drivers
set(STM32_Drivers_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Src/system_stm32h7xx.c
    /home/david/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c
    /home/david/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c
    /home/david/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c
    /home/david/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c
    /home/david/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c
    /home/david/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c
    /home/david/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c
    /home/david/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c
    /home/david/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c
    /home/david/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c
    /home/david/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c
    /home/david/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c
    /home/david/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c
    /home/david/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c
    /home/david/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c
    /home/david/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c
    /home/david/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c
    /home/david/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c
    /home/david/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rng.c
    /home/david/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rng_ex.c
    /home/david/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rtc.c
    /home/david/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rtc_ex.c
    /home/david/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_usart.c
    /home/david/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_usart_ex.c
    /home/david/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c
    /home/david/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c
    /home/david/STM32Cube/Repository/STM32Cube_FW_H7_V1.13.0/Drivers/BSP/STM32H7xx_Nucleo/stm32h7xx_nucleo.c
)

# Drivers Midllewares


set(ThreadX_Src
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/ports/cortex_m7/gnu/src/tx_thread_context_restore.S
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/ports/cortex_m7/gnu/src/tx_thread_context_save.S
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/ports/cortex_m7/gnu/src/tx_thread_interrupt_control.S
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/ports/cortex_m7/gnu/src/tx_thread_interrupt_disable.S
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/ports/cortex_m7/gnu/src/tx_thread_interrupt_restore.S
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/ports/cortex_m7/gnu/src/tx_thread_schedule.S
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/ports/cortex_m7/gnu/src/tx_thread_stack_build.S
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/ports/cortex_m7/gnu/src/tx_thread_system_return.S
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/ports/cortex_m7/gnu/src/tx_timer_interrupt.S
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_initialize_high_level.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_initialize_kernel_enter.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_initialize_kernel_setup.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_block_allocate.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_block_pool_cleanup.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_block_pool_create.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_block_pool_delete.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_block_pool_info_get.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_block_pool_initialize.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_block_pool_prioritize.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_block_release.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_byte_allocate.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_byte_pool_cleanup.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_byte_pool_create.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_byte_pool_delete.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_byte_pool_info_get.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_byte_pool_initialize.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_byte_pool_prioritize.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_byte_pool_search.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_byte_release.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_event_flags_cleanup.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_event_flags_create.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_event_flags_delete.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_event_flags_get.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_event_flags_info_get.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_event_flags_initialize.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_event_flags_set.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_event_flags_set_notify.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_mutex_cleanup.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_mutex_create.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_mutex_delete.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_mutex_get.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_mutex_info_get.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_mutex_initialize.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_mutex_prioritize.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_mutex_priority_change.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_mutex_put.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_queue_cleanup.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_queue_create.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_queue_delete.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_queue_flush.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_queue_front_send.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_queue_info_get.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_queue_initialize.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_queue_prioritize.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_queue_receive.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_queue_send.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_queue_send_notify.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_semaphore_ceiling_put.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_semaphore_cleanup.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_semaphore_create.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_semaphore_delete.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_semaphore_get.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_semaphore_info_get.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_semaphore_initialize.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_semaphore_prioritize.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_semaphore_put.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_semaphore_put_notify.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_thread_create.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_thread_delete.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_thread_entry_exit_notify.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_thread_identify.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_thread_info_get.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_thread_initialize.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_thread_preemption_change.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_thread_priority_change.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_thread_relinquish.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_thread_reset.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_thread_resume.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_thread_shell_entry.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_thread_sleep.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_thread_stack_analyze.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_thread_stack_error_handler.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_thread_stack_error_notify.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_thread_suspend.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_thread_system_preempt_check.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_thread_system_resume.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_thread_system_suspend.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_thread_terminate.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_thread_time_slice.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_thread_time_slice_change.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_thread_timeout.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_thread_wait_abort.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_time_get.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_time_set.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_block_allocate.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_block_pool_create.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_block_pool_delete.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_block_pool_info_get.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_block_pool_prioritize.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_block_release.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_byte_allocate.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_byte_pool_create.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_byte_pool_delete.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_byte_pool_info_get.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_byte_pool_prioritize.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_byte_release.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_event_flags_create.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_event_flags_delete.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_event_flags_get.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_event_flags_info_get.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_event_flags_set.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_event_flags_set_notify.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_mutex_create.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_mutex_delete.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_mutex_get.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_mutex_info_get.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_mutex_prioritize.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_mutex_put.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_queue_create.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_queue_delete.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_queue_flush.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_queue_front_send.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_queue_info_get.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_queue_prioritize.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_queue_receive.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_queue_send.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_queue_send_notify.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_semaphore_ceiling_put.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_semaphore_create.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_semaphore_delete.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_semaphore_get.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_semaphore_info_get.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_semaphore_prioritize.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_semaphore_put.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_semaphore_put_notify.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_thread_create.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_thread_delete.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_thread_entry_exit_notify.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_thread_info_get.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_thread_preemption_change.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_thread_priority_change.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_thread_relinquish.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_thread_reset.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_thread_resume.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_thread_suspend.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_thread_terminate.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_thread_time_slice_change.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_thread_wait_abort.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_timer_activate.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_timer_change.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_timer_create.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_timer_deactivate.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_timer_delete.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_timer_expiration_process.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_timer_info_get.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_timer_initialize.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_timer_system_activate.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_timer_system_deactivate.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/tx_timer_thread_entry.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_timer_activate.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_timer_change.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_timer_create.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_timer_deactivate.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_timer_delete.c
    /home/david/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-AZRTOS-H7/3.5.0/Middlewares/ST/threadx/common/src/txe_timer_info_get.c
)

# Link directories setup
set(MX_LINK_DIRS

)
# Project static libraries
set(MX_LINK_LIBS 
    STM32_Drivers
    ${TOOLCHAIN_LINK_LIBRARIES}
    ThreadX	
)
# Interface library for includes and symbols
add_library(stm32cubemx INTERFACE)
target_include_directories(stm32cubemx INTERFACE ${MX_Include_Dirs})
target_compile_definitions(stm32cubemx INTERFACE ${MX_Defines_Syms})

# Create STM32_Drivers static library
add_library(STM32_Drivers OBJECT)
target_sources(STM32_Drivers PRIVATE ${STM32_Drivers_Src})
target_link_libraries(STM32_Drivers PUBLIC stm32cubemx)


# Create ThreadX static library
add_library(ThreadX OBJECT)
target_sources(ThreadX PRIVATE ${ThreadX_Src})
target_link_libraries(ThreadX PUBLIC stm32cubemx)

# Add STM32CubeMX generated application sources to the project
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${MX_Application_Src})

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE ${MX_LINK_DIRS})

# Add libraries to the project
target_link_libraries(${CMAKE_PROJECT_NAME} ${MX_LINK_LIBS})

# Add the map file to the list of files to be removed with 'clean' target
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map)

# Validate that STM32CubeMX code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()
