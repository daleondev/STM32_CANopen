#include "main.h"
#include "gpio.h"
#include "rng.h"
#include "rtc.h"
#include "tim.h"

#include <tx_api.h>

#include <cstdio>
#include <exception>
#include <string_view>

#include "canopen_cia402_motor_threadx.hpp"
#include "simulated_can_port.hpp"

extern "C"
{
  extern COM_InitTypeDef BspCOMInit;
  extern void SystemClock_Config(void);
  extern void MPU_Config_User(void);
}

void tx_main();

namespace
{

  class MainThreadLogger final : public ThreadXCia402Logger
  {
  public:
    void Info(std::string_view message) override
    {
      std::printf("[CiA402] %.*s\r\n", static_cast<int>(message.size()), message.data());
    }

    void Error(std::string_view message) override
    {
      std::printf("[CiA402][ERR] %.*s\r\n", static_cast<int>(message.size()), message.data());
    }
  };

  [[noreturn]] void BlinkStatus(bool success)
  {
    const ULONG delay = TX_TIMER_TICKS_PER_SECOND / 4 ? TX_TIMER_TICKS_PER_SECOND / 4 : 1;

    BSP_LED_Off(LED_GREEN);
    BSP_LED_Off(LED_RED);

    while (true)
    {
      if (success)
      {
        BSP_LED_Toggle(LED_GREEN);
        BSP_LED_Off(LED_RED);
      }
      else
      {
        BSP_LED_Toggle(LED_RED);
        BSP_LED_Off(LED_GREEN);
      }

      tx_thread_sleep(delay);
    }
  }

} // namespace

void tx_application_define(void *first_unused_memory)
{
  tx_main();
}

void tx_thread_stack_error_notify(TX_THREAD *thread)
{
  printf("Thread %s stack overflow detected\r\n", thread->tx_thread_name);
  Error_Handler();
}

int main(void)
{
  MPU_Config_User();
  SCB_EnableICache();
  SCB_EnableDCache();

  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_RNG_Init();

  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED);
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  BspCOMInit.BaudRate = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits = COM_STOPBITS_1;
  BspCOMInit.Parity = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  HAL_TIM_Base_Start(&htim2);

  tx_kernel_enter();

  Error_Handler();
}

void tx_main()
{
  static TX_THREAD main_thread;
  static UCHAR main_thread_stack[4096];

  static CHAR thread_name[] = "Main Thread";
  tx_thread_create(&main_thread, thread_name, [](ULONG entry_input)
                   {
    (void)entry_input;

    MainThreadLogger logger;
    ThreadXCia402Config config;
    SimulatedCanPort can_port(config.node_id, &logger);

    try
    {
      ThreadXCia402MotorApp app(config, can_port, &logger);
      const int result = app.Run();
      BlinkStatus(result == 0);
    }
    catch (const std::exception &e)
    {
      logger.Error(e.what());
    }
    catch (...)
    {
      logger.Error("Unknown exception in CiA402 main thread.");
    }

    BlinkStatus(false); }, 0, main_thread_stack, sizeof(main_thread_stack), 15, 15, TX_NO_TIME_SLICE, TX_AUTO_START);
}