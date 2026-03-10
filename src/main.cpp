#include "main.h"
#include "gpio.h"
#include "rng.h"
#include "rtc.h"
#include "tim.h"
#include "fdcan.h"

#include <tx_api.h>
#include <cstdio>

#include "Factory/LLDriver/CAN.hpp"
#include "Factory/HLDriver/CANopen.hpp"
#include "Factory/HLDriver/CiA402.hpp"
#include "Factory/HLDriver/NanotecPnD.hpp"
#include "SerialCLI.hpp"

extern "C"
{
  extern COM_InitTypeDef BspCOMInit;
  extern void SystemClock_Config(void);
  extern void MPU_Config_User(void);
}

/* -------------------------------------------------------------------------- */
/* Configuration constants                                                    */
/* -------------------------------------------------------------------------- */
static constexpr uint8_t CANOPEN_NODE_ID = 0x7FU; /* Master node ID */
static constexpr uint16_t CAN_BITRATE = 500U;     /* 500 kbit/s */

/* -------------------------------------------------------------------------- */
/* Thread definitions                                                         */
/* -------------------------------------------------------------------------- */
static TX_THREAD mainThread;
static UCHAR mainThreadStack[4096];

static TX_THREAD canopenThread;
static UCHAR canopenThreadStack[8192];

static TX_THREAD serialThread;
static UCHAR serialThreadStack[4096];

/* -------------------------------------------------------------------------- */
/* Microsecond timer using TIM2                                               */
/* -------------------------------------------------------------------------- */
static uint32_t getTimerUs()
{
  return __HAL_TIM_GET_COUNTER(&htim2);
}

/* -------------------------------------------------------------------------- */
/* Thread entry functions                                                     */
/* -------------------------------------------------------------------------- */

/**
 * @brief Main thread — LED heartbeat indicator.
 */
static void mainThreadEntry(ULONG)
{
  constexpr auto ticks_per_ms = TX_TIMER_TICKS_PER_SECOND / 1000;

  while (true)
  {
    BSP_LED_Toggle(LED_GREEN);
    tx_thread_sleep(1000 / ticks_per_ms);
  }
}

/**
 * @brief CANopen thread — runs the CANopen stack processing at ~1 ms interval.
 */
static void canopenThreadEntry(ULONG)
{
  /* Create drivers via factories */
  auto &ican = Factory::LLDriver::CAN::create(&hfdcan1);
  auto &canopen = Factory::HLDriver::CANopen::create(ican);
  auto &motor = Factory::HLDriver::CiA402::create(canopen);
  auto &nanotec = Factory::HLDriver::NanotecPnD::create(canopen);

  /* Initialize the Serial CLI with CANopen + motor + nanotec references */
  SerialCLI::init(canopen, &motor, &nanotec);

  /* Initialize the CANopen stack */
  if (!canopen.init(CANOPEN_NODE_ID, CAN_BITRATE))
  {
    printf("CANopen init failed — entering error state\r\n");
    BSP_LED_On(LED_RED);
    while (true)
    {
      tx_thread_sleep(1000);
    }
  }

  uint32_t lastTimerUs = getTimerUs();

  while (true)
  {
    /* Calculate time difference */
    uint32_t now = getTimerUs();
    uint32_t timeDiff_us = now - lastTimerUs;
    lastTimerUs = now;

    /* Run synchronous processing (SYNC, RPDO, TPDO) */
    canopen.processSync(timeDiff_us);

    /* Run asynchronous processing (NMT, HB, SDO, Emergency, etc.) */
    uint8_t reset = canopen.process(timeDiff_us);

    /* Update motor driver status from received PDO data */
    motor.update();

    if (reset == 1) /* CO_RESET_COMM */
    {
      printf("CANopen: communication reset requested\r\n");
      canopen.init(CANOPEN_NODE_ID, CAN_BITRATE);
      lastTimerUs = getTimerUs();
    }
    else if (reset == 2) /* CO_RESET_APP */
    {
      printf("CANopen: application reset requested\r\n");
      HAL_NVIC_SystemReset();
    }

    /* Sleep ~1 ms */
    tx_thread_sleep(1);
  }
}

/**
 * @brief Serial communication thread — runs the CLI.
 */
static void serialThreadEntry(ULONG)
{
  /* Small delay to let CANopen thread initialize first */
  tx_thread_sleep(100);

  SerialCLI::run();
}

/* -------------------------------------------------------------------------- */
/* ThreadX application define                                                 */
/* -------------------------------------------------------------------------- */

void tx_application_define(void *first_unused_memory)
{
  (void)first_unused_memory;

  /* Main thread — LED heartbeat (priority 15, lowest of the three) */
  static CHAR mainName[] = "Main Thread";
  tx_thread_create(&mainThread, mainName, mainThreadEntry, 0,
                   mainThreadStack, sizeof(mainThreadStack),
                   15, 15, TX_NO_TIME_SLICE, TX_AUTO_START);

  /* CANopen thread — stack processing (priority 10, highest) */
  static CHAR canopenName[] = "CANopen Thread";
  tx_thread_create(&canopenThread, canopenName, canopenThreadEntry, 0,
                   canopenThreadStack, sizeof(canopenThreadStack),
                   10, 10, TX_NO_TIME_SLICE, TX_AUTO_START);

  /* Serial CLI thread — user interaction (priority 20, lowest) */
  static CHAR serialName[] = "Serial Thread";
  tx_thread_create(&serialThread, serialName, serialThreadEntry, 0,
                   serialThreadStack, sizeof(serialThreadStack),
                   20, 20, TX_NO_TIME_SLICE, TX_AUTO_START);
}

void tx_thread_stack_error_notify(TX_THREAD *thread)
{
  printf("Thread %s stack overflow detected\r\n", thread->tx_thread_name);
  Error_Handler();
}

/* -------------------------------------------------------------------------- */
/* main()                                                                     */
/* -------------------------------------------------------------------------- */
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
  MX_FDCAN1_Init();

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