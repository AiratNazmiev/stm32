#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_cortex.h"

#define CORE_FREQ 48000000

// RCC configuration
static void rcc_config (void) {
  // Main PLL configuration and activation
  // Set FLASH latency
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  // Enable HSI and wait for activation
  LL_RCC_HSI_Enable();
  while (LL_RCC_HSI_IsReady() != 1);

  // Main PLL configuration and activation
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2,
                                LL_RCC_PLL_MUL_12);

  LL_RCC_PLL_Enable();
  while (LL_RCC_PLL_IsReady() != 1);

  // Sysclk activation on the main PLL
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

  // Set APB1 prescaler
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

  // Update CMSIS variable (which can be updated also
  // through SystemCoreClockUpdate function)
  SystemCoreClock = CORE_FREQ;

  return;
}

// GPIO configuration
static void gpio_config (void) {
  // Enable A port
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);

  // Set 0 - 11 pins as outputs
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_11, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_9, LL_GPIO_MODE_OUTPUT);

  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_1, LL_GPIO_MODE_INPUT);

  LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_0, LL_GPIO_PULL_DOWN);
  LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_1, LL_GPIO_PULL_DOWN);

  return;
}


static void setindicator (uint32_t number) {

  uint32_t port_state = 0;

  static uint32_t mask = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | \
                         LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | \
                         LL_GPIO_PIN_6 | LL_GPIO_PIN_7;

  // decoder
  static const uint32_t decoder[] = {

        LL_GPIO_PIN_6 | LL_GPIO_PIN_4 | LL_GPIO_PIN_0 | LL_GPIO_PIN_2 | \
        LL_GPIO_PIN_3 | LL_GPIO_PIN_5,
        // 0
        LL_GPIO_PIN_4 | LL_GPIO_PIN_0,
        // 1
        LL_GPIO_PIN_6 | LL_GPIO_PIN_4 | LL_GPIO_PIN_7 | LL_GPIO_PIN_3 | \
        LL_GPIO_PIN_2,
         // 2
        LL_GPIO_PIN_6 | LL_GPIO_PIN_4 | LL_GPIO_PIN_7 | LL_GPIO_PIN_0 | \
        LL_GPIO_PIN_2, // 3

        LL_GPIO_PIN_5 | LL_GPIO_PIN_7 | LL_GPIO_PIN_4 | LL_GPIO_PIN_0, //4

        LL_GPIO_PIN_6 | LL_GPIO_PIN_5 | LL_GPIO_PIN_0 | LL_GPIO_PIN_2 | \
        LL_GPIO_PIN_7, //5

        LL_GPIO_PIN_6 | LL_GPIO_PIN_5 | LL_GPIO_PIN_7 | LL_GPIO_PIN_0 | \
        LL_GPIO_PIN_2 | LL_GPIO_PIN_3,  //6

        LL_GPIO_PIN_6 | LL_GPIO_PIN_4 | LL_GPIO_PIN_0, //7

        LL_GPIO_PIN_0 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | \
        LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7, //8

        LL_GPIO_PIN_0 | LL_GPIO_PIN_2 | LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | \
        LL_GPIO_PIN_6 | LL_GPIO_PIN_7 //9

    };
  // read current state
  port_state = LL_GPIO_ReadOutputPort(GPIOA);

  const uint8_t max_number = sizeof(decoder) / sizeof(uint32_t);

  // write result
  port_state = (port_state & ~mask) | decoder[number % max_number];
  LL_GPIO_WriteOutputPort(GPIOA, port_state);

  return;
}

static void enableindicator (uint8_t number) {

  uint32_t port_state = 0;

  uint32_t mask = LL_GPIO_PIN_11 | LL_GPIO_PIN_10 | LL_GPIO_PIN_9 | LL_GPIO_PIN_8;

  // decoder
  static const uint32_t decoder[] = {
    LL_GPIO_PIN_11, LL_GPIO_PIN_10, LL_GPIO_PIN_9, LL_GPIO_PIN_8
  };

  // read current state
  port_state = LL_GPIO_ReadOutputPort(GPIOA);

  const uint8_t max_number = sizeof(decoder) / sizeof(uint32_t);

  // write result
  port_state = (port_state & ~mask) | (~decoder[number % max_number] & mask);
  LL_GPIO_WriteOutputPort(GPIOA, port_state);

  return;
}

static void exti_config (void) {

  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE0);
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE1);

  LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_0);
  LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_1);

  LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_1);
  LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_1);

  LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_0);
  LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_0);

  NVIC_EnableIRQ(EXTI0_1_IRQn);
  NVIC_SetPriority(EXTI0_1_IRQn, 0);

  return;
}

uint32_t number = 0;

const uint8_t ROTATION_TRIGGER = 3;

void EXTI0_1_IRQHandler(void)
{

    static const int8_t states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

    static uint8_t enc_trans = 0;

    static int8_t enc_dir = 0;

    uint8_t enc_state = 0x00;

    enc_state = 0x0003 & LL_GPIO_ReadInputPort(GPIOC);
    enc_trans = ((0x03 & enc_trans) << 2) | enc_state;
    enc_dir += states[enc_trans];

    if (enc_dir == ROTATION_TRIGGER) {
        if (number < 9999)
            ++number;
        LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_8);
        LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9);
        enc_dir = 0;
    }

    if (enc_dir == -ROTATION_TRIGGER) {
        if (number > 0)
            --number;
        LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_9);
        LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_8);
        enc_dir = 0;
    }

    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
}

static void systick_config(void) {
    LL_InitTick(48000000, 1000);
    LL_SYSTICK_EnableIT();
    NVIC_SetPriority(SysTick_IRQn, 0);
    return;
}

void SysTick_Handler(void)
{
  static uint8_t digit = 0;

  enableindicator(digit);

  switch (digit) {
    case 0: setindicator(number % 10);
    break;
    case 1: setindicator((number % 100) / 10);
    break;
    case 2: setindicator((number % 1000) / 100);
    break;
    case 3: setindicator(number / 1000);
    break;
    default: ;
  }

  digit = (++digit) % 4;
}

int main(void) {

  rcc_config();
  gpio_config();
  exti_config();
  systick_config();

  enableindicator(0);
  setindicator(0);

  while(1);

  return 0;
}
