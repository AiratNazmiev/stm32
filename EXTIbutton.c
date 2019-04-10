#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_cortex.h"

#define CORE_FREQ 48000000

static void rcc_config (void) {

  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  LL_RCC_HSI_Enable();
  while (LL_RCC_HSI_IsReady() != 1);

  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2,
                                LL_RCC_PLL_MUL_12);

  LL_RCC_PLL_Enable();
  while (LL_RCC_PLL_IsReady() != 1);

  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

  SystemCoreClock = CORE_FREQ;

  return;
}

static void gpio_config (void) {

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_9, LL_GPIO_MODE_OUTPUT);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_0, LL_GPIO_PULL_DOWN);

  return;
}

static void exti_config (void) {

  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);

  LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_0);

  LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_0);

  NVIC_EnableIRQ(EXTI0_1_IRQn);
  NVIC_SetPriority(EXTI0_1_IRQn, 0);

  return;
}

static void systick_config (void) {
  LL_InitTick(CORE_FREQ, 1000);
  LL_SYSTICK_EnableIT();
  //WHY ZERO PRIORITY?!
  NVIC_SetPriority(SysTick_IRQn, 0);
  return;
}

uint32_t milliseconds = 0;

void SysTick_Handler(void)
{
    ++milliseconds;
}

void EXTI0_1_IRQHandler(void)
{
    uint32_t ms = milliseconds;
    static uint32_t ms_old = 0;

    if (ms - ms_old > 50) {
      LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_9);

    }

          ms_old = ms;

    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
}




int main() {
  rcc_config();
  gpio_config();
  exti_config();
  systick_config();

  while (1);

  return 0;
}
