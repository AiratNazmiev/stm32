#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"

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
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

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

  // Set 12 pin as input
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_13, LL_GPIO_MODE_INPUT);

  return;
}

__attribute__((naked)) static void delay_10ms(void)
{
    asm ("push {r7, lr}");
    asm ("ldr r6, [pc, #8]");
    asm ("sub r6, #1");
    asm ("cmp r6, #0");
    asm ("bne delay_10ms+0x4");
    asm ("pop {r7, pc}");
    asm (".word 0x1770"); //60000
}

// __attribute__((naked)) static void delay_1ms(void)
// {
//     asm ("push {r7, lr}");
//     asm ("ldr r6, [pc, #8]");
//     asm ("sub r6, #1");
//     asm ("cmp r6, #0");
//     asm ("bne delay_1ms+0x4");
//     asm ("pop {r7, pc}");
//     asm (".word 0x1770"); //60000
// }

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

int main(void) {

  uint32_t number = 0;
  uint8_t indicator_clk = 0;

  uint8_t button_clicked = 0;
  uint8_t debouncer_clk = 0;

  // RCC and GPIO configurations
  rcc_config();
  gpio_config();

  //setting into 0
  enableindicator(0);
  setindicator(0);

  while(1) {

    //if button firsty judged to be clicked
    if (LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_13)) {
      button_clicked = 1;
      debouncer_clk = 0;
    }

    // if we're waiting for the end of debouncing
    if (button_clicked) {
      ++debouncer_clk;
      delay_10ms();
    }

    // if it's the supposed end of debouncing (50 ms)
    if (debouncer_clk >= 10) {
      ++number;
      button_clicked = 0;
      debouncer_clk = 0;
    }

   if (number >= 10000) number = 0;

   if (indicator_clk == 4) {
      indicator_clk = 0;
   }

    //displaying number
    enableindicator(indicator_clk);

    switch (indicator_clk) {
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

    ++indicator_clk;


    delay_10ms();
  }

  return 0;
}
