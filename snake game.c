#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_spi.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_tim.h"

#define CORE_FREQ 48000000

uint32_t number_of_fruits = 0;

enum statusbar_states {
  SB_0 = 0,
  SB_1 = LL_GPIO_PIN_4,
  SB_2 = LL_GPIO_PIN_5 | SB_1,
  SB_3 = LL_GPIO_PIN_6 | SB_2,
  SB_4 = LL_GPIO_PIN_7 | SB_3,
  SB_5 = LL_GPIO_PIN_10 | SB_4,
  SB_6 = LL_GPIO_PIN_11 | SB_5,
  SB_7 = LL_GPIO_PIN_12 | SB_6,
  SB_8 = LL_GPIO_PIN_13 | SB_7,
  SB_9 = LL_GPIO_PIN_14 | SB_8,
  SB_10 = LL_GPIO_PIN_15 | SB_9,
};

uint32_t tmp_up = 0;
uint32_t tmp_down = 0;
uint32_t tmp_left = 0;
uint32_t tmp_right = 0;

uint32_t snake_speed = 0;

static void set_indicator (uint32_t number);

static void enable_indicator (uint8_t number);

void statusbar_display (uint32_t state);

static void rcc_config() {
    /* Set FLASH latency */
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

    /* Enable HSI and wait for activation*/
    LL_RCC_HSI_Enable();
    while (LL_RCC_HSI_IsReady() != 1);

    /* Main PLL configuration and activation */
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2,
                                LL_RCC_PLL_MUL_12);

    LL_RCC_PLL_Enable();
    while (LL_RCC_PLL_IsReady() != 1);

    /* Sysclk activation on the main PLL */
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

    /* Set APB1 prescaler */
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

    /* Update CMSIS variable (which can be updated also
     * through SystemCoreClockUpdate function) */
    SystemCoreClock = 48000000;
}

static void gpio_config(void) {
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);

    // 7-segment
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_2, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_3, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_9, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_10, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_11, LL_GPIO_MODE_OUTPUT);
    //

    //buttons
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_1, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_2, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_3, LL_GPIO_MODE_INPUT);

    LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_0, LL_GPIO_PULL_UP);
    LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_1, LL_GPIO_PULL_UP);
    LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_2, LL_GPIO_PULL_UP);
    LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_3, LL_GPIO_PULL_UP);
    //

    //statusbar
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_10, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_11, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_12, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_14, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_15, LL_GPIO_MODE_OUTPUT);
    //

    //LEDs
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_9, LL_GPIO_MODE_OUTPUT);
    //

    //encoder
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_0, LL_GPIO_PULL_UP);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_1, LL_GPIO_PULL_UP);
    //

    //

    return;
}

static void exti_config (void) {

  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE1);

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

static void timers_config(void) {
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_5, LL_GPIO_AF_2);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
    LL_TIM_SetPrescaler(TIM2, 479);
    LL_TIM_SetAutoReload(TIM2, 199);
    LL_TIM_OC_SetCompareCH1(TIM2, 1);
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetMode(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1); // handles if cnt reaches defined value
    LL_TIM_SetCounterMode(TIM2, LL_TIM_COUNTERMODE_UP);
    LL_TIM_EnableIT_CC1(TIM2);
    /*
     * Setup NVIC
     */
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 1);
    return;
}

void TIM2_IRQHandler(void) {
    LL_TIM_ClearFlag_CC1(TIM2);
}


const uint8_t ROTATION_TRIGGER = 5;

void EXTI0_1_IRQHandler(void) {

    static const int8_t states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

    static uint8_t enc_trans = 0;

    static int8_t enc_dir = 0;

    uint8_t enc_state = 0x00;

    enc_state = 0x0003 & LL_GPIO_ReadInputPort(GPIOA);
    enc_trans = ((0x03 & enc_trans) << 2) | enc_state;
    enc_dir += states[enc_trans];

    if (enc_dir == ROTATION_TRIGGER) {
        if (snake_speed < 10) {
            ++snake_speed;
            statusbar_display(snake_speed);
        }
        enc_dir = 0;
    }

    if (enc_dir == -ROTATION_TRIGGER) {
      if (snake_speed > 0) {
          --snake_speed;
          statusbar_display(snake_speed);
        }
        enc_dir = 0;
    }

    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
}

static void set_indicator (uint32_t number) {

  uint32_t port_state = 0;

  static uint32_t mask = LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | \
                         LL_GPIO_PIN_7 | LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | \
                         LL_GPIO_PIN_10 | LL_GPIO_PIN_11;

  // decoder
  static const uint32_t decoder[] = {

        LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7 | \
        LL_GPIO_PIN_9 | LL_GPIO_PIN_8,
        // 0
        LL_GPIO_PIN_5 | LL_GPIO_PIN_6,
        // 1
        LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_10 | LL_GPIO_PIN_8 | \
        LL_GPIO_PIN_7,
         // 2
        LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7 | \
        LL_GPIO_PIN_10, // 3

        LL_GPIO_PIN_9 | LL_GPIO_PIN_10 | LL_GPIO_PIN_5 | LL_GPIO_PIN_6, //4

        LL_GPIO_PIN_4 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10 | LL_GPIO_PIN_6 | \
        LL_GPIO_PIN_7, //5

        LL_GPIO_PIN_4 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10 | LL_GPIO_PIN_6 | \
        LL_GPIO_PIN_8 | LL_GPIO_PIN_7,  //6

        LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_6, //7

        LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7 | \
        LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10, //8

        LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7 | \
        LL_GPIO_PIN_10 | LL_GPIO_PIN_9 //9

    };
  // read current state
  port_state = LL_GPIO_ReadOutputPort(GPIOB);

  const uint8_t max_number = sizeof(decoder) / sizeof(uint32_t);

  // write result
  port_state = (port_state & ~mask) | decoder[number % max_number];
  LL_GPIO_WriteOutputPort(GPIOB, port_state);

  return;
}

static void enable_indicator (uint8_t number) {

  uint32_t port_state = 0;

  uint32_t mask = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3;

  // decoder
  static const uint32_t decoder[] = {
    LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3
  };

  // read current state
  port_state = LL_GPIO_ReadOutputPort(GPIOB);

  const uint8_t max_number = sizeof(decoder) / sizeof(uint32_t);

  // write result
  port_state = (port_state & ~mask) | (~decoder[number % max_number] & mask);
  LL_GPIO_WriteOutputPort(GPIOB, port_state);

  return;
}

static void hc595_config(void)
{
    /*
     * Init latch pin
     */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_15, LL_GPIO_MODE_OUTPUT);
    /*
     * Init GPIO pertained to SPI module
     */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    //SPI_MOSI
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_15, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_15, LL_GPIO_AF_0);
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_15, LL_GPIO_SPEED_FREQ_HIGH);
    //SPI_SCK
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_13, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_13, LL_GPIO_AF_0);
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_13, LL_GPIO_SPEED_FREQ_HIGH);
    /*
     * Init SPI
     */
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SPI1);
    LL_SPI_SetMode(SPI2, LL_SPI_MODE_MASTER);
    LL_SPI_SetBaudRatePrescaler(SPI2, LL_SPI_BAUDRATEPRESCALER_DIV8);
    LL_SPI_SetTransferBitOrder(SPI2, LL_SPI_MSB_FIRST);
    LL_SPI_SetDataWidth(SPI2, LL_SPI_DATAWIDTH_8BIT);
    //LL_SPI_SetNSSMode(SPI1, LL_SPI_NSS_HARD_OUTPUT);
    //LL_SPI_EnableNSSPulseMgt(SPI1);
    LL_SPI_Enable(SPI2);
}

static void hc595_set(uint16_t data)
{
    uint16_t counter = 1000;

    /*
     * Send the data
     */
    LL_SPI_TransmitData16(SPI2, data);
    while (!LL_SPI_IsActiveFlag_TXE(SPI2));
    /*
     * Toggle latch pin
     */
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_15);
    while (counter--);
    LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_15);
    return;
}

void statusbar_display (uint32_t state) {
  static uint32_t mask = LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | \
                         LL_GPIO_PIN_7 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | \
                         LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | \
                         LL_GPIO_PIN_15;

  uint32_t sb_curr_state = LL_GPIO_ReadOutputPort(GPIOC);
  uint32_t sb_display = 0;

  switch (state) {
    case 0: sb_display = SB_0;
    break;
    case 1: sb_display = SB_1;
    break;
    case 2: sb_display = SB_2;
    break;
    case 3: sb_display = SB_3;
    break;
    case 4: sb_display = SB_4;
    break;
    case 5: sb_display = SB_5;
    break;
    case 6: sb_display = SB_6;
    break;
    case 7: sb_display = SB_7;
    break;
    case 8: sb_display = SB_8;
    break;
    case 9: sb_display = SB_9;
    break;
    case 10: sb_display = SB_10;
    break;
    default: sb_display = sb_curr_state;
  }

  LL_GPIO_WriteOutputPort(GPIOC, sb_display | (sb_curr_state & ~mask) );

}

void up_button_pressed() {
  number_of_fruits += 1;
  return;
}

void down_button_pressed() {
  number_of_fruits += 2;
  return;
}

void left_button_pressed() {
  number_of_fruits += 3;
  return;
}

void right_button_pressed() {
  number_of_fruits += 4;
  return;
}

const uint32_t piezo_speaker_duration = 500; //ms

uint32_t piezo_speaker_cnt = 0;

typedef enum {FALSE, TRUE} piezo_speaker_flag;

static void systick_config(void) {
    LL_InitTick(48000000, 1000);
    LL_SYSTICK_EnableIT();
    NVIC_SetPriority(SysTick_IRQn, 0);
    return;
}

void SysTick_Handler(void) {
  static uint8_t digit = 0;

  enable_indicator(digit);

  switch (digit) {
    case 0: set_indicator(number_of_fruits % 10);
    break;
    case 1: set_indicator((number_of_fruits % 100) / 10);
    break;
    case 2: set_indicator((number_of_fruits % 1000) / 100);
    break;
    case 3: set_indicator(number_of_fruits / 1000);
    break;
    default: ;
  }

  ++digit;
  digit = digit % 4;

  if (!LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_3)) {
    ++tmp_up;
    if (tmp_up == 50) {
      up_button_pressed();
    }
  } else {
    tmp_up = 0;
  }

  if (!LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_0)) {
    ++tmp_down;
    if (tmp_down == 50) {
      down_button_pressed();
    }
  } else {
    tmp_down = 0;
  }

  if (!LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_1)) {
    ++tmp_left;
    if (tmp_left == 50) {
      left_button_pressed();
    }
  } else {
    tmp_left = 0;
  }

  if (!LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_2)) {
    ++tmp_right;
    if (tmp_right == 50) {
      right_button_pressed();
    }
  }
  else {
    tmp_right = 0;
  }
}

int main(void) {
    rcc_config();
    gpio_config();
    exti_config();
    timers_config();
    hc595_config();
    systick_config();

    hc595_set(0xABCD);

    while (1);
    return 0;
}
