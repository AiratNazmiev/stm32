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

// game values------------------------------

uint8_t ROUND_NUMBER = 0;

const uint8_t MAX_ROUND = 5;

const uint8_t FIRST_PLAYER = 0;

const uint8_t SECOND_PLAYER = 1;

uint32_t FIRST_POINTS = 0;

uint32_t SECOND_POINTS = 0;

uint8_t GAME_ITERATION = 0;

uint8_t GAMER_TURN = 0; //0 or 1

//  delay

uint32_t DELAY = 1000; //ms

uint8_t DELAY_FLAG = 0;

uint32_t DELAY_CNT = 0;

// 0

uint32_t FIRST_LUCK = 0;

uint32_t FIRST_LUCK_FLAG = 0;

uint32_t SECOND_LUCK = 0;

uint32_t SECOND_LUCK_FLAG = 0;

// 1

uint32_t FIRST_DISTANCE = 10;

uint32_t FIRST_DISTANCE_FLAG = 0;

uint32_t SECOND_DISTANCE = 10;

uint32_t SECOND_DISTANCE_FLAG = 0;

// 2

uint8_t SHOOTING_START = 0;

uint8_t FIRST_FIRED = 0;

uint8_t FIRST_SHOT_SECOND = 0;

uint8_t SECOND_FIRED = 0;

uint8_t SECOND_SHOT_FIRST = 0;
//

//-------------------------------------------


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

void up_button_pressed();
void down_button_pressed();
void left_button_pressed();
void right_button_pressed();
//

static void set_indicator (uint32_t number);

static void enable_indicator (uint8_t number);

void statusbar_display (uint32_t state);

uint8_t shot_outcome (uint8_t player);

void delay_randomize();

void points_indicator();

//

uint32_t SEVEN_SEGMENT_NUM = 0;

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
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);

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

    //signal LED
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
    //

    //first player LEDs
    LL_GPIO_SetPinMode(GPIOF, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
    //

    //second player LED
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_14, LL_GPIO_MODE_OUTPUT);
    //

    //encoder
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_0, LL_GPIO_PULL_UP);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_1, LL_GPIO_PULL_UP);
    //

    return;
}

static void exti_config (void) {

  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE1);

  // LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_0);
  // LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_1);

  LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_1);
  LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_1);

  LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_0);
  LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_0);

  NVIC_EnableIRQ(EXTI0_1_IRQn);
  NVIC_SetPriority(EXTI0_1_IRQn, 0);

  return;
}

//------------------------------------------

static void timers_config(void) {
  /*
 * Configure input channel
 */
LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);
LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_5, LL_GPIO_AF_2);
/*
 * Setup timer to output compare mode
 */
LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2); //SOUND OOOOFFFFFFFFFFFFFFFF
LL_TIM_SetPrescaler(TIM2, 479);
LL_TIM_SetAutoReload(TIM2, 999);
LL_TIM_OC_SetCompareCH1(TIM2, 0); //piezo turned off
LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
LL_TIM_OC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
LL_TIM_OC_SetMode(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
LL_TIM_SetCounterMode(TIM2, LL_TIM_COUNTERMODE_UP);
LL_TIM_EnableIT_CC1(TIM2);
LL_TIM_EnableCounter(TIM2);
/*
 * Setup NVIC
 */
NVIC_EnableIRQ(TIM2_IRQn);
NVIC_SetPriority(TIM2_IRQn, 1);
return;
}

uint8_t ENABLE_SOUND = 0;

const uint32_t SOUND_DURATION = 10;

void TIM2_IRQHandler(void) {
  static uint32_t sound_time_passed = 0;

    if (ENABLE_SOUND == 1) {
      ++sound_time_passed;
      LL_TIM_OC_SetCompareCH1(TIM2, 1000);
      if (sound_time_passed > SOUND_DURATION) {
        ENABLE_SOUND = 0;
        sound_time_passed = 0;
        LL_TIM_OC_SetCompareCH1(TIM2, 0);
      }
    }
    LL_TIM_ClearFlag_CC1(TIM2);
}

void shot_sound() { ///////////////////////////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  ENABLE_SOUND = 1;
}

void survivor_sound() {
  //
}

void kill_sound() {
  //
}

//------------------------------------------------

const uint8_t ROTATION_TRIGGER = 5;

uint32_t ENCODER_STATE = 10;

void EXTI0_1_IRQHandler(void) {

    static const int8_t states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

    static uint8_t enc_trans = 0;

    static int8_t enc_dir = 0;

    uint8_t enc_state = 0x00;

    enc_state = 0x0003 & LL_GPIO_ReadInputPort(GPIOA);
    enc_trans = ((0x03 & enc_trans) << 2) | enc_state;
    enc_dir += states[enc_trans];

    if (enc_dir == ROTATION_TRIGGER) {
        if (ENCODER_STATE < 90) {
            ENCODER_STATE += 10;
            SEVEN_SEGMENT_NUM = ENCODER_STATE;
        }
        enc_dir = 0;
    }

    if (enc_dir == -ROTATION_TRIGGER) {
      if (ENCODER_STATE > 10) {
          ENCODER_STATE -= 10;
          SEVEN_SEGMENT_NUM = ENCODER_STATE;
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
        LL_GPIO_PIN_8 | LL_GPIO_PIN_9,
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

// --------------------------------------------------

uint32_t STATUSBAR_STATE = 0;

const uint32_t STATUSBAR_MOVEMEMENT_SPEED = 20;//ms

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

void statusbar_movement() {
  static uint32_t statusbar_cnt = 0;
  statusbar_display(STATUSBAR_STATE);
  ++statusbar_cnt;
  if (statusbar_cnt == STATUSBAR_MOVEMEMENT_SPEED) {
    if (STATUSBAR_STATE == 10){
      STATUSBAR_STATE = 0;
    } else {
      ++STATUSBAR_STATE;
    }
    statusbar_cnt = 0;
  }
}

//---------------------------------------------

const uint32_t PRESS_TRIGGER = 7; //*10 ms

void buttons_Handler() {
  if (!LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_3)) {
    ++tmp_up;
    if (tmp_up == PRESS_TRIGGER) {
      up_button_pressed();
    }
  } else {
    tmp_up = 0;
  }

  if (!LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_0)) {
    ++tmp_down;
    if (tmp_down == PRESS_TRIGGER) {
      down_button_pressed();
    }
  } else {
    tmp_down = 0;
  }

  if (!LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_1)) {
    ++tmp_left;
    if (tmp_left == PRESS_TRIGGER) {
      left_button_pressed();
    }
  } else {
    tmp_left = 0;
  }

  if (!LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_2)) {
    ++tmp_right;
    if (tmp_right == PRESS_TRIGGER) {
      right_button_pressed();
    }
  }
  else {
    tmp_right = 0;
  }
}

void up_button_pressed() {
  switch (GAME_ITERATION) {
    case 0:
      if (!FIRST_LUCK_FLAG) {
        FIRST_LUCK = STATUSBAR_STATE;
        FIRST_LUCK_FLAG = 1;
      }
    break;
    case 1:
      if (!FIRST_DISTANCE_FLAG) {
        FIRST_DISTANCE = ENCODER_STATE;
        FIRST_DISTANCE_FLAG = 1;
      }
    break;
    case 2:
        //NOTHING
    break;
    case 3:
        //NOTHING
    break;
    default: ;
  }
}

void down_button_pressed() {
  switch (GAME_ITERATION) {
    case 0:
      if (FIRST_LUCK_FLAG) {
        SECOND_LUCK = STATUSBAR_STATE;
        SECOND_LUCK_FLAG = 1;
      }
    break;
    case 1:
      if (FIRST_DISTANCE_FLAG) {
        SECOND_DISTANCE = ENCODER_STATE;
        SECOND_DISTANCE_FLAG = 1;
      }
    break;
    case 2:
        //NOTHING
    break;
    case 3:
        //NOTHING
    break;
    default: ;
  }
}

void right_button_pressed() { //first
  switch (GAME_ITERATION) {
    case 0:
      //NOTHING
    break;
    case 1:
      //NOTHING
    break;
    case 2:
      if (SHOOTING_START) {
        if (!SECOND_SHOT_FIRST && !FIRST_FIRED) {
          shot_sound();
          FIRST_FIRED = 1;
          if (shot_outcome(FIRST_PLAYER)) {
            FIRST_SHOT_SECOND = 1;
            ++FIRST_POINTS;
          }
        }
      }
    break;
    case 3:
      //NOTHING
    break;
    default: ;
  }
}

void left_button_pressed() { //second
  switch (GAME_ITERATION) {
    case 0:
      //NOTHING
    break;
    case 1:
      //NOTHING
    break;
    case 2:
      if (SHOOTING_START) {
        if (!FIRST_SHOT_SECOND && !SECOND_FIRED) {
          shot_sound();
          SECOND_FIRED = 1;
          if (shot_outcome(SECOND_PLAYER)) {
            SECOND_SHOT_FIRST = 1;
            ++SECOND_POINTS;
          }
        }
      }
    break;
    case 3:
      //NOTHING
    break;
    default: ;
  }
}

//--------------------------------

void seven_segment_Handler() {
  static uint8_t digit = 0;

  enable_indicator(digit);

  switch (digit) {
    case 0: set_indicator(SEVEN_SEGMENT_NUM % 10);
    break;
    case 1: set_indicator((SEVEN_SEGMENT_NUM % 100) / 10);
    break;
    case 2: set_indicator((SEVEN_SEGMENT_NUM % 1000) / 100);
    break;
    case 3: set_indicator(SEVEN_SEGMENT_NUM / 1000);
    break;
    default: ;
  }

  ++digit;
  digit = digit % 4;
}

//---------------------------------------------------------

static void systick_config(void) {
    LL_InitTick(48000000, 1000);
    LL_SYSTICK_EnableIT();
    NVIC_SetPriority(SysTick_IRQn, 0);
    return;
}


void delay(uint32_t time) {
  ++DELAY_CNT;
  if (DELAY_CNT == time) {
    DELAY_FLAG = 0;
    DELAY_CNT = 0;
  }
}

void luck_iteration() {
  points_indicator();
  switch (GAMER_TURN) {
    case 0:
      statusbar_movement();
      if (FIRST_LUCK_FLAG) {
        GAMER_TURN = 1;
        STATUSBAR_STATE = 0;
        DELAY_FLAG = 1;
        SEVEN_SEGMENT_NUM = FIRST_LUCK;
      }
    break;
    case 1:
      statusbar_movement();
      if (SECOND_LUCK_FLAG){
        GAMER_TURN = 0;
        DELAY_FLAG = 1;
        SEVEN_SEGMENT_NUM = SECOND_LUCK;

        GAME_ITERATION = 1;// NEW ITERATION
      }
    break;
    default: ;
  }
}

void distance_iteration() {
  statusbar_display(SB_0);
  //Enable encoder
  LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_0);
  LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_1);

  switch (GAMER_TURN) {
    case 0:
      if (FIRST_DISTANCE_FLAG) {
        GAMER_TURN = 1;
        ENCODER_STATE = 10;
        DELAY_FLAG = 1;
        SEVEN_SEGMENT_NUM = FIRST_DISTANCE;
      }
    break;
    case 1:
      if (SECOND_DISTANCE_FLAG){
        GAMER_TURN = 0;
        DELAY_FLAG = 1;
        SEVEN_SEGMENT_NUM = SECOND_DISTANCE;

        GAME_ITERATION = 2;// NEW ITERATION

        delay_randomize();// RANDOMIZATION OF DELAY FOR SHOOTING
        //Disable encoder
        LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_0);
        LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_1);
      }
    break;
    default: ;
  }

}

const uint32_t SHOOTING_ITERATION_TIME = 5000; //ms

//uint32_t shooting_time_passed = 0;

void shooting_iteration() {
  SHOOTING_START = 1;
  static uint32_t shooting_time_passed = 0;

  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_8); //signal LED
  LL_GPIO_SetOutputPin(GPIOF, LL_GPIO_PIN_6); //first player
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_14); //second player

  if (SECOND_SHOT_FIRST) {
    kill_sound(); //first is shot
    GAME_ITERATION = 3;// NEW ITERATION
    DELAY_FLAG = 1;
    shooting_time_passed = 0;
    LL_GPIO_ResetOutputPin(GPIOF, LL_GPIO_PIN_6);
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_8);
  } else if (SECOND_FIRED) {
    survivor_sound(); // first is alive
  }

  if (FIRST_SHOT_SECOND) {
    kill_sound(); //second is shot
    GAME_ITERATION = 3;// NEW ITERATION
    DELAY_FLAG = 1;
    shooting_time_passed = 0;
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_14);
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_8);
  } else if (FIRST_FIRED){
    survivor_sound();// second is alive
  }

  if (shooting_time_passed > SHOOTING_ITERATION_TIME) { //time limit
    GAME_ITERATION = 3; // NEW ITERATION
    DELAY_FLAG = 1;
    shooting_time_passed = 0;
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_8);
    survivor_sound(); // first is alive
    survivor_sound();// second is alive
  }

  ++shooting_time_passed;

  DELAY = 1000; //common value <- random
}

void result_iteration() {
  points_indicator();
  GAME_ITERATION = 4;
}

void reloading() {
  ++ROUND_NUMBER;

  GAME_ITERATION = 0;
  GAMER_TURN = 0;
  FIRST_LUCK = 0;
  FIRST_LUCK_FLAG = 0;
  SECOND_LUCK = 0;
  SECOND_LUCK_FLAG = 0;
  FIRST_DISTANCE = 10;
  FIRST_DISTANCE_FLAG = 0;
  SECOND_DISTANCE = 10;
  SECOND_DISTANCE_FLAG = 0;
  SHOOTING_START = 0;
  FIRST_FIRED = 0;
  FIRST_SHOT_SECOND = 0;
  SECOND_FIRED = 0;
  SECOND_SHOT_FIRST = 0;

  ENCODER_STATE = 10;
  STATUSBAR_STATE = 0;

  LL_GPIO_ResetOutputPin(GPIOF, LL_GPIO_PIN_6);
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_14);

  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_8);
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9);
}

uint32_t SEED = 0;

void SysTick_Handler(void) {
  if (ROUND_NUMBER >= MAX_ROUND) {
      points_indicator();
      LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_8);
  } else {

    if (DELAY_FLAG) {
      delay(DELAY);
    } else {

      switch (GAME_ITERATION) {
        case 0: luck_iteration();

        break;
        //
        case 1: distance_iteration();

        break;
        //
        case 2: shooting_iteration();

        break;
        //
        case 3: result_iteration();

        break;
        //
        case 4: reloading();

        break;
        }
    }
    ++SEED;

    if (SEED == 1001) {
      SEED = 0;
    }
  }

  seven_segment_Handler();
  buttons_Handler();
}

//-------------------------------------------------------

void delay_randomize() {
  DELAY = ((SEED % 8) + 1) * 500; //ms
  DELAY_FLAG = 1;
}

// 500 + (45 - distance / 4) * luck -> [500; 900]
uint8_t shot_outcome(uint8_t player) {
  uint32_t distance = FIRST_DISTANCE + SECOND_DISTANCE;
  if (player == FIRST_PLAYER){
    if (500 + (45 - distance) * FIRST_LUCK >= SEED) {
      return 1; //killed
    } else {
      return 0; //missed
    }
  } else {
    if (500 + (45 - distance) * SECOND_LUCK >= SEED) {
      return 1; //killed
    } else {
      return 0; //missed
    }
  }
}

void points_indicator () {
  SEVEN_SEGMENT_NUM = SECOND_POINTS * 100 + FIRST_POINTS;
}

//----------------------------------------------------------

int main(void) {
    rcc_config();
    gpio_config();
    exti_config();
    timers_config();
    systick_config();

    while (1);

    return 0;
}
