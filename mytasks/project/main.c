#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_tim.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_cortex.h"

/**
  * System Clock Configuration
  * The system Clock is configured as follow :
  *    System Clock source            = PLL (HSI/2)
  *    SYSCLK(Hz)                     = 48000000
  *    HCLK(Hz)                       = 48000000
  *    AHB Prescaler                  = 1
  *    APB1 Prescaler                 = 1
  *    HSI Frequency(Hz)              = 8000000
  *    PLLMUL                         = 12
  *    Flash Latency(WS)              = 1
  */
static void rcc_config()
{
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

static void gpio_config(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_2, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_3, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_2, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_3, LL_GPIO_MODE_OUTPUT);
    
    return;
}

static void timer2_config(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_5, LL_GPIO_AF_2);

    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_1, LL_GPIO_AF_2);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
    LL_TIM_SetPrescaler(TIM2, 479);
    LL_TIM_SetAutoReload(TIM2, 999);
    LL_TIM_OC_SetCompareCH1(TIM2, 0);
    LL_TIM_OC_SetCompareCH2(TIM2, 0);
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetMode(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetMode(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
    LL_TIM_SetCounterMode(TIM2, LL_TIM_COUNTERMODE_UP);
    LL_TIM_EnableIT_CC1(TIM2);
    LL_TIM_EnableCounter(TIM2);

    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 1);
    return;
}

static void systick_config(void)
{
    LL_InitTick(48000000, 250);
    LL_SYSTICK_EnableIT();
    NVIC_SetPriority(SysTick_IRQn, 0);

    return;
}

static void timer3_config(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_6, LL_GPIO_AF_1);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_6, LL_GPIO_PULL_UP);

    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_7, LL_GPIO_AF_1);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_7, LL_GPIO_PULL_UP);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
    LL_TIM_SetPrescaler(TIM3, 47999);

    LL_TIM_IC_SetFilter(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV32_N8);
    LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH1,
                          LL_TIM_IC_POLARITY_RISING);
    LL_TIM_IC_SetActiveInput(TIM3, LL_TIM_CHANNEL_CH1,
                             LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
    LL_TIM_EnableIT_CC1(TIM3);

    LL_TIM_IC_SetFilter(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV32_N8);
    LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH2,
                          LL_TIM_IC_POLARITY_RISING);
    LL_TIM_IC_SetActiveInput(TIM3, LL_TIM_CHANNEL_CH2,
                             LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);
    LL_TIM_EnableIT_CC2(TIM3);
    LL_TIM_EnableCounter(TIM3);

    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_SetPriority(TIM3_IRQn, 1);
    return;
}

static void timer1_config(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_8, LL_GPIO_AF_2);
    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_9, LL_GPIO_AF_2);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_8, LL_GPIO_PULL_UP);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_9, LL_GPIO_PULL_UP);

    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);

    LL_TIM_SetEncoderMode(TIM1, LL_TIM_ENCODERMODE_X4_TI12);
    LL_TIM_IC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH1,
                          LL_TIM_IC_POLARITY_FALLING);
    LL_TIM_IC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH2,
                          LL_TIM_IC_POLARITY_FALLING);

    LL_TIM_SetAutoReload(TIM1, 119);
    LL_TIM_EnableCounter(TIM1);
    return;
}

static void timer15_config(void)
{
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM15);
    LL_TIM_SetPrescaler(TIM15, 47999);
    LL_TIM_SetAutoReload(TIM15, 999);
    LL_TIM_SetCounterMode(TIM15, LL_TIM_COUNTERMODE_UP);
    LL_TIM_EnableIT_UPDATE(TIM15);
    LL_TIM_EnableCounter(TIM15);

    NVIC_EnableIRQ(TIM15_IRQn);
    NVIC_SetPriority(TIM15_IRQn, 0);

    LL_TIM_GenerateEvent_UPDATE(TIM15);

    return;
}

//////////////////////////////////////////////////////////////////////////

struct time
{
    uint32_t hrs;
    uint32_t min;
    uint32_t sec;
};

enum SET_MODE
{
    NONE,
    FIRST,
    SECOND
};

enum IND_MODE
{
    HRS_MIN,
    MIN_SEC
};

enum MODE
{
    CLOCK,
    ALARM,
    TIMER
};

void set_timer_hrs(struct time* tm)
{
    LL_TIM_SetCounter(TIM1, tm->hrs * 5);
}

void set_timer_min(struct time* tm)
{
    LL_TIM_SetCounter(TIM1, tm->min * 2);
}

void set_timer_sec(struct time* tm)
{
    LL_TIM_SetCounter(TIM1, tm->sec * 2);
}

void set_hrs(struct time* tm, uint32_t hrs)
{
    tm->hrs = hrs;
}

void set_min(struct time* tm, uint32_t min)
{
    tm->min = min;
}

void set_sec(struct time* tm, uint32_t sec)
{
    tm->sec = sec;
}

void clear_time(struct time* tm)
{
    tm->hrs = 0;
    tm->min = 0;
    tm->sec = 0;
}

void increment_time(struct time* tm)
{
    tm->sec++;
    tm->min += tm->sec / 60;
    tm->sec %= 60;
    tm->hrs += tm->min / 60;
    tm->min %= 60;
    tm->hrs %= 24;
}

void decrement_time(struct time* tm)
{
    if (tm->sec > 0)
    {
        tm->sec--;
        return;
    }
    tm->sec = 59;
    if (tm->min > 0)
    {
        tm->min--;
        return;
    }
    tm->min = 59;
    if (tm->hrs > 0)
    {
        tm->hrs--;
        return;
    }
    tm->hrs = 23;
}


struct time glob_tm  = {0, 0, 0};
struct time alarm_tm = {0, 0, 0};
struct time timer_tm = {0, 0, 0};

int ind_dot = 1;
int ind_first = 1;
int ind_second = 1;

int alarm_alarm = 0;
int alarm_active = 0;

int timer_alarm = 0;
int timer_active = 0;

enum SET_MODE setting_mode = NONE;
enum MODE global_mode = CLOCK;


void toggle(int* var)
{
    *var = (*var == 0 ? 1 : 0);
}

void set_alarm_cur()
{
    alarm_tm.hrs = glob_tm.hrs;
    alarm_tm.min = glob_tm.min;
}

static void set_time_indicator(struct time* tm, enum IND_MODE mode)
{
    static int digit_num = 0;

    static uint32_t mask = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | \
                           LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | \
                           LL_GPIO_PIN_6 | LL_GPIO_PIN_7;

    static const uint32_t decoder[] = {
        LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3 | \
        LL_GPIO_PIN_4 | LL_GPIO_PIN_5, // 0
        LL_GPIO_PIN_1 | LL_GPIO_PIN_2, // 1
        LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_6 | LL_GPIO_PIN_4 | \
        LL_GPIO_PIN_3, // 2
        LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_6 | LL_GPIO_PIN_2 | \
        LL_GPIO_PIN_3, // 3
        LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2, // 4
        LL_GPIO_PIN_0 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3 | LL_GPIO_PIN_5 | \
        LL_GPIO_PIN_6, // 5
        LL_GPIO_PIN_0 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | \
        LL_GPIO_PIN_5 | LL_GPIO_PIN_6, //6
        LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2, //7
        LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3 | \
        LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_6, //8
        LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3 | \
        LL_GPIO_PIN_5 | LL_GPIO_PIN_6 //9
    };

    uint32_t port_state = LL_GPIO_ReadOutputPort(GPIOB);

    static uint32_t digit_mask = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3;
    LL_GPIO_WriteOutputPort(GPIOC, digit_mask);
    
    port_state &= ~mask;

    switch(digit_num)
    {
        case 0:
            if (ind_first)
            {
                LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0);
                port_state |= decoder[(mode == HRS_MIN ? tm->hrs : tm->min) / 10];
            }
            break;
        case 1:
            if (ind_first)
            {
                LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_1);
                port_state |= decoder[(mode == HRS_MIN ? tm->hrs : tm->min) % 10];
            }
            if (ind_dot) 
                port_state |= LL_GPIO_PIN_7;
            break;
        case 2:
            if (ind_second)
            {
                LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_2);
                port_state |= decoder[(mode == HRS_MIN ? tm->min : tm->sec) / 10];
            }
            break;
        case 3:
            if (ind_second)
            {
                LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_3);
                port_state |= decoder[(mode == HRS_MIN ? tm->min : tm->sec) % 10];
            }
            break;
        default:
            break;
    }
    LL_GPIO_WriteOutputPort(GPIOB, port_state);

    digit_num = (digit_num + 1) % 4;
    return;
}

void TIM15_IRQHandler(void)
{
    increment_time(&glob_tm);

    if (alarm_active && setting_mode == NONE &&
        glob_tm.hrs == alarm_tm.hrs && glob_tm.min == alarm_tm.min)
    {
        alarm_alarm = 1;
    }

    if (timer_active)
        decrement_time(&timer_tm);

    if (timer_active && setting_mode == NONE &&
        timer_tm.min == 0 && timer_tm.sec == 0)
    {
        timer_alarm = 1;
        timer_active = 0;
    }

    LL_TIM_ClearFlag_UPDATE(TIM15);
}

void SysTick_Handler(void)
{
    static int counter = 0;

    counter++;
    if (counter % 83 == 0)
    {
        if (setting_mode == FIRST)
            toggle(&ind_first);
        else if (setting_mode == SECOND)
            toggle(&ind_second);
    }

    if (counter % 125 == 0)
        if (setting_mode == NONE)
            toggle(&ind_dot);

    switch (global_mode)
    {
        case CLOCK:
            set_time_indicator(&glob_tm, HRS_MIN);
            break;
        case ALARM:
            set_time_indicator(&alarm_tm, HRS_MIN);
            break;
        case TIMER:
            set_time_indicator(&timer_tm, MIN_SEC);
        default:
            break;
    }

    if (setting_mode == FIRST)
    {
        uint16_t cur_enc = LL_TIM_GetCounter(TIM1);
        switch (global_mode)
        {
            case CLOCK:
                set_hrs(&glob_tm, cur_enc / 5);
                break;
            case ALARM:
                set_hrs(&alarm_tm, cur_enc / 5);
                break;
            case TIMER:
                set_min(&timer_tm, cur_enc / 2);                
            default:
                break;
        }
    }
    else if (setting_mode == SECOND)
    {
        uint16_t cur_enc = LL_TIM_GetCounter(TIM1);
        switch (global_mode)
        {
            case CLOCK:
                set_min(&glob_tm, cur_enc / 2);
                break;
            case ALARM:
                set_min(&alarm_tm, cur_enc / 2);
                break;
            case TIMER:
                set_sec(&timer_tm, cur_enc / 2);
            default:
                break;
        }
    }
}

void TIM3_IRQHandler(void)
{
    if (alarm_alarm)
    {
        alarm_active = 0;
        alarm_alarm = 0;

        LL_TIM_ClearFlag_CC1(TIM3);
        LL_TIM_ClearFlag_CC2(TIM3);
        return;
    }
    if (timer_alarm)
    {
        timer_active = 0;
        timer_alarm = 0;
        clear_time(&timer_tm);

        LL_TIM_ClearFlag_CC1(TIM3);
        LL_TIM_ClearFlag_CC2(TIM3);
        return;
    }

    if (LL_TIM_IsActiveFlag_CC1(TIM3))
    {
        switch (setting_mode)
        {
            case NONE:
                setting_mode = FIRST;
                switch (global_mode) 
                {                    
                    case CLOCK:
                        set_timer_hrs(&glob_tm);
                        break;
                    case TIMER:
                        set_timer_min(&timer_tm);
                    default:
                        break;
                }
                break;
            case FIRST:
                setting_mode = SECOND;
                ind_first = 1;
                switch (global_mode)
                {  
                    case CLOCK:
                        set_timer_min(&glob_tm);
                        break;
                    case ALARM:
                        set_timer_min(&alarm_tm);
                        break;
                    case TIMER:
                        timer_active = 0;
                        set_timer_sec(&timer_tm);
                    default:
                        break;
                }
                break;
            case SECOND:
                setting_mode = NONE;
                ind_second = 1;
                switch (global_mode)
                {
                    case ALARM:
                        alarm_active = 1;
                        global_mode = CLOCK;
                        break;
                    case TIMER:
                        timer_active = 1;
                    default:
                        break;
                }
            default:
                break;
        }
        LL_TIM_ClearFlag_CC1(TIM3);
    }
    else if (LL_TIM_IsActiveFlag_CC2(TIM3))
    {
        switch (global_mode)
        {
            case CLOCK:
                global_mode = ALARM;
                setting_mode = FIRST;
                set_alarm_cur();
                set_timer_hrs(&alarm_tm);
                break;
            case ALARM:
                global_mode = TIMER;
                ind_first = 1;
                ind_second = 1;
                setting_mode = NONE;
                clear_time(&timer_tm);
                break;
            case TIMER:
                global_mode = CLOCK;
                ind_first = 1;
                ind_second = 1;
                setting_mode = NONE;
            default:
                break;
        }
        LL_TIM_ClearFlag_CC2(TIM3);
    }
}

void TIM2_IRQHandler(void)
{
    static int counter = 0;

    if (alarm_alarm || timer_alarm)
    {
        counter = (counter + 1) % 1500;
        LL_TIM_OC_SetCompareCH1(TIM2, counter * 2 / 3);
        LL_TIM_OC_SetCompareCH2(TIM2, counter * 2 / 3);
    }
    else
    {
        LL_TIM_OC_SetCompareCH1(TIM2, 0);
        LL_TIM_OC_SetCompareCH2(TIM2, 0);
    }
    LL_TIM_ClearFlag_CC1(TIM2);
}

int main(void)
{
    rcc_config();
    gpio_config();
    timer2_config();
    timer3_config();
    timer1_config();
    timer15_config();
    systick_config();

    while (1);

    return 0;
}
