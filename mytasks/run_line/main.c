#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"
#include <ctype.h>
#include <string.h>
#include <stdlib.h>

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

/*
 * Clock on GPIOC and set two led pins
 */
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

__attribute__((naked)) static void delay_1ms(void)
{
    asm ("push {r7, lr}");
    asm ("ldr r6, [pc, #8]");
    asm ("sub r6, #1");
    asm ("cmp r6, #0");
    asm ("bne delay_1ms+0x4");
    asm ("pop {r7, pc}");
    asm (".word 0x1770"); //6000
}

static uint32_t decoder(char sym)
{
    sym = toupper(sym);
    uint32_t res = 0;
    switch (sym)
    {
        case 'A':
            res = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | \
                  LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_6;
            break;
        case 'C':
            res = LL_GPIO_PIN_0 | LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | \
                  LL_GPIO_PIN_5;
            break;
        case 'E':
            res = LL_GPIO_PIN_0 | LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | \
                  LL_GPIO_PIN_5 | LL_GPIO_PIN_6;
            break;
        case 'F':
            res = LL_GPIO_PIN_0 | LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | \
                  LL_GPIO_PIN_6;
            break;
        case 'H':
            res = LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_4 | \
                  LL_GPIO_PIN_5 | LL_GPIO_PIN_6;
            break;
        case 'I':
            res = LL_GPIO_PIN_1 | LL_GPIO_PIN_2; 
            break;
        case 'L':
            res = LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | LL_GPIO_PIN_5;
            break;
        case 'O':
            res = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | \
                  LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | LL_GPIO_PIN_5;
            break;
        case 'P':
            res = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_4 | \
                  LL_GPIO_PIN_5 | LL_GPIO_PIN_6;
            break;
        case 'S':
            res = LL_GPIO_PIN_0 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3 | \
                  LL_GPIO_PIN_5 | LL_GPIO_PIN_6;
            break;
        case 'U':
            res = LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3 | \
                  LL_GPIO_PIN_4 | LL_GPIO_PIN_5;
            break;
        case 'Y':
            res = LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3 | \
                  LL_GPIO_PIN_5 | LL_GPIO_PIN_6;
            break;
        default: 
            break;
    }
    return res;
}

static void set_indicator(char* str)
{
    static int digit_num = 0;

    static uint32_t mask = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | \
                           LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | \
                           LL_GPIO_PIN_6;

    uint32_t port_state = LL_GPIO_ReadOutputPort(GPIOB);

    static uint32_t digit_mask = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3;
    LL_GPIO_WriteOutputPort(GPIOC, digit_mask);
    
    switch(digit_num)
    {
        case 0:
            LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0);
            port_state = (port_state & ~mask) | decoder(str[0]);
            break;
        case 1:
            LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_1);
            port_state = (port_state & ~mask) | decoder(str[1]);
            break;
        case 2:
            LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_2);
            port_state = (port_state & ~mask) | decoder(str[2]);
            break;
        case 3:
            LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_3);
            port_state = (port_state & ~mask) | decoder(str[3]);
            break;
        default:
            break;
    }
    LL_GPIO_WriteOutputPort(GPIOB, port_state);

    digit_num = (digit_num + 1) % 4;
    return;
}

int main(void)
{
    char str[] = "hello people";
    
    char* buf = (char*) calloc(1, 64);

    strcat(buf, "    ");
    strcat(buf, str);
    strcat(buf, "    ");

    rcc_config();
    gpio_config();
    
    char* pstr = buf;
    int move = 0;
    while (1)
    {
        set_indicator(pstr);
        delay_1ms();
        move++;
        if (move == 400)
        {
            pstr++;
            move = 0;
        }
        if (pstr[4] == '\0')
        {
            pstr = buf;
        }
    }

    free(buf);
    return 0;
}
