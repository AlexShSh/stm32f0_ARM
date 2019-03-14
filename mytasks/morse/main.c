#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"

#include <string.h>
#include <stdlib.h>
#include <ctype.h>

//#define FLASH_0LAT_DELAY0LAT
//#define FLASH_0LAT_DELAY1LAT
//#define FLASH_1LAT_DELAY0LAT
#define FLASH_1LAT_DELAY1LAT

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
 * Clock on GPIOC and set pin with Blue led connected
 */
static void gpio_config(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_9, LL_GPIO_MODE_OUTPUT);
    return;
}

/*
 * Just set of commands to waste CPU power for a second
 * (basically it is a simple cycle with a predefined number
 * of loops)
 */
__attribute__((naked)) static void delay(void)
{
    asm ("push {r7, lr}");
    asm ("ldr r6, [pc, #8]");
    asm ("sub r6, #1");
    asm ("cmp r6, #0");
    asm ("bne delay+0x4");
    asm ("pop {r7, pc}");
    asm (".word 0x2dc6c0"); //3000000
}

/*
 * Here we call configure all peripherals we need and
 * start blinking upon current mode
 */
 
enum {BUF_SIZE = 1024};

#define CASE(letter, str)  \
case letter:               \
{                          \
    strcat(buf, str);      \
    break;                 \
}
        
void parse_str(const char* str, char* buf)
{
    int i = 0;

    while (str[i] != '\0')
    {
        char ch = tolower(str[i]);
        switch (ch)
        {
            CASE('a', ".-");
            CASE('b', "-...");
            CASE('c', "-.-.");
            CASE('d', "-..");
            CASE('e', ".");
            CASE('f', "..-.");
            CASE('g', "--.");
            CASE('h', "....");
            CASE('i', "..");
            CASE('j', ".---");
            CASE('k', "-.-");
            CASE('l', ".-..");
            CASE('m', "--");
            CASE('n', "-.");
            CASE('o', "---");
            CASE('p', ".--.");
            CASE('q', "--.-");
            CASE('r', ".-.");
            CASE('s', "...");
            CASE('t', "-");
            CASE('u', "..-");
            CASE('v', "...-");
            CASE('w', ".--");
            CASE('x', "-..-");
            CASE('y', "-.--");
            CASE('z', "--..");

            CASE('.', ".-.-.-");
            CASE(',', "--..--");
            CASE('?', "..--..");
            CASE(';', "-.-.-.");
            CASE(':', "---...");
            CASE('!', "-.-.--");

            CASE(' ', " "); 

            default:
            {
                //printf("unknown symbol: %c\n", ch);
            }
        }
        if (str[i] != ' ' && str[i + 1] != ' ' && i < (int) strlen(str) - 1)
            strcat(buf, "_");
            
        i++; 
    }
}

#undef CASE

void show_dot()
{
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_9);
    delay();
    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9);
    delay();
}

void show_dash()
{
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_9);
    delay();
    delay();
    delay();
    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9);
    delay();
}

void show_underscore()
{
    delay();
    delay();
}

void show_space()
{
    for (int i = 0; i < 6; i++)
        delay();
}

void big_delay()
{
    for (int i = 0; i < 30; i++)
        delay();
}

void show_mcode(char* mcode)
{
    int i = 0;
    while (mcode[i] != '\0')
    {
        switch (mcode[i])
        {
            case '.': 
                show_dot();
                break;
            case '-':
                show_dash();
                break;
            case '_':
                show_underscore();
                break;
            case ' ':
                show_space();
            default: {}
        }
        i++;
    }
}


int main(void)
{
    rcc_config();
    gpio_config();
    
    char str[] = "ksrsp";
    char* mcode = (char*) calloc(sizeof(char), BUF_SIZE);
    parse_str(str, mcode);

    while (1) 
    {   
        delay();
        show_mcode(mcode);
        LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9);
        big_delay();
    }
    return 0;
}

