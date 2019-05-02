/*
 * This example demonstrates using I2C with SSD1306
 * oled controller
 */

#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"

#include "xprintf.h"
#include "oled_driver.h"

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
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_9, LL_GPIO_MODE_OUTPUT);
    return;
}

/*
 * Set callback for out device
 */
static void printf_config(void)
{
    xdev_out(oled_putc);
    return;
}

/*
 * Init all periphs and print out something
 */


void oled_draw_rect(int x1, int y1, int x2, int y2, enum color_t color)
{
    for (int x = x1; x <= x2; x++)
        for (int y = y1; y <= y2; y++)
            oled_set_pix(x, y, color);
}

extern unsigned char donov[];

int main(void)
{
    rcc_config();
    gpio_config();
    oled_config();
    printf_config();

    //xprintf("\n     Hello, John!\n\n");
    //xprintf("    Rom4ik the Fox\n");
    //xprintf("    is in a linden\n");
    //xprintf("    SSD1306 driver");

    //oled_pic(donov, 85);

    //oled_pic_dithering(donov);

    oled_draw_rect(10, 10, 30, 30, clWhite);
    oled_draw_rect(11, 11, 29, 29, clBlack);
    oled_draw_rect(12, 12, 28, 28, clWhite);
    oled_draw_rect(13, 13, 27, 27, clBlack);
    oled_draw_rect(14, 14, 26, 26, clWhite);
    oled_draw_rect(15, 15, 25, 25, clBlack);
    oled_draw_rect(16, 16, 24, 24, clBlack);
    oled_draw_rect(17, 17, 23, 23, clWhite);
    oled_draw_rect(18, 18, 22, 22, clBlack);
    oled_draw_rect(19, 19, 21, 21, clWhite);
    oled_draw_rect(20, 20, 20, 20, clBlack);

    oled_update();

    while (1);

    return 0;
}
