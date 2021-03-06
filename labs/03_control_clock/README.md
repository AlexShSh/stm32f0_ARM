## Задание 03 "control_clock"

### Подключение заголовочных файлов

В данной лаборотной работе вам предлагается повысить частоту тактирования микроконтроллера, а также рассмотрение такого параметра FLASH памяти, как задержка обращения.

Итак, для начала необходимо скопировать пустой проект, открыть main.c и подключить к нему несколько библиотек.
Так как мы собираемся использовать модуль тактирования, то подключим:

```c
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
```

Помимо это, чтобы задать частоту тактирования различных шин (APB, AHB), вам необходима следующая библиотека:

```c
#include "stm32f0xx_ll_bus.h"
```

Ну и напоследок для демонстрации необходимо будет мигать светодиодом, установленным на отладочной плате (подробнее о GPIO в следующих лекциях), поэтому:

```c
#include "stm32f0xx_ll_gpio.h"
```

Теперь перейдем к написанию функции для установки частоты тактирования, создайте пустую функцию rcc_config перед main:

```c
static void rcc_config()
{
}
```

Ключевое слово **static** необходимо, чтобы функция не была видна из других модулей, таким образом она может вызвана только в пределах данного файла.

### Настройка модуля тактирования 

Для начала выставите минимальную задержку обращения к FLASH памяти:
```c
LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
```

Согласно описанию, на отладочной плане нет внешнего источника тактирования для контроллера, поэтому еще раз переключитесь на источник внутреннего тактирования и дождитесь стабилизации частоты:

```c
LL_RCC_HSI_Enable();
while (LL_RCC_HSI_IsReady() != 1);
```

Так как частота внутреннего генератора всего 8МГц, настройте PLL, чтобы получить 48МГц. Например, можно поделить входную частоту на 2 и домножить на 12. Как обычно, после настройки включите сам модуль и дождитесь его готовности:

```c
LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2,
                            LL_RCC_PLL_MUL_12);
LL_RCC_PLL_Enable();
while (LL_RCC_PLL_IsReady() != 1);
```

Теперь настройте делитель для шины AHB и выставите PLL как источник для системной частоты:

```c
LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);
```

Напоследок, установите частоту для шины APB:

```c
LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
```

Функция для настройки модуля тактирования готова, добавьте ее вызов в main. После загрузки данного кода микроконтроллер будет работать на частоте 48МГц!

### Настройка вывода со светодиодом

Теперь помигаем светодиодом, чтобы микроконтроллер начал подавать хоть какие-то признаки жизни. Для этого придется сначала инициализировать соответсвующий порт и пин, куда подключен светодиод. Для этого добавьте просто следующую функцию перед main:

```c
static void gpio_config(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
    return;
}
```

Что здесь происходит в подробностях станет ясно позже, пока лишь можно сказать, что этот код настраивает один вывод в модуле портов ввода-вывода так, чтобы можно было управлять светодиодом. Добавьте вызов функции в main после настройки модуля тактирования.

Чтобы светодиод начал мигать, необходимо в цикле включать и выключать соответствующий вывод, добавьте эту процедуру в бесконечный цикл main'а:

```c
LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_8);
LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_8);
```

Готово! Загрузите программу и ... светодиод просто горит! Почему?

### Написание функции задержки

Вспоминаем, что частота тактирования микроконтроллера 48 МГц, а это значит, что он выполняет примерно 48 миллионов инструкций (не все так просто, но в первом приближении будем считать, что это так). А значит светодиод мигает просто с огромной частотой в нескольок мегагерц, поэтому мерцание незаметно.

Для решения данной проблемы скопируем функцию ниже, которая исполняется ровно 48М тактов, формируя задержку в 1 сек:

```c
__attribute__((naked)) static void delay(void)
{
    asm ("push {r7, lr}");    // Сохраняем регистр-ссылку LR и R7 в стек (чтобы вернуться обратно)
    asm ("ldr r6, [pc, #8]"); // Загружаем число 0x5b8d80 в регистр R6
    asm ("sub r6, #1");       // вычитаем 1
    asm ("cmp r6, #0");       // Проверяем на 0
    asm ("bne delay+0x4");    // Если не 0, то возвращаемся на строчку 3
    asm ("pop {r7, pc}");     // Выгружаем LR и R7 в регистры PC и R7,
                              // тем самым возвращаясь в место вызова функции
    asm (".word 0x5b8d80"); //6000000
#endif
}
```

Данная функция может показаться непростой для понимания, но на самом деле это цикл, который исполняется 6М раз. Каждый проход цикла занимает в среднем 8 тактов:

```c
asm ("sub r6, #1");
asm ("cmp r6, #0");
asm ("bne delay+0x4");
```

Добавьте вызов данной функции между переключениями светодиода и загружаем прошивку. Теперь светодиод переключается каждую секунду. Для сравнения попробуйте убрать вызов инициализации системы тактирования и сравните полученные результаты.

Попробуйте поиграться с константой для задержки и добиться переключения каждые 10 сек. Используйте сторонний таймер для проверки.

### Регулирование задержки обращения к памяти

Согласно документации от производителя на частоте 48 МГц рекомендуется увеличить задержку обращения к FLASH памяти, поэтому в функции rcc_config выставите параметр LL_FLASH_LATENCY_1 вместо LL_FLASH_LATENCY_0.

Загрузите прошивку и засеките время переключения на таймере.
Как видно, оно стало больше, так как микроконтроллер после запроса каждой инструкции пропускает один такт.
Поэтому формально проход цикла в среднем занимает больше не 8 тактов, а 11 (8 + по такту на каждую инструкцию).
Учитывая этот факт, подрегулируйте константу так, чтобы частота мигания стала как была с LL_FLASH_LATENCY_0.

Так зачем же вставлять лишний такт, когда все работает и без него на высокой частоте?

Дело в том, что это режим, на котором производитель гарантирует стабильную работу, любые выходы за пределы не означают неработоспобность, ровно как и работоспособность. Более того, с помощью PLL можно выставить частоту тактирования больше 48МГц, и все будет работать. Для проверки, попробуйте поиграться как с парамерами PLL, так и с задержкой работы FLASH памяти и найти предельные параметры, при которых программа еще исполняется.
