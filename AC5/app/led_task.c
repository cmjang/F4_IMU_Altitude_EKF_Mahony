#include "led_task.h"
#include "cmsis_os.h"
#include "main.h"



void led_task(void const * argument)
{
    osDelay(500);

    while(1)
    {
        
        osDelay(100);
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

    }
}


