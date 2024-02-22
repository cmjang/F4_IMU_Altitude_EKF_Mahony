/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "imu_temp_control_task.h"
#include "vofa.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId TEMP_IMUHandle;
uint32_t TEMP_IMUBuffer[ 1024 ];
osStaticThreadDef_t TEMP_IMUControlBlock;
osThreadId VofaHandle;
uint32_t VofaBuffer[ 512 ];
osStaticThreadDef_t VofaControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
osThreadId LEDHandle;
/* USER CODE END FunctionPrototypes */

void imu_temp_control_task(void const * argument);
void Vofa_Task(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of TEMP_IMU */
  osThreadStaticDef(TEMP_IMU, imu_temp_control_task, osPriorityRealtime, 0, 1024, TEMP_IMUBuffer, &TEMP_IMUControlBlock);
  TEMP_IMUHandle = osThreadCreate(osThread(TEMP_IMU), NULL);

  /* definition and creation of Vofa */
  osThreadStaticDef(Vofa, Vofa_Task, osPriorityIdle, 0, 512, VofaBuffer, &VofaControlBlock);
  VofaHandle = osThreadCreate(osThread(Vofa), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_imu_temp_control_task */
/**
  * @brief  Function implementing the TEMP_IMU thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_imu_temp_control_task */
__weak void imu_temp_control_task(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN imu_temp_control_task */
		Imu_Init();
  /* Infinite loop */
  for(;;)
  {
		INS_Task();
    osDelay(1);
  }
  /* USER CODE END imu_temp_control_task */
}

/* USER CODE BEGIN Header_Vofa_Task */
/**
* @brief Function implementing the Vofa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Vofa_Task */
void Vofa_Task(void const * argument)
{
  /* USER CODE BEGIN Vofa_Task */
  /* Infinite loop */
  for(;;)
  {
		vofa_start();
    osDelay(2);
  }
  /* USER CODE END Vofa_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
