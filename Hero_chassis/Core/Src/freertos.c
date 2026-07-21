/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "FreeRTOS.h"
#include "queue.h"
#include "remote_task.hpp"
#include "chassis_task.hpp"
#include "can_send_task.hpp"
#include "usart.h"

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
/* Definitions for start_name */
osThreadId_t start_nameHandle;
const osThreadAttr_t start_name_attributes = {
  .name = "start_name",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
BaseType_t start_remote_control;
BaseType_t start_can_send;
BaseType_t start_chassis;

TaskHandle_t xRemoteHandle;
TaskHandle_t xCanSendHandle;
TaskHandle_t xGimbalHandle;




/* USER CODE END FunctionPrototypes */

void start(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* creation of start_name */
 // start_nameHandle = osThreadNew(start, NULL, &start_name_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	
	start_can_send = xTaskCreate(can_send_task, "CAN_Send_Task", 1024, NULL, osPriorityAboveNormal, &xCanSendHandle);
  start_chassis = xTaskCreate(chassis_task, "Chassis_Task", 256, NULL, osPriorityAboveNormal, &xGimbalHandle);
  start_remote_control = xTaskCreate(remote_task, "Remote_Control_Task", 256, NULL, osPriorityAboveNormal, &xRemoteHandle);
	
	
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_start */
/**
  * @brief  Function implementing the start_name thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_start */
void start(void *argument)
{
  /* USER CODE BEGIN start */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END start */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

