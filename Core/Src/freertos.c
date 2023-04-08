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
#include "usart.h"
#include "bsp_can.h"
#include "CAN_receive.h"
#include "struct_typedef.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint32_t commond_buffer[24] = {0x000004, 0x000005, 0x000006, 0x000007, 0x000040, 0x000050, 0x000060, 0x000070, 0x000400, 0x000500, 0x000600, 0x000700,
														 0x004000, 0x005000, 0x006000, 0x007000, 0x040000, 0x050000, 0x060000, 0x070000, 0x400000, 0x500000, 0x600000, 0x700000};
uint8_t order_buffer[100] = {0};
uint8_t finish_flag = 0;
uint8_t random_flag = 0;

extern TIM_HandleTypeDef htim2;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId DMA_RXTaskHandle;
osThreadId RED_LEDTask;
osThreadId Order_dectTaskHandle;
osThreadId Random_moveTaskHandle;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void DMA_RX(void const * argument);
void Redled_Toggle(void const * argument);
void Order_dect(void const * argument);
void Random_move(void const * argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	osThreadDef(DMA_RXTask, DMA_RX, osPriorityNormal, 0, 128);
  DMA_RXTaskHandle = osThreadCreate(osThread(DMA_RXTask), NULL);
	
	osThreadDef(RED_LEDTask, Redled_Toggle, osPriorityNormal, 0, 64);
  RED_LEDTask = osThreadCreate(osThread(RED_LEDTask), NULL);
	
	osThreadDef(Order_detectTask, Order_dect, osPriorityNormal, 0, 128);
  Order_dectTaskHandle = osThreadCreate(osThread(Order_detectTask), NULL);
	
	osThreadDef(Random_moveTask, Random_move, osPriorityNormal, 0, 128);
  Random_moveTaskHandle = osThreadCreate(osThread(Random_moveTask), NULL);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
	for(;;)
	{
    osDelay(1);
	}
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void DMA_RX(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Receive loop */
	for(;;)
	{
		if(recv_end_flag ==1)
		{
			//HAL_UART_Transmit_DMA(&huart1,rx_buffer,rx_len);
			rx_len=0;
			recv_end_flag=0;
			HAL_UART_Receive_DMA(&huart1,rx_buffer,BUFFER_SIZE);
			if(rx_buffer[0] == 0x40)
			{
				for(uint8_t i=0; rx_buffer[i] != 0x5A; i++)
				{
					order_buffer[i] = rx_buffer[i];
				}
				finish_flag = 0;
			}
		}
		osDelay(500);
	}
  /* USER CODE END StartDefaultTask */
}

void Order_dect(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	static uint32_t Motor_move = 0;
	static uint16_t order_index = 0;
  /* Toggle loop */
	for(;;)
	{
		if(order_buffer[0] == 0x40 && finish_flag == READY_RESTORE)  						//当指令集的起始位等于@，且完成标志符为0，进入还原进程
		{
			Motor_move = 0;
			printf("receive restore order:'%s'\r\n", order_buffer);
			finish_flag = 2;                           						//运行中
			for(uint8_t i=1; order_buffer[i] != 0; i++)
			{
				order_index = order_buffer[i]-0x41;
				Motor_move = commond_buffer[order_index];
				CAN_cmd_motor(Motor_move, 20);
				osDelay(250);
			}
			finish_flag = 1;																			//完成
			for(uint8_t a=0; a < 50; a++)
			{
				order_buffer[a] = 0;
			}
		}
		
		else if(order_buffer[0] == 0x3F && finish_flag == READY_RESTORE)  			//当指令集的起始位等于？，且完成标志符为0，进入随机打乱进程
		{
			Motor_move = 0;
			printf("receive random order:'%s'\r\n", order_buffer);
			finish_flag = 2;                                			//运行中
			for(uint8_t i=1; order_buffer[i] != '0'; i++)
			{
				order_index = order_buffer[i]-0x41;
				Motor_move = commond_buffer[order_index];
				CAN_cmd_motor(Motor_move, 5);
				osDelay(600);
			}
			finish_flag = 1;																			//完成
			for(uint8_t a=0; a < 50; a++)
				order_buffer[a] = 0;
		}
		osDelay(300);
	}
  /* USER CODE END StartDefaultTask */
}

void Random_move(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Toggle loop */
	static uint32_t rng_val = 0;
	for(;;)
	{
		if(random_flag && finish_flag != RUNING_RESTORE)     								//判断打乱指令，且不在电机运行
		{	
			order_buffer[0] = '?' ;
			order_buffer[21] = 'Z';
			for(uint8_t i=1; i < 21; i++)
			{
			//rng_val = HAL_RNG_GetRandomNumber(&hrng);
			rng_val = rng_val % 24;
			order_buffer[i] = rng_val + 'A';
			//printf("KEY_0->RNG_GetRandomNumber:%lu\r\n",rng_val);
			}
			if(random_flag == 1) random_flag = 0;
		}
		osDelay(500);
	}
  /* USER CODE END StartDefaultTask */
}

void Redled_Toggle(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Toggle loop */
	static uint16_t led_value = 0;
	for(;;)
	{
		while (led_value < 500)
	  {
		  led_value++;
		  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, led_value);    //修改比较值，修改占空比
//		  TIM1->CCR1 = pwm_value;    //与上方作用相同，直接修改寄存器
		  osDelay(10);
	  }
	  while (led_value > 5)
	  {
		  led_value--;
		  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, led_value);    //修改比较值，修改占空比
//		  TIM1->CCR1 = pwmVal;    	 //与上方作用相同，直接修改寄存器
		  osDelay(10);
	  }
		osDelay(100);
	}
  /* USER CODE END StartDefaultTask */
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
