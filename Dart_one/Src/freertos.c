/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "PID.h"
#include "motor.h"
#include "CAN_receive.h"


#define MOTOR_SPEED_PID_KP  20.0f
#define MOTOR_SPEED_PID_KI  0.1f
#define MOTOR_SPEED_PID_KD  0.0f
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
extern dart_control dart;
//can_flag 判断是否通信成功
uint8_t can_flag = 0;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId dart_shootHandle;
osThreadId dart_relodeHandle;
osThreadId dart_tb660Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void dart_shoot_init(void const * argument);
void dart_reload_init(void const * argument);
void dart_tb660_init(void const * argument);

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

  /* definition and creation of dart_shoot */
  osThreadDef(dart_shoot, dart_shoot_init, osPriorityIdle, 0, 128);
  dart_shootHandle = osThreadCreate(osThread(dart_shoot), NULL);

  /* definition and creation of dart_relode */
  osThreadDef(dart_relode, dart_reload_init, osPriorityIdle, 0, 128);
  dart_relodeHandle = osThreadCreate(osThread(dart_relode), NULL);

  /* definition and creation of dart_tb660 */
  osThreadDef(dart_tb660, dart_tb660_init, osPriorityIdle, 0, 128);
  dart_tb660Handle = osThreadCreate(osThread(dart_tb660), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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

/* USER CODE BEGIN Header_dart_shoot_init */
/**
* @brief 驱动电机发射飞镖.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_dart_shoot_init */
void dart_shoot_init(void const * argument)
{
  /* USER CODE BEGIN dart_shoot_init */
  // dart_control dart;
  dart_shoot_reset(&dart);
  //启动飞镖pid初始化
  for(int i=0;i<4;i++)
  {
    pid_init(&dart.motor_speed_pid[i],5,0.01,0,30000,30000);
  }
  // dart_control_init(&dart);
  /* Infinite loop */
  for(;;)
  {
    //遥控器右边s2按键控制飞镖发射
    //s1拨动至0或者1时飞镖停止
    //s1拨动至2时飞镖全速发射
    if(rc_ctrl.rc.s[0]==1) //飞标发射
    {
      //给出速度期望值
      dart.motor[0].target_speed = 10000;
      dart.motor[1].target_speed = 10000;
      dart.motor[2].target_speed = -10000;
      dart.motor[3].target_speed = -10000;
    }
    else 
    {
            //给出速度期望值
      dart.motor[0].target_speed =  0;
      dart.motor[1].target_speed =  0;
      dart.motor[2].target_speed =  0;
      dart.motor[3].target_speed =  0;
    }
          //将期望值带入pid
    for(int i=0;i<4;i++)
    {
    dart.motor[i].give_voltage = pid_calc(&dart.motor_speed_pid[i], 
                                                      dart.motor[i].target_speed, 
                                                      dart.motor[i].motor_measure.speed_rpm);
    }
    //can信号发送至电机
    CAN_cmd_motor(0,
                  dart.motor[0].give_voltage,
                  dart.motor[1].give_voltage,
                  dart.motor[2].give_voltage,
                  dart.motor[3].give_voltage);
    osDelay(1);
  }
  /* USER CODE END dart_shoot_init */
}

/* USER CODE BEGIN Header_dart_reload_init */
/**
* @brief Function 将飞镖送上发射位.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_dart_reload_init */
void dart_reload_init(void const * argument)
{
  /* USER CODE BEGIN dart_reload_init */
  dart_reload_reset(&dart);
  /* Infinite loop */
  for(;;)
  {
    if(rc_ctrl.rc.s[1]==1)//向上装填
    {
      //给出速度期望值
      dart.motor[4].target_speed = 100;
    }
    if(rc_ctrl.rc.s[1]==2)//向下
    {
      //给出速度期望值
      dart.motor[4].target_speed = -100;
    }
    else
    {
      //给出速度期望值
      dart.motor[4].target_speed = 0;
    }
      //将期望值带入pid
    dart.motor[4].give_voltage = pid_calc(&dart.motor_speed_pid[4], 
                                                      dart.motor[4].target_speed, 
                                                      dart.motor[4].motor_measure.speed_rpm);
    
    //can信号发送至电机
    CAN_cmd_motor(1,dart.motor[4].give_voltage,0,0,0);
    osDelay(1);
  }
  /* USER CODE END dart_reload_init */
}

/* USER CODE BEGIN Header_dart_tb660_init */
/**
* @brief 控制tb660步进电机调整飞镖角度
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_dart_tb660_init */
void dart_tb660_init(void const * argument)
{
  /* USER CODE BEGIN dart_tb660_init */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END dart_tb660_init */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
