/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdbool.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <rosidl_runtime_c/string_functions.h>
#include "usart.h"
#include "stm32f4xx_hal.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

typedef struct {
	int32_t rpm[3];
}MotorTargetRPM_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//micro-ROS関連グローバル変数
rcl_publisher_t encoder_rpm_publisher;
rcl_subscription_t pwm_control_subscriber;
std_msgs__msg__Int32MultiArray encoder_rpm_msg; //encder_rpmトピック
std_msgs__msg__Int32MultiArray pwm_control_msg; //pwm_controlトピック
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

// エンコーダ関連
volatile int32_t encoder_counts[3] = {0, 0, 0}; // 現在のエンコーダカウント
volatile int32_t prev_encoder_counts[3] = {0, 0, 0}; // 前回のエンコーダカウント
volatile uint32_t last_rpm_calc_time[3] = {0, 0, 0}; // RPM計算時の最終時刻
const float RPM_CALC_INTERVAL_S = 0.1f; // RPM計算周期 (秒) - 100ms
const int RPM_CALC_INTERVAL_MS = 100;   // RPM計算周期 (ミリ秒)

// PWM制御関連
const uint32_t PWM_MAX_DUTY = 1000; // PWMタイマーのARR値 (例:1000) - CubeMX設定と一致させる

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId_t motorControlTaskHandle;
uint32_t motorControlTaskBuffer[3000];
osStaticThreadDef_t motorControlTaskControlBlock;
const osThreadAttr_t motorControlTask_attributes = {
  .name = "motorControlTask",
  .cb_mem = &motorControlTaskControlBlock,
  .cb_size = sizeof(motorControlTaskControlBlock),
  .stack_mem = &motorControlTaskBuffer[0],
  .stack_size = sizeof(motorControlTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t encoderTaskHandle;
uint32_t encoderTaskBuffer[3000];
osStaticThreadDef_t encoderTaskControlBlock;
const osThreadAttr_t encoderTask_attributes = {
  .name = "encoderTask",
  .cb_mem = &encoderTaskControlBlock,
  .cb_size = sizeof(encoderTaskControlBlock),
  .stack_mem = &encoderTaskBuffer[0],
  .stack_size = sizeof(encoderTaskBuffer),
  .priority = (osPriority_t) osPriorityBelowNormal, // micro-ROSより少し低くても良い
};

osMessageQueueId_t motorTargetRPMQueueHandle;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 3000 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

void StartMotorControlTask(void *argument);
void StartEncoderTask(void *argument);

// PWM制御コールバック関数
void pwm_control_callback(const void *msgin);
// Helper function to set motor speed and direction
void set_motor_speed(uint8_t motor_index, int32_t rpm);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

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
	motorTargetRPMQueueHandle = osMessageQueueNew(5, sizeof(MotorTargetRPM_t), NULL);
	  if (motorTargetRPMQueueHandle == NULL) {
	      printf("Failed to create motorTargetRPMQueueHandle\n");
	      while(1);
	  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  motorControlTaskHandle = osThreadNew(StartMotorControlTask, NULL, &motorControlTask_attributes);
    encoderTaskHandle = osThreadNew(StartEncoderTask, NULL, &encoderTask_attributes);
    if (motorControlTaskHandle == NULL || encoderTaskHandle == NULL) {
  	  printf("Failed to create tasks\n");
  	  while(1);
    }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	// micro-ROSの設定
	  rmw_uros_set_custom_transport(
	    true,
		(void *) &huart4,
		cubemx_transport_open,
		cubemx_transport_close,
		cubemx_transport_write,
		cubemx_transport_read
	  );

	  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	  freeRTOS_allocator.allocate = microros_allocate;
	  freeRTOS_allocator.deallocate = microros_deallocate;
	  freeRTOS_allocator.reallocate = microros_reallocate;
	  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

	  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
	    printf("Error on default allocators (line %d)\n", __LINE__);
	  }

	  // micro-ROSのセットアップ
	  allocator = rcl_get_default_allocator();
	  node = rcl_get_zero_initialized_node();
	  //初期化設定の作成
	  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
	  //ノードの作成
	  RCCHECK(rclc_node_init_default(&node, "f446_omuni_node", "", &support));
	  //encoder_rpm publisherの作成
	  RCCHECK(rclc_publisher_init_default(
	    &encoder_rpm_publisher,
	    &node,
	    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
	    "/encoder_rpm"));

	  //pwm_control subscriberの作成
	  RCCHECK(rclc_subscription_init_default(
		  &pwm_control_subscriber,
		  &node,
		  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
		  "/pwm_control"));

	  // Initialize messages
	    // /encoder_rpm message
	    static int32_t encoder_rpm_data_buffer[3]; // Ensure buffer is valid for message lifetime
	    encoder_rpm_msg.data.data = encoder_rpm_data_buffer;
	    encoder_rpm_msg.data.size = 0; // Will be set to 3 before publishing
	    encoder_rpm_msg.data.capacity = 3;
	    //std_msgs__msg__Int32MultiArray__init(&encoder_rpm_msg); // More robust initialization if available

	    // /pwm_control message
	    static int32_t pwm_control_data_buffer[3]; // Buffer for incoming data
	    pwm_control_msg.data.data = pwm_control_data_buffer;
	    pwm_control_msg.data.size = 0;
	    pwm_control_msg.data.capacity = 3;
	    //std_msgs__msg__Int32MultiArray__init(&pwm_control_msg);

	    // create executor
	    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator)); // 1 handle for subscriber
	    RCCHECK(rclc_executor_add_subscription(&executor, &pwm_control_subscriber, &pwm_control_msg, &pwm_control_callback, ON_NEW_DATA));

	    printf("micro-ROS DefaultTask initialized.\n");
  for(;;)
  {
	  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)); // 10ms timeout
	  osDelay(10); // Yield for other tasks
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void StartMotorControlTask(void *argument)
{
  /* USER CODE BEGIN StartMotorControlTask */
  MotorTargetRPM_t received_rpms;
  printf("MotorControlTask started.\n");

  // Start PWM for all motors
  HAL_TIM_PWM_Start(MD1_PWM_TIMER, MD1_PWM_CHANNEL);
  HAL_TIM_PWM_Start(MD2_PWM_TIMER, MD2_PWM_CHANNEL);
  HAL_TIM_PWM_Start(MD3_PWM_TIMER, MD3_PWM_CHANNEL);

  /* Infinite loop */
  for(;;)
  {
    if (osMessageQueueGet(motorTargetRPMQueueHandle, &received_rpms, NULL, osWaitForever) == osOK) {
        printf("MotorControlTask: Received RPMs: M1=%ld, M2=%ld, M3=%ld\n", received_rpms.rpm[0], received_rpms.rpm[1], received_rpms.rpm[2]);
        set_motor_speed(0, received_rpms.rpm[0]); // Motor 1
        set_motor_speed(1, received_rpms.rpm[1]); // Motor 2
        set_motor_speed(2, received_rpms.rpm[2]); // Motor 3
    }
  }
  /* USER CODE END StartMotorControlTask */
}
void StartEncoderTask(void *argument)
{
  /* USER CODE BEGIN StartEncoderTask */
  printf("EncoderTask started.\n");

  // Start encoder timers
  HAL_TIM_Encoder_Start(ENC1_TIMER, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(ENC2_TIMER, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(ENC3_TIMER, TIM_CHANNEL_ALL);

  // Initialize last calculation time
  for (int i=0; i<3; ++i) {
      last_rpm_calc_time[i] = HAL_GetTick();
      prev_encoder_counts[i] = 0; // Or read initial counter value
      if (i==0) prev_encoder_counts[i] = __HAL_TIM_GET_COUNTER(ENC1_TIMER);
      if (i==1) prev_encoder_counts[i] = __HAL_TIM_GET_COUNTER(ENC2_TIMER);
      if (i==2) prev_encoder_counts[i] = __HAL_TIM_GET_COUNTER(ENC3_TIMER);
  }


  /* Infinite loop */
  for(;;)
  {
    uint32_t current_time = HAL_GetTick();
    bool publish_now = false;

    // RPM Calculation (example for one encoder, repeat for all 3)
    // For simplicity, assume all calculations happen close enough in time
    // A more robust way would be per-encoder timing.
    if ( (current_time - last_rpm_calc_time[0]) >= RPM_CALC_INTERVAL_MS ) { // Check for encoder 0, assume others are similar
        publish_now = true;
        for(int i=0; i<3; ++i) {
            TIM_HandleTypeDef* current_enc_timer;
            if (i==0) current_enc_timer = ENC1_TIMER;
            else if (i==1) current_enc_timer = ENC2_TIMER;
            else current_enc_timer = ENC3_TIMER;

            int32_t current_raw_count = (int16_t)__HAL_TIM_GET_COUNTER(current_enc_timer); // Read as signed 16-bit
            int32_t count_diff = current_raw_count - (int16_t)(prev_encoder_counts[i] & 0xFFFF); // Handle wrap-around for 16-bit timer

            if (count_diff > (PULSES_PER_REV / 2) && count_diff > 0) { // Heuristic for underflow with 16-bit counter
                 count_diff -= 65536; // Timer counted down past 0
            } else if (count_diff < -(PULSES_PER_REV / 2) && count_diff < 0) { // Heuristic for overflow
                 count_diff += 65536; // Timer counted up past 65535
            }

            uint32_t time_diff_ms = current_time - last_rpm_calc_time[i];
            if (time_diff_ms == 0) time_diff_ms = 1; // Avoid division by zero

            // RPM = (pulses / pulses_per_rev) / (time_ms / 1000 / 60)
            // RPM = (pulses * 60000) / (pulses_per_rev * time_ms)
            encoder_rpm_msg.data.data[i] = (int32_t)(( (float)count_diff * 60000.0f) / ( (float)PULSES_PER_REV * (float)time_diff_ms ));

            prev_encoder_counts[i] = current_raw_count;
            last_rpm_calc_time[i] = current_time;
        }
    }

    if (publish_now) {
        encoder_rpm_msg.data.size = 3;
        RCSOFTCHECK(rcl_publish(&encoder_rpm_publisher, &encoder_rpm_msg, NULL));
        printf("EncoderTask: Published RPMs: M1=%ld, M2=%ld, M3=%ld\n", encoder_rpm_msg.data.data[0], encoder_rpm_msg.data.data[1], encoder_rpm_msg.data.data[2]);
    }
    osDelay(10);
  }
  /* USER CODE END StartEncoderTask */
}

/* USER CODE END Application */

