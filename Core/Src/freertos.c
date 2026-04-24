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
#include "cmsis_os.h"
#include "main.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwip.h"
#include "usart.h"
#include <lwip/ip_addr.h>
#include <lwip/netif.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>
#include <std_msgs/msg/int32.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <uxr/client/transport.h>

#include "app.h"

extern struct netif gnetif;
void MX_LWIP_Process(void);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void debug_print(const char *msg) { 
  HAL_UART_Transmit(&huart3, (const uint8_t*)msg, strlen(msg), HAL_MAX_DELAY); 
}
void debug_print_error(const char *msg, int rc) {
  char buf[128];
  snprintf(buf, sizeof(buf), "Error: %s, rc: %d\r\n", msg, rc);
  HAL_UART_Transmit(&huart3, (const uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* UDP トランスポート関数の宣言 */
bool cubemx_transport_open(struct uxrCustomTransport *transport);
bool cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport,
                              const uint8_t *buf, size_t len, uint8_t *err);
size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf,
                             size_t len, int timeout, uint8_t *err);

/* エラーハンドリング用のマクロ */
#define RCCHECK(fn)                                                            \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      debug_print_error(#fn, temp_rc);                                         \
      while (1) {                                                              \
        osDelay(100);                                                          \
      }                                                                        \
    }                                                                          \
  }
#define RCSOFTCHECK(fn)                                                        \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      debug_print_error(#fn, temp_rc);                                         \
    }                                                                          \
  }

/* micro-ROS関連の変数 */
rcl_publisher_t publisher;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
std_msgs__msg__Int32 pub_msg;

/* メモリ管理用のプロトタイプ宣言 */
extern void *microros_allocate(size_t size, void *state);
extern void microros_deallocate(void *pointer, void *state);
extern void *microros_reallocate(void *pointer, size_t size, void *state);
extern void *microros_zero_allocate(size_t number_of_elements,
                                    size_t size_of_element, void *state);

rcl_subscription_t subscriber;
std_msgs__msg__Int32 sub_msg;
rclc_executor_t executor;
extern CAN_HandleTypeDef hcan1;
volatile uint8_t shared_can_data[8] = {0, 0, 0, 0,
                                       0, 0, 0, 0}; // 送信データの共有バッファ
osThreadId canTxTaskHandle; // CAN送信タスクのハンドル
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[3000];
osStaticThreadDef_t defaultTaskControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void led_subscription_callback(const void *msgin) {
  on_led_subscription_callback(msgin);
}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const *argument);
void StartCanTxTask(void const *argument);

extern void MX_USB_DEVICE_Init(void);
extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize) {
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
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 3000,
                    defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(canTxTask, StartCanTxTask, osPriorityNormal, 0, 256);
  canTxTaskHandle = osThreadCreate(osThread(canTxTask), NULL);
  /* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument) {
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN StartDefaultTask */
  setup_ros_and_app();

  /* ネットワークインターフェースのリンクアップを待機（リンクしたらすぐ抜ける）
   */
  debug_print("[NET] Waiting for Ethernet link...\r\n");
  while (!netif_is_up(&gnetif) || !netif_is_link_up(&gnetif)) {
    osDelay(10);
  }
  debug_print("[NET] Ethernet ready\r\n");
  osDelay(100); /* socket API等の初期化待ちとしてわずかに待機 */

  /* UDPカスタムトランスポートの設定 */
  rmw_uros_set_custom_transport(
      false, "192.168.1.5", /* <- PCのAgent IPアドレス（環境に合わせて変更） */
      cubemx_transport_open, cubemx_transport_close, cubemx_transport_write,
      cubemx_transport_read);
  debug_print("[microROS] UDP transport configured\r\n");

  /* カスタムアロケータの設定 */
  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate = microros_zero_allocate;
  rcutils_set_default_allocator(&freeRTOS_allocator);

  /* micro-ROSのセットアップ */
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "f7_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
      &subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "/led_cmd"));

  RCCHECK(rclc_publisher_init_default(
      &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "/from_f767zi"));

  /* Executorの作成 */
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &sub_msg,
                                         &led_subscription_callback,
                                         ON_NEW_DATA));

  pub_msg.data = 0;
  debug_print("[microROS] Ready and running\r\n");

  /* Infinite loop */
  for (;;) {
    pub_msg.data++;
    RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));

    /* Executorをスピンさせて受信処理を行う (タイムアウト100ms) */
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

    osDelay(10); /* FreeRTOSのタスクを切り替えるための少しの待機 */
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void StartCanTxTask(void const *argument) {
  for (;;) {
    /* C610側でCAN送信用の処理を行う */
    loop_can_task();

    /* 20ms待機 (他のタスクに処理を譲る) */
    osDelay(20);
  }
}
/* USER CODE END Application */
