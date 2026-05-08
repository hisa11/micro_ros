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
#include <std_msgs/msg/string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <uxr/client/transport.h>

#include "app.h"
#include "key.h"

extern struct netif gnetif;
extern UART_HandleTypeDef huart3;
void MX_LWIP_Process(void);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void debug_print(const char *msg) {
  HAL_UART_Transmit(&huart3, (const uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}
void debug_print_error(const char *msg, int rc) {
  char buf[128];
  snprintf(buf, sizeof(buf), "Error: %s, rc: %d\r\n", msg, rc);
  HAL_UART_Transmit(&huart3, (const uint8_t *)buf, strlen(buf), HAL_MAX_DELAY);
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

rcl_subscription_t button_subscription;
rcl_subscription_t sticks_subscription;

std_msgs__msg__String button_subscription_msg;
std_msgs__msg__String sticks_subscription_msg;

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

// C互換の関数ラッパー
extern void key_puress_c(const char *msg);
extern void stick_parse(const char *msg);

void button_subscription_callback(const void *msgin) {
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;

  // メッセージサイズをチェック
  if (msg == NULL || msg->data.size == 0) {
    return;
  }

  // バッファをコピーして null 終端を保証
  static char temp_buffer[256];
  size_t copy_size = (msg->data.size < sizeof(temp_buffer) - 1)
                         ? msg->data.size
                         : sizeof(temp_buffer) - 1;
  memcpy(temp_buffer, msg->data.data, copy_size);
  temp_buffer[copy_size] = '\0'; // null 終端を追加

  // printf("[button] received: %s\r\n", temp_buffer);
  fflush(stdout);

  key_puress_c((const char *)temp_buffer);
}

void sticks_subscription_callback(const void *msgin) {
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;

  // メッセージサイズをチェック
  if (msg == NULL || msg->data.size == 0) {
    return;
  }

  // バッファをコピーして null 終端を保証
  static char temp_buffer[256];
  size_t copy_size = (msg->data.size < sizeof(temp_buffer) - 1)
                         ? msg->data.size
                         : sizeof(temp_buffer) - 1;
  memcpy(temp_buffer, msg->data.data, copy_size);
  temp_buffer[copy_size] = '\0'; // null 終端を追加

  // // printf("[sticks] received: %s\r\n", temp_buffer);
  fflush(stdout);

  stick_parse((const char *)temp_buffer);
}

void StartCanTxTask(void const *argument);
void StartPIDTask(void const *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const *argument);

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
  osThreadDef(pidTask, StartPIDTask, osPriorityNormal, 0, 256);
  osThreadCreate(osThread(pidTask), NULL);
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

  rcl_ret_t ret = rcutils_set_default_allocator(&freeRTOS_allocator);
  if (ret != RCL_RET_OK) {
    debug_print_error("rcutils_set_default_allocator", ret);
  }

  /* micro-ROSのセットアップ */
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "f7_node", "", &support));

  RCCHECK(rclc_subscription_init_best_effort(
      &button_subscription, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "/controller/buttons"));

  RCCHECK(rclc_subscription_init_best_effort(
      &sticks_subscription, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "/controller/sticks"));

  /* String型メッセージの受信バッファを初期化 */
  static char button_buffer[256];
  button_subscription_msg.data.data = button_buffer;
  button_subscription_msg.data.capacity = sizeof(button_buffer);

  static char sticks_buffer[256];
  sticks_subscription_msg.data.data = sticks_buffer;
  sticks_subscription_msg.data.capacity = sizeof(sticks_buffer);

  /* Executorの作成 */
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &button_subscription, &button_subscription_msg,
      &button_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &sticks_subscription, &sticks_subscription_msg,
      &sticks_subscription_callback, ON_NEW_DATA));

  pub_msg.data = 0;
  debug_print("[microROS] Ready and running\r\n");

  /* Infinite loop */

  for (;;) {
    // /* * Executorをスピンさせる (タイムアウト 1000ms)
    //  * PCからデータが来るまで、最大1秒間ここでタスクが「待機（ブロック）」します。
    //  * データが来た場合は、1秒待たずにすぐ抜けてコールバックが実行されます。
    //  */
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1000));

    // /* 現在の時刻を取得 */
    // uint32_t current_time = HAL_GetTick();

    // /* 前回送信時から1000ms (1秒) 以上経過していたら送信する */
    // if (current_time - last_send_time >= 1000) {
    //   pub_msg.data++;
    //   RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
    //   last_send_time = current_time; // 送信時刻を更新
    // }

    // /* * osDelay(10)
    // は外しても構いません（spin_some内でブロックされるため）。
    //  * 他のFreeRTOSタスクに確実に処理を譲りたい場合は、osDelay(1)
    //  程度残しても良いです。
    //  */
    // osDelay(1);
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
void StartPIDTask(void const *argument) { pid_control_loop(argument); }
/* USER CODE END Application */
