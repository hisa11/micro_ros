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
#include <rosidl_runtime_c/string_functions.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <uxr/client/transport.h>

extern struct netif gnetif;
void MX_LWIP_Process(void);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void debug_print(const char *msg) { printf("%s", msg); }
void debug_print_error(const char *msg, int rc) {
  printf("Error: %s, rc: %d\r\n", msg, rc);
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
std_msgs__msg__String msg;

/* メモリ管理用のプロトタイプ宣言 */
extern void *microros_allocate(size_t size, void *state);
extern void microros_deallocate(void *pointer, void *state);
extern void *microros_reallocate(void *pointer, size_t size, void *state);
extern void *microros_zero_allocate(size_t number_of_elements,
                                    size_t size_of_element, void *state);

rcl_subscription_t subscriber;
std_msgs__msg__Int32 sub_msg;
rclc_executor_t executor;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[8192];
osStaticThreadDef_t defaultTaskControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void led_subscription_callback(const void *msgin) {
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;

  if (msg->data == 1) {
    // 1を受信したら緑LED(LD1)を点灯
    HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
    debug_print("[microROS] LED ON\r\n");
  } else if (msg->data == 0) {
    // 0を受信したら緑LED(LD1)を消灯
    HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);
    debug_print("[microROS] LED OFF\r\n");
  }
}
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
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 8192,
                    defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

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
void StartDefaultTask(void const *argument) {
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN StartDefaultTask */

  /* ネットワークインターフェースを上げるための待機 */
  /* PHYがリンクアップするまで待つ（通常5-10秒必要） */
  debug_print("[NET] Waiting for Ethernet link (10 seconds)...\r\n");

  /* この待機中も定期的にLWIPプロセスを実行する必要があります */
  uint32_t wait_start = HAL_GetTick();
  uint32_t last_log = wait_start;
  while ((HAL_GetTick() - wait_start) < 10000) {

    /* 1秒毎にログを出す */
    if ((HAL_GetTick() - last_log) >= 1000) {
      char buf[100];
      snprintf(buf, sizeof(buf), "[NET] Waiting... %lu sec (up:%d link:%d)\r\n",
               (HAL_GetTick() - wait_start) / 1000, netif_is_up(&gnetif),
               netif_is_link_up(&gnetif));
      debug_print(buf);
      last_log = HAL_GetTick();
    }

    osDelay(10);
  }

  debug_print("[NET] Ethernet initialization complete\r\n");

  /* socket APIが初期化されるまでさらに待機 */
  debug_print("[NET] Waiting for socket API to be ready (3 seconds)...\r\n");
  uint32_t socket_wait_start = HAL_GetTick();
  while ((HAL_GetTick() - socket_wait_start) < 3000) {
    osDelay(10);
  }
  debug_print("[NET] Socket API initialization complete\r\n");

  /* UDPカスタムトランスポートの設定 */
  debug_print("[microROS] Setting custom UDP transport...\r\n");
  rmw_uros_set_custom_transport(
      false, "192.168.1.5", /* <- PCのAgent IPアドレス（環境に合わせて変更） */
      cubemx_transport_open, cubemx_transport_close, cubemx_transport_write,
      cubemx_transport_read);
  debug_print("[microROS] UDP transport configured\r\n");

  /* カスタムアロケータの設定（FreeRTOSのヒープを使用） */
  debug_print("[microROS] Setting custom allocator...\r\n");
  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate = microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
    debug_print("[ERROR] Failed to set allocator\r\n");
  } else {
    debug_print("[microROS] Custom allocator set successfully\r\n");
  }

  /* micro-ROSのセットアップ開始 */
  debug_print("[microROS] Getting default allocator...\r\n");
  allocator = rcl_get_default_allocator();
  debug_print("[microROS] Initializing ROS2 support...\r\n");

  /* rclcの初期化 */
  rcl_ret_t support_init_ret = rclc_support_init(&support, 0, NULL, &allocator);
  if (support_init_ret != RCL_RET_OK) {
    debug_print_error("rclc_support_init", support_init_ret);
    while (1) {
      osDelay(100);
    }
  }
  debug_print("[microROS] Support initialized\r\n");

  /* ノードの作成 */
  RCCHECK(rclc_node_init_default(&node, "f7_node", "", &support));
  debug_print("[microROS] Node created\r\n");
  RCCHECK(rclc_subscription_init_default(
      &subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "/led_cmd"));
  debug_print("[microROS] Subscriber created\r\n");

  /* Executorの作成 (コールバックを処理するための機構) */
  /* 第3引数の「1」は、管理するハンドルの数(今回はSubscriber 1つ) */
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &sub_msg,
                                         &led_subscription_callback,
                                         ON_NEW_DATA));
  /* Publisherの作成 */
  RCCHECK(rclc_publisher_init_default(
      &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "/from_f767zi"));
  debug_print("[microROS] Publisher created\r\n");

  /* メッセージデータの初期化 */
  // Zennの記事のように1〜20の配列を用意
  uint8_t txbuf[20] = {1,  2,  3,  4,  5,  6,  7,  8,  9,  10,
                       11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
  char str_buf[128] = {0};
  int offset = 0;
  for (int i = 0; i < 20; i++) {
    // 文字列としてカンマ区切りで格納
    offset +=
        snprintf(str_buf + offset, sizeof(str_buf) - offset, "%d,", txbuf[i]);
  }

  rosidl_runtime_c__String__init(&msg.data);
  rosidl_runtime_c__String__assignn(&msg.data, str_buf, strlen(str_buf));
  debug_print("[microROS] Ready - entering main loop\r\n");

  /* Infinite loop */
  for (;;) {
    /* 既存の送信処理（不要なら消しても構いません） */
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

    /* Executorをスピンさせて受信処理を行う (タイムアウト100ms) */
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

    osDelay(10); /* FreeRTOSのタスクを切り替えるための少しの待機 */
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
