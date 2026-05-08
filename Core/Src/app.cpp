#include "app.h"
#include "FreeRTOS.h"
#include "can.h"
#include "cmsis_os.h"
#include "main.h"
#include "task.h"

// ユーザーライブラリのインクルード
#include "VESC.hpp"
#include "c610.hpp"
#include "pid.hpp"
#include "key.h"

#define VESC_ID 12
int target_rpm = 0;
// VESC vesc(, VESC_ID);

// ROS 用インクルード
extern "C" {
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>
extern void debug_print(const char *msg);
}

// ============== C++ でのグローバルなインスタンス宣言 ==============
// C610 dji(&hcan1);
VESC vesc(&hcan1, VESC_ID);
// PIDコントローラー (P, I, D, 速度制御モード)
PID dji_pid(1.0, 0.1, 0.00, PID::Mode::VELOCITY);

// PID制御ループのタスク
extern "C" void pid_control_loop(void const *argument) {
  // 10ms周期で動くため、dtは0.01秒で固定してしまうのが最も安定します
  const double dt = 0.01;
  dji_pid.set_dt(dt);

  for (;;) {
    // 1. C610から現在のRPMを取得
    // int current_rpm = (int)dji.get_rpm(1);

    // 2. PID計算を行い、必要なパワー（-16000 ~ 16000）を算出
    // double control_output = dji_pid.do_pid(current_rpm);

    // 3. 計算結果をモーターに適用
    // dji.set_power(1, (int)control_output);

    // 10ms待機（これにより正確な100Hzループになる）
    osDelay(10);
  }
}

// CAN送信タスクのループ処理
extern "C" void loop_can_task(void) {
  if(Circle == 1)
  {
    debug_print("[CAN] Circle Pressed\r\n");
  }
  // C610側でパワーの送信処理
  //   dji.send_message();
  vesc.set_rpm(target_rpm);
  //   vesc.update(); // VESCからのステータスを更新
}

extern "C" void setup_ros_and_app(void) {
  // ★重要: PIDの出力上限をC610の仕様（-10000〜10000）に合わせる
  dji_pid.set_output_limits(-10000, 10000);
  CAN_FilterTypeDef filterConfig;
  filterConfig.FilterBank = 0;
  filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  filterConfig.FilterIdHigh = 0x0000;
  filterConfig.FilterIdLow = 0x0000;
  filterConfig.FilterMaskIdHigh = 0x0000;
  filterConfig.FilterMaskIdLow = 0x0000;
  filterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; // 受信データをFIFO0に入れる
  filterConfig.FilterActivation = ENABLE;
  filterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &filterConfig) != HAL_OK) {
    debug_print("[CAN] Filter Config ERROR\r\n");
  }
  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    debug_print("[CAN] Start ERROR\r\n");
  }
  debug_print("[CAN] Started\r\n");
}

extern "C" void on_led_subscription_callback(const void *msgin) {
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;

  // 受信した値をそのまま目標RPMとして設定する
  taskENTER_CRITICAL();
  target_rpm = msg->data;
  taskEXIT_CRITICAL();

  // PCから届いたタイミングでのみ出力し、通信のリズムを確認できるようにする
  char debug_buf[64];
  snprintf(debug_buf, sizeof(debug_buf),
           "[microROS] Received! Target RPM: %d\r\n", target_rpm);
  debug_print(debug_buf);
}
