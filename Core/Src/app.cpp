#include "app.h"
#include "FreeRTOS.h"
#include "can.h"
#include "cmsis_os.h"
#include "main.h"
#include "task.h"

// ユーザーライブラリのインクルード
#include "c610.hpp"
#include "pid.hpp"

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
C610 dji(&hcan1);
// PIDコントローラー (P, I, D, 速度制御モード)
PID dji_pid(1.0, 0.1, 0.00, PID::Mode::VELOCITY);

// ROSコールバック処理
extern "C" void on_led_subscription_callback(const void *msgin) {
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;

  if (msg->data == 1) {
    debug_print("[microROS] Recive 1 -> Set Target RPM: 3000\r\n");
    // モーターに直接パワーを指示するのではなく、PIDの「目標RPM」を設定する
    taskENTER_CRITICAL();
    dji_pid.set_goal(8000); // 例: 3000 RPMを目標にする
    taskEXIT_CRITICAL();
  } else {
    debug_print("[microROS] Recive 0 -> Stop\r\n");
    taskENTER_CRITICAL();
    dji_pid.set_goal(-2000); // 目標を0 RPM（停止）にする
    taskEXIT_CRITICAL();
  }
}

// PID制御ループのタスク
extern "C" void pid_control_loop(void const *argument) {
  // 10ms周期で動くため、dtは0.01秒で固定してしまうのが最も安定します
  const double dt = 0.01;
  dji_pid.set_dt(dt);

  for (;;) {
    // 1. C610から現在のRPMを取得
    int current_rpm = (int)dji.get_rpm(1);

    // 2. PID計算を行い、必要なパワー（-16000 ~ 16000）を算出
    double control_output = dji_pid.do_pid(current_rpm);

    // 3. 計算結果をモーターに適用
    // 1. 文字列を格納するためのバッファ（配列）を用意
    char debug_buf[64];

    // 2. snprintfを使って、数値を埋め込んだ文字列をバッファに作成
    snprintf(debug_buf, sizeof(debug_buf), "MotorSpeed: %d\r\n",
             (int)current_rpm);

    // 3. 完成した文字列をdebug_printに渡す
    debug_print(debug_buf);
    dji.set_power(1, (int)control_output);

    // 10ms待機（これにより正確な100Hzループになる）
    osDelay(10);
  }
}

// CAN送信タスクのループ処理
extern "C" void loop_can_task(void) {
  // C610側でパワーの送信処理
  dji.send_message();
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
