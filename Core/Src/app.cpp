#include "app.h"
#include "main.h"
#include "can.h"
#include "FreeRTOS.h"
#include "task.h"

// ユーザーライブラリのインクルード
#include "c610.hpp"

// 外から持ってくるもの (lwip等) は C の関数なので extern "C" ブロックなどで囲むのが安全ですが
// stm32側のヘッダは基本的に対応済みです

// ROS 用インクルード (C言語で書かれているので extern "C" で囲む)
extern "C" {
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>
extern void debug_print(const char *msg);
}

// ============== C++ でのグローバルなインスタンス宣言 ==============
C610 dji(&hcan1);  // C++らしくオブジェクトを宣言

// ROSコールバック処理
extern "C" void on_led_subscription_callback(const void *msgin) {
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;

    if (msg->data == 1) {
        debug_print("[microROS] Recive 1\r\n");
        // C++のクラスメソッドを呼び出し
        taskENTER_CRITICAL();
        dji.set_power(1, 4000);
        dji.set_power(2, 4000);
        dji.set_power(3, -4000);
        dji.set_power(4, -4000);
        taskEXIT_CRITICAL();
    } else {
        debug_print("[microROS] Recive 0\r\n");
        taskENTER_CRITICAL();
        dji.set_power(1, 0);
        dji.set_power(2, 0);
        dji.set_power(3, 0);
        dji.set_power(4, 0);
        taskEXIT_CRITICAL();
    }
}

// CAN送信タスクのループ処理
extern "C" void loop_can_task(void) {
    // taskENTER_CRITICAL();
    // ここでまとめて送信
    dji.send_message();
    // taskEXIT_CRITICAL();
}

extern "C" void setup_ros_and_app(void) {
    // CAN1 の開始など、C++で書いた初期化を含めたい場合はここに書く
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        debug_print("[CAN] Start ERROR\r\n");
        // Error_Handler();
    }
    debug_print("[CAN] Started\r\n");
}
