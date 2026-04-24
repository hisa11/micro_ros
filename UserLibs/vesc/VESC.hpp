#ifndef VESC_HPP
#define VESC_HPP

#include "main.h"
#include "can.h"
#include "can_utils.hpp"
#include <algorithm>

/// @brief VESC ESCコントローラークラス
class VESC {
 public:
  /// @brief コンストラクタ (共有CANオブジェクト版)
  /// @param can_ref 共有するCANオブジェクトへの参照
  /// @param vesc_id VESCのID
  VESC(CAN_HandleTypeDef *can_ref, uint8_t vesc_id)
      : can(can_ref),
        vesc_id(vesc_id),
        rpm(0),
        current(0.0f),
        duty(0.0f),
        voltage(0.0f),
        temp_fet(0.0f),
        temp_motor(0.0f),
        current_in(0.0f) {}

  /// @brief 初期化処理
  void init() { printf("VESC (ID:%d) Initialized.\n", vesc_id); }

  /// @brief RPM速度を設定
  /// @param target_rpm 目標RPM
  void set_rpm(int32_t target_rpm) { send_command(COMM_SET_RPM, target_rpm); }

  /// @brief 電流を設定
  /// @param target_current 目標電流 [A]
  void set_current(float target_current) {
    int32_t current_milliamps = (int32_t)(target_current * 1000.0f);
    send_command(COMM_SET_CURRENT, current_milliamps);
  }

  /// @brief デューティサイクルを設定
  /// @param target_duty 目標Duty [0.0-1.0]
  void set_duty(float target_duty) {
    int32_t duty_scaled = (int32_t)(target_duty * 100000.0f);
    send_command(COMM_SET_DUTY, duty_scaled);
  }

  /// @brief ブレーキ電流を設定
  /// @param brake_current ブレーキ電流 [A]
  void set_brake(float brake_current) {
    int32_t brake_milliamps = (int32_t)(brake_current * 1000.0f);
    send_command(COMM_SET_CURRENT_BRAKE, brake_milliamps);
  }

  /// @brief 位置を設定 (サーボモード)
  /// @param target_position 目標位置 [度: 0-360]
  void set_position(float target_position) {
    int32_t position_scaled = (int32_t)(target_position * 1000000.0f);
    send_command(COMM_SET_POS, position_scaled);
  }

  /// @brief VESCからのデータを受信して更新
  void update() {
    CANMessage rxMsg;

    // バッファ内の全メッセージを読み出す
    while (can_read(can, rxMsg)) {
      uint8_t id_vesc = rxMsg.id & 0xFF;
      uint8_t id_cmd = (rxMsg.id >> 8) & 0xFF;

      // 対象のVESCからのステータスパケットを解析
      if (id_vesc == vesc_id) {
        if (id_cmd == CAN_PACKET_STATUS) {
          parse_status(rxMsg);
        } else if (id_cmd == CAN_PACKET_STATUS_2) {
          parse_status2(rxMsg);
        } else if (id_cmd == CAN_PACKET_STATUS_3) {
          parse_status3(rxMsg);
        } else if (id_cmd == CAN_PACKET_STATUS_4) {
          parse_status4(rxMsg);
        }
      }
    }
  }

  /// @brief RPMを取得
  /// @return RPM値
  int32_t read_rpm() const { return rpm; }

  /// @brief 電流を取得
  /// @return 電流 [A]
  int read_current() const { return current; }

  /// @brief デューティサイクルを取得
  /// @return Duty [%]
  int read_duty() const { return duty; }

  /// @brief 電圧を取得
  /// @return 電圧 [V]
  int read_voltage() const { return voltage; }

  /// @brief 温度を取得 (FETおよびモーター)
  /// @return 温度 [℃]
  int read_temp_fet() const { return temp_fet; }

  /// @brief モーター温度を取得
  /// @return 温度 [℃]
  int read_temp_motor() const { return temp_motor; }

  /// @brief モーター電流を取得 (Status4から)
  /// @return 電流 [A]
  float read_current_in() const { return current_in; }

 private:
  CAN_HandleTypeDef *can;
  uint8_t vesc_id;

  // 受信データ
  int32_t rpm;
  float current;
  float duty;
  float voltage;
  float temp_fet;
  float temp_motor;
  float current_in;  // Status4から取得するモーター電流

  // VESCコマンドID
  static const uint8_t COMM_SET_DUTY = 0;
  static const uint8_t COMM_SET_CURRENT = 1;
  static const uint8_t COMM_SET_CURRENT_BRAKE = 2;
  static const uint8_t COMM_SET_RPM = 3;
  static const uint8_t COMM_SET_POS = 4;

  // VESCステータスID
  static const uint8_t CAN_PACKET_STATUS = 9;
  static const uint8_t CAN_PACKET_STATUS_2 = 14;
  static const uint8_t CAN_PACKET_STATUS_3 = 15;
  static const uint8_t CAN_PACKET_STATUS_4 = 16;

  /// @brief コマンドを送信
  /// @param command コマンドID
  /// @param value 値
  void send_command(uint8_t command, int32_t value) {
    uint32_t tx_id = (uint32_t)command << 8 | vesc_id;

    char txData[4];
    txData[0] = (value >> 24) & 0xFF;
    txData[1] = (value >> 16) & 0xFF;
    txData[2] = (value >> 8) & 0xFF;
    txData[3] = (value >> 0) & 0xFF;

    CANMessage txMsg(tx_id, txData, 4, CANMessage::CANData, CANMessage::CANExtended);
    
    // 送信がバスに書き込まれるまで待つ
    while (!can_write(can, txMsg)) {
      HAL_Delay(1);
    }
  }

  /// @brief ステータス1を解析 (RPM, 電流, Duty)
  /// @param msg CANメッセージ
  void parse_status(const CANMessage& msg) {
    if (msg.len < 8) return;

    // RPM (Byte 0-3)
    rpm = (int32_t)((msg.data[0] << 24) | (msg.data[1] << 16) |
                    (msg.data[2] << 8) | msg.data[3]);

    // 電流 (Byte 4-5, スケール: ×10)
    int16_t current_raw = (int16_t)((msg.data[4] << 8) | msg.data[5]);
    current = (float)current_raw / 10.0f;

    // Duty (Byte 6-7, スケール: ×1000)
    int16_t duty_raw = (int16_t)((msg.data[6] << 8) | msg.data[7]);
    duty = (float)duty_raw / 10.0f;  // %表示
  }

  /// @brief ステータス2を解析 (Ah, Ahチャージ)
  /// @param msg CANメッセージ
  void parse_status2(const CANMessage& msg) {
    // 必要に応じて実装
  }

  /// @brief ステータス3を解析 (Wh, Whチャージ)
  /// @param msg CANメッセージ
  void parse_status3(const CANMessage& msg) {
    // 必要に応じて実装
  }

  /// @brief ステータス4を解析 (温度など)
  /// @param msg CANメッセージ
  void parse_status4(const CANMessage& msg) {
    if (msg.len < 8) return;

    // FET温度 (Byte 0-1, スケール: ×10)
    int16_t temp_fet_raw = (int16_t)((msg.data[0] << 8) | msg.data[1]);
    temp_fet = (float)temp_fet_raw / 10.0f;

    // モーター温度 (Byte 2-3, スケール: ×10)
    int16_t temp_motor_raw = (int16_t)((msg.data[2] << 8) | msg.data[3]);
    temp_motor = (float)temp_motor_raw / 10.0f;

    // Current In (入力電流) (Byte 4-5, スケール: ×10)
    int16_t current_in_raw = (int16_t)((msg.data[4] << 8) | msg.data[5]);
    current_in = (float)current_in_raw / 10.0f;
  }
};

#endif  // VESC_HPP
