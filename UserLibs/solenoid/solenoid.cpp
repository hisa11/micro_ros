#include "solenoid.hpp"
#include "main.h"
#include "can.h"
#include "can_utils.hpp"
#include <algorithm>

solenoid::solenoid(CAN_HandleTypeDef *can, int id) : can(can), id(id) {
    for (int i = 0; i < 8; i++) {
        status[i] = false;
    }
}

bool solenoid::sendmessage() {
    // status配列の内容をCANで送信
    uint8_t solenoid_byte = 0;
    for (int i = 0; i < 8; i++) {
            if (status[i]) solenoid_byte |= (1 << i);
        }
    return can_write(can, CANMessage(id, (const char*)&solenoid_byte, 1));
}

void solenoid::data(int id, bool state) {
    // 指定されたIDのステータスを更新
    if (id >= 0 && id < 8) {
        status[id] = state;
    }
}

bool solenoid::getstatus(int id) {
    // 指定されたIDのステータスを取得
    if (id >= 0 && id < 8) {
        return status[id];
    }
    return false; // 無効なIDの場合はfalseを返す
}