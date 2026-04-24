#ifndef SOLENOID_HPP
#define SOLENOID_HPP
#include "main.h"
#include "can.h"
#include "can_utils.hpp"
#include <algorithm>

class solenoid {
    public:
        solenoid(CAN_HandleTypeDef *can, int id);
        bool sendmessage(); // メンバ関数に修正
        void data(int id,bool state);
        bool getstatus(int id);

        bool status[8];
        CAN_HandleTypeDef *can; // CAN参照をメンバ変数として保持
        int id;



};

#endif