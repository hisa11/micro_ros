#include "FP.hpp"

bool FP::send()
{
    return can_write(can, CANMessage{send_id, reinterpret_cast<const uint8_t *>(pwm), 8});
}

void FP::read(const CANMessage &msg)
{
    if (msg.format == CANMessage::CANStandard && msg.type == CANMessage::CANData && msg.len == sizeof(receive[0]) && send_id < msg.id && msg.id <= send_id + 5)
    {
        receive[msg.id - send_id - 1].set(msg.data);
    }
}