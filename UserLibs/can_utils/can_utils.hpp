#ifndef CAN_UTILS_HPP
#define CAN_UTILS_HPP

#include "main.h"

struct CANMessage {
    uint32_t id;
    uint8_t data[8];
    uint8_t len;
    
    // mbed compatibility
    enum Format { CANStandard, CANExtended };
    enum Type { CANData, CANRemote };
    
    Format format;
    Type type;
    
    CANMessage() : id(0), len(0), format(CANStandard), type(CANData) {}
    CANMessage(uint32_t id, const uint8_t* data, uint8_t len, Format format = CANStandard, Type type = CANData) 
        : id(id), len(len), format(format), type(type) {
        for(int i=0; i<len; i++) this->data[i] = data[i];
    }
    CANMessage(uint32_t id, const char* data, uint8_t len, Format format = CANStandard, Type type = CANData) 
        : id(id), len(len), format(format), type(type) {
        for(int i=0; i<len; i++) this->data[i] = data[i];
    }
};

inline bool can_write(CAN_HandleTypeDef *hcan, const CANMessage &msg) {
    CAN_TxHeaderTypeDef tx_header;
    tx_header.StdId = (msg.format == CANMessage::CANStandard) ? msg.id : 0;
    tx_header.ExtId = (msg.format == CANMessage::CANExtended) ? msg.id : 0;
    tx_header.IDE = (msg.format == CANMessage::CANStandard) ? CAN_ID_STD : CAN_ID_EXT;
    tx_header.RTR = (msg.type == CANMessage::CANData) ? CAN_RTR_DATA : CAN_RTR_REMOTE;
    tx_header.DLC = msg.len;
    tx_header.TransmitGlobalTime = DISABLE;
    
    uint32_t tx_mailbox;
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0) {
        if (HAL_CAN_AddTxMessage(hcan, &tx_header, (uint8_t*)msg.data, &tx_mailbox) == HAL_OK) {
            return true;
        }
    }
    return false;
}


inline bool can_read(CAN_HandleTypeDef *hcan, CANMessage &msg) {
    CAN_RxHeaderTypeDef rx_header;
    if (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, msg.data) == HAL_OK) {
            msg.id = (rx_header.IDE == CAN_ID_STD) ? rx_header.StdId : rx_header.ExtId;
            msg.len = rx_header.DLC;
            msg.format = (rx_header.IDE == CAN_ID_STD) ? CANMessage::CANStandard : CANMessage::CANExtended;
            msg.type = (rx_header.RTR == CAN_RTR_DATA) ? CANMessage::CANData : CANMessage::CANRemote;
            return true;
        }
    }
    if (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1) > 0) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, msg.data) == HAL_OK) {
            msg.id = (rx_header.IDE == CAN_ID_STD) ? rx_header.StdId : rx_header.ExtId;
            msg.len = rx_header.DLC;
            msg.format = (rx_header.IDE == CAN_ID_STD) ? CANMessage::CANStandard : CANMessage::CANExtended;
            msg.type = (rx_header.RTR == CAN_RTR_DATA) ? CANMessage::CANData : CANMessage::CANRemote;
            return true;
        }
    }
    return false;
}
#endif
