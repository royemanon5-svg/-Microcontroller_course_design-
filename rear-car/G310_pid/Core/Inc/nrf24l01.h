#ifndef __NRF24L01_H
#define __NRF24L01_H

#include "main.h"

#define NRF24L01_PAYLOAD_SIZE       13U
#define NRF24L01_ACK_PAYLOAD_SIZE   9U
#define NRF24L01_ACK_MAGIC          0xA5U
#define NRF24L01_PACKET_TIMEOUT_MS  300U

#define FRONT_CAR_FLAG_RIGHT_ANGLE  0x01U
#define FRONT_CAR_FLAG_CORNER_EXIT  0x02U
#define FRONT_CAR_FLAG_WAIT_REAR    0x04U
#define FRONT_CAR_FLAG_CORNER_APPROACH 0x08U
#define FRONT_CAR_CORNER_ID_SHIFT   4U
#define FRONT_CAR_CORNER_ID_MASK    0xF0U

typedef enum {
    NRF24L01_REAR_STATE_NORMAL = 0,
    NRF24L01_REAR_STATE_APPROACH,
    NRF24L01_REAR_STATE_TURNING,
    NRF24L01_REAR_STATE_CATCHUP,
    NRF24L01_REAR_STATE_DONE
} NRF24L01_RearState;

typedef struct {
    int16_t speed; //前车速度
    int16_t turn; //前车转向量
    int16_t yaw; //前车航向角
    uint32_t path_ticks; //前车累计编码器路程
    uint8_t flags; //前车状态标志
    uint8_t seq; //包序号
    uint8_t checksum; //校验和
} NRF24L01_Packet;

typedef enum {
    NRF24L01_OK = 0,
    NRF24L01_NO_DATA,
    NRF24L01_CHECKSUM_ERROR,
    NRF24L01_SPI_ERROR
} NRF24L01_Status;

NRF24L01_Status NRF24L01_Init(void);
NRF24L01_Status NRF24L01_ReadPacket(NRF24L01_Packet *packet);
NRF24L01_Status NRF24L01_QueueAckStatus(uint8_t corner_id,
                                       uint8_t rear_state,
                                       uint32_t rear_path_ticks);
uint8_t NRF24L01_IsConnected(uint32_t now_ms);
uint32_t NRF24L01_LastPacketTick(void);
uint8_t NRF24L01_ReadRegister(uint8_t reg);

#endif
