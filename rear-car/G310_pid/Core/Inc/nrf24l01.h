#ifndef __NRF24L01_H
#define __NRF24L01_H

#include "main.h"

#define NRF24L01_PAYLOAD_SIZE      8U
#define NRF24L01_PACKET_TIMEOUT_MS 300U

typedef struct {
    int16_t speed; //前车速度
    int16_t turn; //前车转向量
    int16_t yaw; //前车航向角
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
uint8_t NRF24L01_IsConnected(uint32_t now_ms);
uint32_t NRF24L01_LastPacketTick(void);

#endif
