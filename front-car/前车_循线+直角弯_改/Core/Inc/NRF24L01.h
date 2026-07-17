#ifndef __NRF24L01_H
#define __NRF24L01_H

#include "main.h"

#define NRF24L01_PAYLOAD_SIZE       13U
#define NRF24L01_ACK_PAYLOAD_SIZE   9U
#define NRF24L01_ACK_MAGIC          0xA5U

#define FRONT_CAR_FLAG_RIGHT_ANGLE  0x01U
#define FRONT_CAR_FLAG_CORNER_EXIT  0x02U
#define FRONT_CAR_FLAG_WAIT_REAR    0x04U
#define FRONT_CAR_FLAG_CORNER_APPROACH 0x08U
#define FRONT_CAR_CORNER_ID_SHIFT   4U
#define FRONT_CAR_CORNER_ID_MASK    0xF0U

typedef enum
{
    NRF24L01_REAR_STATE_NORMAL = 0,
    NRF24L01_REAR_STATE_APPROACH,
    NRF24L01_REAR_STATE_TURNING,
    NRF24L01_REAR_STATE_CATCHUP,
    NRF24L01_REAR_STATE_DONE
} NRF24L01_RearState;

typedef struct
{
    uint8_t corner_id;
    uint8_t rear_state;
    uint8_t ack_seq;
    uint32_t rear_path_ticks;
} NRF24L01_RearAck;

typedef enum
{
    NRF24L01_TX_OK = 0,
    NRF24L01_TX_MAX_RETRY,
    NRF24L01_TX_TIMEOUT
} NRF24L01_TxResult;

uint8_t NRF24L01_Init(void);
NRF24L01_TxResult NRF24L01_SendCarData(int16_t speed,
                                      int16_t turn,
                                      int16_t yaw_x10,
                                      uint32_t path_ticks,
                                      uint8_t flags);
uint8_t NRF24L01_ReadRegister(uint8_t reg);
uint8_t NRF24L01_LastSequence(void);
uint8_t NRF24L01_GetRearAck(NRF24L01_RearAck *ack, uint32_t max_age_ms);

#endif
