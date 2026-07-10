#ifndef __NRF24L01_H
#define __NRF24L01_H

#include "main.h"

#define NRF24L01_PAYLOAD_SIZE  8U

typedef enum
{
    NRF24L01_TX_OK = 0,
    NRF24L01_TX_MAX_RETRY,
    NRF24L01_TX_TIMEOUT
} NRF24L01_TxResult;

uint8_t NRF24L01_Init(void);
NRF24L01_TxResult NRF24L01_SendCarData(int16_t speed,
                                      int16_t turn,
                                      int16_t yaw_x10);
uint8_t NRF24L01_ReadRegister(uint8_t reg);
uint8_t NRF24L01_LastSequence(void);

#endif
