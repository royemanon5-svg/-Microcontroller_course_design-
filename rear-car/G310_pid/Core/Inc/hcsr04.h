#ifndef __HCSR04_H
#define __HCSR04_H

#include "main.h"

#define HCSR04_INVALID_DISTANCE_CM 0U

typedef enum {
    HCSR04_STATUS_OK = 0,
    HCSR04_STATUS_NO_ECHO,
    HCSR04_STATUS_ECHO_STUCK_HIGH
} HCSR04_Status;

void HCSR04_Init(void);
uint16_t HCSR04_ReadDistanceCm(void);
HCSR04_Status HCSR04_LastStatus(void);

#endif
