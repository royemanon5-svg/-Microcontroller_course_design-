#ifndef __RBUFFER_H
#define __RBUFFER_H

#include "stm32f1xx_hal.h" 
#include <stdint.h>


typedef struct
{
    uint8_t *buffer;      
    uint32_t buffer_size; 
    uint32_t read_index;  
    uint32_t write_index; 
} rbuffer_t;


typedef enum
{
    RB_EMPTY,
    RB_FULL,
    RB_HALFFULL, 
} rbuffer_state_t;


#define rbuffer_space_len(rb) ((rb)->buffer_size - rbuffer_data_len(rb))

void rbuffer_init(rbuffer_t *rb, uint8_t *pool, int32_t size);
void rbuffer_reset(rbuffer_t *rb);

uint32_t rbuffer_put(rbuffer_t *rb, const uint8_t *ptr, uint32_t length);
uint32_t rbuffer_put_force(rbuffer_t *rb, const uint8_t *ptr, uint32_t length);
uint32_t rbuffer_putchar(rbuffer_t *rb, const uint8_t ch);
uint32_t rbuffer_putchar_force(rbuffer_t *rb, const uint8_t ch);

uint32_t rbuffer_del(rbuffer_t *rb, uint32_t length);
uint32_t rbuffer_get(rbuffer_t *rb, uint8_t *ptr, uint32_t length);
uint32_t rbuffer_peek(rbuffer_t *rb, uint8_t *ptr, uint32_t length);
uint32_t rbuffer_getchar(rbuffer_t *rb, uint8_t *ch);
uint32_t rbuffer_data_len(rbuffer_t *rb);

rbuffer_state_t rbuffer_status(rbuffer_t *rb);

#ifdef __cplusplus
}
#endif

#endif
