#include "rbuffer.h"

void rbuffer_init(rbuffer_t *rb, uint8_t *pool, int32_t size)
{
    rb->buffer = pool;
    rb->buffer_size = size;
    rb->read_index = 0;
    rb->write_index = 0;
}

void rbuffer_reset(rbuffer_t *rb)
{
    rb->read_index = 0;
    rb->write_index = 0;
}

uint32_t rbuffer_data_len(rbuffer_t *rb)
{
    if (rb->write_index >= rb->read_index)
    {
        return rb->write_index - rb->read_index;
    }
    else
    {
        return rb->buffer_size - (rb->read_index - rb->write_index);
    }
}

uint32_t rbuffer_put(rbuffer_t *rb, const uint8_t *ptr, uint32_t length)
{
    uint32_t i;
    uint32_t space_len = rbuffer_space_len(rb);
    if (length > space_len)
    {
        length = space_len;
    }

    for (i = 0; i < length; i++)
    {
        rb->buffer[rb->write_index] = ptr[i];
        rb->write_index = (rb->write_index + 1) % rb->buffer_size;
    }
    return i;
}

uint32_t rbuffer_put_force(rbuffer_t *rb, const uint8_t *ptr, uint32_t length)
{
    uint32_t i;
    for (i = 0; i < length; i++)
    {
        if (rbuffer_space_len(rb) == 0)
        {
            rb->read_index = (rb->read_index + 1) % rb->buffer_size;
        }
        rb->buffer[rb->write_index] = ptr[i];
        rb->write_index = (rb->write_index + 1) % rb->buffer_size;
    }
    return i;
}

uint32_t rbuffer_putchar(rbuffer_t *rb, const uint8_t ch)
{
    if (rbuffer_space_len(rb) == 0)
    {
        return 0; 
    }
    rb->buffer[rb->write_index] = ch;
    rb->write_index = (rb->write_index + 1) % rb->buffer_size;
    return 1;
}

uint32_t rbuffer_putchar_force(rbuffer_t *rb, const uint8_t ch)
{
    if (rbuffer_space_len(rb) == 0)
    {
        rb->read_index = (rb->read_index + 1) % rb->buffer_size;
    }
    rb->buffer[rb->write_index] = ch;
    rb->write_index = (rb->write_index + 1) % rb->buffer_size;
    return 1;
}

uint32_t rbuffer_del(rbuffer_t *rb, uint32_t length)
{
    uint32_t data_len = rbuffer_data_len(rb);
    if (length > data_len)
    {
        length = data_len;
    }
    rb->read_index = (rb->read_index + length) % rb->buffer_size;
    return length;
}

uint32_t rbuffer_get(rbuffer_t *rb, uint8_t *ptr, uint32_t length)
{
    uint32_t i;
    uint32_t data_len = rbuffer_data_len(rb);
    if (length > data_len)
    {
        length = data_len;
    }

    for (i = 0; i < length; i++)
    {
        ptr[i] = rb->buffer[rb->read_index];
        rb->read_index = (rb->read_index + 1) % rb->buffer_size;
    }
    return i;
}

uint32_t rbuffer_peek(rbuffer_t *rb, uint8_t *ptr, uint32_t length)
{
    uint32_t i;
    uint32_t temp_index = rb->read_index;
    uint32_t data_len = rbuffer_data_len(rb);
    if (length > data_len)
    {
        length = data_len;
    }

    for (i = 0; i < length; i++)
    {
        ptr[i] = rb->buffer[temp_index];
        temp_index = (temp_index + 1) % rb->buffer_size;
    }
    return i;
}

uint32_t rbuffer_getchar(rbuffer_t *rb, uint8_t *ch)
{
    if (rbuffer_data_len(rb) == 0)
    {
        return 0; 
    }
    *ch = rb->buffer[rb->read_index];
    rb->read_index = (rb->read_index + 1) % rb->buffer_size;
    return 1;
}

rbuffer_state_t rbuffer_status(rbuffer_t *rb)
{
    if (rbuffer_data_len(rb) == 0)
    {
        return RB_EMPTY;
    }
    else if (rbuffer_space_len(rb) == 0)
    {
        return RB_FULL;
    }
    else
    {
        return RB_HALFFULL;
    }
}
