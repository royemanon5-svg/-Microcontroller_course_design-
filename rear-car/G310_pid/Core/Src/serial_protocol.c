#include "serial_protocol.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#define HEAD 0xAA
#define TAIL 0x55

static uint8_t check_sum(uint8_t* data, uint32_t len)
{
	uint8_t check_sum = 0x00;
	
	for(uint32_t i = 0; i < len; i++){
		check_sum += data[i];
	}
	return check_sum;
}


int32_t packet_is_valid(uint8_t* data, uint32_t len, uint32_t* redundant)
{
		if((data == NULL)||(len == 0)||(redundant == NULL)){
			return -1;
		}
		uint32_t index = 0;
		for(index = 0; index < len; index++){
			if(data[index]==HEAD){
				break;
			}
		}
		*redundant = index;
		
		if((len - index) < 3){
			return -2;
		}
		uint16_t payload_len = 0x00;
		memcpy(&payload_len, &data[index+1], 2);
		if((len - index) < (payload_len +5)){
			return -2;
		}
		if((data[index + 3 + payload_len + 1] != TAIL)||(check_sum(&data[index+1], 2 + payload_len ) != data[index + 3 + payload_len]))
		{
			return -3;
		}
		return 0;
		
}



uint32_t packet_length(uint8_t* data, uint32_t len)
{
	if((data == NULL) || (data[0] != HEAD) || (len < 5)){
		return 0;
	}
	
	uint16_t payload_len = 0;
	memcpy(&payload_len, &data[1], 2);
	
	return (payload_len + 5);
}

int32_t packet_encode(uint8_t* payload, uint32_t len, uint8_t* packet_buff, uint32_t buff_len)
{
	if((payload == NULL)|| (packet_buff == NULL) || ((len + 5) > buff_len)){
		return -1;
	}
	
	uint16_t payload_len = len;
	uint32_t index = 0;
	packet_buff[index++] = HEAD;
	memcpy(&packet_buff[index], &payload_len, 2);
	index += 2;
	
	memcpy(&packet_buff[index], payload, len);
	index += len;
	
	uint8_t check = check_sum(&packet_buff[1], 2 + len);
	packet_buff[index++] = check;
	packet_buff[index++] = TAIL;
	
	return index;
	
}

int32_t packet_decode(uint8_t* data, uint32_t len, uint8_t* payload_buff, uint32_t buff_len)
{
	if((data == NULL)||(data[0] != HEAD)|| (len < 5) || (payload_buff == NULL)){
		return -1;
	}
	uint16_t payload_len = 0x00;
	memcpy(&payload_len, &data[1], 2);
	if((len < (payload_len + 5)) || (buff_len < payload_len)){
		return -1;
	}
	memcpy(payload_buff, &data[3], payload_len);
	return payload_len;
}



















