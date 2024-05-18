#ifndef _ringbuffer_
#define _ringbuffer_

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct RINGBUFFER
{
    float* Data;  //ring buffer data
    uint16_t Counter;   //counts data in buffer
    uint16_t Head;    
    uint16_t Tail;
    uint16_t Buffer_Size;
}RINGBUFFER;

void Ring_buffer_init(RINGBUFFER* ringbuffer, uint16_t buffer_size);
void Ring_buffer_push(RINGBUFFER* ringbuffer, float data);
void Ring_buffer_pop(RINGBUFFER* ringbuffer, float* data);
uint16_t Ring_buffer_get_capacity(RINGBUFFER* ringbuffer);
void Ring_buffer_clear(RINGBUFFER* ringbuffer);
void buffer_print(RINGBUFFER* ringbuffer);

#endif 