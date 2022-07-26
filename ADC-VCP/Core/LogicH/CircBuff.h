#pragma once

//#ifdef CircBuff_H
//#define CircBuff_H


#include "main.h"

typedef struct queue {
    uint8_t *buffer;
    size_t   buffer_size;
    size_t   head;
    size_t   tail;
    size_t   bytes_avail;
} queue_t ;





 int BuffInit();
 int put(struct queue *q, uint8_t *data, size_t size);
 int get(struct queue *q, uint8_t *data, size_t size);

//#endif
