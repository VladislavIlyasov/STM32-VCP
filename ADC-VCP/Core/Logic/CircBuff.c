
 // #include "../LogicH/CircBuff.h"

  #include "../LogicH/CircBuff.h"
//#include "usb_device.h"

//#include "CircBuff.h"

#define max(x, y) ( (x) > (y) ? (x) : (y) )
#define min(x, y) ( (x) < (y) ? (x) : (y) )



struct queue queueIn, queueOut;

int BuffInit(){

	//// Check this f



	queueIn.buffer      = malloc(128);
			queueIn.buffer_size = 128;
			queueIn.head        = 0;
			queueIn.tail        = 0;
			queueIn.bytes_avail = 0;

			queueOut.buffer      = malloc(256);
			queueOut.buffer_size = 256;
			queueOut.head        = 0;
			queueOut.tail        = 0;
			queueOut.bytes_avail = 0;

}


int put(queue_t *q, uint8_t *data, size_t size) {
    if(q->buffer_size - q->bytes_avail < size){
        return 0;
    }
const size_t part1 = min(size,q->buffer_size - q->tail);
const size_t part2 = size - part1;

memcpy(q->buffer + q->tail, data,         part1);
memcpy(q->buffer,           data + part1, part2);

q->tail = (q->tail + size) % q->buffer_size;
q->bytes_avail += size;
return 1;

}



int get(queue_t *q, uint8_t *data, size_t size) {
    if(q->bytes_avail < size){
        return 0;
    }
const size_t part1 = min(size, q->buffer_size - q->head);
const size_t part2 = size - part1;

memcpy(data,         q->buffer + q->head, part1);
memcpy(data + part1, q->buffer,           part2);

q->head = (q->head + size) % q->buffer_size;
q->bytes_avail -= size;
return 1;

}


