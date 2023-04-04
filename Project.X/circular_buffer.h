// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef MY_CIRCULAR_BUFFER_LIB_H
#define MY_CIRCULAR_BUFFER_LIB_H

#include <xc.h> // include processor files - each processor file is guarded.
#include <stdlib.h>

typedef struct circular_buffer {
    char* container;      // the container of the items
    int size;             // the size of the container
    int count;            // number of items in the buffer
    int head;             // index to the first element
    int tail;             // index to the last element
} circular_buffer;

void cb_init(volatile circular_buffer *cb, char *arr, int size);
void cb_free(volatile circular_buffer *cb);
int cb_push_back(volatile circular_buffer *cb, char item);
int cb_push_back_string(volatile circular_buffer *cb, char* string);
int cb_pop_front(volatile circular_buffer *cb, char* item);

#endif
