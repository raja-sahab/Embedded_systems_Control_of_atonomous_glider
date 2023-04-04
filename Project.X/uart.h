// This is a guard condition so that contents of this file are not included
// more than once.
#ifndef MY_UART_LIB_H
#define	MY_UART_LIB_H

#include "circular_buffer.h"
#include <xc.h> // include processor files - each processor file is guarded.

void uart_init(int baudrate, volatile circular_buffer *in_buffer, volatile circular_buffer *out_buffer);
void uart_main_loop();
void uart_send(char* message);

#endif