
#include "spi.h"

void spi_put_char(char c) {
    while(SPI1STATbits.SPITBF == 1);
    SPI1BUF = c;
}

void spi_put_string(char* str) {
    int i = 0;
    for(i = 0; str[i] != '\0'; i++) {
        spi_put_char(str[i]);
    }
}

void spi_move_cursor(int row, int column) {
    switch (row) {
        case 0:
            spi_put_char(0x80 + column);
            return;
        case 1:
            spi_put_char(0xC0 + column);
            return;
    }
}

void spi_clear_first_row() {
    spi_move_cursor(FIRST_ROW, 0);
    int i = 0;
    for(i = 0; i < 16; i++) {
        spi_put_char(' ');
    }
}

void spi_clear_second_row() {
    spi_move_cursor(SECOND_ROW, 0);
    int i = 0;
    for(i = 0; i < 16; i++) {
        spi_put_char(' ');
    }
}

