// uart.c
// Author: Alexander Voronin, av1024@gmail.com
// custom interrupt-driven buffered UART implementation

#include <avr/io.h>
#include <avr/interrupt.h>

#include "uart.h"

#ifndef INLINE
 #define INLINE   inline __attribute__((__always_inline__))
#endif
#ifndef NOINLINE
 #define NOINLINE __attribute__((__noinline__))
#endif
#ifndef UNUSED
 #define UNUSED __attribute__((__unused__))
#endif

#define UART_TX_BUF_MASK (UART_TX_BUF_SIZE-1)
#define UART_RX_BUF_MASK (UART_RX_BUF_SIZE-1)

static volatile struct {
    uint8_t in, out, cnt;
    uint8_t buf [UART_TX_BUF_SIZE];
} _tx;

static volatile struct {
    uint8_t in, out, cnt;
    uint8_t buf [UART_RX_BUF_SIZE];
} _rx;

/*
 * Initialize USART speed, mode, interrupts via direct UBRR value
 * Use macro from .h for human-readable speed definitions
 */
void uart_init_b(uint16_t ubrr) {
    // baud rate
    UBRRH = (uint8_t) (ubrr >> 8);
    UBRRL = (uint8_t) ubrr;

    // Clear Double-speed mode
    UCSRA = 0; //(1<<U2X);

    // Enable rx interrupts and tx/rx lines
    UCSRB =   (1<<RXCIE)
            | (1<<TXCIE)
            | (1<<RXEN)
            | (1<<TXEN)
            ;

    // Frame format 8-N-1
    #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega88__)
      UCSRC = (1<<UCSZ00) | (1<<UCSZ01);
    #else
      UCSRC = (1<<URSEL) | (1<<UCSZ00) | (1<<UCSZ01);
    #endif

    // RS485 support
    #ifdef RS485PIN
        RS485PORTD |= (1<<RS485PIN);
        RS485PORT &= ~(1<<RS485PIN);
    #endif
}

/*
 * Put next char into output buffer and start sending
 * If compiled with UART_BLOCK may block until output buffer has room for character
 */
NOINLINE uint8_t uart_putch(uint8_t data) {
    register uint8_t sreg = SREG;

    // Return failure for non-blocking mode
    #ifdef UART_BLOCK
        #warning *** UART IN BLOCKING MODE ***
        while(_tx.cnt >= UART_TX_BUF_SIZE) { ; }
    #else
        if (_tx.cnt >= UART_TX_BUF_SIZE) return 0;
    #endif

    cli();
    #ifdef RS485PORT
        RS485PORT |= (1<<RS485PIN); // switch to TX
    #endif

    if (_tx.cnt) {
        _tx.buf[_tx.in] = data;
        _tx.in = (_tx.in + 1) & UART_TX_BUF_MASK;
    } else {
        UDR = data;
    }
    _tx.cnt++;

    SREG=sreg;
    return 1;
}

/*
 * Read next char from input buffer,
 * Returns 0 if no data available
 */
uint8_t uart_getch(void) {
    register uint8_t ch = 0;
    register uint8_t sreg;
    if (_rx.cnt > 0) {
        sreg = SREG;
        cli();
        ch = _rx.buf[_rx.out];
        _rx.out = (_rx.out + 1) & UART_RX_BUF_MASK;
        _rx.cnt--;
        SREG = sreg;
    }
    return ch;
}

INLINE uint8_t uart_rx_count(void) {
    return _rx.cnt;
}

INLINE uint8_t uart_tx_count(void) {
    return _tx.cnt;
}

void uart_rx_flush(void) {
    UNUSED uint8_t dummy;
    while ( UCSRA & (1<<RXC) ) dummy = UDR;
    _rx.cnt = 0;
}

void uart_tx_flush(void) {
    while (_tx.cnt) ;
    //while ( (UCSRB & (1<<TXCIE)) ) ;
}

NOINLINE uint8_t uart_print(const char *str) {
    while (*str ) {
        if (!uart_putch(*str++)) return 0;
    }
    return 1;
}

uint8_t uart_print_hex(uint8_t b) {
    uint8_t c = b >> 4;
    if (c > 9) c += 7;
    if (!uart_putch(c + '0')) return 0;
    c = b & 0x0F;
    if (c > 9) c += 7;
    return uart_putch(c + '0');
}

uint8_t uart_print_dec(uint8_t b) {
    uint8_t c, x, z;
    c = (b / 100); // x100
    x = b % 100;
    z = x / 10;    // x10
    if (c)
        if (!uart_putch(c + '0')) return 0;     // >= 100
    if (b > 9)
        if (!uart_putch(z + '0')) return 0; // >= 10
    return uart_putch('0' + (x % 10));     // 0..9
}

uint8_t uart_print_bin(uint8_t b) {
    for(uint8_t i=0;i<8;i++) {
        if ( b & (0x80 >> i) ) {
            if (!uart_putch('1')) return 0;
        }
        else
            if (!uart_putch('0')) return 0;
    }
    return 1;
}

uint8_t uart_print_mem(uint8_t *mem, uint8_t count) {
    while(count--)
        if (!uart_print_hex(*mem++)) return 0;
    return 1;
}

uint8_t uart_print_mem_d(uint8_t *mem, uint8_t count, char delim) {
    for(register uint8_t i=0;i<count;i++) {
        if (i)
            if (!uart_putch(delim)) return 0;
        if (!uart_print_hex(*mem++)) return 0;
    }
    return 1;
}

uint8_t uart_print_p(const char *str) {
    uint8_t c;
    do {
        c = pgm_read_byte(str++);
        if (c != 0)
            if (!uart_putch(c)) return 0;
    } while (c);
    return 1;
}

ISR(USART0_TXC_vect) {
    _tx.cnt--;
    if (_tx.cnt) {
        UDR = _tx.buf[_tx.out];
        _tx.out = (_tx.out + 1) & UART_TX_BUF_MASK;
    } else {
#ifdef RS485PIN
    RS485PORT &= ~(1<<RS485PIN);
#endif
    ;
    }
}


ISR(USART0_RXC_vect) {
    uint8_t tmp;
    _rx.buf[_rx.in] = UDR;
    tmp = (_rx.in + 1) &  UART_RX_BUF_MASK;
    if (tmp != _rx.out) {
        _rx.in = tmp;
        _rx.cnt++;
    }
}
