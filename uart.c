#include <avr/io.h>
#include "uart.h"
#include <util/setbaud.h>

void uart_init(void) {
  UBRR0H = UBRRH_VALUE;
  UBRR0L = UBRRL_VALUE;
#if USE_2X
  UCSR0A |= (1 << U2X0);
#else
  UCSR0A &= ~(1 << U2X0);
#endif
  UCSR0B |= (1<<RXEN0)  | (1<<TXEN0);
  UCSR0C |= (1<<UCSZ00) | (1<<UCSZ01);
}

void uart_putc(unsigned char c) {
  while (!(UCSR0A & (1<<UDRE0)));
  UDR0 = c;
}

void uart_puts(char *s) {
  while(*s) {
    uart_putc(*s);
    s++;
  }
}

unsigned char uart_available(void) {
  return (UCSR0A & (1<<RXC0));
}

unsigned char uart_getc(void) {
  return UDR0;
}
