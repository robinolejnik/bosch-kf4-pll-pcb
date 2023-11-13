#ifndef UART_H
#define UART_H

#ifdef __cplusplus
extern "C" {
#endif


  #define BAUD 38400
  
  void uart_init(void);
  void uart_putc(unsigned char c);
  void uart_puts(char *s);
  unsigned char uart_available(void);
  unsigned char uart_getc(void);

/**@}*/
#ifdef __cplusplus
} // extern "C"
#endif

#endif // UART_H 

