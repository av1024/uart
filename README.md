Custom implementstion of USART for ATMega.
==========================================

Moved to bitbucket
------------------

https://bitbucket.org/av1024/uart

 - RS485 driving supported
 - Configuration defines for ATMega 88/168/328 but shoul work with other on USART0
 
 Usage:

```
// setup
uart_init(38400); // Initialize as 8-N-1, 38400 via define

// using
uart_print("ABC");
uart_print_p(PSTR("Progmem string"));
// or
uart_pprint("Progmem string");

// finish sending before sleep:
uart_tx_flush();
sleep_cpu();
```
