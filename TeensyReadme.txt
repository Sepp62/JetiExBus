Changes in Teensy libraries
===========================

Changes in Teensy HardwareSerial library to automatically enable/disable receiver
---------------------------------------------------------------------------------
// comment 
in: ...\Arduino\hardware\teensy\avr\cores\teensy3\serial2.c
/* 

#ifdef HAS_KINETISK_UART1_FIFO

  #define C2_ENABLE      UART_C2_TE | UART_C2_RE | UART_C2_RIE | UART_C2_ILIE
#else

  #define C2_ENABLE      UART_C2_TE | UART_C2_RE | UART_C2_RIE

#endif

#define C2_TX_ACTIVE     C2_ENABLE | UART_C2_TIE

#define C2_TX_COMPLETING C2_ENABLE | UART_C2_TCIE

#define C2_TX_INACTIVE   C2_ENABLE

*/

// new code:


#ifdef HAS_KINETISK_UART1_FIFO

  #define C2_RXENABLE    UART_C2_RE | UART_C2_RIE | UART_C2_ILIE

#else

  #define C2_RXENABLE    UART_C2_RE | UART_C2_RIE

#endif

#define C2_TX_ACTIVE     UART_C2_TE | UART_C2_TIE

#define C2_TX_COMPLETING UART_C2_TE | UART_C2_TCIE

#define C2_TX_INACTIVE   C2_RXENABLE

