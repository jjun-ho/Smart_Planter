/*
 * uart.h
 *
 * Created: 2022-05-10 오후 2:02:04
 *  Author: mics
 */ 


#ifndef UART_H_
#define UART_H_

	void UART1_initialize_polling(void);
	void UART1_initialize_interrupt(void);
	void UART_TX(unsigned char data);
	void UART_RX(void);
	//void UART_TX_string(unsigned char *str);
	void UART_TX_string(char *str);
	void USART_NUM(int nNum);
	unsigned char* DEC_TO_CHAR(long data);


#endif /* UART_H_ */