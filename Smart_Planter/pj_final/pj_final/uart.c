/*
 * uart.c
 *
 * Created: 2022-05-10 오후 2:03:48
 *  Author: mics
 */ 

#include <avr/io.h>
static unsigned char save_data[15] = {0};
	
	
void UART1_initialize_polling(void)
{
	UCSR1A = 0x00;
	UCSR1B = (1<<RXEN1)|(1<<TXEN1);
	UCSR1C = (1<<UCSZ11)|(1<<UCSZ10);
	//비동기, parity 0, 8bit, 1stopbit
	
	//baud rate
	UBRR1H = 0x00;
	UBRR1L = 103;
	
}


void UART1_initialize_interrupt(void)
{
	UCSR1A = 0x00;
	UCSR1B = (1<<RXCIE1)|(1<<RXEN1)|(1<<TXEN1);
	UCSR1C = (1<<UCSZ11)|(1<<UCSZ10);
	//비동기, parity 0, 8bit, 1stopbit
	
	//baud rate
	UBRR1H = 0x00;
	UBRR1L = 103;
	
}


void UART_TX(unsigned char data)
{
	while(!(UCSR1A & (1<<UDRE1)));
	UDR1 = data;
}

unsigned char UART_RX(void)
{
	while(!(UCSR1A & (1<<RXC1)));
	return UDR1;
}

void UART_TX_string(char *str)
{
	for(int i = 0 ; str[i] ; i++)
	{
		UART_TX(str[i]);
	}
	//while(*str)
	//{
		//while(!(UCSR1A & (1<<UDRE1)));
		//UDR1 = *str;
		//str++;
	//}
}

void USART_NUM(int nNum)
{
	UART_TX(nNum / 10000 + 48);
	UART_TX((nNum % 10000) / 1000 + 48);
	UART_TX((nNum % 1000) / 100 + 48);
	UART_TX((nNum % 100) / 10 + 48);
	UART_TX((nNum % 10) + 48);
}




unsigned char* DEC_TO_CHAR(long data)
{
	unsigned char i, j, temp;
	
	for(int k = 0 ; k < 15 ; k++)
	{
		save_data[k] = 0;
	}
	
	if(data > 0)
	{
		save_data[0] = '+';
		
		for(i=1 ; data!=0 ; i++)
		{
			save_data[i] = (data % 10) + '0';
			data /= 10;
			if(data == 0)	
			{
				break;
			}
		}
	}
	else if(data < 0)
	{
		data *= -1;
		save_data[0] = '-';
		for(i = 1 ; data!=0 ; i++)
		{
			save_data[i] = (data % 10) + '0';
			data /= 10;
			if(data == 0)
			{
				break;
			}
		}
	}
	else if(data == 0)
	{
		save_data[0] = '+';
		save_data[1] = '0';
		return save_data;
	}
	
	i++;
	for(j=1 ; j<=(i/2) ; j++)
	{
		temp = save_data[j];
		save_data[j] = save_data[i-j];
		save_data[i-j] = temp;
	}
	
	return save_data;
}