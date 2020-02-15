/*
 * atmega8_RX.cpp
 *
 * Created: 1/3/2016 4:07:26 PM
 *  Author: lenovo
 */ 

#define EVEN 0
#define ODD 1

#include <math.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

unsigned char RecieveUART(void);
void TransmitUART(uint8_t data);

int main(void)
{
	unsigned char data = 0;
	int baud = 9600;
	
	DDRC = 0xFF;
	PORTC = 0;
	DDRB = 0xFF;
	PORTB = 0;
	
		UCSRA &= ~(1 << U2X);
	
		uint16_t UBRRValue = lrint((F_CPU /(16L * baud)) - 1) ;
	
		//Put the upper part of the baud number here (bits 8 to 11)
		UBRRH = (unsigned char) (UBRRValue >> 8);

		//Put the remaining part of the baud number here
		UBRRL = (unsigned char) UBRRValue;

		//Enable the receiver and transmitter
		UCSRB = (1 << RXEN) | (1 << TXEN);
	
		UCSRC |= (1 << USBS); //Sets 2 stop bits
		UCSRC |= (1 << UPM1); //Sets parity to EVEN
		UCSRC |= (3 << UCSZ0); //Alternative code for 8-bit data length
	
	while(1)
    {
		while (! (UCSRA & (1 << RXC)) );
		if (UDR == 0xFF)
		{
			while (! (UCSRA & (1 << RXC)) );
			if (UDR == 0xEE)
			
			PORTB = 0xFF;
			_delay_ms(700);
			PORTB = 0;
			_delay_ms(700);
			
		}
		
    }
}

unsigned char RecieveUART(void)
{
	while (! (UCSRA & (1 << RXC)) );
	return UDR;
}

void TransmitUART(uint8_t data)
{
	while (! (UCSRA & (1 << UDRE)) );
	UDR = data;
}



