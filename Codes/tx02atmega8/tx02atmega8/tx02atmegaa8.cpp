/*
 * remoteATMEGA8for_atmega328.cpp
 *
 * Created: 1/4/2016 1:17:19 PM
 *  Author: lenovo
 */ 
#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

void TransmitUART(uint8_t data);

int a=0;
uint8_t b=0;

int main(void)
{
	DDRB  = 0xFF;
	PORTB = 0;
	
	DDRC  = 0;
	PORTC = 0;
	
	DDRD  = 0b11111100;
	PORTD = 0;
	
	int baud = 9600;	
	UCSR0A &= ~(1 << U2X0);		
	uint16_t UBRRValue = lrint((F_CPU /(16L * baud)) - 1);		
	UBRR0H = (unsigned char) (UBRRValue >> 8);
	UBRR0L = (unsigned char) UBRRValue;
	UCSR0B |= (1 << RXEN0) | (1 << TXEN0);
	UCSR0B |= (1 << TXCIE0) | (1 << RXCIE0);
		
	UCSR0C |= (1 << USBS0); //Sets 2 stop bits
	UCSR0C |= (1 << UPM01); //Sets parity to EVEN
	UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01); //Alternative code for 8-bit data length
	
	ADCSRA |= (1<<ADPS2) | (1<<ADIE);
	ADMUX  |= (1<<ADLAR) |(1<<REFS0);
	ADCSRA |= (1<<ADEN);
	
	TCCR1A |= (1<<WGM11) |(1<<COM1A1) |(1<<COM1B1);
	TCCR1B |= (1<<WGM12) |(1<<WGM13) |(1<<CS10);
	ICR1=19999; //top value
		
	sei();
	
	_delay_ms(1000);
	
	TransmitUART(0xFF);
		
	ADCSRA |= (1<<ADSC);
	
	while(1);
//    {
        //TODO:: Please write your application code
		//TransmitUART(0xEF);
		//_delay_ms(1500);
		//TransmitUART(0xFE);
		//_delay_ms(1500); 
//	}		
}

void TransmitUART(uint8_t data)
{
	while (! (UCSR0A & (1 << UDRE0)) );
	UDR0 = data;
}

ISR (USART_TX_vect)
{
	PORTC ^= (1<<PINC0);
}

ISR (ADC_vect)
{
	uint8_t the_low = ADCL;
	uint8_t the_high = ADCH;
	uint16_t ten_bit_val = the_high<<2 | the_low>>6;
	a=ten_bit_val;
	if (a>1000)
	{
		a=1000;
	}
	

	switch (ADMUX)
	{
		case 0x60:
		b=0;
		OCR1A = a;
		TransmitUART(0x60);
		TransmitUART(the_low);
		TransmitUART(the_high);
		ADMUX = 0x61;
		break;
		case 0x61:
		b=1;
		OCR1B = a;
		TransmitUART(0x61);
		TransmitUART(the_low);
		TransmitUART(the_high);
		ADMUX = 0x60;
		break;
	}
	
	ADCSRA |= (1<<ADSC);
}