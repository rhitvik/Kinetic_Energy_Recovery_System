/*
 * KERS.cpp
 *
 * Created: 12/29/2015 3:00:39 PM
 *  Author: RHITVIK
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>

uint8_t data;
uint8_t data1;
uint8_t low;
uint8_t high;
uint16_t datamain;
uint8_t count1;
uint8_t count2;

void motor(uint8_t side,uint8_t dir);
void debounce(void);

int main(void)
{
	uint8_t pressed=0;
	int baud = 9600;
	
	DDRB = 0xFF;
	PORTB= 0;
	DDRC = 0xFF;
	PORTC= 0;
	DDRD = 0b11111100;
	PORTD= 0;
	
	TCCR1A |= (1<<WGM11) |(1<<COM1A1) |(1<<COM1B1);
	TCCR1B |= (1<<WGM12) |(1<<WGM13) |(1<<CS10);
	ICR1=19999; //top value
	//OCR1A=duty; // 20% duty
	

	UCSR0A &= ~(1 << U2X0);
	uint16_t UBRRValue = lrint((F_CPU /(16L * baud)) - 1) ;
	UBRR0H = (unsigned char) (UBRRValue >> 8);
	UBRR0L = (unsigned char) UBRRValue;
	UCSR0B |= (1 << RXEN0) | (1 << TXEN0);
	//UCSR0B |= (1 << RXCIE0) | (1 << TXCIE0);
	UCSR0C |= (1 << USBS0); //Sets 2 stop bits
	UCSR0C |= (1 << UPM01); //Sets parity to EVEN
	UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01); //Alternative code for 8-bit data length
	
	sei();
	
	TCCR1A |= (1<<WGM11) |(1<<COM1A1) |(1<<COM1B1);
	TCCR1B |= (1<<WGM12) |(1<<WGM13) |(1<<CS10);
	ICR1=19999;	
	
	TCCR2A |= (1<<WGM20) |(1<<WGM21) |(1<<COM2A1) |(1<<COM2B1);
	TCCR2B |= (1<<CS21) |(1<<CS22) ;   //|(1<<WGM22) |(1<<CS20)
	TCNT2 = 0;
	
	OCR2A = 128;
	OCR2B = 128;
	
	TCCR0A |= (1<<WGM00) |(1<<WGM01) |(1<<COM0A1) |(1<<COM0B1);
	TCCR0B |= (1<<CS02);	
	TCNT0 = 0;
	
	OCR0A = 128;
	OCR0B = 128;
	
	motor(1,1);
	motor(2,1);
	motor(3,1);
	motor(4,1);
	
	sei();
	
	while(1)
	{
		while (! (UCSR0A & (1 << RXC0)) );
		{
			data = UDR0;
			if (data == 0xFF)
			
			loop1:
			while(2)
			{
				while (! (UCSR0A & (1 << RXC0)) );
				data1 = UDR0;
				
				while (! (UCSR0A & (1 << RXC0)) );
				low = UDR0;
				
				while (! (UCSR0A & (1 << RXC0)) );
				high = UDR0;
				
				datamain = high<<2 | low>>6;
				
				if (data1==0x60)
				{
					count1++;
					if (count1>=10)
					{
						count1=0;
						OCR1A= datamain*4;
					}
					
					goto loop1;
				}
				
				else
				if (data1==0x61)
				{
					count2++;
					if (count2>=10)
					{
						count2=0;
						OCR1B= datamain*4;
					}
					
					goto loop1;
				}
			}
		}
	}	
	
}

void motor (uint8_t side,uint8_t dir)
{	
	if (side==1 && dir==1)
	{
		PORTC |= (1<<PINC0);
		PORTC &=~(1<<PINC1);
	}
	
	else
	if (side==1 && dir==2)
	{
		PORTC |= (1<<PINC1);
		PORTC &=~(1<<PINC0);	
	}
	else	
	if (side==1 && dir==0)	
	{
		PORTC &=~(1<<PINC0);
		PORTC &=~(1<<PINC1);
	}
	else
	if (side==2 && dir==1)
	{
		PORTC |= (1<<PINC2);
		PORTC &=~(1<<PINC3);
	}
	else
	if (side==2 && dir==2)
	{
		PORTC |= (1<<PINC3);
		PORTC &=~(1<<PINC2);
	}
	else
	if (side==2 && dir==0)
	{
		PORTC &=~(1<<PINC3);
		PORTC &=~(1<<PINC2);
	}
	//////////////
	if (side==3 && dir==1)
	{
		PORTD |= (1<<PIND4);
		PORTD &=~(1<<PIND7);
	}
	
	else
	if (side==3 && dir==2)
	{
		PORTD |= (1<<PIND7);
		PORTD &=~(1<<PIND4);
	}
	else
	if (side==3 && dir==0)
	{
		PORTD &=~(1<<PIND4);
		PORTD &=~(1<<PIND7);
	}
	else
	if (side==4 && dir==1)
	{
		PORTC |= (1<<PINC4);
		PORTC &=~(1<<PINC5);
	}
	else
	if (side==4 && dir==2)
	{
		PORTC |= (1<<PINC5);
		PORTC &=~(1<<PINC4);
	}
	else
	if (side==4 && dir==0)
	{
		PORTC &=~(1<<PINC4);
		PORTC &=~(1<<PINC5);
	}
}

void debounce(void)
{
	unsigned int pressed_confidence_level=0;
	pressed_confidence_level++;
	if(pressed_confidence_level>=500 )//put a debouncing value///////////////
	{
		pressed_confidence_level=0;
	}
}

