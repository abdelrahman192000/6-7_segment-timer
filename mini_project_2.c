/*
 * mini_project_2.c
 *
 *  Created on: Sep 17, 2022
 *      Author: abdelrahman adel
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

unsigned char flag =0,segment_num[6];

void TIMER_1_Init (void)
{
	TCCR1A=(1<<FOC1A)|(1<<FOC1B);//NOT PWM
	TCCR1B=(1<<WGM12)|(1<<CS10)|(1<<CS12);//prescaler =1024 and mode is ctc
	TCNT1=0; //intial value is 0
	OCR1A=976;
	TIMSK |=(1<<OCIE1A);//enable comapre interrupt
	SREG |=(1<<7);//enable interrupt bit


}

ISR (TIMER1_COMPA_vect)
{



	flag=1;
}

void INT0_Init(void)
{
	DDRD  &=~(1<<2);
	PORTD |=(1<<2);
	MCUCR |= (1<<ISC01); //set falling edge
	GICR |=(1<<INT0);



}
ISR (INT0_vect)
{
	//all variable set to value zero
	segment_num[0]=0;
	segment_num[1]=0;
	segment_num[2]=0;
	segment_num[3]=0;
	segment_num[4]=0;
	segment_num[5]=0;



}
void INT1_Iint (void )
{
	DDRD &=(1<<3);
	PORTD |=(1<<3);
	MCUCR |=(1<<ISC11)|(1<<ISC10); //set rising edge
	GICR |= (1<<INT1);



}
ISR (INT1_vect)
{

	TCCR1B &=~ (1<<CS10) &~(1<<CS12);//No clock source (Timer/Counter stopped)

}
void INT2_Init (void )
{
	DDRB &=~(1<<2);
	PORTB |=(1<<2);
	MCUCSR &=~(1<<ISC2);// set as falling edge
	GICR |=(1<<INT2);



}
ISR (INT2_vect)
{
	TCCR1B |= (1<<CS10) |(1<<CS12);//clock source again  with 1024 presccaller(counter start )

}


int main ()
{
	DDRC   |=(1<<0)|(1<<1)|(1<<2)|(1<<3);
	PORTC &=~(1<<9)&~(1<<1)&~(1<<2)&~(1<<3);
	DDRA    |=0x3F;//configure first 6 pins as output
	PORTA &=~(1<<0) &~(1<<1) &~(1<<2) &~(1<<3) &~(1<<4) &~(1<<5);



	INT0_Init();
	INT1_Iint();
	INT2_Init();

	TIMER_1_Init();

	while (1)
	{
		if (flag==1)
		{

			segment_num[5]++;


			if(segment_num[5]==10)
			{
				segment_num[4]++;
				segment_num[5]=0;
			}
			if(segment_num[5]==0 && segment_num[4]==6)
			{
				segment_num[4]=0;
				segment_num[3]++;
			}
			if(segment_num[3]==10)
			{
				segment_num[2]++;
				segment_num[3]=0;
				segment_num[4]=0;
				segment_num[5]=0;
			}
			if(segment_num[3]==0 && segment_num[2]==6)
			{
				segment_num[3]=0;
				segment_num[4]=0;
				segment_num[5]=0;
				segment_num[2]=0;
				segment_num[1]++;
			}
			if(segment_num[1]==10)
			{
				segment_num[0]++;
				segment_num[1]=0;
				segment_num[2]=0;
				segment_num[3]=0;
				segment_num[4]=0;
				segment_num[5]=0;
			}
			flag=0;
		}

		PORTA =(1<<0);

		PORTC=(PORTC & 0xF0)|(segment_num[0]);
		_delay_ms(2);


		PORTA =(1<<1);
		PORTC =(PORTC & 0xF0)|(segment_num[1]);
		_delay_ms(2);


		PORTA =(1<<2);
		PORTC = (PORTC & 0xF0)|(segment_num[2]);
		_delay_ms(2);


		PORTA =(1<<3);
		PORTC = (PORTC & 0xF0)|(segment_num[3]);
		_delay_ms(2);


		PORTA =(1<<4);
		PORTC = (PORTC & 0xF0)|(segment_num[4]);
		_delay_ms(2);


		PORTA =(1<<5);
		PORTC = (PORTC & 0xF0)|(segment_num[5]);
		_delay_ms(2);

	}

}








