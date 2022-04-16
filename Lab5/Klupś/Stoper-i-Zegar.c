#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/sfr_defs.h>
#include <math.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "HD44780.h"


#ifndef _BV
#define _BV(bit)				(1<<(bit))
#endif

#ifndef inb
#define	inb(addr)			(addr)
#endif

#ifndef outb
#define	outb(addr, data)	addr = (data)
#endif

#ifndef sbi
#define sbi(reg,bit)		reg |= (_BV(bit))
#endif

#ifndef cbi
#define cbi(reg,bit)		reg &= ~(_BV(bit))
#endif

#ifndef tbi
#define tbi(reg,bit)		reg ^= (_BV(bit))
#endif

//  Gotowe zaimplementowane:
 #define 	bit_is_set(sfr, bit)   (_SFR_BYTE(sfr) & _BV(bit))
 #define 	bit_is_clear(sfr, bit)   (!(_SFR_BYTE(sfr) & _BV(bit)))
 #define 	loop_until_bit_is_set(sfr, bit)   do { } while (bit_is_clear(sfr, bit))
 #define 	loop_until_bit_is_clear(sfr, bit)   do { } while (bit_is_set(sfr, bit))



// MIN/MAX/ABS macros
#define MIN(a,b)			((a<b)?(a):(b))
#define MAX(a,b)			((a>b)?(a):(b))
#define ABS(x)				((x>0)?(x):(-x))


uint8_t i = 0;
uint8_t j = 0;
//uint8_t k = 0;

volatile uint8_t minuty, sekundy;
volatile uint16_t liczba7Seg;
volatile char znaki[4];
volatile uint8_t flaga = 0;

void Info()
{
	char text[50];
	LCD_GoTo(0,0);
	sprintf(text , "PTM_2021");
	LCD_WriteText(text);
	LCD_GoTo(0,1);
	sprintf(text , "252870,252919");
	LCD_WriteText(text);
	_delay_ms(4000);
	LCD_Clear();

}


char cyfra[10] = { 0b1111110, 0b0110000, 0b1101101, 0b1111001, 0b0110011,
		0b1011011, 0b1011111, 0b1110000, 0b1111111, 0b1111011};



//Inicjalizacja Timer1 do wywolywania przerwania z czêstotliwoœci¹ 2Hz
void TimerInit() {
	//Wybranie trybu pracy CTC z TOP OCR1A
	sbi(TCCR1B,WGM12);
	//Wybranie dzielnika czestotliwosci
	sbi(TCCR1B,CS12);
	//Zapisanie do OCR1A wartosci odpowiadajacej 0,1s
	OCR1A = 31.25;
	//Uruchomienie przerwania OCIE1A
	sbi(TIMSK,OCIE1A);
}

//Inicjalizacja portow do obs³ugi wyswietlacza 7 segmentowego
void seg7Init() {

	//Inicjalizacja segmentu
	DDRC = 0xFF;
	PORTC = 0x00;

}

//Wyswietla na wyswietlaczu 7 segmentowym cyfre z argumentu
void seg7ShowCyfra(uint8_t cyfraDoWyswietlenia) {
	PORTC = cyfra[cyfraDoWyswietlenia];	//co to robi - wytlumaczyc prowadzacemu

}



void ADC_init()
{
	//Rejestr Admux
	sbi(ADMUX,REFS0);
	cbi(ADMUX,REFS1);
	//Rejestr ADCSRA Mniejsze od 100kHz
	sbi(ADCSRA,ADPS0);
	sbi(ADCSRA,ADPS1);
	sbi(ADCSRA,ADPS2);
	//Rejestr ADCSRA Aden
	sbi(ADCSRA,ADEN);
	//ADMUX bity Mux - ustawienie kana³u
	cbi(ADMUX,MUX0);
	cbi(ADMUX,MUX1);
	cbi(ADMUX,MUX2);
	cbi(ADMUX,MUX3);
	cbi(ADMUX,MUX4);
}

uint16_t ADC_10bit(uint8_t x)
{

	sbi(ADCSRA,ADSC);

	while(bit_is_clear(ADCSRA,ADSC));

	return(ADC);
}

uint32_t ADC_measure(uint8_t x)
{
    return 500*((uint32_t)ADC_10bit(x))/1024;
}

//Zmienne dla stopera
volatile uint16_t ms = 0;
volatile uint16_t s = 0;
volatile uint16_t m = 0;
volatile uint16_t h = 0;


void stoper () {

	LCD_Initalize();
	LCD_Home();

	char text[40];

	TimerInit();
	OCR1A = 31.25;
	LCD_Clear();

	sbi(DDRD,PD1);


	sei(); //funkcja uruchamia globalne przerwania

	while(1) {

		LCD_GoTo(0,0);
		sprintf(text , "%d:%d:%d"  , m , s , ms);
		LCD_WriteText(text);


	}

}

//Przerwania dla stopera OCR1A = 31.25;
//ISR(TIMER1_COMPA_vect) {
//
//	ms++;
//			if(ms == 1000){
//				s++;
//				i++;
//				ms = 0;
//			}
//			if(s == 60){
//				 m++;
//				s = 0;
//			}
//			if (i == 1 || i == 3 || i == 5 || i == 7 || i == 9)
//			{
//				sbi(PORTD,PD1);
//			}
//			else
//			{
//				cbi(PORTD,PD1);
//			}
//
//
//
//
//}


//Zmienne dla Zegara
volatile uint16_t msek = 0;
volatile uint16_t sek = 0;
volatile uint16_t min = 0;


void zegar ()
{

	char text1[10];
	char text2[10];

	LCD_Initalize();
	LCD_Home();
	seg7Init();
	TimerInit();
	LCD_Clear();


	sbi(DDRD,PD1);

	sei();

	while(1) {
		if(sek < 10){
			sprintf(text1, ":0%d", sek);
		}
		else{
			sprintf(text1, ":%d", sek);
		}

		if(min < 10){
			sprintf(text2, "0%d", min);
		}
		else{
			sprintf(text2, "%d", min);
		}


		LCD_GoTo(2, 0);
		LCD_WriteText(text1);			//Wyswietla sekundy

		LCD_GoTo(0, 0);
		LCD_WriteText(text2);			//Wyswietla minuty




	}


}

//Przerwanie dla zegara
ISR(TIMER1_COMPA_vect) {
	msek++;
	if (msek == 1000) {
		sek++;
		msek = 0;
		j++;
		seg7ShowCyfra(j);
	}
	if (sek == 60)
	{
		sek = 0;
		min++;
	}
	if (j == 10)
	{
		j = 0;
		seg7ShowCyfra(0b1111110);
	}
	if (msek > 0 && msek < 200)
	{
		sbi(PORTD,PD1);
	}
	else
	{
		cbi(PORTD,PD1);
	}



}

int main ()
{
	//stoper();
	zegar();
}




