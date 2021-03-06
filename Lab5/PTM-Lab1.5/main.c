#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/sfr_defs.h>
#include <math.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <inttypes.h>
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

//Napi?cie referencyjne 5V

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
	//ADMUX bity Mux - ustawienie kana?u
	cbi(ADMUX,MUX0);
	cbi(ADMUX,MUX1);
	cbi(ADMUX,MUX2);
	cbi(ADMUX,MUX3);
	cbi(ADMUX,MUX4);
}

void ADC_init1()
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
	//ADMUX bity Mux - ustawienie kana?u
	sbi(ADMUX,MUX0);
	cbi(ADMUX,MUX1);
	cbi(ADMUX,MUX2);
	cbi(ADMUX,MUX3);
	cbi(ADMUX,MUX4);
}

uint16_t ADC_10bit()
{

	sbi(ADCSRA,ADSC);

	while(bit_is_clear(ADCSRA,ADSC));

	return(ADC);
}

uint32_t ADC_measure()
{

	if(ADC_10bit() != 0){

    return 1 + 500*((uint32_t)ADC_10bit())/1024;

	}
	return  500*((uint32_t)ADC_10bit())/1024;

}

//uint32_t ADC_measure1()
//{
//    return 500*((uint32_t)ADC_10bit(PA1))/1024;
//}



/*

// Zadanie na 3
int main() {
	ADC_init();
	LCD_Initalize();
	LCD_Home();
	uint32_t c=0;
	//LCD_Clear();
	char text[20];

	while(1) {
		c = ADC_10bit();
		LCD_Clear();
		LCD_GoTo(0,0);
		sprintf(text , "Pomiar: %"PRIu32" " , c );
		LCD_WriteText(text);
		_delay_ms(1000);
	}

}

*/
/*
//Zadanie na 4

int main() {
	ADC_init();
	LCD_Initalize();
	LCD_Home();
	uint32_t c=0;
	//LCD_Clear();
	char text[20];

	while(1) {
		c = ADC_measure();
		LCD_Clear();
		LCD_GoTo(0,1);
		sprintf(text , "Pomiar: %"PRIu32" V", c );
		LCD_WriteText(text);
		_delay_ms(1000);
	}

}
*/

/*
//Zadanie na 4,5
int main() {
	ADC_init();
	LCD_Initalize();
	LCD_Home();
	uint32_t c=0;
	//LCD_Clear();
	char text[20];

	sbi(DDRC,PC3);        //Stan wysoki na dziodzie
	//sbi(PORTC,PC3);

	while(1) {
		c = ADC_measure();
		LCD_Clear();
		LCD_GoTo(0,0);
		sprintf(text , "Pomiar: %"PRIu32" V" , c );
		LCD_WriteText(text);

		if (c > 250)
		{
			sbi(PORTC,PC3);
			LCD_GoTo(0,1);
			sprintf(text , "on");
			LCD_WriteText(text);
			_delay_ms(1000);
		}
		else
		{
			cbi(PORTC,PC3);
			LCD_GoTo(0,1);
			sprintf(text , "off");
			LCD_WriteText(text);
			_delay_ms(1000);
		}
	}

}
*/

//Zadanie na 5

int main()
{

	LCD_Initalize();		//Inicjalizacja LCD
	LCD_Home();
	uint32_t c = 0;			//Zmienna pomocnicza
	uint32_t x = 0;			//Zmienna pomocnicza
	uint8_t h = 50;			//Zmienna przechowujaca zakres
	LCD_Clear();			//Czyszczenie wyswietlacza
	char text[20];			//Tablica znakow
	sbi(DDRC,PC3);			//Ustawienie stanu na porcie C3
	sbi(DDRC,PC4);			//Ustawienie stanu na porcie C4
	while(1) {

		ADC_init();			//Inicjalizacja do 1 pomiaru
		_delay_ms(20);
		c = ADC_measure();	//Pomiar 1 napiecia

		ADC_init1();		//Inicjalizacja do 2 pomiaru
		_delay_ms(20);
		x = ADC_measure();	//Pomiar 1 napiecia

		_delay_ms(150);

		sprintf(text , "V1:%"PRIu32"V, V2:%"PRIu32"V" , c , x );		//Zapisuje wartosci do tablicy
		LCD_GoTo(0,0);													//Ustawia pozycje na 1 wiersz
		LCD_Clear();													//Usuwa stare napisy z wyswietlacza
		LCD_WriteText(text);											//Przypisuje nowe wartosci do wyswietlacza


		if (x > c + (h/2) )												//Sprawdzanie warunkow histerezy
		{
			sbi(PORTC,PC4);												//Stan wysoki na C4

		}
		else if (x < c - (h/2))											//Sprawdzanie warunkow histerezy
		{
			cbi(PORTC,PC4);												//Stan niski na C4

		}


//		if (c > 250)													//Sprawdzanie warunkow komparatora
//		{
//			sbi(PORTC,PC3);												//Stan wysoki na wyjsciu C3
//			LCD_GoTo(0,1);												//Ustawia pozycje na 2 wiersz
//			sprintf(text , "on");										//Zapisuje napis "on" w tabeli znakow
//			LCD_WriteText(text);										//Wyswietla napis "on"
//			_delay_ms(100);
//		}
//		else
//		{
//			cbi(PORTC,PC3);												//Ustala stan niski na wyjsciu C3
//			LCD_GoTo(0,1);												//Ustawia pozycje na 2 wiersz
//			sprintf(text , "off");										//Zapisuje napis "off" w tabeli znakow
//			LCD_WriteText(text);										//Wyswietla napis "on"
//			_delay_ms(100);
//		}



	}
	return 0;
}

