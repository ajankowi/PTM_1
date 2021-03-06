#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/sfr_defs.h>
#include <math.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

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

/*
 *  Gotowe zaimplementowane:
 #define 	bit_is_set(sfr, bit)   (_SFR_BYTE(sfr) & _BV(bit))
 #define 	bit_is_clear(sfr, bit)   (!(_SFR_BYTE(sfr) & _BV(bit)))
 #define 	loop_until_bit_is_set(sfr, bit)   do { } while (bit_is_clear(sfr, bit))
 #define 	loop_until_bit_is_clear(sfr, bit)   do { } while (bit_is_set(sfr, bit))

 */

// MIN/MAX/ABS macros
#define MIN(a,b)			((a<b)?(a):(b))
#define MAX(a,b)			((a>b)?(a):(b))
#define ABS(x)				((x>0)?(x):(-x))

volatile uint8_t minuty, sekundy;
volatile uint16_t liczba7Seg;

volatile char znaki[4];

char cyfra[10] = { 0b1111110, 0b0110000, 0b1101101, 0b1111001, 0b0110011,
		0b1011011, 0b1011111, 0b1110000, 0b1111111, 0b1111011 };

//Inicjalizacja Timer1 do wywolywania przerwania z częstotliwością 2Hz
void TimerInit() {
	//Wybranie trybu pracy CTC z TOP OCR1A
	sbi(TCCR1B,WGM12);
	//Wybranie dzielnika czestotliwosci
	sbi(TCCR1B,CS12);
	//Zapisanie do OCR1A wartosci odpowiadajacej 0,5s
	OCR1A=31250;
	//Uruchomienie przerwania OCIE1A
	sbi(TIMSK,OCIE1A);
}

//Inicjalizacja portow do obsługi wyswietlacza 7 segmentowego
void seg7Init() {
	//Inicjalizacja segmentu
	DDRC = 0xff;
	PORTC = 0x00;

}

//Wyswietla na wyswietlaczu 7 segmentowym cyfre z argumentu
void seg7ShowCyfra(uint8_t cyfraDoWyswietlenia) {
	PORTC = cyfra[cyfraDoWyswietlenia];	//co to robi - wytlumaczyc prowadzacemu

}

uint8_t i = 10;
uint8_t j = 0;
uint8_t k = 0;


int main() {

	/**************************
	**************************
	*****  Zadanie na 3  *****
	**************************
	************************** */

	//  Przerwania – jest to mechanizm, który pozwala mikrokontrolerowi
	//  na przerwanie bieżąco wykonywanych zadań na skutek otrzymania jakiegoś zdarzenia, które wymaga pilnej obsługi.

	//  Można je wykorzystac do:

	//  liczniki – przepełnienie licznika, odliczenie ustalonej ilości impulsów
	//  moduł komunikacji – zdarzenie informujące o zakończeniu transmisji


	//			OCR1A=15625;

/*	TimerInit();
	seg7Init();

	sei(); //funkcja uruchamia globalne przerwania
	sbi(DDRB,PB0);

	while (1) {
		//for (uint8_t i = 0; i < 10; i++) {
		//	seg7ShowCyfra(i);

	//	}
		_delay_ms(5000);
	}

	return 0;
*/


	/**************************
	**************************
	*****  Zadanie na 4  *****
	**************************
	************************** */
/*
	TimerInit();
	seg7Init();

	sei(); //funkcja uruchamia globalne przerwania
	sbi(DDRB,PB0);

	while (1) {
		for (uint8_t i = 0; i < 10; i++) {
			seg7ShowCyfra(i);
			_delay_ms(500);
		}

	}

	return 0;

*/



	/**************************
	**************************
	*****  Zadanie na 5  *****
	**************************
	************************** */

	//Ustawienie PD0 jako PULL-UP
	cbi(DDRD, 0);
	sbi(PORTD, 0);

	//Ustawienie PB4 jako PULL-UP
	cbi(DDRB, 4);
	sbi(PORTB, 4);

	//Ustawienie PB5 jako wyjście stanu niskiego
	sbi(DDRB, 5);
	cbi(PORTB, 5);


	TimerInit();		//Uruchamia funkcje TimerInit()
	seg7Init();			//Uruchamia funkcje seg7Init()


	sbi(DDRB,PB0);


	while(bit_is_set(PIND,0)){		//Sprawdza czy przycisk został wcisniety

	}

	_delay_ms(500);

	sei(); //funkcja uruchamia globalne przerwania


	while (1) {



		if(bit_is_clear(PIND,0)){		//Sprawdza czy przycisk został wcisniety

			sbi(PORTB, 5);			//Zmienia jako wyjście stan wysoki
			j++;
			_delay_ms(500);
		}
		if(j > 1){
			cbi(PORTB, 5);			//Zmienia jako wyjście stan niski
			j = 0;					//Ustawia zmienna pomocnicza j na 0
		}



	}

	return 0;


}








//Funkcja uruchamiana z przerwaniem po przepelnieniu licznika w timer1
ISR(TIMER1_COMPA_vect) {



	if(bit_is_clear(PINB,4)){	//Sprawdza wartosc na pinie B5
		i--;
		seg7ShowCyfra(i);
	}
	else{
		i = 10;					//Ustawia zmienna pomocnicza na 10
	}

	if(i == 0){

		sbi(PORTB,PB0);			//Zapala diode
		_delay_ms(5000);		//Odczekuje 5sek
		cbi(PORTB,PB0);			//Gasi diode
		i = 10;					//Ustawia zmienna pomocnicza na 10
	}

}
