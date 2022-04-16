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
#include <stdbool.h>



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

//Napiêcie referencyjne 5V

volatile uint32_t i;

//Funkcja inicjuje miernik sygna³u analogowego
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

//Funkcja zwraca wartosc odczytanego sygnalu analogowego
uint16_t ADC_10bit()
{

	sbi(ADCSRA,ADSC);

	while(bit_is_clear(ADCSRA,ADSC));

	return(ADC);
}

//Funkcja przelicza wartosc odczytana na volty
uint32_t ADC_measure()
{

	if(ADC_10bit() != 0){

    return 1 + 500*((uint32_t)ADC_10bit())/1024;

	}
	return  500*((uint32_t)ADC_10bit())/1024;

}

//Funcja wyswietla poprawna wartosc napiecia w V
void wyswietl(){

	char text[20];			//Tablica znakow
	uint32_t a = 0;			//Zmienna pomocnicza
	uint32_t b = 0;			//Zmienna pomocnicza
	uint32_t x = 0;			//Zmienna pomocnicza

	x = ADC_measure();		//Przypisanie wartosci odczytanej do x

	a = x/100;				//Obliczenie wartosci setek
	b = x - 100 * a;		//Obliczenia wartosci po przecinku

	LCD_GoTo(0,1);
	sprintf(text , " Pomiar: %"PRIu32".%"PRIu32" V" , a, b );		//Wyswietlenie pomiaru
	LCD_WriteText(text);
}

//Tablica zankow przechowujaca liczby do wyswietlacza
char cyfra[11] = { 0b1111110, 0b0110000, 0b1101101, 0b1111001, 0b0110011,
		0b1011011, 0b1011111, 0b1110000, 0b1111111, 0b1111011, 0b0000001};

//Inicjalizacja portow do obs³ugi wyswietlacza 7 segmentowego
void seg7Init() {
	//Inicjalizacja segmentu
	DDRC = 0xff;
	PORTC = 0x00;
}

//Wyswietla na wyswietlaczu 7 segmentowym cyfre z argumentu
void seg7ShowCyfra(uint8_t cyfraDoWyswietlenia) {
	PORTC = cyfra[cyfraDoWyswietlenia];
}

//Inicjalizacja Timer1 do wywolywania przerwania z czêstotliwoœci¹ 100Hz
void TimerInit() {
	//Wybranie trybu pracy CTC z TOP OCR1A
	sbi(TCCR1B,WGM12);
	//Wybranie dzielnika czestotliwosci
	sbi(TCCR1B,CS12);
	//Zapisanie do OCR1A wartosci odpowiadajacej 0,01s
	OCR1A=312.5;
	//Uruchomienie przerwania OCIE1A
	sbi(TIMSK,OCIE1A);
}

//Inicjalizacja Zegara do wywolywania przerwania z czêstotliwoœci¹ 1Hz
void ZegarInit() {
	//Wybranie trybu pracy CTC z TOP OCR1A
	sbi(TCCR1B,WGM12);
	//Wybranie dzielnika czestotliwosci
	sbi(TCCR1B,CS12);
	//Zapisanie do OCR1A wartosci odpowiadajacej 1s
	OCR1A=31250;
	//Uruchomienie przerwania OCIE1A
	sbi(TIMSK,OCIE1A);
}

//Funcja wyswietla informacje o programie
void info(){

	char text[20];			//Tablica znakow
	int wyjdz = 1;			//Zmienna pomocnicza

	LCD_Clear();
	sprintf(text , "Program PTM 2021" );	//Wyswietlenie napisu
	LCD_GoTo(0,0);
	LCD_WriteText(text);

	sprintf(text , "252919, 252870" );		//Wyswietlenie numerow indeksow
	LCD_GoTo(0,1);
	LCD_WriteText(text);
	for(int i = 1; (i <= 800)&&(wyjdz); i++){		//Petla umozliwia nagle wyjscie z funkcji


		cbi(PORTD, 0);								//Ustawia wyjœcie stan niski
		_delay_ms(5);
													//Sprawdza ktory przycisk jest wlaczony
		if(bit_is_clear(PIND,2)){
			wyjdz = 0;
		}
		sbi(PORTD,0);
	}
}

//Funkcja liczby
void liczby(){

	int wyjdz = 1;					//Zmienna pomocnicza
	int pierwsza = 1;				//Zmienna pomocnicza
	uint8_t a = 0;					//Zmienna pomocnicza
	char text[20];					//Tablica znakow

	seg7Init();						//Uruchamia funkcje seg7Init()

	while(1 && wyjdz){							//Petla sprawdzajaca czy zostal wcisniety X
		_delay_ms(100);

		cbi(PORTD, 0);							//Ustawia wyjœcie stan niski
		_delay_ms(5);
												//Sprawdza ktory przycisk jest wlaczony
		if(bit_is_clear(PIND,2)){
			wyjdz = 0;
		}
		if(bit_is_clear(PIND,3)){				//Przycisk up
			if(a == 50){
				a = 50;
			}
			else{
				a++;
			}
		}
		sbi(PORTD, 0);							//Ustawia wyjœcie stan wysoki

		cbi(PORTD, 1);							//Ustawia wyjœcie stan niski
		_delay_ms(5);
												//Sprawdza ktory przycisk jest wlaczony
		if(bit_is_clear(PIND,3)){
			if(a == 0){
				a = 0;
			}
			else{
				a--;							//Przycisk down
			}
		}
		sbi(PORTD, 1);							//Ustawia wyjœcie stan wysoki

		LCD_Clear();
		LCD_GoTo(0,0);
		sprintf(text , "Liczba: %"PRIu8" ", a);		//Wypisanie wartosci danej liczby
		LCD_WriteText(text);

		if(a > 9){
			seg7ShowCyfra(10);						//Wyswietlenie "-"
		}
		else{
			seg7ShowCyfra(a);						//Wyswietla liczby od 1-9 na wyswietlaczu 7 segmentowym
		}

		if(a % 2){									//Sprawdza czy liczba jest nieparzysta
			cbi(PORTA,4);							//Gasi LED 1
			sbi(PORTA,5);							//Zapala LED 2
		}
		else{										//Liczba jest parzysta
			sbi(PORTA,4);							//Zapala LED 1
			cbi(PORTA,5);							//Gasi LED 2
		}

		if(a < 2){									//Jezeli liczba jest mniejsza od 1 to nie jest liczba pierwsza
			pierwsza = 0;
		}

		if(a >= 2){
			for(uint8_t i = 2; i*i <= a; i++)
				if(a % i == 0){
					pierwsza = 0;		 //gdy znajdziemy dzielnik, to dana liczba nie jest pierwsza
				}
		}
		if(pierwsza == 0){				//Jezeli liczba nie jest licza pierwsza
			cbi(PORTA,6);				//Gasi LED 3
			pierwsza = 1;
		}
		else{
			sbi(PORTA,6);				//Zapala LED 3
		}
	}

	cbi(PORTA,4);						//Ustawia stan niski na A4
	cbi(PORTA,5);						//Ustawia stan niski na A5
	cbi(PORTA,6);						//Ustawia stan niski na A6
	PORTC = 0x00;						//Ustawia stan niski na portach C
}

//Funkcja liczy czas z dokladnoscia do 0,01s
void stoper () {

	TimerInit();						//Uruchamia funkcje TimerInit()

	char text[20];						//Tablica znakow
	uint32_t ms = 0;					//Zmienna pomocnicza
	uint32_t s = 0;						//Zmienna pomocnicza
	uint32_t k = 1;						//Zmienna pomocnicza
	int licznik = 0;					//Zmienna pomocnicza
	int kasuj = 1;						//Zmienna pomocnicza
	int zatrzymaj = 0;					//Zmienna pomocnicza
	int wyjdz = 1;						//Zmienna pomocnicza



	sprintf(text , "Stoper: 00:00");			//Wypisanie na wyswietlaczu 00:00
	LCD_GoTo(0,0);
	LCD_Clear();
	LCD_WriteText(text);
	_delay_ms(80);

	while(1 && wyjdz){									//Petla sprawdza czy zostal wcisniety przycisk X
		while((k != i) && zatrzymaj && wyjdz){			//Petla sprawdza czy zostal wcisniety przycisk X lub OK
			k = i;										//Przypisanie wartosci z przerwania
			ms = i - licznik * 100;						//Obliczanie dokladnej wartosci setnych sekund

			if((i - licznik * 100) >= 100){
				tbi(PORTA,4);							//Ustrawia stan niski i wysoki na porcie A4
				licznik++;								//Zwiekszenie zakresu licznik
				s++;									//Zwiekszenie zakresu s
				ms = i-licznik*100;						//Obliczenie wartosci milisekund
			}

			if(s == 60){								//Jezli dojdzie do 60 sek
				s = 0;									//Zeruje wartosc s
			}

			if(s < 10){									//Umozliwia poprawne wyswietlanie stopera dla wartosi mniejszych od 10
				sprintf(text , "Stoper: 0%"PRIu32":%"PRIu32" ",s ,ms );
				LCD_GoTo(0,0);
				LCD_WriteText(text);
				_delay_ms(80);
			}
			else{										//Umozliwia poprawne wyswietlanie stopera dla wartosi wiekszych od 10
				sprintf(text , "Stoper: %"PRIu32":%"PRIu32" ",s ,ms );
				LCD_GoTo(0,0);
				LCD_WriteText(text);
				_delay_ms(80);
			}
			cbi(PORTD, 0);								//Ustawia wyjœcie stan niski
			_delay_ms(5);
														//Sprawdza ktory przycisk jest wlaczony
			if(bit_is_clear(PIND,2)){
				wyjdz = 0;
			}
			sbi(PORTD,0);

			cbi(PORTD, 1);								//Ustawia wyjœcie stan niski
			_delay_ms(5);
														//Sprawdza ktory przycisk jest wlaczony
			if(bit_is_clear(PIND,2)){
				zatrzymaj = 0;
				kasuj = 0;
				_delay_ms(100);
			}
			sbi(PORTD,1);
		}

		cli();										//Wylacza przerywanie

		cbi(PORTD, 0);								//Ustawia wyjœcie stan niski
		_delay_ms(5);
													//Sprawdza ktory przycisk jest wlaczony
		if(bit_is_clear(PIND,2)){
			wyjdz = 0;
		}
		sbi(PORTD,0);

		if(kasuj == 1){									//Sprawdzenie warunkow
			cbi(PORTD, 1);								//Ustawia wyjœcie stan niski
			_delay_ms(5);								//Sprawdza ktory przycisk jest wlaczony
			if(bit_is_clear(PIND,2)){
				zatrzymaj = 1;
				_delay_ms(50);
				sei();									//funkcja uruchamia globalne przerwania
			}
			sbi(PORTD,1);
		}
		else{
			cbi(PORTD, 1);								//Ustawia wyjœcie stan niski
			_delay_ms(5);								//Sprawdza ktory przycisk jest wlaczony
			if(bit_is_clear(PIND,2)){
				kasuj = 1;								//Wyzerowanie wartosci
				i = 0;
				ms = 0;
				s = 0;
				licznik = 0;

				sprintf(text , "Stoper: 00:00");		//Wyswietlenie zresetowanego stopera
				LCD_GoTo(0,0);
				LCD_Clear();
				LCD_WriteText(text);
				_delay_ms(100);
			}
			sbi(PORTD,1);

		}

	}

}

//Funcja pelni role zegara zliczajacego czas z dokladnoscia do 1sek za pomoca przerwan
void zegar(){

	ZegarInit();					//Uruchamia funkcje ZegarInit()
	seg7Init();						//Uruchamia funkcje seg7Init()
	i = 0;							//Wyzerowanie zmiennej globalnej
	int wyjdz = 1;					//Zmienna pomocnicza
	int k = 0;						//Zmienna pomocnicza
	int j = 0;						//Zmienna pomocnicza
	char text[20];					//Zmienna pomocnicza
	uint32_t m = 0;					//Zmienna pomocnicza

	LCD_GoTo(0,0);
	LCD_Clear();					//Czyszczenie wyswietlacza

	sei();							//Uruchamienie przewania
	while(1 && wyjdz){

		cbi(PORTD, 0);						//Ustawia wyjœcie stan niski
		_delay_ms(5);
														//Sprawdza ktory przycisk jest wlaczony
		if(bit_is_clear(PIND,2)){
			wyjdz = 0;
		}
		sbi(PORTD, 0);						//Ustawia wyjœcie stan wysoki

		if(wyjdz == 0){						//Jezeli zostal wcisniety przycisk X
			PORTC = 0x00;					//Gasi wyswietlacz 7 segmentowy
		}


		while(1 && (k != i)){				//Petla jest uruchamiana dokladnie co 1sek
			j++;							//Zwiekszenie zakresu o 1
			k = i;							//Podstawienie wartosci i pod zmienna k
			if(i > 59){						//Warunek umozliwia poprawne wyswietlanie sekund
				m++;						//Zwiekszenie ilosci minut o 1
				i = 0;						//Wyzerowanie zmiennej
			}
			if(m > 59){						//Warunek umozliwia poprawne wyswietlanie minut
				m = 0;						//Wyzerwoanie zmiennej
			}

			if(j < 10){						//Sprawdza czy j jest mniejsze od 10
				seg7ShowCyfra(j);			//Wyswietla liczbe od 0 do 9
				if(j == 9){					//Warunek sprawdza czy j jest rowne 9
					j = -1;					//Podtsawinie -1 pod zmienna j
				}
			}

			if(i < 10 && m < 10){			//Umozliwia poprawen wyswietlanie zegara
				sprintf(text ,"Zegar: 0%"PRIu32":0%"PRIu32" ",m ,i);		//Wyswietlanie godziny
				LCD_GoTo(0,0);
				LCD_WriteText(text);

			}
			else if(i < 10){				//Umozliwia poprawen wyswietlanie zegara
				sprintf(text ,"Zegar: %"PRIu32":0%"PRIu32" ",m ,i);			//Wyswietlanie godziny
				LCD_GoTo(0,0);
				LCD_WriteText(text);

			}
			else if(m < 10){				//Umozliwia poprawen wyswietlanie zegara
				sprintf(text ,"Zegar: 0%"PRIu32":%"PRIu32" ",m ,i);			//Wyswietlanie godziny
				LCD_GoTo(0,0);
				LCD_WriteText(text);

			}
			sbi(PORTA,6);						//Ustawia stan wysoki na porcie A6
			_delay_ms(200);
			cbi(PORTA,6);						//Ustawia stan niski na porcie A6
		}
	}
}

//Funcja mierzy napiêcie
void miernik(){

	ADC_init();											//Inicjalizacja pomiaru

	int wyjdz = 1;										//Podtsawienie 1 pod zmienna wyjdz
	uint32_t c = 0;										//Zmienna pomocnicza
	char text[20];										//Tablica znakow

	while(1 && wyjdz) {									//Petla dziala tak dlugo az zostanie wcisniety przycisk X
			c = ADC_10bit();							//Zwraca wartosc pomiaru
			LCD_Clear();
			LCD_GoTo(0,0);
			sprintf(text , " ADC: %"PRIu32" " , c );	//Wystwielenie pomiaru na wyswietlaczu
			LCD_WriteText(text);

			LCD_GoTo(0,1);
			wyswietl();

			cbi(PORTD, 0);										//Ustawia wyjœcie stan niski
			_delay_ms(5);
																//Sprawdza ktory przycisk jest wlaczony
			if(bit_is_clear(PIND,2)){
				wyjdz = 0;										//Podstawienie 0 pod zmienna wyjdz
			}
			sbi(PORTD,0);										//Ustawia stan wysoki na porcie D0


			_delay_ms(100);
		}
}

//Funkcja umozliwia wybranie odpowiedniego prograsmu
void menu(){

	char text[20];			//Tablica znakow
	int i = 1;
	uint16_t ok = 0;

	sbi(DDRD, 0);			//Wyjœcie stan wysoki porty PD0-PD1
	sbi(PORTD, 0);
	sbi(DDRD, 1);
	sbi(PORTD, 1);

							//Ustawienie PD2-PD3 jako PULL-UP
	cbi(DDRD, 2);
	sbi(PORTD, 2);
	cbi(DDRD, 3);
	sbi(PORTD, 3);

	while(1){
		_delay_ms(100);

		cbi(PORTD, 0);						//Ustawia wyjœcie stan niski
		_delay_ms(5);
											//Sprawdza ktory przycisk jest wlaczony
		if(bit_is_clear(PIND,2)){

		}
		if(bit_is_clear(PIND,3)){			//Przycisk up
			i--;
			if(i < 0){
				i = 0;
			}
		}
		sbi(PORTD, 0);						//Ustawia wyjœcie stan wysoki


		cbi(PORTD, 1);						//Ustawia wyjœcie stan niski
		_delay_ms(5);
											//Sprawdza ktory przycisk jest wlaczony
		if(bit_is_clear(PIND,2)){
			ok = 1;
		}
		if(bit_is_clear(PIND,3)){
			i++;							//Przycisk down
			if(i > 5){
				i = 5;
			}
		}
		sbi(PORTD, 1);							//Ustawia wyjœcie stan wysoki

		if(i == 1){								//Sprawdza wybrana opcje

			LCD_Clear();
			sprintf(text , "1. Info" );			//Wyswietla na wyswietlaczu napis "1. Info"
			LCD_GoTo(0,0);
			LCD_WriteText(text);
			_delay_ms(10);

			if(ok == 1){						//Uruchamia dana funkcje po kliknieciu przycisku ok
				info();							//Uruchamia funkcje info()
				ok = 0;							//Wyzerowanie zmiennej ok
			}
		}
		if(i == 2){								//Sprawdza wybrana opcje

			LCD_Clear();
			sprintf(text , "2. Liczby" );		//Wyswietla na wyswietlaczu napis "2. Liczby"
			LCD_GoTo(0,0);
			LCD_WriteText(text);
			_delay_ms(10);

			if(ok == 1){						//Uruchamia dana funkcje po kliknieciu przycisku ok
				liczby();						//Uruchamia funkcje liczby()
				ok = 0;							//Wyzerowanie zmiennej ok
			}
		}
		if(i == 3){								//Sprawdza wybrana opcje

			LCD_Clear();
			sprintf(text , "3. Stoper" );		//Wyswietla na wyswietlaczu napis "3. Stoper"
			LCD_GoTo(0,0);
			LCD_WriteText(text);
			_delay_ms(10);

			if(ok == 1){						//Uruchamia dana funkcje po kliknieciu przycisku ok
				stoper();						//Uruchamia funkcje stoper()
				ok = 0;							//Wyzerowanie zmiennej ok
			}
		}
		if(i == 4){								//Sprawdza wybrana opcje

			LCD_Clear();
			sprintf(text , "4. Zegar" );		//Wyswietla na wyswietlaczu napis  "4. Zegar"
			LCD_GoTo(0,0);
			LCD_WriteText(text);
			_delay_ms(10);

			if(ok == 1){						//Uruchamia dana funkcje po kliknieciu przycisku ok
				zegar();						//Uruchamia funkcje zegar()
				ok = 0;							//Wyzerowanie zmiennej ok
			}
		}
		if(i == 5){

			LCD_Clear();
			sprintf(text , "5. Miernik" );		//Wyswietla na wyswietlaczu napis "5. Miernik"
			LCD_GoTo(0,0);
			LCD_WriteText(text);
			_delay_ms(10);

			if(ok == 1){						//Uruchamia dana funkcje po kliknieciu przycisku ok
				miernik();						//Uruchamia funkcje miernik()
				ok = 0;							//Wyzerowanie zmiennej ok
			}
		}
	}
}


int main(){

	LCD_Initalize();		//Inicjalizacja LCD
	LCD_Home();
	LCD_Clear();			//Czyszczenie wyswietlacza





	sbi(DDRA,4);			//Ustawia stanu wysokiego na DDRA 4
	sbi(DDRA,5);			//Ustawia stanu wysokiego na DDRA 5
	sbi(DDRA,6);			//Ustawia stanu wysokiego na DDRA 6

	info();					//Uruchamia funkcje INFO

	menu();					//Uruchamia funkcje MENU

	return 0;				//Konczy program

}

//Funkcja uruchamiana z przerwaniem po przepelnieniu licznika w timer1
ISR(TIMER1_COMPA_vect) {
	i++;					//Zwiekszenie zakresu o 1 co dana ilosc sekund
}
