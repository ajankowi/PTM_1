#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/sfr_defs.h>
#include <math.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "HD44780.h"

#ifndef _BV
#define _BV(bit)				(1<<(bit))
#endif
#ifndef sbi
#define sbi(reg,bit)		reg |= (_BV(bit))
#endif

#ifndef cbi
#define cbi(reg,bit)		reg &= ~(_BV(bit))
#endif


//Funkcja zamienia char na int
uint16_t charToInt(char c){
	uint16_t num = 0;


	num = c - '0';

	return num;
}

//Funkcja zwraca wartosc 1 liczby
uint16_t policz1(char tablica[]){

	uint16_t num = 0;
	uint16_t liczba = 0;		//Zmienne pomocnicze
	char Tab_liczb[20];

	while (tablica[num] != '+' && tablica[num] != '-'){		//Petla sprawdza kiedy jest + lub -
		Tab_liczb[num] = tablica[num];						//Przepisanie wartosci
		num++;
	}

	liczba = atoi(Tab_liczb);								//char to int



	return liczba;





}

//Funkcja zwraca wartosc 2 liczby
uint16_t policz2(char tablica[]){

	uint16_t a = 0;
	uint16_t b = 0;
	uint16_t c = 0;				//Zmienne pomocnicze
	uint16_t liczba = 0;
	char Tab_liczb[20];

	while (tablica[a] != '+' && tablica[a] != '-'){		//Sprawdza kiedy jest + lub -
		a++;

	}
	b = a+1;

	while (tablica[b] != '='){							//Sprawdza kiedy jest =
		Tab_liczb[c] = tablica[b];						//Przypisanie wartosci do tablicy
		b++;
		c++;
	}
	Tab_liczb[c] = ' ';									//Usuwa zbedny znak

	liczba = atoi(Tab_liczb);							//Char to int

	return  liczba;										//Zwraca wartosc

}

//Funkcja zwraca wynik obliczen
uint16_t obliczenia(char tablica[], uint16_t liczba1, uint16_t liczba2){

	uint16_t wynik = 0;
	uint16_t mun = 0;

	while (tablica[mun] != '='){
		if(tablica[mun] == '+'){
			return wynik = liczba1 + liczba2;
		}
		if(tablica[mun] == '-'){
			return wynik = liczba1 - liczba2;
		}

		mun++;
	}

	return wynik;
}










int main() {

	 	//	ZADANIE NA 3


		/*******************
		 ****   ZEGAR   ****
		 *******************
		 * */



	LCD_Initalize();
	LCD_Home();
	char text[20];
	char sek[5];
	char min[5];
	char godz[5];

	uint16_t s = 54;
	uint16_t m = 59;
	uint16_t h = 23;

	//sbi(PORTD, PD6); //Ustawia pull Up
	//sbi(PORTC, PC0);

	while (1) {


		s++;			//Dodawanie 1sek po przej?ciu p?tli
		if(s == 60){
			m ++;		//Dodaje 1min po dojsciu do 60sek
			s = 0;		//Wyzerowuje sekundy
		}
		if(m == 60){
			h ++;		//Dodaje 1h po dojsciu do 60min
			m = 0;		//Wyzerowuje minuty
		}
		if(h == 24){
			h = 0 ;		//Wyzerowuje godziny
		}




		if(s < 10){
			sprintf(sek, ":0%d", s);
		}
		else{
			sprintf(sek, ":%d", s);
		}

		if(m < 10){
			sprintf(min, ":0%d", m);
		}
		else{
			sprintf(min, ":%d", m);
				}
		if(h < 10){
			sprintf(godz, "0%d", h);
		}
		else{
			sprintf(godz, "%d", h);
		}

		sprintf(text, "Clock");

		LCD_Clear();

		LCD_GoTo(12, 0);
		LCD_WriteText(sek);			//Wyswietla sekundy

		LCD_GoTo(9, 0);
		LCD_WriteText(min);			//Wyswietla minuty

		LCD_GoTo(7, 0);
		LCD_WriteText(godz);		//Wyswietla godziny

		LCD_GoTo(0, 0);
		LCD_WriteText(text);		//Wyswietla napis "Clock"

		LCD_GoTo(0, 1);
		sprintf(text, "252919, 252870");
		LCD_WriteText(text);

		_delay_ms(1000);
	}


			//	ZADANIE NA 4

			/********************************
			 * ******************************
			 * ********  Klawiatura  ********
			 * ******************************
			 * ******************************
			 * */
/*

	LCD_Initalize();
	LCD_Home();
	char napis[20] = "789-456+123C*0#= ";
	char text[20];

	uint16_t i = 16;
	uint16_t a = 0;


	sbi(PORTD, PD6); //Ustawia pull Up
	sbi(PORTC, PC0);

	//Moje

	sbi(DDRC, 2);

	cbi(DDRC, 4);
	sbi(PORTC, 4);

	sbi(DDRC, 6);
	cbi(PORTC, 6);


	//Testy
	sbi(DDRD, 0);	//Wyj?cie stan wysoki porty PD0-PD3
	sbi(PORTD, 0);
	sbi(DDRD, 1);
	sbi(PORTD, 1);
	sbi(DDRD, 2);
	sbi(PORTD, 2);
	sbi(DDRD, 3);
	sbi(PORTD, 3);


	//Ustawienie PD4-PD7 jako PULL-UP
	cbi(DDRD, 4);
	sbi(PORTD, 4);
	cbi(DDRD, 5);
	sbi(PORTD, 5);
	cbi(DDRD, 6);
	sbi(PORTD, 6);
	cbi(DDRD, 7);
	sbi(PORTD, 7);





	while(1){
		_delay_ms(100);

		cbi(PORTD, 0);	//Ustawia wyj?cie stan niski
		_delay_ms(5);

		if(bit_is_clear(PIND,7)){
			i = 0;
		}
		if(bit_is_clear(PIND,6)){
			i = 1;
		}
		if(bit_is_clear(PIND,5)){
			i = 2;
		}
		if(bit_is_clear(PIND,4)){
			i = 3;
		}
		sbi(PORTD, 0);	//Ustawia wyj?cie stan wysoki



		cbi(PORTD, 1);	//Ustawia wyj?cie stan niski
		_delay_ms(5);

		if(bit_is_clear(PIND,7)){
			i = 4;
		}
		if(bit_is_clear(PIND,6)){
			i = 5;
		}
		if(bit_is_clear(PIND,5)){
			i = 6;
		}
		if(bit_is_clear(PIND,4)){
			i = 7;
		}
		sbi(PORTD, 1);	//Ustawia wyj?cie stan wysoki



		cbi(PORTD, 2);	//Ustawia wyj?cie stan niski
		_delay_ms(5);

		if(bit_is_clear(PIND,7)){
			i = 8;
		}
		if(bit_is_clear(PIND,6)){
			i = 9;
		}
		if(bit_is_clear(PIND,5)){
			i = 10;
		}
		if(bit_is_clear(PIND,4)){
			i = 11;
		}
		sbi(PORTD, 2);	//Ustawia wyj?cie stan wysoki



		cbi(PORTD, 3);	//Ustawia wyj?cie stan niski
		_delay_ms(5);

		if(bit_is_clear(PIND,7)){
			i = 12;
		}
		if(bit_is_clear(PIND,6)){
			i = 13;
		}
		if(bit_is_clear(PIND,5)){
			i = 14;
		}
		if(bit_is_clear(PIND,4)){
			i = 15;
		}
		sbi(PORTD, 3);	//Ustawia wyj?cie stan wysoki

		if(i == 16){
			LCD_Clear();
			LCD_GoTo(0, 0);
			sprintf(text, "-1" );

			LCD_WriteText(text);
			_delay_ms(20);
		}

		if(i != 16){
			LCD_Clear();
			LCD_GoTo(0, 1);


			sprintf(text, "Wcisnieto: %c",napis[i] );

			LCD_WriteText(text);

			_delay_ms(20);
			i = 16;
			a++;
		}
	}


*/



























	//	ZADANIE NA 5

	/********************************
	 * ******************************
	 * ********  Kalkulator  ********
	 * ******************************
	 * ******************************
	 * */


/*


	LCD_Initalize();			//Tablice przechowujace wartosci
		LCD_Home();
		char napis[20] = "789-456+123C*0#= ";
		char text[20];
		char wynik[20];

								//Zmienne pomocnicze
		uint16_t i = 16;
		uint16_t a = 0;
		uint16_t liczba1 = 0;
		uint16_t liczba2 = 0;
		uint16_t odpowiedz = 0;
		uint16_t kasuj = 0;


		sbi(PORTD, PD6); //Ustawia pull Up
		sbi(PORTC, PC0);



		sbi(DDRC, 2);

		cbi(DDRC, 4);
		sbi(PORTC, 4);

		sbi(DDRC, 6);
		cbi(PORTC, 6);



		sbi(DDRD, 0);	//Wyj?cie stan wysoki porty PD0-PD3
		sbi(PORTD, 0);
		sbi(DDRD, 1);
		sbi(PORTD, 1);
		sbi(DDRD, 2);
		sbi(PORTD, 2);
		sbi(DDRD, 3);
		sbi(PORTD, 3);


		//Ustawienie PD4-PD7 jako PULL-UP
		cbi(DDRD, 4);
		sbi(PORTD, 4);
		cbi(DDRD, 5);
		sbi(PORTD, 5);
		cbi(DDRD, 6);
		sbi(PORTD, 6);
		cbi(DDRD, 7);
		sbi(PORTD, 7);





		while(1){
			_delay_ms(100);

			cbi(PORTD, 0);				//Ustawia wyj?cie stan niski
			_delay_ms(5);
										//Sprawdza ktory przycisk jest wlaczony
			if(bit_is_clear(PIND,7)){
				i = 0;
			}
			if(bit_is_clear(PIND,6)){
				i = 1;
			}
			if(bit_is_clear(PIND,5)){
				i = 2;
			}
			if(bit_is_clear(PIND,4)){
				i = 3;
			}
			sbi(PORTD, 0);				//Ustawia wyj?cie stan wysoki



			cbi(PORTD, 1);				//Ustawia wyj?cie stan niski
			_delay_ms(5);
										//Sprawdza ktory przycisk jest wlaczony
			if(bit_is_clear(PIND,7)){
				i = 4;
			}
			if(bit_is_clear(PIND,6)){
				i = 5;
			}
			if(bit_is_clear(PIND,5)){
				i = 6;
			}
			if(bit_is_clear(PIND,4)){
				i = 7;
			}
			sbi(PORTD, 1);				//Ustawia wyj?cie stan wysoki



			cbi(PORTD, 2);				//Ustawia wyj?cie stan niski
			_delay_ms(5);
										//Sprawdza ktory przycisk jest wlaczony
			if(bit_is_clear(PIND,7)){
				i = 8;
			}
			if(bit_is_clear(PIND,6)){
				i = 9;
			}
			if(bit_is_clear(PIND,5)){
				i = 10;
			}
			if(bit_is_clear(PIND,4)){
				i = 11;
			}
			sbi(PORTD, 2);				//Ustawia wyj?cie stan wysoki



			cbi(PORTD, 3);				//Ustawia wyj?cie stan niski
			_delay_ms(5);
										//Sprawdza ktory przycisk jest wlaczony
			if(bit_is_clear(PIND,7)){
				i = 12;
			}
			if(bit_is_clear(PIND,6)){
				i = 13;
			}
			if(bit_is_clear(PIND,5)){
				i = 14;
			}
			if(bit_is_clear(PIND,4)){
				i = 15;
			}
			sbi(PORTD, 3);				//Ustawia wyj?cie stan wysoki




			if(i != 16){				//Sprawdza czy jest wcisniety jakikolwiek przycisk


				if(kasuj < 2){
					LCD_Clear();
					LCD_GoTo(0, 0);
					wynik[a] = napis[i];		//Zapisuje wcisniete przyciski w tablicy

					LCD_WriteText(wynik);		//Wyswietla zapisane przyciski

					_delay_ms(20);

				}


				if(wynik[a] == '='){			//Sprawdza czy zostal wcisniety znak =

					kasuj ++;					//Zmienna umozliwiajaca kasowanie
					if(kasuj < 2){
						liczba1 = policz1(wynik);			//Pobiera 1 liczbe
						liczba2 = policz2(wynik);			//Pobiera 2 liczbe
						odpowiedz = obliczenia(wynik,liczba1,liczba2);		//Zwraca wynik rowannia

						LCD_GoTo(0, 1);
						sprintf(text, "Wynik: %d",odpowiedz);				//Wyswietla wynik
						LCD_WriteText(text);
						_delay_ms(20);
					}
					else{
						LCD_Clear();							//Kasuje cala pamiec tablic
						kasuj = 0;
						for(uint16_t x = 0; x < 20; x++){
							text[x] = ' ';
							wynik[x] = ' ';
							a = -1;
						}
					}

				}

				i = 16;
				a++;				//Kolejny elment tablicy

			}

		} */

}
