#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>

#ifndef _BV
#define _BV(bit)				(1<<(bit))
#endif
#ifndef sbi
#define sbi(reg,bit)		reg |= (_BV(bit))
#endif

#ifndef cbi
#define cbi(reg,bit)		reg &= ~(_BV(bit))
#endif

int main() {

/*		ZADANIE 1
###############################
#######  Miganie dioda  #######
###############################
								*/
/*	// 1 STPSOB
	sbi(DDRD,PD0);
	sbi(DDRD,PD1);
	sbi(DDRD,PD2);
	sbi(DDRD,PD3);
	sbi(DDRD,PD4);
	sbi(DDRD,PD5);
	sbi(DDRD,PD6);
	sbi(DDRD,PD7);

	while (1) {
		sbi(PORTD,PD0);
		sbi(PORTD,PD1);
		sbi(PORTD,PD2);
		sbi(PORTD,PD3);
		sbi(PORTD,PD4);
		sbi(PORTD,PD5);
		sbi(PORTD,PD6);
		sbi(PORTD,PD7);

		_delay_ms(500);

		cbi(PORTD,PD0);
		cbi(PORTD,PD1);
		cbi(PORTD,PD2);
		cbi(PORTD,PD3);
		cbi(PORTD,PD4);
		cbi(PORTD,PD5);
		cbi(PORTD,PD6);
		cbi(PORTD,PD7);

		_delay_ms(500);
	}
*/


/*	// 2 SPOSOB
	DDRD = 0xFF;
	while(1){
		PORTD = 0xFF;
		_delay_ms(500);
		PORTD = 0x00;
		_delay_ms(500);
	 }
*/

	// 3 SPOSOB
/*	for(int i = 0; i<8; i++){
		DDRD |= (1<<i); //w??czenie pin?w jako wyj?cie
	}

	while(1){

		for(int i = 0; i<8; i++){
			PORTD &= ~(1<<i);
		}
	    _delay_ms(500);

	    for(int i = 0; i<8; i++){
	    PORTD |= (1<<i);
	    }
	    _delay_ms(500);
	}
*/




	/*		ZADANIE 2
	###############################
	######  Miganie dioda II ######
	###############################
									*/

/*	for(int i = 0; i<8; i++){
		sbi(DDRD,i);
	}
	while (1) {
		for(int i = 0; i<8; i++){
			sbi(PORTD,i);
		}
		_delay_ms(1000);

		for(int i = 0; i<8; i++){
			cbi(PORTD,i);
		}

			_delay_ms(1000);
		}

*/






	/*		ZADANIE 3
	###############################
	######  Biegajaca dioda #######
	###############################
									*/
/*	for(int i = 0; i<8; i++){
		sbi(DDRD,i);
	}

	while (1) {
		for(int i = 0; i<8; i++){

			sbi(PORTD,i);
			_delay_ms(500);
			cbi(PORTD,i);
			_delay_ms(200);
		}
		for(int i = 6; i>0; i--){

			sbi(PORTD,i);
			_delay_ms(500);
			cbi(PORTD,i);
			_delay_ms(200);
		}

	}
*/




	/*		ZADANIE 4
	###############################
	########## Przycisk  ##########
	###############################
									*/



/*	cbi(DDRC,PC0);
	sbi(PORTC,PC0);

	DDRD = 0xFF;


	while(1){
		if(bit_is_clear(PINC,PC0)){
			PORTD = 0xFF;
		}
		else{
			PORTD = 0x00;
		}
		_delay_ms(500);
	}
*/



	/*		ZADANIE 5
		#################################
		#####  Magiczne przyciski   #####
		#################################
										*/
/*
	cbi(DDRC,PC0);
	sbi(PORTC,PC0);
	cbi(DDRC,PC1);
	sbi(PORTC,PC1);

	for(int i = 0; i<8; i++){
		sbi(DDRD,i);
	}

	while (1) {
		for(int i = 0; i<8;){
			if(bit_is_clear(PINC,PC1)){
				sbi(PORTD,i);
				_delay_ms(500);
				cbi(PORTD,i);
				_delay_ms(200);
				i++;
			}
			else if(bit_is_clear(PINC,PC0)){
					PORTD = 0xFF;
					_delay_ms(200);
					}
				else{
					PORTD = 0x00;
					_delay_ms(200);
				}

		}
		for(int i = 6; i>0;){
			if(bit_is_clear(PINC,PC1)){
				sbi(PORTD,i);
				_delay_ms(500);
				cbi(PORTD,i);
				_delay_ms(200);
				i--;
			}
			else if(bit_is_clear(PINC,PC0)){
					PORTD = 0xFF;
					_delay_ms(200);
				}
				else{
					PORTD = 0x00;
					_delay_ms(200);
				}
		}

	}
*/

}
