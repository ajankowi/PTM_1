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
	sbi(DDRD, PD7);

	while (1) {
		sbi(PORTD, PD7)
		_delay_ms(500);
		cbi(PORTD, PD7);
		_delay_ms(500);

	}
}
