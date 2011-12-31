/*  
*   Student Name:   Craig Tataryn
*   Student#:	    6601643
*   User ID:	    umtatar4
*   Assignment #:   1
*   Question #:	    1
*   Date:	    Oct 16 2004
*/	
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/signal.h>

/* globals */
typedef enum {LEFT,RIGHT} direction;
unsigned char _currLED = 0x01; //start off with LED 00000001
direction _dir = RIGHT;

//SIGNAL (SIG_OUTPUT_COMPARE0) {
SIGNAL (SIG_OVERFLOW0) {
    TCNT0 = 12; //bump the counter up by 12 ticks to reduce our overflow frequency to 250.28ms
    switch (_currLED) {
	case 0x80: //10000000
	case 0x01: //00000001
	    _dir = !_dir;
    }

    switch (_dir) {
	case LEFT:
	    _currLED <<= 1;
	    break;
	case RIGHT:
	   _currLED >>= 1;
    }
    /* 
     * I know complementing is a wasted op but I feel for readability it is nicer to think of the value
     * within _currLED in terms of a 1 (in a given bit position) to mean LED ON.
     * PORT behaviour is opposite of this thus the need for complimenting.  I could have done this without
     * the complement by starting my _currLED at 0x7E and then testing for my direction change boundaries as:
     *     0x7E and 0x3F
    */
    PORTB = ~_currLED;
}

int main(void) {
    DDRB = 0xff;	//Direction of PORTB set to OUT
    PORTB = ~_currLED;   //set the first LED on, it will remain on until the interrupt shifts it
    
    cli();	    //Disable Interrupts until we have setup our registers properly

    //TIMSK |= _BV(OCIE0);	    //Enable output compare interrupt
    TIMSK |= _BV(TOIE0);
    TCCR0 |= _BV(CS00) | _BV(CS02); //set our timer clock to /1024 so our timer would overflow every 262.144ms if we were to enabled that interrupt
    //TCCR0 |= _BV(WGM01);	    //set output compare mode on counter to allow us to get closer to 250ms (i.e. get rid of 12.144ms)   
    /* 
     * set the output compare register for timer0 to 244 because 1 tick is almost 1ms with a 1024 divider on the timer. 
     * So instead of relying on an overflow every 256 ticks (262.144ms), we want to be alerted to an interrupt every 256-12==244ticks
     * This will force a CTC interrupt every 250.28ms without the need for keeping a count of overflows in our ISR.  We just change the LED
     * every time the ISR fires
     */
    //OCR0 = 0xF4; //244
    TCNT0 = 12; //instead of output compare, just bump the counter up by 12 ticks to reduce our overflow frequency to 250.28ms
    sei();//Enable Interrupts so our timer CTC interrupt fires

    for (;;) {
	//loop forever let our ISR do the work
    }
}
