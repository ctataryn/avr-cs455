/*
*   Student Name:   Craig Tataryn
*   Student#:       6601643
*   User ID:        umtatar4
*   Assignment #:   2
*   Question #:     2
*   Date:           Oct 30 2004
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/signal.h>

/* Setup Overflow Reset */
#define MAX_OVR 50

#define TEMP_OFFSET 20
/* VREF is 4.95 * 1000 */
uint16_t VREF = 4950;

/* globals */
typedef enum {false=0,true=-1} boolean;
/* use as indexer into debounce array */
volatile unsigned int m_ovrCtr = 0;

/*
 *  When an A to D conversion is ready for us calculate the temp using:
 *  T = (((Vin * 5) / VREF) - 1.375) / 0.0225
 *  Where: Vin = ADCVal/1024 * VREF
 *  
 *  Initially we scale the elements of the equation by 1000 for precision
 *  In the final part of the calculation we scale by 10,000 to bring the value
 *  0.0225 up to 225
 *  
 *  Keep getting wrong result for T though, not sure why :(
*/
SIGNAL (SIG_ADC) {
    unsigned char low = ADCL;
    unsigned char high = ADCH;
    unsigned int Vin = 0;
    unsigned short int ADCVal = 0;
    unsigned long T = 0;

    ADCVal = (high<<8) | low; 

    Vin = (ADCVal * 1000) * VREF;
   
    Vin = Vin / 1024000;// <== this seems to blank out Vin

    T = (((Vin * 5000) / VREF) - 1375);
   
    /* T is scaled by 1000, multiply by 10 to get it to 10,000 */
    T = (T * 10) / 225;

    T = T - TEMP_OFFSET;

    PORTB = ~(T & 0xFF);//show low byte
}

INTERRUPT (SIG_OVERFLOW1) {
    m_ovrCtr=0;
}

int main(void) {
    DDRB = 0xff;
    PORTB = 0xff;
    DDRA = 0x00;

    //DDRD = 0x00;//Direction of PORTD is set to in for our switches

    cli(); //disable interrupts until we have finished setting the ones we want to fire   

    TIMSK |=  _BV(TOIE1); //setup 16bit timer 1
    TCCR1B |= _BV(CS11); //prescaler to /1

    /* enable ADC, enable autotrigger conversion, enable ADC interrupt */
    ADCSRA |= _BV(ADEN) | _BV(ADATE) | _BV(ADIE);
    
    /* have the ADC trigger when Timter 1 overflows */
    SFIOR |= _BV(ADTS2) | _BV(ADTS1);

    sei();
    for (;;) {
    }
}
