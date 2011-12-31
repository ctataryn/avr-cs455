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

/* Debounce Consts */
#define DB_ON_LIMIT  2
#define DB_ON_MAX    2
#define DB_OFF_LIMIT 1
#define DB_OFF_MIN   1
/* Setup Overflow Reset */
#define MAX_OVR 50
/* Compare To values for timer */
#define OFF_CT_VAL 0xff
#define FULL_ON_CT_VAL 0x01
/* determines how much to speed/slow the fan */
#define STEP_VAL 4

/* globals */
typedef enum {false=0,true=-1} boolean;
/* use as indexer into debounce array */
typedef enum {SW1,SW2} button;
volatile unsigned int m_ovrCtr = 0;
/* Button debounce values */
unsigned char m_btnDBval[2] = {0,0};
/* On/Off status of the buttons */
boolean m_btnState[2] = {false,false};
/* Pins on the Port we should test for button presses */
unsigned char m_btnPin[2] = {PINC1,PINC2};
/*  indicates whether we are supplying power
 *  to the fan or whether the fan is off
*/
boolean fanOn = false;
/* current position in the timer where we set the fan on */
volatile int ON_CT_VAL = FULL_ON_CT_VAL;


/* PWM chart
 * When ON_CT_VAL is reached, CT unit toggles fan on.
 * When OFF_CT_VAL is reached, CT unit toggles fan off.

     Fan at 100% RPM    Fan at approx 40% RPM   Fan Off
         /                      /              /
        /                      /              /
If ON_CT_VAL is:              /       If ON_CT_VAL is:
 FULL_ON_CT_VAL          ON_CT_VAL      OFF_CT_VAL
       |                     |              |
       |---------------------+--------------|
       +<----- FAN OFF ----->+<-- FAN ON -->|
       |                     |              |
                                           \
        --------------Timer Counter---------X
                                           /
*/
SIGNAL (SIG_OUTPUT_COMPARE2) {
    if (ON_CT_VAL == OFF_CT_VAL) {
	TCCR2 &= ~_BV(COM20); //turn off PWM
	fanOn = false;
    } else {
	fanOn = !fanOn;
	TCCR2 |= _BV(COM20); //make sure PWM is enabled
	OCR2 = fanOn ? OFF_CT_VAL : ON_CT_VAL;
    }
}

INTERRUPT (SIG_OVERFLOW0) {
    m_ovrCtr++;

}

boolean getButtonState(char dbVal_, boolean currentButtonState_) {

    if (dbVal_ >= DB_ON_LIMIT) {
        return true;
    } else if (dbVal_ <= DB_OFF_LIMIT) {
        return false;
    } else {
	return currentButtonState_;
    }

}
void debounceButtonX(button btn_) {
    if (~PINC & _BV(m_btnPin[btn_])) {
        if (++m_btnDBval[btn_] > DB_ON_MAX) {
            m_btnDBval[btn_] = DB_ON_MAX;
        }
    } else if (m_btnDBval[btn_]!=DB_OFF_MIN) {
        m_btnDBval[btn_]--;
    }
}

void updateButtonState() {
    debounceButtonX(SW1);
    debounceButtonX(SW2);
    m_btnState[SW1] = getButtonState(m_btnDBval[SW1], m_btnState[SW1]);
    m_btnState[SW2] = getButtonState(m_btnDBval[SW2], m_btnState[SW2]);

}

/* check to see if it's time to check our button states */
boolean checkState() {
    
    updateButtonState();
    if (m_ovrCtr >= MAX_OVR) { 
        m_ovrCtr = 0;
        return true;
    } else {
        return false;
    }
}

int main(void) {
    DDRC = 0x00;//input
    DDRB = 0xff;
    PORTB = 0xff;
    DDRD = 0xff; //Direction of PORTB set to OUT
    PORTD &= ~_BV(7); //Star off, when CT hits it will toggle

    //DDRD = 0x00;//Direction of PORTD is set to in for our switches

    cli(); //disable interrupts until we have finished setting the ones we want to fire   
    /* Set CT mode on timer 2 */
    //TCCR2 = _BV(WGM21);
    /* toggle OC2 on compare match */
    TCCR2 |= _BV(COM20);
    TCCR2 |= _BV(CS20) | _BV(CS21) | _BV(CS22);
    /* Enable compare interrupts for timer 0 */
    TIMSK |=  _BV(OCIE2); 
    TIMSK |=  _BV(TOIE0); //setup 8bit timer 0
    TCCR0 |= _BV(CS01); //prescaler to /8 to get 2ms/ovr
    /*	set teh compare register, initially duty
     *	cycle is on par with overflow time
    */
    OCR2 = FULL_ON_CT_VAL;
    /* disable alternate function of PINC1 */
    TWCR &= ~_BV(TWEN);
    /*	alternate function of PINC2 is disabled
     *	by disabling JTAGEN fuse during programming
    */
    sei();

    for (;;) {
	/*  Show the speed of the fan on the LEDs
	 *  by lighting an LED to represent the
	 *  percentage of max RPM of the fan
	 *  The farther to the right  the LED is
	 *  the faster the fan is going
	*/
	PORTB = ~_BV(ON_CT_VAL/32);
	if (checkState()) {
	    if (m_btnState[SW1]) {
		if (ON_CT_VAL + STEP_VAL < OFF_CT_VAL) {
		    ON_CT_VAL += STEP_VAL;
		} else {
		    ON_CT_VAL = OFF_CT_VAL;
		}
	    } else if (m_btnState[SW2]) {
		if (ON_CT_VAL - STEP_VAL > FULL_ON_CT_VAL) {
		    ON_CT_VAL -= STEP_VAL;
		} else {
		    ON_CT_VAL = FULL_ON_CT_VAL;
		}
	    }
	}
    }
}
