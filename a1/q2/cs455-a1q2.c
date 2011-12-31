/*
*   Student Name:   Craig Tataryn
*   Student#:       6601643
*   User ID:        umtatar4
*   Assignment #:   1
*   Question #:     2
*   Date:           Oct 16 2004
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/signal.h>
#include <avr/eeprom.h>
/* globals */
typedef enum {false,true} boolean;
const unsigned int CLK_DIV = 8;
const unsigned int OVR_MS = 524;//number of milliseconds between overflows
volatile unsigned int _ovrCtr = 0;
unsigned char _eeAddr = 0;
boolean _btn2up = false;
boolean _btn3up = false;

void eeWrite(unsigned char data_, unsigned int  epromAddr_)
{
    while(EECR & (_BV(EEWE)))
    ; //loop until the eprom is ready

    cli();//no interrupts should happen when value is being written!
    EEAR = epromAddr_;  //set the address to write the byte to (0..255)
    EEDR = data_;       //set the data which will be written

    EECR |= _BV(EEMWE); //set the EEPROM master write bit, this enables a write to EEPROM when the EEPROM write bit (EEWE) is set

    EECR |= _BV(EEWE);  //start the write of the data in EEDR to the address in EEPROM equal to the value in EEAR
    //in 4 clock ticks our value will be written and both EEMWE and EEWE will be flipped back to 0 in EECR
    sei();
}

void eeWriteXBytes(unsigned char* val_,unsigned int startAddr_,unsigned int numBytes_) {
    unsigned char i;
    for (i=0;i<numBytes_;i++) {
    //write b[i] to EEPROM
        eeWrite(val_[i],startAddr_ + i);
    }
}

INTERRUPT (SIG_INTERRUPT0) {
    _btn2up = !_btn2up;
    if (_btn2up) {
	unsigned long currMS = 0;
	cli();  //turn off interrupts while reading the clock since behind the scenes we are reading the high/low bytes of the register
		//and we don't want the timer updating when this happens
	unsigned int timerVal = TCNT1;
	sei(); //enable interrupts so our timer can continue;
	currMS = (_ovrCtr * OVR_MS) + (timerVal/(CLK_DIV * 1000)); 
	//cast a pointer to an 8bit structure so we can get at the individule bytes of our 32bit number
	unsigned char *bytes = (unsigned char*)&currMS;
	eeWriteXBytes(bytes,_eeAddr,4);
	_eeAddr +=5; //[XX][XX][XX][XX][FF][XX]........
	PORTB = ~_BV(2); //turn LED2 on when button pressed
    } else {
	PORTB = 0xFF;//turn the LEDs off when button is released
    }
}

INTERRUPT (SIG_INTERRUPT1) {
    _btn3up = ! _btn3up;
    if (_btn3up) {
	TCNT1 = 0x0000; //reset the timer value
	_eeAddr = 0;
	_ovrCtr = 0;
	_btn2up = false;
	PORTB = ~_BV(3); //turn LED3 on when button pressed
    } else {
	PORTB = 0xFF;
    }
}

SIGNAL (SIG_OVERFLOW1) {
    _ovrCtr++;
}

int main(void) {
    DDRB = 0xff; //Direction of PORTB set to OUT
    PORTB = 0xff; //Turn all LEDs off

    cli(); //disable interrupts until we have finished setting the ones we want to fire   
    /* 
     * GICR - General Interrupt Control Register (bits indicate which interrupts to enable)
     * MCUCR - Control Register
     *	-> bits ISC00 = 1, ISC01 = 0 -	indidcates we want an interrupt to fire when a logical change 
     *					happens with our button (for button connected to PORTD pin 2)
     *  -> bits ISC10 = 1, ISC11 = 0 -	indicates we want an interrupt to fire when a logical change
     *					happens with our button (for button connected to PORTD pin3)
     */
    GICR |= _BV(INT0) | _BV(INT1);
    MCUCR |= _BV(ISC00); 
    MCUCR |= _BV(ISC10); 

    /*
     * 16bit timer, 1Mhz clock
     * timer overflows every 2^16 ticks = 65536/sec == 65.536/ms
     * set presaler to /8, overflow happens every 524.288.  We do this 
     * to gain more precision (i.e. .288ms is better than .536ms)
     */ 
    TIMSK |=  _BV(TOIE1); //setup 16bit timer 1
    TCCR1B |= _BV(CS11); //prescaler to /8 to get 524.288ms/ovr
    sei();//Enable Interrupts

    for (;;) {
    }
	
}
