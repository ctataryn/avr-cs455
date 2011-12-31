/*
*   Student Name:   Craig Tataryn
*   Student#:       6601643
*   User ID:        umtatar4
*   Assignment #:   1
*   Question #:     3
*   Date:           Oct 16 2004
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/signal.h>
#include <avr/eeprom.h>
/* globals */
typedef enum {false,true} boolean;
typedef enum {BTN0,BTN1} button;//use as indexer into debounce array
typedef enum {S_0,S_1,S_2,S_3,S_4,S_LED0_TGL,S_LED1_TGL,S_LED2_TGL,S_LED3_TGL,S_STROBE_LED1,S_ALL_LEDS_OFF} state;
const unsigned int CLK_DIV = 8;
const unsigned int OVR_MS = 2;//number of milliseconds between overflows
volatile unsigned int _ovrCtr = 0;
unsigned char _btnDBval[2] = {0,0}; //Button debounce values
boolean _btnState[2] = {false,false};
state _currentState = S_0;
unsigned int _btnTimeout[2] = {0,0};
unsigned int _btnTimer[2] = {0,0};

/*
unsigned int min(unsigned int x, unsigned int y) {
    return x<y ? x : y;
}
*/
SIGNAL (SIG_OVERFLOW0) {
    _ovrCtr++;

    _btnTimer[BTN0] += OVR_MS;
    if (_btnTimer[BTN0] > _btnTimeout[BTN0]) {
	_btnTimer[BTN0] = _btnTimeout[BTN0] + 1;
    }
    //_btnTimer[BTN0] = min(_btnTimer[BTN0],_btnTimeout[BTN0]);
    _btnTimer[BTN1] += OVR_MS;
    if (_btnTimer[BTN1] > _btnTimeout[BTN1]) {
        _btnTimer[BTN1] = _btnTimeout[BTN1] + 1;
    }
}

boolean getButtonState(char dbVal_, boolean currentButtonState_) {

    if (dbVal_ >= 10) {
        return true;
    } else if (dbVal_ <= 4) {
        return false;
    } else {
	return currentButtonState_;
    }

}
void updateButtonState() {
    if (~PIND & _BV(PIND0)) {
	//_btnDBval[BTN0] = min(++_btnDBval[BTN0],12);
	if (++_btnDBval[BTN0] > 12) {
	    _btnDBval[BTN0] = 12;
	}
    } else if (_btnDBval[BTN0]!=0) {
	_btnDBval[BTN0]--;
    }

    if (~PIND & _BV(PIND1)) {
        //_btnDBval[BTN0] = min(++_btnDBval[BTN0],12);
        if (++_btnDBval[BTN1] > 12) {
            _btnDBval[BTN1] = 12;
        }
    } else if (_btnDBval[BTN1]!=0) {
        _btnDBval[BTN1]--;
    }

    _btnState[BTN0] = getButtonState(_btnDBval[BTN0], _btnState[BTN0]);
    _btnState[BTN1] = getButtonState(_btnDBval[BTN1], _btnState[BTN1]);

}

/* check to see if it's time to check our button states */
boolean checkState() {
    updateButtonState();
    if (_ovrCtr >= 50) { //100/OVR_MS) {
        _ovrCtr = 0;
        return true;
    } else {
        return false;
    }
}

int main(void) {
    DDRB = 0xff; //Direction of PORTB set to OUT
    PORTB = 0xff; //Turn all LEDs off

    DDRD = 0x00;//Direction of PORTD is set to in for our switches

    cli(); //disable interrupts until we have finished setting the ones we want to fire   
    /*
     * 8bit timer, 1Mhz clock
     * timer overflows every 2^8 ticks = 256tks == .256ms/ovr
     * set presaler to /8, overflow happens every ~2ms.  
     * 
     */ 
    TIMSK |=  _BV(TOIE0); //setup 8bit timer 0
    TCCR0 |= _BV(CS01); //prescaler to /8 to get 2ms/ovr
    sei();//Enable Interrupts

    for (;;) {
	if (checkState()) {
	    //FSM *******************************
	    switch (_currentState) {
		case S_0:
		    //clear our timers
		    _btnTimeout[BTN0] = 0;
		    _btnTimer[BTN0] = 0;
		    _btnTimeout[BTN1] = 0;
		    _btnTimer[BTN1] = 0;
		    if (_btnState[BTN0]) {
			_btnTimeout[BTN0] = 2000;
			_btnTimer[BTN0] = 0;
			_currentState = S_1;
			break;
		    }
		    if (_btnState[BTN1]) {
			unsigned char leds = PORTB;
                        PORTB = leds ^ _BV(1); //flip LED 1	
			_currentState = S_4;
			_btnTimeout[BTN1] = 100;
                        _btnTimer[BTN1] = 0;
			break;
		    }
		    break;
		case S_4:
		    if (_btnState[BTN1] && _btnState[BTN0]) {
			_currentState = S_LED3_TGL;
			break;
		    }
		    
		    if (_btnState[BTN1]) {
                        if (_btnTimer[BTN1] >= _btnTimeout[BTN1]) { //SW0 held down for 2 secs
                            _btnTimeout[BTN1] = 1000;
                            _btnTimer[BTN1] = 0; //stop button timer
                            //probably have to switch to a state that waits for release
                            _currentState = S_STROBE_LED1;
                        }
                    } else {
			_currentState = S_0;
		    }
		    break;
		case S_1:
		    
		    if (_btnState[BTN0] && _btnState[BTN1]) {
			_currentState = S_LED3_TGL;
			break;
		    }
			
		    if (_btnState[BTN0]) { 
			if (_btnTimer[BTN0] >= _btnTimeout[BTN0]) { //SW0 held down for 2 secs
			    _btnTimeout[BTN0] = 0;
			    _btnTimer[BTN0] = 0; //stop button timer
			    PORTB = 0xFF; //turn all LEDs off
			    //probably have to switch to a state that waits for release
			    _currentState = S_ALL_LEDS_OFF;
			} 
		    } else { //SW0 
			_btnTimeout[BTN0] = 250;
			_btnTimer[BTN0] = 0;
			_currentState = S_2;
		    }
		    break;
		case S_2:
		    if (_btnState[BTN0]) {
			if (_btnTimer[BTN0] < _btnTimeout[BTN0]) { //SW0-SW0
			    _currentState = S_3;
			    _btnTimer[BTN0] = 0;
			    _btnTimeout[BTN0] = 2000;
			}
		    } else {// if (_btnTimer[BTN0] >= _btnTimeout[BTN0]) {
			unsigned char leds = PORTB;
			PORTB = leds ^ _BV(0); //flip LED 0
			_btnTimeout[BTN0] = 0;
			_btnTimer[BTN0] = 0;
			_currentState = S_LED0_TGL;
		    }
		    break;
		case S_3: 
                    if (_btnState[BTN0]) {
                        if (_btnTimer[BTN0] >= _btnTimeout[BTN0]) {
                            _btnTimeout[BTN0] = 0; //stop button timer
			    _btnTimer[BTN0] = 0;
                            PORTB = 0xff; //turn all LEDs off
                            //probably have to switch to a state that waits for release
                            _currentState = S_ALL_LEDS_OFF;
                        }
                    } else {
                        _btnTimeout[BTN0] = 0;
                        _btnTimer[BTN0] = 0;
			unsigned char leds = PORTB;
                        PORTB = leds ^ _BV(2); //flip LED 2
			_currentState = S_LED2_TGL;
                    }
		    break;
                case S_LED3_TGL:
		    {
		    unsigned char leds = PORTB;
                    PORTB = leds ^ _BV(3); //flip LED 3
		    _currentState = S_0;
		    }
		    break;
		case S_STROBE_LED1:
		    if (_btnState[BTN1]) {
			if (_btnTimer[BTN1] >= _btnTimeout[BTN1]) {
			    _btnTimer[BTN1] = 0;//reset the timer
			    unsigned char leds = PORTB;
			    PORTB = leds ^ _BV(1); //flip LED 1
			}  
		    } else {
			_btnTimer[BTN1] = 0;
			_btnTimeout[BTN1] = 0;
			_currentState = S_0;
		    }
		    break;
		case S_ALL_LEDS_OFF:
                case S_LED0_TGL:
                case S_LED1_TGL:
                case S_LED2_TGL:
		   if (!_btnState[BTN0]) {
			_currentState = S_0;
		    }
		    break;
	    }
	}
    }
	
}
