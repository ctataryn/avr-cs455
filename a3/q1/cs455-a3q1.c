/*
*   Student Name:   Craig Tataryn
*   Student#:       6601643
*   User ID:        umtatar4
*   Assignment #:   3
*   Question #:     1
*   Date:           Nov 12 2004
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
#define MAX_OVR 195

//Fast PWM/Clear OC2 on compare match and set OC2 at TOP, divide timer by 1024
#define PWM_MODE _BV(WGM20) | _BV(WGM21) | _BV(COM21) | _BV(CS21) | _BV(CS20)
//just set timer0 to normal operation
#define TIMER0_SETTINGS _BV(CS00)
/* converts KMH to PID input */
#define VALUE_SCALE 10
/* maximum duty cycle value */
#define MAX_DC 0xFF
/* fastest we can go */
#define MAX_OUTPUT 0x3FF
#define NUM_POT_SAMPLES 16
static unsigned short int  potval[NUM_POT_SAMPLES];
static unsigned char pot_index = 0;
static unsigned char pot_samples = 0;
static unsigned long avg_pot = 0;
/* VREF is 4.95 * 1000 */
uint16_t VREF = 4950;

/* globals */
typedef enum {false=0,true=-1} boolean;
/* use as indexer into debounce array */
typedef enum {
    SW1,
    SW2,
    SW3,
    SW4,
    SW5
} Button;

/* states */
typedef enum {
    NONE,
    SW1_ON,
    SW1_OFF,
    SW2_ON,
    SW2_OFF,
    SW3_ON,
    SW3_OFF,
    SW4_ON,
    SW4_OFF,
    SW5_ON,
    SW5_OFF
} State;

volatile State m_currState = NONE;
volatile State m_prevState = NONE;


volatile unsigned int m_ovrCtr = 0;
/* Button debounce values */
unsigned char m_btnDBval[5] = {DB_OFF_LIMIT,DB_OFF_LIMIT,DB_OFF_LIMIT,DB_OFF_LIMIT,DB_OFF_LIMIT};
/* On/Off status of the buttons */
boolean m_btnState[5] = {false,false,false,false,false};
/* Pins on the Port we should test for button presses */
unsigned char m_btnPin[5] = {PINC1,PINC2,PINC3,PINC4,PINC5};

volatile unsigned char m_duty_cycle = 0x00;

//Too many globals as it is,grouping all PID vars into a struct
typedef struct
{
    /* P */
    unsigned int pGain; 
    /* I */
    int iState, iGain, iGainFactor; 
    unsigned int iMax, iMin;
    /* D */
    unsigned int dGain;
    int dState;

    unsigned int desiredState;
    unsigned int currentState;
} PID;

volatile PID m_pid;

SIGNAL (SIG_ADC) {
    unsigned char low = ADCL;
    unsigned char high = ADCH;
    unsigned short int ADCVal = 0;

    ADCVal = (high<<8) | low; 

    //if (m_currState != SW5_ON) {
    //}
    avg_pot -= potval[pot_index];
    potval[pot_index] = ADCVal;
    avg_pot += potval[pot_index];
    if ( pot_samples < NUM_POT_SAMPLES )
	pot_samples++;

    m_pid.currentState = avg_pot/NUM_POT_SAMPLES;

    pot_index = (pot_index+1) % NUM_POT_SAMPLES;
   /* 
    m_pid.currentState = ADCVal;
    low = ADCVal/10;
*/
}

INTERRUPT (SIG_OVERFLOW0) {
    m_ovrCtr++;
}

INTERRUPT(SIG_OVERFLOW2)
{
  OCR2 = m_duty_cycle;
}

void setupTimer0() {
  // set timer count rate and go
  TCCR0 = TIMER0_SETTINGS;
  TIMSK |= _BV(TOIE0);

}

void setupTimer2() {
  OCR2 = m_duty_cycle;
  TCCR2 = PWM_MODE;
  TIMSK |= _BV(TOIE2);
}

void setupADC() {
    /* enable ADC, enable autotrigger conversion, enable ADC interrupt */
    ADCSRA |= _BV(ADEN) | _BV(ADATE) | _BV(ADIE);

    /* have the ADC trigger when Timter 0 overflows */
    SFIOR |= _BV(ADTS2); 
}

void setupPID() {

    m_pid.desiredState = 0;
    m_pid.currentState = 0;
    m_pid.pGain = 1;
    m_pid.iState = 0;
    m_pid.iGain = 1;
    m_pid.iGainFactor = 100;
    m_pid.iMax = MAX_OUTPUT;
    m_pid.iMin  = 0x00;
    m_pid.dGain = 50;
    m_pid.dState  = 0;

    
}

void disableJTAG() {
    MCUCSR |= _BV(JTD);
    MCUCSR |= _BV(JTD);
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
void debounceButtonX(Button btn_) {
    if (~PINC & _BV(m_btnPin[btn_])) {
        if (++m_btnDBval[btn_] > DB_ON_MAX) {
            m_btnDBval[btn_] = DB_ON_MAX;
        }
    } else if (m_btnDBval[btn_]!=DB_OFF_MIN) {
        m_btnDBval[btn_]--;
    }
}

void updateButtonState() {
    int i;
    for (i=SW1;i<=SW5;i++) {
	debounceButtonX(i);
	m_btnState[i] = getButtonState(m_btnDBval[i], m_btnState[i]);
    }

}

/* check to see if it's time to check our button states */
boolean checkState() {
    if (m_ovrCtr >= MAX_OVR) {
        m_ovrCtr = 0;
        return true;
    } else {
        return false;
    }
}

long updatePID(int error, unsigned int currentState) {
    long pTerm;
    long iTerm;
    long dTerm;

    m_pid.iState += error;
    if (m_pid.iState > m_pid.iMax)
	m_pid.iState = m_pid.iMax;
    else if (m_pid.iState < m_pid.iMin)
	m_pid.iState = m_pid.iMin;
    
    pTerm = m_pid.pGain * error;
    iTerm = (m_pid.iGain * m_pid.iState * m_pid.iGainFactor)/m_pid.iGainFactor;
    dTerm = m_pid.dGain * (m_pid.dState - m_pid.currentState);

    m_pid.dState = m_pid.currentState;
    return pTerm + iTerm + dTerm;

}

void processState() {
    long output;
    int desiredState = m_pid.desiredState;
    
    updateButtonState();
   
    //Added an extra button to the mix, if you press SW5 it shows you the 
    //desired speed
    if (m_currState != SW5_ON) {
	//PORTB = ~(m_pid.currentState/10);
	PORTB = ~((unsigned char) m_duty_cycle);
	//PORTB = (unsigned char) ~sizeof(long);
    }
    switch (m_currState) {
    
	case NONE:
	    m_prevState = m_currState;
	    if (m_btnState[SW1]) {
		m_currState = SW1_ON;
	    } else if (m_btnState[SW2]) {
		m_currState = SW2_ON;
	    } else if (m_btnState[SW3]) {
		m_currState = SW3_ON;
	    } else if (m_btnState[SW4]) {
		m_currState = SW4_ON;
	    } else if (m_btnState[SW5]) {
		m_currState = SW5_ON;
	    }
	    break;
        case SW1_ON:
            if (!m_btnState[SW1]) {
                m_currState = SW1_OFF;
            } else if (m_prevState != SW1_ON) {
                m_prevState = SW1_ON;
                desiredState += (-1 * VALUE_SCALE);
            }
            break;
        case SW1_OFF:
            m_currState = NONE;
            break;
	case SW2_ON:
	    if (!m_btnState[SW2]) {
		m_currState = SW2_OFF;
	    } else if (m_prevState != SW2_ON) {
		m_prevState = SW2_ON;
		desiredState += (1 * VALUE_SCALE);
	    }
	    break;
	case SW2_OFF:
	    m_currState = NONE;
	    break;
        case SW3_ON:
            if (!m_btnState[SW3]) {
                m_currState = SW3_OFF;
            } else if (m_prevState != SW3_ON) {
                m_prevState = SW3_ON;
                desiredState += (-5 * VALUE_SCALE);
            }
            break;
        case SW3_OFF:
            m_currState = NONE;
            break;
        case SW4_ON:
            if (!m_btnState[SW4]) {
                m_currState = SW4_OFF;
            } else if (m_prevState != SW4_ON) {
                m_prevState = SW4_ON;
                desiredState += (5 * VALUE_SCALE);
            }
            break;
        case SW4_OFF:
            m_currState = NONE;
            break;
	//if you press SW5 it will display the desiredState
	case SW5_ON:
            if (!m_btnState[SW5]) {
                m_currState = SW5_OFF;
            } else if (m_prevState != SW5_ON) {
		m_prevState = SW5_ON;
		PORTB = ~(desiredState/10); 
            }
            break;
	case SW5_OFF:
	    m_currState = NONE;
	    break;
    }

    if (desiredState > MAX_OUTPUT)
	desiredState = MAX_OUTPUT;

    if (desiredState < 0)
	desiredState = 0;

    m_pid.desiredState = desiredState;
    output = updatePID(m_pid.desiredState - m_pid.currentState, m_pid.currentState);

    //m_duty_cycle = MAX_DC * (output / MAX_OUTPUT);
    if (output < 0x00)
	output = 0x00;
    else if (output > MAX_OUTPUT)
	output = MAX_OUTPUT;

    m_duty_cycle = ((MAX_DC*100L) * (output*100L / MAX_OUTPUT *100L))/100L;
    //m_duty_cycle = output;
    

}

int main(void) {
    DDRB = 0xff;    //LEDs
    PORTB = 0xff;
    DDRA = 0x00;    //Direction of PORTA set to IN for ADCVal
    DDRD = 0xff;    //Direction of PORTB set to OUT
    PORTD &= ~_BV(7); //Start off, when CT hits it will toggle


    //DDRD = 0x00;//Direction of PORTD is set to in for our switches

    cli(); //disable interrupts until we have finished setting the ones we want to fire   

    disableJTAG();
    setupTimer0();
    setupTimer2();
    setupADC();
    setupPID();
    sei();
    for (;;) {
	if (checkState()) {
	    processState();
	}
    }
}
