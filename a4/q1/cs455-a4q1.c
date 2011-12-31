/*
*   Student Name:   Craig Tataryn
*   Student#:       6601643
*   User ID:        umtatar4
*   Assignment #:   4
*   Question #:     1
*   Date:           Dec 5 2004
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/signal.h>
#include <avr/sleep.h>

/* Debounce Consts */
#define DB_ON_LIMIT  2
#define DB_ON_MAX    2
#define DB_OFF_LIMIT 1
#define DB_OFF_MIN   1
/* 200 gives us 50 times per second */
/* we count 5 of these for key presses to see if the user has held it for .25 of a second */
#define MAX_OVR 200

//Fast PWM/Clear OC2 on compare match and set OC2 at TOP, divide timer by 1024
#define PWM_MODE _BV(WGM20) | _BV(WGM21) | _BV(COM21) | _BV(CS22) | _BV(CS21) | _BV(CS20)
//just set timer0 to normal operation
#define TIMER0_SETTINGS _BV(CS00)
/* maximum duty cycle value */
#define MAX_DC 0xFF
/* number of buttons we are debouncing */
#define NUM_BUTTONS 1

#define DC_THRESHOLD 10
#define BIT_START_DC	53UC
#define BIT_1_DC	117UC
#define BIT_0_DC	181UC
#define BIT_END_DC	245UC	

/* globals */
typedef enum {false=0,true=-1} boolean;
/* use as indexer into debounce array */
typedef enum {
    SW0,
    SW1,
    SW2,
    SW3,
    SW4,
    SW5
} Button;

/* states */
typedef enum {
    NONE,
    SW0_PRESS,
    SW0_RELEASE,
} State;

volatile State m_currState = NONE;
volatile State m_prevState = NONE;

volatile unsigned int m_ovrCtr = 0;
/* Button debounce values */
unsigned char m_btnDBval[6] = {DB_OFF_LIMIT,DB_OFF_LIMIT,DB_OFF_LIMIT,DB_OFF_LIMIT,DB_OFF_LIMIT,DB_OFF_LIMIT};
/* On/Off status of the buttons */
boolean m_btnState[6] = {false,false,false,false,false,false};
/* Pins on the Port we should test for button presses */
unsigned char m_btnPin[6] = {PINC0,PINC1,PINC2,PINC3,PINC4,PINC5};
Button m_debounceButtons[1] = {SW0};

volatile unsigned char m_duty_cycle = 0x00;

#define MAX_FRAMES 10
typedef struct {
    unsigned int data;
    unsigned char numbits;
    unsigned char currbit;
} Frame;

typedef struct  {
    Frame qdata[MAX_FRAMES];
    unsigned char insert_pos; //current index in queue to push to
    unsigned char TOQ; //top of queue
} Queue;

Queue m_rx_queue;
Queue m_tx_queue;

Frame m_print_frame = {0x0000,0,15};
Frame m_rx_frame = {0x0000,0,15};
Frame m_scratch_tx_frame = {0x0000,0,15};
Frame m_tx_frame = {0x0000,0,15};

unsigned char m_prevPD2 = 0;
boolean m_found_start_bit = false;

unsigned char m_timer = 0;
boolean m_frameInProgress = false;

boolean m_in_tx = false;

void frame_init(Frame* f_) {
    f_->data = 0x0000;
    f_->numbits = 0;
    f_->currbit = 15;
}
void frame_write(Frame* f_,unsigned char val_) {
    if (val_ == 1) {
	//15 first bit to be tx'd therefore first bit to be written to
	f_->data |= _BV(15 - f_->numbits);
    }
    f_->numbits++;

}
void init_queue(Queue* q_) {
    q_->insert_pos = 0;
    q_->TOQ = 0; 
}

void push(Queue* q_, Frame val) {
    q_->qdata[q_->insert_pos] = val;
    q_->insert_pos = (q_->insert_pos + 1) % MAX_FRAMES;
}

boolean isEmpty(Queue* q_) {
    return (q_->TOQ == q_->insert_pos) ? true : false;
}

Frame remove(Queue* q_) {
    Frame retFrame;
    if (!isEmpty(q_)) {
	retFrame = q_->qdata[q_->TOQ];
	q_->TOQ = (q_->TOQ + 1) % MAX_FRAMES;
    }
    return retFrame;
} 


INTERRUPT (SIG_OVERFLOW0) {
    m_ovrCtr++;
}
/* figure out if we need to PWM a tx bit */
INTERRUPT (SIG_OVERFLOW2) {
    if (m_in_tx) {
	if (m_tx_frame.currbit == m_tx_frame.numbits) {
	    OCR2 = BIT_END_DC;
	    m_in_tx = false;
	} else if (m_tx_frame.data & _BV(m_tx_frame.currbit)){
	    OCR2 = BIT_1_DC;
	} else {
	    OCR2 = BIT_0_DC;
	}
	m_tx_frame.currbit++;
    } else { 
	//get the next frame to send (if any)
	if (!isEmpty(&m_tx_queue)) {
	    m_tx_frame = remove(&m_tx_queue);
	    OCR2 = BIT_START_DC;
	    m_in_tx = true;
	}
    }
    OCR2 = 1;
}
void setupTimer2();

/* handle rx of bits on PD2 */
INTERRUPT (SIG_INTERRUPT0) {
    /* tried making m_found_start_bit static, but compiler freaked out */
    unsigned char currPD2 = (PORTD & _BV(PD2));
    cli();
    unsigned char bitTimer = TCNT2;
    sei();

    if (m_prevPD2 == 1 && currPD2 == 0) {
	if (bitTimer >= BIT_START_DC && bitTimer <= BIT_START_DC + DC_THRESHOLD) {
		m_found_start_bit = true;
		frame_init(&m_rx_frame);
	} else if (bitTimer >= BIT_1_DC && bitTimer <= BIT_1_DC + DC_THRESHOLD) {
	    if (m_found_start_bit) {
		frame_write(&m_rx_frame,1);
	    }
	} else if (bitTimer >= BIT_0_DC && bitTimer <= BIT_0_DC + DC_THRESHOLD) {
	    if (m_found_start_bit) {
		frame_write(&m_rx_frame,0);
	    }
	} else if (bitTimer >= BIT_END_DC && bitTimer <= BIT_END_DC + DC_THRESHOLD) {
	    if (m_found_start_bit) {
	        m_found_start_bit = false;
		push(&m_rx_queue,m_rx_frame);
	    }
	} else {
	    PORTB &= ~_BV(7);
	}  
    } else {
	setupTimer2();
    }

    m_prevPD2 = currPD2;
}

void setupTimer0() {
  // set timer count rate and go
  TCCR0 = TIMER0_SETTINGS;
  TIMSK |= _BV(TOIE0);

}

/* hook into PD2 signals */
void setupInterrupt0() {
    /*
     * GICR - General Interrupt Control Register (bits indicate which interrupts to enable)
     * MCUCR - Control Register
     *  -> bits ISC00 = 1, ISC01 = 0 -  indidcates we want an interrupt to fire when a logical change
     *                                  happens with our button (for button connected to PORTD pin 2)
     */
    GICR |= _BV(INT0); 
    MCUCR |= _BV(ISC00);
}

/* timer 2 is used to PWM PD7 for tx */
void setupTimer2() {
  OCR2 = 0x00;
  TCCR2 = PWM_MODE;
  TIMSK |= _BV(TOIE2);
}

void setupQueues() {
    init_queue(&m_rx_queue);
    init_queue(&m_tx_queue);
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
    Button btn;
    for (i=0;i<NUM_BUTTONS;i++) {
	debounceButtonX(btn = m_debounceButtons[i]);
	m_btnState[btn] = getButtonState(m_btnDBval[btn], m_btnState[btn]);
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

void printFrameIfAvail() {
    if (m_print_frame.numbits > 0) {
	m_print_timer++;
	if (m_print_timer <= 25 && (m_print_frame.data & _BV(m_print_frame.currbit))) {
	    //we are printing a 1, for 1 second
	    PORTB = ~_BV(0);
	} else if (m_print_timer > 25 && (m_print_frame.data & _BV(m_print_frame.currbit))) {
	    m_in_pause = true;
	    PORTB = 0;
	} else if (m_print_timer <= 5) {
	    //we are printing a 1 for .25 seconds
	    PORTB = ~_BV(0);
	} else if (m_print_timer > 5 && (m_print_frame.data & _BV(m_print_frame.currbit))) { 
    } else {
	m_print_frame = remove(&m_rx_queue);
	m_print_timer = 0;
    }

}
void processState() {
    updateButtonState();
    printFrameIfAvail(); 
    switch (m_currState) {
	case NONE:
	    if (m_btnState[SW0]) {
		m_currState = SW0_PRESS;
		m_timer = 0;
		m_frameInProgress = true;
	    } else if (m_frameInProgress) {
		if (m_timer <= 20) {
                    m_timer++;
                }
		if (m_timer > 20) {
		    push(&m_tx_queue, m_scratch_tx_frame);
		    m_frameInProgress = false;
		    frame_init(&m_scratch_tx_frame);
		    m_timer = 0;
		}
	    }
	break;
        case SW0_PRESS:
            if (m_btnState[SW0]) {
		//this function is called every .05 secs
		//6 calls to this function with SW0 pressed
		//means the button was pressed for longer than .25secs
		if (m_timer <= 5) {
		    m_timer++;
		}
            } else {
		m_currState = SW0_RELEASE;
	    }
        break;
	case SW0_RELEASE:
	    if (m_timer <=5) {
                //tx a dot (0)
                frame_write(&m_scratch_tx_frame,0);
            } else {
                //tx a dash (1)
                frame_write(&m_scratch_tx_frame,1);
            }
	    m_currState = NONE;
	break;
    }
}

int main(void) {
    DDRB = 0xff;    //LEDs
    PORTB = 0xff;
    DDRD = _BV(PD7);
    PORTD |= _BV(PD7); //start with PD7 low


    //DDRD = 0x00;//Direction of PORTD is set to in for our switches

    cli(); //disable interrupts until we have finished setting the ones we want to fire   

    disableJTAG();
    setupTimer0();
    setupTimer2();
    setupInterrupt0();
    setupQueues();

    set_sleep_mode( SLEEP_MODE_IDLE );

    sei();
    for (;;) {
	sleep_mode();
	if (checkState()) {
	    processState();
	}
    }
}
