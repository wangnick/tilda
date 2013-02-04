// tildatiny.c
// (C) 2013 Sebastian Wangnick
//
// Attiny861 I2C-controlled PWM generator for two brushed DC motors, in my case Lego 71427.
// Returns current motor speeds (measured via Back-EMF) and battery power to the I2C master.
// Motors are driven via H-Bridge, in my case SN754410.
//
// Part of the Tilda project, a self-balancing two-wheel Lego robot, with Raspberry Pi as I2C master.
// Refer to http://code.google.com/p/tilda for further details.
// 
// TODO: Restart motors after sample-and-hold whilst last conversion is is progress.
// TODO: Try to sample in PWM pause?

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/crc16.h>

#include "usiTwiSlave.h"

/*
  H-Bridge SN754410
*/
const int P12EN = PB3; // Right motor PWM output
const int P34EN = PB5; // Left motor PWM output
const int P2A = PB4; // Right motor backwards output - 1A right motor forwards output is negated from this in hardware
const int P4A = PB6; // Left motor backwards output - 3A left motor forwards output is negated from this in hardware

const int PIR = PCINT3; // Infrared receiver, PA3
const int PDISTTRIG = PA2; // IR distance measurement HC-SR04, trigger
const int PDISTECHO = PA2; // IR distance measurement HC-SR04, echo
// Note: The HC-SR04 Echo line is pulled down with about 36 Ohms.
// A 3.3V Attiny has not enough power on the output pin to pull the HC-SR04 Echo line high enough to prepare the Trigger, thus PDISTTRIG!=PDISTECHO
// At 5V Attiny sources 60mA to the Echo line to pull the Trigger to 2.2V, which is sufficient for the HC-SR04.
const int PLED = PA0; // LED

const unsigned char PWMMAXDUTY = 0x7F;
unsigned char dutyright, dutyleft, dirright, dirleft;

enum {V12, V34, VCC, VSIZE} adckind;
int adcval[VSIZE];
// Following terminals are connected via 33kOhm / 10kOhm to GND voltage splitters
//     Right motor forwards positive Back-EMF: ADC3 (PA4)
//     Right motor backwards positive Back-EMF: ADC4 (PA5)
//     Left motor forwards positive Back-EMF: ADC5 (PA6)
//     Left motor backwards positive Back-EMF: ADC6 (PA7)
//     Battery positive voltage terminal: ADC1 (PA1)
const unsigned long ADCMUX[] = {3, 5, 1};
const unsigned long ADCMUXNEG[] = {4, 6};
enum {START, AWAIT, SETTLE, MEASURE, MEASUREPOS, MEASURENEG} adcstate;
const unsigned long ADCTIME[] = {0, 6000, 250, 5, 5, 5};
unsigned long adctime, lastadc;
	
const int I2C_SLAVE_ADDR = 0x26;           // Our i2c slave address (38)
const unsigned long I2CTIME [] = {50000, 2000};
unsigned long lasti2c;
int i2c = false;
unsigned char i2cdata[3];
int i2ccount = 0;
int led;

const unsigned long LEDTIME [] = {500000, 100000};
unsigned long ledtime[2], lastled;

#define modbit(val,bitnum,bitval) val = val & ~(1<<bitnum) | ((bitval)<<bitnum)

volatile unsigned long timer0 = 0;

ISR (TIMER0_OVF_vect) {
	timer0++;
}

#define TIMER16BIT

static inline unsigned int tcnt0 () __attribute__((always_inline));
static inline unsigned int tcnt0 () {
	unsigned int result = TCNT0L;
#ifdef TIMER16BIT
	result |= TCNT0H<<8;
#endif
	return result; 
}

unsigned long micros () {
	unsigned long result;
	unsigned char sreg_i = SREG & (1<<SREG_I);
	cli();
	result = tcnt0();
#ifdef TIMER16BIT	
	if ((TIFR & (1<<TOV0)) && result<0x8000) result += 0x10000;
	result += timer0<<16;
#else
	if ((TIFR & (1<<TOV0)) && result<0x80) result += 0x100;
	result += timer0<<8;
#endif
	SREG |= sreg_i;
	return result;
}

void setup_micros () {
	TCCR0A = 0;
#ifdef TIMER16BIT
	// Set Timer 0 to 16 bit
	TCCR0A |= 1<<TCW0;
#endif
	// Enable Timer 0 overflow interrupt
	TIMSK = 1<<TOIE0;
	// Activate Timer 0 clock, prescaler 1/8 -> 1us,1MHz
	TCCR0B = 1<<CS01;
	// Enable interrupts
	sei();
}

void drive (signed char speedright, signed char speedleft) {
	OCR1B = abs(speedright);
	OCR1D = abs(speedleft);
	unsigned char portb = PORTB;
	modbit(portb,P2A,speedright<0);
	modbit(portb,P4A,speedleft<0);
	PORTB = portb;
}

void setup_motors () {
	DDRB |= 1<<P12EN | 1<<P34EN | 1<<P2A | 1<<P4A;
	//PORTB |= 1<<P12EN | 1<<P34EN;
	//for (unsigned long now = micros(); micros()-now<3000; ) {
		//led = !led;
		//modbit(PORTA,PLED,led);
	//}	
	//PORTB &= ~(1<<P12EN | 1<<P34EN);
	
	// The Phase and Frequency Correct PWM Mode (PWM1A/PWM1B = 1 and WGM11:10 = 01) provides
	// a high resolution Phase and Frequency Correct PWM waveform generation option. The
	// Phase and Frequency Correct PWM mode is based on a dual-slope operation. The counter
	// counts repeatedly from BOTTOM to TOP (defined as OCR1C) and then from TOP to BOTTOM.

	// The Timer/Counter Overflow Flag (TOV1) is set each time the counter reaches BOTTOM. The
	// Interrupt Flag can be used to generate an interrupt each time the counter reaches the BOTTOM
	// value.
	
	// Setting the COM1x1:0 bits to two will produce a non-inverted PWM and setting the COM1x1:0
	// to three will produce an inverted PWM output.
	
	// The actual values will only be visible on the port pin if the data direction for the 
	// port pin is set as output.

	// Activate OC1B non-inverted PWM.
	TCCR1A = (1<<COM1B1) | (1<<PWM1B);
	// Activate OC1D non-inverted PWM.
	TCCR1C = (1<<COM1D1) | (1<<PWM1D);
	// Select Past PWM waveform generation
	TCCR1D = 0;
	// Set up max duty cycle
	OCR1C = PWMMAXDUTY;
	// Set up duty cycle
	drive(0,0);
	// Activate timer 1 clock, pre-scaler 1/4 -> 0.5us/2MHz per duty step, 62us/16kHz at 128 duty steps -> Pitch not audible
	TCCR1B = (1<<CS11) | (1<<CS10);
	// Activate timer 1 clock, pre-scaler 1/8 -> 1us/1MHz per duty step, 125us/8kHz at 128 duty steps -> High pitch
	//TCCR1B = (1<<CS12);
	// Activate timer 1 clock, pre-scaler 1/16 -> 2us/500kHz per duty step, 250us/4kHz at 128 duty steps -> Medium pitch
	// TCCR1B = (1<<CS12) | (1<<CS10);
	// Activate timer 1 clock, pre-scaler 1/32 -> 4us/250kHz per duty step, 500us/2kHz at 128 duty steps -> Unnerving medium pitch
	//TCCR1B = (1<<CS12) | (1<<CS11);
	// Activate timer 1 clock, pre-scaler 1/64 -> 8us/125kHz per duty step, 1ms/1kHz at 128 duty steps -> Rather loud
	//TCCR1B = (1<<CS12) | (1<<CS11) | (1<<CS10);
	// Activate timer 1 clock, pre-scaler 1/128 -> 16us/62kHz per duty step, 2ms/500Hz at 128 duty steps -> Rather loud
	//TCCR1B = (1<<CS13);
	// Activate timer 1 clock, pre-scaler 1/256 -> 32us/31kHz per duty step, 4ms/250Hz at 128 duty steps -> Nasty snarring sound under load
	//TCCR1B = (1<<CS13) | (1<<CS10);
}

void setup_adc() {
	// set ADC prescale factor to 32
	// 8 MHz / 32 = 4 us/250kHz, not inside the desired 50-200 KHz range but sufficient.
	// One measurement takes 13 cycles = 52us. The Back-EMF settles after 250us (MOTORSETTLETIME).
	// We need two measurement, one for each motor terminal. Thus, each motor is non-powered for 350us.
	// We measure every 18ms, this makes a 2% loss of maximum torque.
	ADCSRA = (1<<ADEN) | (1<<ADPS1) | (1<<ADPS0);
	// set ADC to bipolar mode for differential conversions
	// set ADC to left-adjusted
	// ADCSRB |= (1<<BIN) | (1<<ADLAR);
	lastadc = micros();
	adcstate = START;
	adctime = 0;
}

void do_adc (unsigned long time) {
	if (time-lastadc<adctime) return;
	lastadc = time;
	static int posval;
	switch (adcstate) {
		case START:
		adckind = VCC;
		ADMUX = ADCMUX[adckind];
		// Fall through
		case AWAIT:
		if (adckind==VCC) {
			ADCSRA |= (1<<ADSC);
			adcstate = MEASURE;
		} else {
			if (adckind==V12) {
				TCCR1A &= ~(1<<COM1B1);
			} else {
				TCCR1C &= ~(1<<COM1D1);
			}
			adcstate = SETTLE;
		}
		break;
		case SETTLE:
		ADCSRA |= (1<<ADSC);
		adcstate = MEASUREPOS;
		break;
		case MEASURE:
		if (!(ADCSRA & (1<<ADSC))) {
			posval = ADCL;
			posval |= ADCH<<8;
			adcval[adckind] = posval;
			adcstate = AWAIT;
		}
		break;
		case MEASUREPOS:
		if (!(ADCSRA & (1<<ADSC))) {
			posval = ADCL;
			posval |= ADCH<<8;
			ADMUX = ADCMUXNEG[adckind];
			ADCSRA |= (1<<ADSC);
			adcstate = MEASURENEG;
		}
		break;
		case MEASURENEG:
		if (!(ADCSRA & (1<<ADSC))) {
			int val = ADCL;
			val |= ADCH<<8;
			adcval[adckind] = posval-val;
			if (adckind==V12) {
				TCCR1A |= (1<<COM1B1);
			} else {
				TCCR1C |= (1<<COM1D1);
			}
			adcstate = AWAIT;
		}
		break;
	}
	if (adcstate==AWAIT) {
		adckind = (adckind+1)%VSIZE;
		ADMUX = ADCMUX[adckind];
	}
	adctime = ADCTIME[adcstate];
}

void setup_led() {
	led = 1;
	DDRA |= 1<<PLED;
	modbit(PORTA,PLED,led);
	for (unsigned long time = micros(); micros()-time<500000; ) ;
	modbit(PORTA,PLED,!led);
	for (unsigned long time = micros(); micros()-time<500000; ) ;
	modbit(PORTA,PLED,led);
	ledtime[0] = LEDTIME[0];
	ledtime[1] = LEDTIME[1];
	lastled = micros();
}

void do_led(unsigned long time) {
	if (time-lastled>ledtime[led]) {
		led = !led;
		modbit(PORTA,PLED,led);
		lastled = time;
	}
}

enum {IR_IDLE, IR_GOTMARK, IR_GOTSPACE, IR_STOP, IR_ERR};
#define IR_BUF 100
#define IR_MARK 0
#define IR_SPACE 1<<PIR
#define IR_GAP 5000

struct Ir {
	unsigned char state;          // state machine
	unsigned long time;
	unsigned int buf[IR_BUF]; // raw data
	unsigned char len;         // counter of entries in rawbuf
} ir;

void setup_ir () {
	ir.state = IR_IDLE;
	ir.len = 0;
	ir.time = micros()/50;
	PCMSK0 = 1<<PIR;
	PCMSK1 = 0;
	GIMSK = 1<<PCIE1;
}

	/*
	unsigned char data = PORTA & Ir.SPACE;
	unsigned long time = micros();
	if (len<IRBUF) {
		unsigned int delta = (time-lasttime>0xFFFF)? 0xFFFF: time-lasttime;
		if (len==0 && (data==Ir.SPACE || delta<GAP)) return;
		if (len>0 && data==Ir.MARK && delta>=GAP) {
			decodeir();
			len = 0;
		}		
		ir.buf[ir.len++] = delta;
		lasttime = time;
	}
	*/


enum {DIST_TRIGGER, DIST_MEASURE_START, DIST_MEASURE_END, DIST_IDLE, DIST_STATES};
const unsigned long DIST_TIME[] = {1000, 40000, 0, 65000}; // Time for MEASURE_START completion < 16 bit. 
	// Don't trigger more often than every 65ms (bounces from previous trigger might still be on their way back).

struct Dist {
	unsigned char state;
	unsigned long time;
    unsigned long echotime;
	unsigned int dist;
} dist;
 
ISR (PCINT_vect) {
	unsigned long time = micros();
    switch (dist.state) {
    case DIST_TRIGGER:
        dist.echotime = time;
        dist.state++; // DIST_MEASURE_START
        break;
    case DIST_MEASURE_START:
        dist.dist = time-dist.echotime;
        dist.state++; // DIST_MEASURE_END
        break;
    }
}

void setup_dist () {
	dist.dist = 0;
	dist.state = DIST_MEASURE_END;
	PCMSK0 = 1<<PDISTECHO;
	PCMSK1 = 0;
	DDRA &= ~(1<<PDISTECHO); // Echo Input (High-Z)
}

void do_dist (unsigned long time) {
	if (time-dist.time<DIST_TIME[dist.state]) return;
    cli();
	dist.state = (dist.state+1)%DIST_STATES;
	switch (dist.state) {
	case DIST_MEASURE_START: // Measurement failed, no echo start in time
        // Fall thru
	case DIST_MEASURE_END: // Measurement failed, no echo end in time
		dist.dist = 0;
        dist.state = DIST_IDLE;
		// Fall thru
	case DIST_IDLE: // Disable interrupt
		GIMSK &= ~(1<<PCIE1);
		break;
	case DIST_TRIGGER: 
		; unsigned int now = tcnt0();
		PORTA |= 1<<PDISTTRIG; // Trigger (Input) PULLUP to prepare for Output HIGH, avoiding a short LOW
		DDRA |= 1<<PDISTTRIG; // Trigger Output (HIGH).
		while (tcnt0()-now<10) continue; // This is crucial to keep short, as we are sourcing >40mA from the Trigger pin if it is combined with Echo
		PORTA &= ~(1<<PDISTTRIG); // Trigger (Output) LOW
		while (tcnt0()-now<18) continue; // Echo needs 4.5 us to go low
		DDRA &= ~(1<<PDISTTRIG); // Trigger Input (High-Z)
		GIMSK |= 1<<PCIE1; // Enable interrupt
		dist.time = time;
		break;
	}
	sei();
}

void setup_i2c() {
	// USIPP = 1<<USIPOS; // Switch USI to alternate pins PA0 and PA2. Must also activate __AVR_ATtiny861__ALT__ in usiTwiSlave.c!
	usiTwiSlaveInit(I2C_SLAVE_ADDR);
	i2c = false;
	i2ccount = 0;
}

#define send(val) usiTwiTransmitByte(val); checksum = _crc_ibutton_update(checksum,val);

void do_i2c (unsigned long time) {
	if (usiTwiDataInReceiveBuffer()) {
		if (usiTwiDataInTransmitBuffer()) {
			usiTwiReceiveByte();
		} else {
			i2c = true;
			ledtime[0] = ledtime[1] = LEDTIME[1];
			lasti2c = time;
			i2cdata[i2ccount++] = usiTwiReceiveByte();
			if (i2ccount==3) {
				unsigned char checksum = _crc_ibutton_update(_crc_ibutton_update(0xFF,i2cdata[0]),i2cdata[1]);
				if (i2cdata[2]!=checksum) {
					checksum = 0xFF;
					send(0x00);
					send(0x00);
					send(0x00);
					send(0x00);
					send(0x00);
					send(0x00);
					send(0x00);
					send(0x00);
					send(checksum);
				} else {
					checksum = 0xFF;
					send(adcval[V12]>>8);
					send(adcval[V12]&0xFF);
					send(adcval[V34]>>8);
					send(adcval[V34]&0xFF);
					send(adcval[VCC]>>8);
					send(adcval[VCC]&0xFF);
					send(dist.dist>>8);
					send(dist.dist&0xFF);
					send(checksum);
					if (i2cdata[0]==0x80) {
						if (i2cdata[1]>=1 && i2cdata[1]<=15) {
							TCCR1B = TCCR1B & 0xF0 | i2cdata[1];
						} else if (i2cdata[1]==0) {
							ledtime[0] = ledtime[1] = LEDTIME[1];
						} else {
							ledtime[0] = ledtime[1] = (unsigned long)i2cdata[1]<<3;
						}
					} else {
						drive(i2cdata[0],i2cdata[1]);
					}
				}
				i2ccount = 0;
			}
		}
	}
	if (i2c && time-lasti2c>I2CTIME[i2ccount?1:0]) {
		if (i2ccount) {
			i2ccount = 0;
		} else {
			i2c = false;
			usiTwiSlaveInit(I2C_SLAVE_ADDR); //Reset I2C library, including receive and transfer buffers.
			drive(0,0);
			ledtime[0] = LEDTIME[0];
			ledtime[1] = LEDTIME[1];
		}
	}
	
}

void setup() {
	setup_micros();
	setup_led();
	setup_adc();
	setup_motors();
	setup_dist();
	setup_i2c();
}

int main(void) {
	setup();
	while (1) {
		unsigned long time = micros();
		do_i2c(time);
		do_adc(time);
		do_led(time);
		do_dist(time);
	}
}

