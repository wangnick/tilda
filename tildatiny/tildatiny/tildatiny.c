#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/crc16.h>

#define F_CPU 8000000
#include <util/delay.h>

#include "usiTwiSlave.h"

#define I2C_SLAVE_ADDR  0x26            // i2c slave address (38)

/*
  H-Brücke 754410
*/
const int P12EN = PB3; // Right motor PWM logic
const int P34EN = PB5; // Left motor PWM logic
const int P2A = PB4; // Right motor backwards logic - 1A right motor forwards logic is negated from this in hardware
const int P4A = PB6; // Left motor backwards logic - 3A left motor forwards logic is negated from this in hardware

// Following terminals are connected via 33kOhm / 10kOhm to GND voltage splitters
const int MUX1Y = 3; // Right motor forwards positive Back-EMF (ADC3, PA4) 
const int MUX2Y = 4; // Right motor backwards positive Back-EMF (ADC4, PA5)
const int MUX3Y = 5; // Left motor forwards positive Back-EMF (ADC5, PA6) 
const int MUX4Y = 6; // Left motor backwards positive Back-EMF (ADC6, PA7)
const int MUXVCC = 1; // Battery voltage terminal (ADC1, PA1)

const int PLED = PA3;

const unsigned char PWMMAXDUTY = 0x7F;
unsigned char dutyright, dutyleft, dirright, dirleft;

enum {V12, V34, VCC, VSIZE} adckind;
int adcval[VSIZE];
enum {AWAIT, SETTLE, MEASURE, MEASUREPOS, MEASURENEG} adcstate;
const unsigned long ADCTIME[] = {6000, 250, 5, 5, 5};
unsigned long adctime, lastadc;
	
unsigned long lasti2c;
int i2c = false;
unsigned char i2cdata[3];
int i2ccount = 0;
int led = 0;

const unsigned long I2CTIME [] = {50000, 2000};
const unsigned long LEDTIME [] = {500000, 100000};
unsigned long ledtime[2], lastled;


// TODO: Restart motors after sample-and-hold whilst last conversion is is progress.
// TODO: Try to sample in PWM pause?

#define modbit(val,bitnum,bitval) val = val & ~(1<<bitnum) | ((bitval)<<bitnum)

volatile unsigned long timer0 = 0;

ISR (TIMER0_OVF_vect) {
	timer0++;
}

//#define TIMER16BIT

unsigned long micros () {
	unsigned long result;
	unsigned char sreg_i = SREG & (1<<SREG_I);
	cli();
	result = TCNT0L;
#ifdef TIMER16BIT	
	result |= (unsigned int)TCNT0H<<8; // Without the cast the uint8_t is promoted to signed int,
										// which will get negative after the shift if TCNT0H>=0x80. Phew.
	if ((TIFR & (1<<TOV0)) && result<0xFFFF) result += 0x10000;
	result += timer0<<16;
#else
	if ((TIFR & (1<<TOV0)) && result<0xFF) result += 0x100;
	result += timer0<<8;
#endif
	SREG |= sreg_i;
	return result;
}

void setup_micros () {
#ifdef TIMER16BIT
	// Set Timer 0 to 16 bit
	TCCR0A = 1<<TCW0;
#endif
	// Enable Timer 0 interrupt
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
	PORTB |= 1<<P12EN | 1<<P34EN;
	for (unsigned long now = micros(); micros()-now<3000; ) {
		led = !led;
		modbit(PORTA,PLED,led);
	}	
	PORTB &= ~(1<<P12EN | 1<<P34EN);
	
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
	// Activate timer 1 clock, pre-scaler 1/4 -> 0.5us/2MHz per duty step, 62us/16kHz at 128 duty steps -> Too low torque
	//TCCR1B = (1<<CS11) | (1<<CS10);
	// Activate timer 1 clock, pre-scaler 1/8 -> 1us/1MHz per duty step, 125us/8kHz at 128 duty steps -> High pitch, still too low torque
	//TCCR1B = (1<<CS12);
	// Activate timer 1 clock, pre-scaler 1/16 -> 2us/500kHz per duty step, 250us/4kHz at 128 duty steps -> Reasonable pitch, still too low torque
	//TCCR1B = (1<<CS12) | (1<<CS10);
	// Activate timer 1 clock, pre-scaler 1/32 -> 4us/250kHz per duty step, 500us/2kHz at 128 duty steps -> Reasonable pitch and torque
	TCCR1B = (1<<CS12) | (1<<CS11);
	// Activate timer 1 clock, pre-scaler 1/64 -> 8us/125kHz per duty step, 1ms/1kHz at 128 duty steps -> Good torque but rather loud
	//TCCR1B = (1<<CS12) | (1<<CS11) | (1<<CS10);
	// Activate timer 1 clock, pre-scaler 1/128 -> 16us/62kHz per duty step, 2ms/500Hz at 128 duty steps -> Good torque but rather loud
	//TCCR1B = (1<<CS13);
	// Activate timer 1 clock, pre-scaler 1/256 -> 32us/31kHz per duty step, 4ms/250Hz at 128 duty steps -> Good torque but nasty snarring sound under load
	//TCCR1B = (1<<CS13) | (1<<CS10);
}

void adc () {
	static int posval;
	switch (adcstate) {
	case AWAIT:
		if (adckind==VCC) {
			ADMUX = MUXVCC;
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
		ADMUX = adckind==V12?MUX1Y:MUX3Y;
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
			ADMUX = adckind==V12?MUX2Y:MUX4Y;
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
	}
	adctime = ADCTIME[adcstate];
}

void setup_adc() {
	// set ADC prescale factor to 32
	// 8 MHz / 32 = 4 us/250kHz, not inside the desired 50-200 KHz range but sufficient.
	// One measurement takes 13 cycles = 52us. The Back-EMF settles after 250us (MOTORSETTLETIME).
	// We need two measurement, one for each motor terminal. Thus, the motor is non-powered for 350us.
	// We measure every 18ms, this makes a 2% loss of maximum torque.
	ADCSRA = (1<<ADEN) | (1<<ADPS1) | (1<<ADPS0);
	// set ADC to bipolar mode for differential conversions
	// set ADC to left-adjusted
	// ADCSRB |= (1<<BIN) | (1<<ADLAR);
	lastadc = micros();
	adctime = 0;
	adckind = VCC;
	adcstate = AWAIT;
}

void setup_led() {
	DDRA |= 1<<PLED;
	modbit(PORTA,PLED,led);
	ledtime[0] = LEDTIME[0];
	ledtime[1] = LEDTIME[1];	
	lastled = micros();
}

void setup_i2c() {
	USIPP |= 1<<USIPOS; // Switch USI to alternate pins PA0 & PA2
	usiTwiSlaveInit(I2C_SLAVE_ADDR);
}

void setup() {
	setup_micros();
	setup_led();
	setup_adc();
	setup_motors();
	setup_i2c();
}

#define send(val) usiTwiTransmitByte(val); checksum = _crc_ibutton_update(checksum,val);

// TODO: Use fast digital write.

void loop() {
	unsigned long time = micros();
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
					send(checksum);
				} else {
					checksum = 0xFF;
					send(adcval[V12]>>8);
					send(adcval[V12]&0xFF);
					send(adcval[V34]>>8);
					send(adcval[V34]&0xFF);
					send(adcval[VCC]>>8);
					send(adcval[VCC]&0xFF);
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
	if (time-lastadc>adctime) {
		adc();
		lastadc = time;
	}
	
	if (time-lastled>ledtime[led]) {
		led = !led;
		modbit(PORTA,PLED,led);
		lastled = time;
	}
	
	//modbit(PORTA,PLED,0);
	//_delay_us(1);
	//modbit(PORTA,PLED,1);
	//_delay_us(1);
	//modbit(PORTA,PLED,0);
	//_delay_us(1);
	//for (int i=0; i<32; i++) {
		//modbit(PORTA,PLED,!!(time&0x80000000));
		//_delay_us(1);
		//time <<= 1;
	//}
	//modbit(PORTA,PLED,0);
	//_delay_us(1);
	//modbit(PORTA,PLED,1);
	//_delay_us(1);
	//modbit(PORTA,PLED,0);
	//_delay_us(1);
//
}

int main(void) {
	setup();
	while (1) {
		loop();
	}
}

