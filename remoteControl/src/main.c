/*
 * Acrobot.c
 *
 * Created: 10/25/2016 12:24:06 PM
 * Author : J. Diego Caporale, Matt Oslin, Garrett Wenger
 */ 


#define	F_CPU 16000000	    //CPU Clock Freq [Hz]
#define TIMER_0_PRSCL 256	//Timer0 Prescaler
#define TIMER_3_PRSCL 64	//Timer3 Prescaler

#define CTRL_FREQ 250		//Control loop clock frequency

#define clr(reg,bit)		reg &= ~(1<<(bit))

#include "m_general.h"
#include "m_bus.h"
#include "m_usb.h"
// #include "m_imu.h"
#include <stdlib.h>
#include "m_rf.h"

#define ADDRESS 20
#define CHANNEL 1
#define PACKET_LENGTH 10

#define NUMADCS 2
volatile uint16_t rawADCCounts[NUMADCS];

// Initialize helper functions
void setup(void);
void adc_init(void);
void timer0_init(void);
void timer3_init(void); // Millisecond timer
uint32_t millis(void); // Returns current milliseconds count

// Global flags for interrupts
volatile bool CTRLreadyFlag = FALSE;	// Frequency control flag for control loop
//volatile bool isCommandReady = FALSE; // RF command flag

volatile uint32_t milliseconds = 0;
volatile unsigned char buffer[PACKET_LENGTH];

int main(void) {
	setup();

	m_usb_init();

	clr(DDRB, 2);
	set(PORTB, 2);

	clr(DDRB, 3);
	set(PORTB, 3);

	uint32_t sendCount = 0;
	//Main process loop
    while (1) {
		//Control Loop at CTRL_FREQ frequency//
		if (CTRLreadyFlag) {
			//DEBUG CTRL FREQUENCY TEST//
			
			CTRLreadyFlag = FALSE; //Reset flag for interupt
			
			if(sendCount%100 == 0) {
				// m_green(TOGGLE);
				int i;
				if (!check(PINB,2)) {
					m_green(ON);
					m_red(OFF);
					for (i = 0; i < PACKET_LENGTH; i++) {
						buffer[i] = 0xA8;
					}
				} else if (!check(PINB,3)) {
					m_green(OFF);
					m_red(ON);
					for (i = 0; i < PACKET_LENGTH; i++) {
						buffer[i] = 0xA4;
					}
				} else {
					m_green(OFF);
					m_red(OFF);
					buffer[0] = 0xA7;
					buffer[1] = (char) (rawADCCounts[0]>>2)&0xFF;
					buffer[2] = (char) (rawADCCounts[1]>>2)&0xFF;
					for (i = 3; i < PACKET_LENGTH; i++) {
						buffer[i] = 0x00;
					}

					m_usb_tx_uint((uint8_t)buffer[1]);
					m_usb_tx_string(" ");
					m_usb_tx_uint((uint8_t)buffer[2]);
					m_usb_tx_string("\n");
				}

				m_rf_send(ADDRESS, buffer, PACKET_LENGTH);
			}
			// 	if (check(PINB,2) == 0) {
			// 		// m_red(OFF);
			// 		bool newPacket = FALSE;
			// 		char tempPhi = phi*1000.0/100.0;
			// 		char tempTheta = theta*1000.0/150.0;
			// 		tempTheta = tempTheta > 5 ? 5 : (tempTheta < -5 ? -5 : tempTheta);
			// 		tempPhi = tempPhi > 5 ? 5 : (tempPhi < -5 ? -5: tempPhi);

			// 		if(tempPhi != buffer[1]) {
			// 			buffer[1] = tempPhi;
			// 			newPacket = TRUE;
			// 			// m_rf_send(ADDRESS, buffer, PACKET_LENGTH);
			// 			// m_red(TOGGLE);
			// 		}
			// 		if(tempTheta != buffer[0]) {
			// 			buffer[0] = tempTheta;
			// 			newPacket = TRUE;
			// 		}
			// 		if(newPacket) {
			// 			m_rf_send(ADDRESS, buffer, PACKET_LENGTH);
			// 			m_red(TOGGLE);
			// 		}
			// 		// buffer[1] = 
			// 		// m_usb_tx_int(1000*theta);
			// 		// m_usb_tx_string(" ");
			// 		// m_usb_tx_int(1000*phi);
			// 		// m_usb_tx_string("\n");
			// 	} else {
			// 		buffer[0] = 0;
			// 		buffer[1] = 0;
			// 		m_rf_send(ADDRESS, buffer, PACKET_LENGTH);
			// 		// m_red(ON);
			// 		m_usb_tx_string("Controller off\n");
			// 	}
			// }
			sendCount++;
			
		}
	}
}

void setup(void) {
	// SET CLOCK TO 16MHz
	m_clockdivide(0);
	
	// Initialize ADC and Timers
	timer0_init(); // Timer0 is our control loop clock
	timer3_init();
	
	adc_init();
	// Initalize all necessary MAEVARM utilities
	m_bus_init();
	// m_imu_init(ACCEL_SCALE,GYRO_SCALE);
	m_rf_open(CHANNEL, 21, PACKET_LENGTH); // For RF comms 
	
	m_green(ON); // Ready LED
	m_disableJTAG(); //Allows use of some of the portF

	// Enable global interrupts
	sei();
	m_wait(50);
}

void timer0_init() {
	//ENABLE MODE 2 UP TO OCR0A, RESET TO 0x00
	clr(TCCR0B, WGM02);
	set(TCCR0A, WGM01);
	clr(TCCR0A, WGM00);
	
 	//SET OCR0A FOR DESIRED FREQ CONTROL
 	OCR0A = (F_CPU/CTRL_FREQ)/TIMER_0_PRSCL;	
	
	//Enable interrupt when TCNT1 Overflows
	set(TIMSK0 , OCIE0A );

	//ENABLE TIMER 0 WITH 256 PRESCALER 
	set(TCCR0B, CS02);
	clr(TCCR0B, CS01);
	clr(TCCR0B, CS00);
}

uint32_t millis(void) {
	return milliseconds;
}

void timer3_init(void) {
	// Count up to OCR3A, then reset
	clr(TCCR3B, WGM33);
	set(TCCR3B, WGM32);
	clr(TCCR3A, WGM32);
	clr(TCCR3A, WGM30);

	OCR3A = 250; // (16 MHz / 64) * (1 ms) = 250

	set(TIMSK3, OCIE3A); // Interrupt when TCNT0 = OCR0A

	// Timer on, prescaler to /64
	clr(TCCR3B, CS32); 
	set(TCCR3B, CS31);
	set(TCCR3B, CS30);
}

ISR(TIMER3_COMPA_vect) {
	milliseconds++;
}

//Interrupt for CTRL_FREQ frequency control loop
ISR(TIMER0_COMPA_vect) {
	CTRLreadyFlag = TRUE;
}

void adc_init() {
	//Set voltage reference to Vcc
	set(ADMUX, REFS0);
	clr(ADMUX, REFS1);

	//Set ADC Prescaler to /128 (125kHz ADCClock at 16MHz Sysclock)
	//This will translate to 9kHz read speed for all the ADCS (2.4kHz per ADC)
	set(ADCSRA, ADPS0);
	set(ADCSRA, ADPS2);
	set(ADCSRA, ADPS1);

	//Disable Digital Inputs on F4,5,6,7
	set(DIDR0, ADC4D);
	set(DIDR0, ADC5D);
	// set(DIDR0, ADC6D);
	// set(DIDR0, ADC7D);

	//Set ADC Interrupt
	set(ADCSRA, ADIE);

	//Do not use  ADC in FREE RUNNING MODE
	clr(ADCSRA, ADATE);
	
	//Set which ADC to use first
	clr(ADCSRB, MUX5);
	set(ADMUX , MUX2);
	clr(ADMUX , MUX1);
	clr(ADMUX , MUX0);

	// ENABLE ADC AND RUN FIRST READING
	set(ADCSRA, ADEN);
	set(ADCSRA, ADSC);
}

//INTERRUPT HANDLER ADC
ISR(ADC_vect) {	
	static uint8_t ADCIndex = 0;
	rawADCCounts[ADCIndex] = ADC;
	//Choose which ADC to run next and set ADMUX register
	ADCIndex = (ADCIndex + 1) % NUMADCS;
	ADMUX = 0x44+ADCIndex;

	//Start next ADC read
	set(ADCSRA, ADSC);
}