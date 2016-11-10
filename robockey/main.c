/*
 * Robockey
 *
 * Created: 10/25/2016 6:31:06 PM
 * Author : J. Diego Caporale, Matt Oslin, Garrett Wenger, Jake Welde
 */ 


#define	F_CPU 16000000	    //CPU Clock Freq [Hz]
#define PI 3.14159265       // The constant that defines the ratio between diameter and circumference


#define ABS(X)   (X < 0 ?-X : X)
#define MIN(X,Y) (X < Y ? X : Y)
#define set(reg,bit)		reg |= (1<<(bit))
#define clr(reg,bit)		reg &= ~(1<<(bit))

// If we want RF
// #define CHANNEL 1
// #define MY_ADDRESS 0x5D
// #define PACKET_LENGTH 3

#include "m_general.h"
#include "m_bus.h"
#include "m_usb.h"
//#include "m_imu.h"
#include "m_rf.h"
#include "motor_driver.h"
#include "ADC_driver.h"


// Initialize helper functions

void timer3_init(void); // Millisecond timer
void usb_read_command(void);
uint32_t millis(void); // Returns current milliseconds count


// Global flags for interrupts

volatile bool CTRLreadyFlag = FALSE;	// Frequency control flag for control loop
//volatile bool isCommandReady = FALSE; // RF command flag

//Global Variable
volatile uint16_t rawADCCounts[NUMADCS];	// Array of raw ADC values
volatile uint32_t milliseconds = 0;


int main(void) {
	// SET CLOCK TO 16MHz
	m_clockdivide(0);
	
	// Initialize ADC and Timers
	adc_init();    // Initializes first ADC read for encoder
	timer0_init(); // Timer0 is our control loop clock
	timer3_init();
	
	// Initalize all necessary MAEVARM utilities
	m_bus_init();
	//m_rf_open(CHANNEL, MY_ADDRESS, PACKET_LENGTH); // For RF comms 
	m_usb_init(); // USB COMs for debug
	
	m_green(ON); // Ready LED
	m_disableJTAG(); //Allows use of some of the portF

	// Enable global interrupts
	sei();
	
	//Initialize Variables
	const float deltaT = 1.0/CTRL_FREQ;
	uint16_t countUSB = 0; //Used to not bog down processer or terminal with USB Transmissions
	//unsigned int usbIn, charCount;

	//Main process loop
    while (1) //Stay in this loop forever
    {
		//Control Loop at CTRL_FREQ frequency//
		if (CTRLreadyFlag)
		{
			//DEBUG CTRL FREQUENCY TEST//
			//set(PORTC,6);
			
			CTRLreadyFlag = FALSE; //Reset flag for interupt

			//LOW PASS FILTER FOR ADC//
			int i;
			for(i = 0; i < NUMADCS; i++) {
				oldFiltADCCounts[i] = filtADCCounts[i];
				filtADCCounts[i] = alpha*filtADCCounts[i] + (1-alpha)*rawADCCounts[i];
			}

			// USB DEBUG//
			// Send USB information for DEBUG every 100 control loop cycles
			// if (countUSB%100 == 0) {	
			//  	m_red(TOGGLE);
			//
	 		//		m_usb_tx_push();
			// }

			//Iterate countUSB
			//countUSB++;

		}
		//RF Command inputs
// 		if (isCommandReady)
// 		{	
// 			isCommandReady = FALSE;
// 		}
	}
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

	//ENABLE TIMER 0 WITH 1024 PRESCALER 
	set(TCCR0B, CS02);
	clr(TCCR0B, CS01);
	set(TCCR0B, CS00);
}


void usb_read_command() {	
	char buff[8];
	unsigned int indx = 0;
	int val = 0;
	int i;

	while(m_usb_rx_available()&&indx<8){
		buff[indx] = m_usb_rx_char();
		indx++;
	}
	
	for(i=indx-1; i > 0; i--){
		val += ((int)buff[i]-'0')*pow(10, indx-i-1);//Introduces mistakes in integer math (rounds down)
		m_usb_tx_int((int)buff[i]-'0');
		m_usb_tx_string("\n");
	}
	switch(buff[0]){
		case 'P':
			kpth = val;
			m_usb_tx_string("\n");
			m_usb_tx_string("KP: ");
			m_usb_tx_int(kpth);
			m_usb_tx_string("\n");
			break;
		case 'D':
			kdth = val;
			m_usb_tx_string("\n");
			m_usb_tx_string("KD: ");
			m_usb_tx_int(kdth);
			m_usb_tx_string("\n");
			break;
		case 'B':
			beta = val;
			m_usb_tx_string("\n");
			m_usb_tx_string("1/Beta: ");
			m_usb_tx_int(1.0/beta);
			m_usb_tx_string("\n");
			break;
		default :
			m_usb_tx_string("NO DATA");
	}
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

//INTERRUPT HANDLER ADC
// Interrupt to inform main loop that ADC is ready to read
// In theory 9.6 kHz (VALIDATED without load)
ISR(ADC_vect)
{
	adc_read(rawADCCounts);
}

//RF Command Interupt Handler.
// ISR(INT2_vect){
// 	isCommandReady = TRUE;
// 	m_rf_read(buffer,PACKET_LENGTH);// pull the packet
// 	
// }

//Interrupt for CTRL_FREQ frequency control loop
ISR(TIMER0_COMPA_vect) {
	CTRLreadyFlag = TRUE;
}

ISR(TIMER3_COMPA_vect) {
	milliseconds++;
}