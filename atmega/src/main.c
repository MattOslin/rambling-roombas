/*
 * Acrobot.c
 *
 * Created: 10/25/2016 12:24:06 PM
 * Author : J. Diego Caporale, Matt Oslin, Garrett Wenger
 */ 


#define	F_CPU 16000000	    //CPU Clock Freq [Hz]

#define TIMER_0_PRSCL 1024	//Timer0 Prescaler
#define TIMER_1_PRSCL 1		//Timer1 Prescaler
#define TIMER_3_PRSCL 64	//Timer3 Prescaler
#define PWM_MAX 0x00FF		//PWM Duty Cycle max (inversely proportional with frequency)
#define BETA .003			//Complimentary Filter Weighting Term


#define ACCEL_SCALE 1		//0 (+/-2g), 1 (+/-4g), 2 (+/-8g), 3 (+/-16g)
#define GYRO_SCALE 1		//0 (+/-250deg/s), 1 (+/-500deg/s), 2 (+/-1000deg/s) , 3 (+/-1000deg/s) 

#define NUMADCS 4			// Number of ADCs to be read
#define CTRL_FREQ 400		//Control loop clock frequency
#define ALPHA  0.5			// ADC LowPass Weighting term
#define PI 3.14159265       // The constant that defines the ratio between diameter and circumference
#define SHIFT 5
#define ACCEL_OFFSET_MAX 0.05

#define KP_TH 4000
#define KD_TH 70
#define KP_EN 0.000005
#define KD_EN 0.00005
#define ACCEL_OFFSET 0.0	//DEPENDS ON CENTER OF MASS
#define MOTOR_MATCH 0.10    //amount to add to motor B

#define clr(reg,bit)		reg &= ~(1<<(bit))

// If we want RF
// #define CHANNEL 1
// #define MY_ADDRESS 0x5D
// #define PACKET_LENGTH 3

#include "m_general.h"
#include "m_bus.h"
#include "m_usb.h"
#include "m_imu.h"
#include <stdlib.h>
// #include "m_rf.h"



// Initialize helper functions
void drive(int a, int b); 
void stop(void);
void adc_init(void);
void timer0_init(void);
void timer1_init(void);
void timer3_init(void); // Millisecond timer
void motor_init(void);
void usb_read_command(void);
uint32_t millis(void); // Returns current milliseconds count


// Global flags for interrupts

volatile bool ADCreadyFlag  = FALSE;	// ADC ready to use flag (not currently used)
volatile bool CTRLreadyFlag = FALSE;	// Frequency control flag for control loop
//volatile bool isCommandReady = FALSE; // RF command flag

//Global Variable
volatile uint16_t rawADCCounts[NUMADCS];	// Array of raw ADC values

int IMURawData[9];						// Array for IMU data [ax,ay,az,gx,gy,gz,mx,my,mz]
volatile int kpth = KP_TH;
volatile int kdth = KD_TH;
volatile float kpen = KP_EN;
volatile float kden = KD_EN;
volatile float beta = BETA;
volatile float alpha = ALPHA;
volatile uint32_t milliseconds = 0;

//INTERRUPT HANDLER ADC
// Interrupt to inform main loop that ADC is ready to read
// WAY TOO MUCH STUFF IN HERE. Will need to be changed when Hall Effect is integrated
// In theory 9.6 kHz (VALIDATED without load)
ISR(ADC_vect) {	
	static uint8_t ADCIndex = 0;
	rawADCCounts[ADCIndex] = ADC;
	//Choose which ADC to run next and set ADMUX register
	ADCIndex = (ADCIndex + 1) % NUMADCS;
	ADMUX = 0x44+ADCIndex;

	//Start next ADC read
	set(ADCSRA, ADSC);
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

int main(void) {
	// SET CLOCK TO 16MHz
	m_clockdivide(0);
	
	// Initialize ADC and Timers
	adc_init();    // Initializes first ADC read for encoder
	timer0_init(); // Timer0 is our control loop clock
	motor_init(); //Initializes Timer1 and the GPIO's necessary to run the motor driver
	timer3_init();
	// ENABLE GPIO OUTPUT FOR DEBUG AND FREQ TESTING
	set(DDRC,6);
	set(DDRC,7);
	
	// Initalize all necessary MAEVARM utilities
	m_bus_init();
	m_imu_init(ACCEL_SCALE,GYRO_SCALE);
	//m_rf_open(CHANNEL, MY_ADDRESS, PACKET_LENGTH); // For RF comms 
	m_usb_init(); // USB COMs for debug
	
	m_green(ON); // Ready LED
	m_disableJTAG(); //Allows use of some of the portF

	// Enable global interrupts
	sei();
	
	//Initialize Variables
	float az,gy,theta,error;
	float lastError = 0;
	theta = 0;
	const float g = 32767 / pow(2,(1+ACCEL_SCALE));    // Gravity magnitude in current scale
	const float maxOmega = 32767/(250*(1+GYRO_SCALE)); // Omega Scaling Term
	const float deltaT = 1.0/CTRL_FREQ;
	uint16_t count = 0; //Used to not bog down processer or terminal with USB Transmissions
	int balanceCommand = 0;
	uint16_t filtADCCounts[NUMADCS];	// Array of Filtered ADC values
	uint16_t oldFiltADCCounts[NUMADCS]; //previous counts
	int velocity[NUMADCS];				// array of encoder velocities
	long encoderA = 0;					//encoder counts
	long encoderB = 0;
	float accelOffset = ACCEL_OFFSET;   //upright positition
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
			
			//READ IMU RAW DATA//
			// ORIGINALLY USED m_imu_raw(), BUT IT RESTRICTED FREQUENCY TO 400Hz. 
			// By ignoring magnetometer readings achieved ~600-650 Hz.
			// Probe C6 to see.
			// This is also why the control clock has been reduced so much.
			
			m_imu_accel(IMURawData);
			m_imu_gyro(&IMURawData[3]);
			
			// clr(PORTC,6);
			
			//SCALE IMU READINGS// 
			// Acceleration in g's
			// Angular Velocity in rads/s 
			// ax = IMURawData[0]/g;
			// ay = IMURawData[1]/g;
			az = IMURawData[2]/g - accelOffset;
			//gx = IMURawData[3]/maxOmega;
			gy = IMURawData[4]/maxOmega*PI/180.0-.0067;
			// gz = IMURawData[5]/maxOmega;
			
			//COMPLIMENTARY FILTER FOR ANGLE//
			theta = (1-beta) * (theta - gy*deltaT) + beta*(-az);

			//LOW PASS FILTER FOR ADC//
			int i;
			for(i = 0; i < NUMADCS; i++) {
				oldFiltADCCounts[i] = filtADCCounts[i];
				filtADCCounts[i] = alpha*filtADCCounts[i] + (1-alpha)*rawADCCounts[i];
			}

			//ENCODER UPDATE//
			velocity[0] = filtADCCounts[1]<512 ? oldFiltADCCounts[0] - filtADCCounts[0] : filtADCCounts[0] - oldFiltADCCounts[0];
			velocity[1] = filtADCCounts[0]<512 ? filtADCCounts[1] - oldFiltADCCounts[1] : oldFiltADCCounts[1] - filtADCCounts[1];
			velocity[2] = filtADCCounts[3]<512 ? filtADCCounts[2] - oldFiltADCCounts[2] : oldFiltADCCounts[2] - filtADCCounts[2];
			velocity[3] = filtADCCounts[2]<512 ? oldFiltADCCounts[3] - filtADCCounts[3] : filtADCCounts[3] - oldFiltADCCounts[3];

			encoderA += velocity[0] + velocity[1];
			encoderB += velocity[2] + velocity[3];	

			accelOffset = ACCEL_OFFSET - kpen*(encoderA+encoderB) - kden*(velocity[0]+velocity[1]+velocity[2]+velocity[3]);
			// accelOffset = accelOffset > ACCEL_OFFSET_MAX ? ACCEL_OFFSET_MAX : accelOffset;
			// accelOffset = accelOffset < -ACCEL_OFFSET_MAX ? -ACCEL_OFFSET_MAX : accelOffset;
			// int16_t xDist = (encoderA + encoderB);
			// xDist = xDist > 6000 ? 6000 : xDist;
			// xDist = xDist < -6000 ? -6000 : xDist;
			
			error = theta - accelOffset;
			
			//PD DRIVE LOOP//
			balanceCommand = kpth*error + kdth*(error-lastError);
			// balanceCommand = balanceCommand > PWM_MAX ? PWM_MAX : balanceCommand;
			if(abs(theta) < .5) {
				drive(balanceCommand,(1+MOTOR_MATCH)*balanceCommand);
			} else {
				stop();
			}
			lastError = error;
			

			// USB DEBUG//
			// Send USB information for DEBUG every 100 control loop cycles
			// if (count%100 == 0) {	
			//  	m_red(TOGGLE);
			//  	m_usb_tx_long(millis());
			//  	m_usb_tx_char(' ');
			//  	m_usb_tx_int(1000*accelOffset);
			//  	m_usb_tx_char(' ');
			// 	// m_usb_tx_uint(filtADCCounts[2]);
			// 	// m_usb_tx_string(" ");
			// 	// m_usb_tx_uint(filtADCCounts[3]);
	  //   		// m_usb_tx_string(" ");
	  //   		// m_usb_tx_int(1000*ax);
	  //   		// // m_usb_tx_string(" ");
	  //   		// // m_usb_tx_int(1000*ay);
	  //   		// m_usb_tx_string(" ");
	  //   		// m_usb_tx_int(1000*az);
	  //   		// // m_usb_tx_string(" ");
	  //   		// // m_usb_tx_int(1000*gx);
	  //   		// m_usb_tx_string(" ");
	  //   		// m_usb_tx_int(1000*gy);
	  //   		// // m_usb_tx_string(" ");
	  //   		// // m_usb_tx_int(1000*gz);
	  //   		// m_usb_tx_string(" ");
	  //   		// m_usb_tx_int(1000*theta);
			// 	// m_usb_tx_string("az:  ");
			// 	// m_usb_tx_int(1000*az);
					
			// 	// m_usb_tx_string("  gy:  ");
			// 	// m_usb_tx_int(1000*gy);

 		// 		// m_usb_tx_string("  theta:  ");
 		// 		// m_usb_tx_int(1000.0*theta);

 		// 		// 	m_usb_tx_string("  kp:  ");
 		// 		// 	m_usb_tx_int(kpth);

			// 	// m_usb_tx_string("  kd:  ");
			// 	// m_usb_tx_int(kdth);

 	 //  		m_usb_tx_string("\n");
 		// // // 		//m_usb_tx_push();
			// }
			//Iterate count
			count++;
			
		}
		
		// if(m_usb_rx_available())
		// {
		// 	usb_read_command();
		// }
// 			m_usb_tx_uint(kpth);
// 			m_usb_tx_string(" ");
		
// 				while (m_usb_rx_available())
// 				{
// 					usbIn += (pow(10,charCount))*((int)m_usb_rx_char() -'0');
// 					m_usb_tx_uint(usbIn);
// 					charCount++;
// 				}
		//RF Command inputs
// 		if (isCommandReady)
// 		{	
// 			isCommandReady = FALSE;
// 		}
	}
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
	set(DIDR0, ADC6D);
	set(DIDR0, ADC7D);

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

void motor_init() {
	timer1_init();// Timer1 PWM Used for Motor PWM

	//ENABLE GPIO OUTPUT B1-5
	set(DDRB,1);
	set(DDRB,2);
	set(DDRB,3);
	set(DDRB,4);
	set(DDRB,5);
}

void timer1_init() {
	//ENABLE GPIO OUTPUT B6,7 for PWM
	set(DDRB,6);
	set(DDRB,7);

	//ENABLE MODE 15 ( UP to OCR1A, reset to 0x0000 PWM mode)
	set(TCCR1A, WGM10);
	set(TCCR1A, WGM11);
	set(TCCR1B, WGM12);
	set(TCCR1B, WGM13);

	//ENABLE COMPARE OUTPUTS B and C, SET AT ROLLOVER
	clr(TCCR1A, COM1B0);
	set(TCCR1A, COM1B1);

	clr(TCCR1A, COM1C0);
	set(TCCR1A, COM1C1);

	//Set OCRIA Max Frequency (31.4kHz) with 256 resolution for duty cycle
	OCR1A = PWM_MAX;

	//SET OCR1B Default Duty Cycle to 0;
	OCR1B = 0;
	OCR1C = 0;
		
	//ENABLE TIMER 1 WITH PRESCALER = 1
	clr(TCCR1B, CS12);
	clr(TCCR1B, CS11);
	set(TCCR1B, CS10);
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

void drive(int a, int b) {
	const uint8_t shift = SHIFT;
	set(PORTB, 1);
	if(a >= 0) {
		set(PORTB, 4);
		clr(PORTB, 5);
		OCR1B = (a+shift) > PWM_MAX ? PWM_MAX : (a+shift);
	} else {
		clr(PORTB, 4);
		set(PORTB, 5);
		OCR1B = (a-shift) < -PWM_MAX ? PWM_MAX : -(a-shift);
	}

	if(b >= 0) {
		set(PORTB, 2);
		clr(PORTB, 3);
		OCR1C = (b+shift) > PWM_MAX ? PWM_MAX : (b+shift);
	} else {
		clr(PORTB, 2);
		set(PORTB, 3);
		OCR1C = (b-shift) < -PWM_MAX ? PWM_MAX : -(b-shift);
	}


	// balanceCommand = balanceCommand > PWM_MAX ? PWM_MAX : balanceCommand;
	// OCR1B = abs(a);
	// OCR1C = abs(b);
}

// STOPS MOTOR DRIVER OPERATION
void stop() {
	clr(PORTB, 1);
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

ISR(TIMER3_COMPA_vect) {
	milliseconds++;
}