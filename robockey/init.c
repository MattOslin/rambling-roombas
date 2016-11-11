#include "init.h"

void m2_init() {
	// SET CLOCK TO 16MHz
	m_clockdivide(0);
	// Initialize ADC and Timers
	adc_init();    // Initializes first ADC read for encoder
	timer0_init(); // Timer0 is our control loop clock
	timer1_init(); // Timer1 PWM Used for Motor PWM
	timer3_init(); // Timer3 Used for millis clock
	// Initalize all necessary MAEVARM utilities
	m_bus_init();
	//m_rf_open(CHANNEL, MY_ADDRESS, PACKET_LENGTH); // For RF comms 
	m_usb_init(); // USB COMs for debug
	
	m_green(ON); // Ready LED
	m_disableJTAG(); //Allows use of some of the portF

	// Enable global interrupts
	sei();
}
void dd_init(dd *rob) {
	motor_GPIO_setup();

	rob->SF.reg = (uint8_t *) (&PORTB);
	rob->SF.bit = 0;
	
	rob->enable.reg = (uint8_t *) (&PORTB);
	rob->enable.bit = 1;
	/***********
	 * MOTOR 2 Specifics
	***********/
	rob->M1.dirControl1 = 2;
	rob->M1.dirControl2 = 3;
	rob->M1.dutyCycleRegister = (uint16_t*) (&OCR1C);
	rob->M1.command = 0;
	rob->M1.encA.reg = (uint8_t *) (&PIND);
	rob->M1.encA.bit = 3;
	rob->M1.encB.reg = (uint8_t *) (&PIND);
	rob->M1.encB.bit = 5;
	rob->M1.kp = CL_VEL_KP;
	rob->M1.ki = CL_VEL_KI;
	rob->M1.kd = CL_VEL_KD;

	/***********
	 * MOTOR 2 Specifics
	***********/
	rob->M2.dirControl1 = 4;
	rob->M2.dirControl2 = 5;
	rob->M2.dutyCycleRegister = (uint16_t*) (&OCR1B);
	rob->M2.command = 0;
	rob->M2.encA.reg = (uint8_t *) (&PINE);
	rob->M2.encA.bit = 6;
	rob->M2.encB.reg = (uint8_t *) (&PINC);
	rob->M2.encB.bit = 7;
	rob->M2.kp = CL_VEL_KP;
	rob->M2.ki = CL_VEL_KI;
	rob->M2.kd = CL_VEL_KD;
}

void motor_GPIO_setup() {

	//ENABLE GPIO OUTPUT B0-7
	clr(DDRB, 0); // SF: Active Low Fault Detection
	set(PORTB,0); // SF: Enable Pull Up

	set(DDRB, 1); // EN: Enable Pin
	set(DDRB, 2); // M1 DIR1 -> IN1 :  
	set(DDRB, 3); // M1 DIR2 -> IN2 : 
	set(DDRB, 4); // M2 DIR1 -> IN1 : 
	set(DDRB, 5); // M2 DIR2 -> IN2 : 
	set(DDRB, 6); // M1 PWM  ->  D2 : 
	set(DDRB, 7); // M2 PWM  ->  D2 :

	clr(DDRD, 3); // M1 Enc A Input Interrupt Pin
	set(PORTD,3); // M1 Enc A Enable Pull Up
	clr(DDRD, 5); // M1 Enc B Input Interrupt Pin
	set(PORTD,5); // M1 Enc B Enable Pull Up

	set(EIMSK,  INT3);
	clr(EICRA, ISC31);
	set(EICRA, ISC30);

	// clr(DDRE, 6); // M2 Enc A Input Interrupt Pin
	// set(PORTE,6); // M2 Enc A Enable Pull Up
	// clr(DDRC, 7); // M2 Enc B Input Interrupt Pin
	// set(PORTC,7); // M2 Enc B Enable Pull Up

	// clr(EICRA, ISC61);
	// set(EICRA, ISC60);
	// set(EIMSK,  INT6);
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

void timer1_init() {
	//ENABLE GPIO OUTPUT B6,7 for PWM
	set(DDRB,6);  // OC1B output
	set(DDRB,7);  // OC1C output

	//ENABLE MODE 15 ( UP to OCR1A, reset to 0x0000 PWM mode)
	set(TCCR1B, WGM13);
	set(TCCR1B, WGM12);
	set(TCCR1A, WGM11);
	set(TCCR1A, WGM10);

	//ENABLE COMPARE OUTPUTS B and C, SET AT ROLLOVER
	clr(TCCR1A, COM1B0);
	set(TCCR1A, COM1B1);

	clr(TCCR1A, COM1C0);
	set(TCCR1A, COM1C1);

	//Set OCRIA Max Frequency (PWM_FREQ Hz) with PWM_MAX resolution for duty cycle
	OCR1A = PWM_MAX ;

	//SET OCR1B Default Duty Cycle to 0;
	OCR1B = 0;
	OCR1C = 0;
		
	//ENABLE TIMER 1 WITH PRESCALER = 1
	clr(TCCR1B, CS12);
	clr(TCCR1B, CS11);
	set(TCCR1B, CS10);
}

void timer3_init(void) {
	// Count up to OCR3A, then reset
	clr(TCCR3B, WGM33);
	set(TCCR3B, WGM32);
	clr(TCCR3A, WGM32);
	clr(TCCR3A, WGM30);

	OCR3A = (F_CPU/1000)/TIMER_3_PRSCL; // (16 MHz / 64) * (1 ms) = 250

	set(TIMSK3, OCIE3A); // Interrupt when TCNT0 = OCR0A

	// Timer on, prescaler to /64
	clr(TCCR3B, CS32); 
	set(TCCR3B, CS31);
	set(TCCR3B, CS30);
}

uint32_t millis(void) {
	return milliseconds;
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
			//kpth = val;
			m_usb_tx_string("\n");
			m_usb_tx_string("KP: ");
			//m_usb_tx_int(kpth);
			m_usb_tx_string("\n");
			break;
		case 'D':
			//kdth = val;
			m_usb_tx_string("\n");
			m_usb_tx_string("KD: ");
			//m_usb_tx_int(kdth);
			m_usb_tx_string("\n");
			break;
		case 'B':
			//beta = val;
			m_usb_tx_string("\n");
			m_usb_tx_string("1/Beta: ");
			//m_usb_tx_int(1.0/beta);
			m_usb_tx_string("\n");
			break;
		default :
			m_usb_tx_string("NO DATA");
	}
}