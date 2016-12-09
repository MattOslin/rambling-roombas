#include "init.h"

extern uint32_t milliseconds;

void timer0_init(void);
void timer1_init(void);
void timer3_init(void); // Ping sensor timer
void timer4_init(void); // Millisecond timer
void motor_GPIO_setup(void);

void m2_init() {
	// SET CLOCK TO 16MHz
	m_clockdivide(0);
	// Initialize ADC and Timers
	adc_init();    // Initializes first ADC read for photosensors

	timer0_init(); // Timer0 is our control loop clock
	timer1_init(); // Timer1 PWM Used for Motor PWM
    timer3_init(); // Timer3 Used for ping sensor triggering pulse
    timer4_init(); // Timer4 Used for millis clock

	// Initalize all necessary MAEVARM utilities
	m_bus_init();
	m_usb_init(); // USB COMs for debug

    //while(!m_usb_isconnected()); // wait for a connection
	
	localize_init();

	m_disableJTAG(); //Allows use of some of the portF

	set(DDRB,1); // Solenoid fire pin
	set(DDRB,4); // Puck detection range change
	set(DDRD,5); // LED Red
	set(DDRD,6); // LED Blue


	// Enable global interrupts
	sei();
	// m_green(ON); // Ready LED

}

void dd_init(dd *rob) {

//	#ifdef USE_EEP_ADDRESS
		rob->myAddress = eeprom_read_byte(&eepAddress);
//	#else
//		rob->myAddress = MY_ADDRESS;
//	#endif
	m_rf_open(CHANNEL, rob->myAddress, PACKET_LENGTH);
	
	motor_GPIO_setup();
	rob->enable = FALSE;
	rob->nxtSt = 0;
	rob->ev = 0;
	rob->direction = (int8_t) eeprom_read_byte(&eepDirection);
	rob->team = eeprom_read_byte(&eepTeam);

	/***********
	 * MOTOR 1 Specifics
	***********/
	rob->M1.direct1.reg = (uint8_t *) (&PORTB);
	rob->M1.direct1.bit = 3; // GPIO Out to B3

	rob->M1.dutyCycleRegister = (uint16_t*) (&OCR1C); // Register for changing dutycycle
	
	rob->M1.command = 0;  // initialize command to motor to zero

  // switched

  rob->M1.encA.reg = (uint8_t *) (&PINE); // Encoder A Input Pin E6
  rob->M1.encA.bit = 6;

  rob->M1.encB.reg = (uint8_t *) (&PINB); // Encoder B Input Pin B0
  rob->M1.encB.bit = 0;
	/***********
	 * MOTOR 2 Specifics
	***********/
	rob->M2.direct1.reg = (uint8_t *) (&PORTB);
	rob->M2.direct1.bit = 2; // GPIO Out to B2

	rob->M2.dutyCycleRegister = (uint16_t*) (&OCR1B); // Register for changing dutycycle
	
	rob->M2.command = 0;  // initialize command to motor to zero

  rob->M2.encB.reg = (uint8_t *) (&PIND); // Encoder A Input Pin D3
  rob->M2.encB.bit = 3;

  rob->M2.encA.reg = (uint8_t *) (&PIND); // Encoder B Input Pin D4
  rob->M2.encA.bit = 4;
}

void motor_GPIO_setup() {

	//ENABLE GPIO OUTPUT B0-7

	// set(DDRB, 1); // EN: Enable Pin
	set(DDRB, 3); // M1 DIR1 -> IN1 and !IN2 : 
	set(DDRB, 2); // M2 DIR1 -> IN1 and !IN2 : 
	set(DDRB, 7); // M1 PWM  ->  D2 : 
	set(DDRB, 6); // M2 PWM  ->  D2 :

	clr(DDRD, 3); // M1 Enc A Input Interrupt Pin
	set(PORTD,3); // M1 Enc A Enable Pull Up
	clr(DDRD, 4); // M1 Enc B Input Pin
	set(PORTD,4); // M1 Enc B Enable Pull Up

	clr(EICRA, ISC31); // Set interrupt to trigger on pin change
	set(EICRA, ISC30);
	set(EIMSK,  INT3); // M1 Enc A enable interupt

	clr(DDRE, 6); // M2 Enc A Input Interrupt Pin
	set(PORTE,6); // M2 Enc A Enable Pull Up
	clr(DDRB, 0); // M2 Enc B Input Pin
	set(PORTB,0); // M2 Enc B Enable Pull Up

	clr(EICRB, ISC61); // M2 Enc A enable interupt
	set(EICRB, ISC60); // Set interrupt to trigger on pin change
	set(EIMSK,  INT6);
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
		
	//ENABLE TIMER 1 WITH PRESCALER = 8
	clr(TCCR1B, CS12);
	set(TCCR1B, CS11);
	clr(TCCR1B, CS10);
}

void timer3_init(void) {
  // Mode 7: Up to 0x03FF, PWM mode
  clr(TCCR3B, WGM33);
  set(TCCR3B, WGM32);
  set(TCCR3A, WGM31);
  set(TCCR3A, WGM30);

  set(DDRC,6);    // set output compare pin to output
  clr(DDRC,7);    // set input capture pin to input

  // output compare: set at rollover, clear at OCR3A
  set(TCCR3A,COM3A1);
  clr(TCCR3A,COM3A0);

  OCR3A = 1; // short pulse to trigger sensor
  clr(TCCR3B,ICES3); // do input capture on falling edge

  set(TIMSK3, ICIE3); // Interrupt on input capture event

  // Timer on, prescaler to 1024
  set(TCCR3B, CS32);
  clr(TCCR3B, CS31);
  set(TCCR3B, CS30);
}


void timer4_init(void) {
  // Count up to OCR4C, then reset
  clr(TCCR4D,WGM41);
  clr(TCCR4D,WGM40);

  OCR4C = 250; // (F_CPU/1000)/TIMER_4_PRSCL; // (16 MHz / 64) * (1 ms) = 250

  set(TIMSK4,TOIE4); // enable overflow interrupt

  // Timer on, prescaler to 128
  clr(TCCR4B,CS43);
  set(TCCR4B,CS42);
  set(TCCR4B,CS41);
  set(TCCR4B,CS40);

}

uint32_t millis(void) {
	return milliseconds;
}

// float atan2_aprox(float x, float y) {

// 	const float k = .28125;

// 	if (x == 0 || y == 0){
// 		if (x == y){
// 			return 0;
// 		}
// 		else{
// 			if (x == 0){
// 				return y > 0 ? PI/2 : -PI/2;
// 			}
// 			else{
// 				return x > 0 ? 0 : PI;
// 			}
// 		}
// 	}

// 	if (ABS(x) > ABS(y)){
// 		if( x > 0 ){
// 				//1
// 				//8
// 			return (x * y / ( y * y + k * x * x));
// 		}
// 		else{
// 				//4
// 				//5
// 			return (PI + x * y / ( y * y + k * x * x));
// 		}
// 	}
// 	else{
// 		if( y > 0 ){
// 				//2
// 				//3
// 			return ((PI / 2) - x * y / ( x * x + k * y * y));
// 		}
// 		else{
// 				//6
// 				//7
// 			return (-(PI / 2) - x * y / ( x * x + k * y * y));
// 		}
// 	}
// }


void shoot_puck(dd *rob, pk *puck){
	float topGoalAng=0;
	float botGoalAng=0;
	float distToGoal = GOAL_Y - (rob->global.y)*rob->direction;
	if( distToGoal < SHT_THRSH_FAR && (distToGoal > SHT_THRSH_NEAR) && (ABS(rob->global.x)<(GOAL_X + SHT_THRSH_NEAR))){
		topGoalAng = atan2(rob->direction * distToGoal, GOAL_X - rob->global.x );
		botGoalAng = atan2(rob->direction * distToGoal, -GOAL_X - rob->global.x );
	 	// m_usb_tx_string(" botGoal: ");
	 	// m_usb_tx_int(100 * botGoalAng);
	 	// m_usb_tx_string(" topGoal: ");
	 	// m_usb_tx_int(100 * topGoalAng);
	 	
		if (rob->direction == POS_Y && rob->global.th > topGoalAng && rob->global.th < botGoalAng && rob->ping > SHT_DIST_THRSH){
			rob->solenoid = ON;
		}
		else if (rob->direction == NEG_Y && rob->global.th < topGoalAng && rob->global.th > botGoalAng && rob->ping > SHT_DIST_THRSH){
			rob->solenoid = ON;
		}
	}
	return;
}

void solenoid_update(dd *rob){
	static uint32_t time;
	static bool isFiring = FALSE;
	static bool isCooldown = FALSE;
	if (isFiring){
		if ((millis() - time) > SOL_ON_TIME ){
			SOLENOID(OFF);
			isFiring = FALSE;
			time = millis();
			isCooldown = TRUE;
		}
	}
	else if (isCooldown){
		if ((millis() - time) > SOL_COOLDOWN_TIME){
			isCooldown = FALSE;
		}
	}
	else if (rob->solenoid){
		rob->solenoid = 0;
		time = millis();
		SOLENOID(ON);
		isFiring = TRUE;
	}

}

bool system_check(dd *rob){

	set(PORTD,5); // LED Red
	m_wait(500);
	set(PORTD,6); // LED Blue
	m_wait(500);
	clr(PORTD,5); // LED Red
	m_wait(500);
	clr(PORTD,6); // LED Red
	m_wait(500);
	set(PORTB,1); // Solenoid fire pin
	
	m_wait(150);
	clr(PORTB,1); // Solenoid fire pin

	m_wait(500);
	rob->veloDesired = .2;
	rob->omegaDesired = 0;
	dd_update(rob);
	
	m_wait(500);
	rob->veloDesired = -.2;
	rob->omegaDesired = 0;
	dd_update(rob);

	m_wait(500);
	rob->veloDesired = 0;
	rob->omegaDesired = .2;
	dd_update(rob);

	m_wait(500);
	rob->veloDesired = 0;
	rob->omegaDesired = -.2;
	dd_update(rob);

	m_wait(500);
	rob->veloDesired = 0;
	rob->omegaDesired = 0;
	dd_update(rob);

	int i;
	for(i=0;i<3;i++){
		set(PORTD,5); // LED Red
		m_wait(100);
		clr(PORTD,5);
		m_wait(100);
	}
	return TRUE;
}



void wall_adjust(dd *rob){
	float k = .5;

	if (rob->global.x > 100){
		if (ABS(rob->global.th) < (80 * PI / 180)){
			rob->omegaDesired += k * rob->global.th;
		}
	}
	else if(rob->global.x <-100){
		if (ABS(ANG_REMAP(rob->global.th - PI)) < (80 * PI / 180)){
			rob->omegaDesired += k * ANG_REMAP(PI - rob->global.th);	
		}
	}

}
