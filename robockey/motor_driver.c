
#define TIMER_1_PRSCL 1		// Timer1 Prescaler
#define PWM_FREQ 5000		// PWM Frequency (only set freq or max not both)
#define PWM_MAX (F_CPU/(TIMER_1_PRSCL*PWM_FREQ)	// PWM Duty Cycle max (inversely proportional with frequency)
#define OL_MOTOR_MATCH 0    // Open loop motor matching calibration
#define MOTOR_COMMAND_MAX 1000

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
	OCR1A = PWM_MAX;

	//SET OCR1B Default Duty Cycle to 0;
	OCR1B = 0;
	OCR1C = 0;
		
	//ENABLE TIMER 1 WITH PRESCALER = 1
	clr(TCCR1B, CS12);
	clr(TCCR1B, CS11);
	set(TCCR1B, CS10);
}

void motor_init() {

	//ENABLE GPIO OUTPUT B1-7
	set(DDRB,0); // SF: Active Low Fault Detection
	set(DDRB,1); // EN:      Enable Pin
	set(DDRB,2); // M1 IN1:  
	set(DDRB,3); // M1 IN2: 
	set(DDRB,4); // M2 IN1: 
	set(DDRB,5); // M2 IN2: 
	set(DDRB,6); // M1 PWM D2: 
	set(DDRB,7); // M2 PWM D2:
	set(DDRD,3); // M1 Encoder A Interupt
	set(DDRD,5); // M1 Encoder B
	set(DDRE,6); // M2 Encoder A Interupt
	set(DDRC,7); // M2 Encoder B 

	timer1_init(); // Timer1 PWM Used for Motor PWM

}

void motor_drive(int a, int b) {

	a = ( (float) PWM_MAX * a ) / MOTOR_COMMAND_MAX + OL_MOTOR_MATCH;
	b = ( (float) PWM_MAX * b ) / MOTOR_COMMAND_MAX - OL_MOTOR_MATCH;
	

	if(a >= 0) {
		set(PORTB, 2);
		clr(PORTB, 3);
		OCR1B = (a) > PWM_MAX ? PWM_MAX : (a);
	} else {
		clr(PORTB, 2);
		set(PORTB, 3);
		OCR1B = (a) < -PWM_MAX ? PWM_MAX : -(a);
	}

	if(b >= 0) {
		set(PORTB, 4);
		clr(PORTB, 5);
		OCR1C = (b) > PWM_MAX ? PWM_MAX : (b);
	} else {
		clr(PORTB, 4);
		set(PORTB, 5);
		OCR1C = (b) < -PWM_MAX ? PWM_MAX : -(b);
	}
}

// ENABLES MOTOR DRIVER OPERATION
void motor_enable() {
	set(PORTB, 1);
}

// STOPS MOTOR DRIVER OPERATION
void motor_disable() {
	clr(PORTB, 1);
}
