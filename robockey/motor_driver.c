
#define TIMER_1_PRSCL 1		// Timer1 Prescaler
#define PWM_FREQ 5000		// PWM Frequency (only set freq or max not both)
#define PWM_MAX (F_CPU/(TIMER_1_PRSCL*PWM_FREQ)	// PWM Duty Cycle max (inversely proportional with frequency)
#define OL_MOTOR_MATCH 0    // Open loop motor matching calibration
#define MOTOR_COMMAND_MAX 1000
#define ABS(X)   (X < 0 ?-X : X)
#define MIN(X,Y) (X < Y ? X : Y)

#define set(reg,bit)		reg |= (1<<(bit))
#define clr(reg,bit)		reg &= ~(1<<(bit))

typedef struct motor {
	int    command;
	double veloDesired;
	double prevError;
	double integError;
	double dirEncoder;
	double veloEncoder;
	double countEncoder;
	uint8_t *dutyCycleRegister;
	uint8_t *dirControl1;
	uint8_t *dirControl2;
	double kp;
	double ki;
	double kd;
} motor;


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

void motor_update(motor *m){
	
	if (m->command >= 0)
	{
		set(PORTB,dirControl1);
		clr(PORTB,dirControl2);
	}
	else
	{
		clr(PORTB,dirControl1);
		set(PORTB,dirControl2);
	}
	*(m->dutyCycleRegister) = MIN( ABS(m->command) , MOTOR_COMMAND_MAX );
}
/*
*
*
*  Modified from http://upgrayd.blogspot.com/p/pid-velocity-motor-controller.html functions
*	encoder_update
*	do_pid
*/
void encoder_update(motor *m, int A, int B){
	// Determine direction and update encoder count from the logic levels of the encoder's A and B outputs.
	if(A == 1){
		// Rising edge of A.
		if(B == 1){
			m->dirEncoder = 1;
			m->countEncoder ++;
		}
		else{
			m->dirEncoder = -1;
			m->countEncoder --;
		}
	}
	else{
		// Falling edge of A.
		if(B == 1){
			m->dirEncoder = -1;
			m->countEncoder --;
		}
		else{
			m->dirEncoder = 1;
			m->countEncoder ++;
		}
	}
}
void drive_CL(motor *m){

	float p_error;	// The difference between the desired velocity and current velocity.
	float i_error;	// The sum of errors over time.
	float d_error;	// The difference between the previous proportional error and the current proportional error.

	float p_output;	// The proportional component of the output.
	float i_output;	// The integral component of the output. This term dampens the output to prevent overshoot and oscillation.
	float d_output;	// The derivative component of the output. This term is responsible for accelerating the output if the error is large.

	float output;	// The sum of the proportional, integral and derivative terms.

	// Calculate the three errors.
	p_error = m->veloDesired - m->veloEncoder;
	i_error = m->integError;
	d_error = p_error - m->prevError;

	// Calculate the three components of the PID output.
	p_output = m->kp * p_error;
	i_output = m->ki * i_error;
	d_output = m->kd * d_error;

	// Sum the three components of the PID output.
	output = p_output + i_output + d_output;

	// Update the previous error and the integral error.
	m->prevError   = p_error;
	m->integError += p_error;

	m->command = (int) output;
}
