
#include "init.h"



// ENABLES MOTOR DRIVER OPERATION
void dd_enable(*rob) {
	set(*(rob->enable.reg),rob->enable.bit);
}

// STOPS MOTOR DRIVER OPERATION
void dd_disable() {
	clr(*(rob->enable.reg),rob->enable.bit);
}

void motor_update(motor *m){
	
	if (m->command >= 0)
	{
		set(PORTB,m->dirControl1);
		clr(PORTB,m->dirControl2);
	}
	else
	{
		clr(PORTB,m->dirControl1);
		set(PORTB,m->dirControl2);
	}
	
	*(m->dutyCycleRegister) = PWM_MAX * MIN( ABS( m->command ) , MOTOR_COMMAND_MAX ) / MOTOR_COMMAND_MAX;
	encoder_velocity(m);
}
/*
*
*
*  Modified from http://upgrayd.blogspot.com/p/pid-velocity-motor-controller.html functions
*	encoder_update
*	do_pid
*/
void encoder_update(motor *m){
	// Determine direction and update encoder count from the logic levels of the encoder's A and B outputs.
	if(check(*(m->encA.reg), m->encA.bit) == 1){
		// Rising edge of A.
		if(check(*(m->encB.reg), m->encB.bit) == 1){
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
		if(check(*(m->encB.reg), m->encB.bit) == 1){
			m->dirEncoder = -1;
			m->countEncoder --;
		}
		else{
			m->dirEncoder = 1;
			m->countEncoder ++;
		}
	}
}
void encoder_velocity(motor *m)
{
	m->veloEncoder = ABS(m->countEncoder);
	m->countEncoder = 0;
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

void get_fault_status(dd *rob) {
	rob->fault = check(*(rob->SF.reg),rob->SF.bit);
}