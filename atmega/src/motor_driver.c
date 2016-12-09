#include "motor_driver.h"
#include "init.h"

void command_update(motor *m, int newCommand);

void motor_update(motor *m) {

	if (m->command >= 0) {	
		set( *(m->direct1.reg), m->direct1.bit );
		// clr( *(m->direct2.reg), m->direct2.bit );
	} else {
		clr( *(m->direct1.reg), m->direct1.bit );
		// set( *(m->direct2.reg), m->direct2.bit );
	}

	*(m->dutyCycleRegister) = (uint16_t)PWM_MAX * (MIN( ABS( m->command ) , MOTOR_COMMAND_MAX ) / (float)MOTOR_COMMAND_MAX);

	
}
/*
*
*
*  Modified from http://upgrayd.blogspot.com/p/pid-velocity-motor-controller.html functions
*	encoder_update
*	do_pid
*/
void encoder_update(motor *m, int dir){
	// Determine direction and update encoder count from the logic levels of the encoder's A and B outputs.
	if(check(*(m->encA.reg), m->encA.bit) != check(*(m->encB.reg), m->encB.bit)) {
		m->countEncoder += dir;
	} else {
		m->countEncoder -= dir;
	}
}

void encoder_velocity(motor *m) {
	m->veloEncoder =  (1 - ALPHA_EN_LPF) * m->veloEncoder + ALPHA_EN_LPF * m->countEncoder;
	m->countEncoder = 0;
}

void drive_CL(motor *m) {
//  command_update(m, (int) (m->veloDesired * MOTOR_COMMAND_MAX));



//   m->veloEncoder  : ticks / s
//   m-> veloDesired : ticks / control frequency
//   m-> command     : 0-1000
//   ENC_RES         : ticks / rev
//   MOTOR_SPEED_MAX : rev / s
//   CTRL_FREQ       : 1 / s

	float veloEncoderScale = m->veloEncoder * (float) CTRL_FREQ / ((float) ENC_RES * (float) MOTOR_SPEED_MAX);

	float diff = veloEncoderScale - m->veloDesired;
	float p_error = MOTOR_COMMAND_MAX * diff;
	float kp = .6;
	float kd = 4;

	float control = m->command + kp * p_error + kd * (p_error - m->prevError);

	command_update(m, (int) control);

	m->prevError = p_error;
}

void drive_OL(motor *m) {
	command_update(m, (int) (m->veloDesired * MOTOR_COMMAND_MAX));
}

void command_update(motor *m, int newCommand) {
	if (newCommand < 0) {
		m->command = MAX(newCommand,-MOTOR_COMMAND_MAX);
	} else{
		m->command = MIN(newCommand, MOTOR_COMMAND_MAX);
	}
	
}
