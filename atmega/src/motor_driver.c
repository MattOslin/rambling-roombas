#include "motor_driver.h"
#include "init.h"

void command_update(motor *m, int newCommand);

void motor_update(motor *m){

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
  }//	if(check(*(m->encA.reg), m->encA.bit) == 1){
//		// Rising edge of A.
//		if(check(*(m->encB.reg), m->encB.bit) == 1){
//			//m->dirEncoder = 1;
//			m->countEncoder-=dir;
//		}
//		else{
//			//m->dirEncoder = -1;
//			m->countEncoder+=dir;
//		}
//	}
//	else{
//		// Falling edge of A.
//		if(check(*(m->encB.reg), m->encB.bit) == 1){
//			//m->dirEncoder = -1;
//			m->countEncoder+=dir;
//		}
//		else{
//			//m->dirEncoder = 1;
//			m->countEncoder-=dir;
//		}
//	}
}

void encoder_velocity(motor *m)
{
	m->veloEncoder =  (1 - ALPHA_EN_LPF) * m->veloEncoder + ALPHA_EN_LPF * m->countEncoder;
	m->countEncoder = 0;
}
void drive_CL(motor *m){


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

//  m_usb_tx_int(m->veloEncoder);
//  m_usb_tx_string(" ");
//  m_usb_tx_int(veloEncoderScale*1000);
//  m_usb_tx_string(" ");
//  m_usb_tx_int(m->veloDesired*1000);
//  m_usb_tx_string(" ");
//  m_usb_tx_int(diff*1000);
//  m_usb_tx_string(" ");

  float control = m->command + kp * p_error + kd * (p_error - m->prevError);
//  m_usb_tx_int(control);
//  m_usb_tx_string(" ");
//
//  m_usb_tx_int(m->veloDesired*1000);
//  m_usb_tx_string(" ");
//  m_usb_tx_int(m->veloEncoder*100);
//  m_usb_tx_string(" ");
//  m_usb_tx_int(p_error*100);
//  m_usb_tx_string(" ");
//  m_usb_tx_int(control);

//  m_usb_tx_string("\n");

  command_update(m, (int) control);

  m->prevError = p_error;


//  command_update(m, (int) (m->veloDesired * MOTOR_COMMAND_MAX));

//	float p_error;	// The difference between the desired velocity and current velocity.
//	float i_error;	// The sum of errors over time.
//	float d_error;	// The difference between the previous proportional error and the current proportional error.
//
//	float p_output;	// The proportional component of the output.
//	float i_output;	// The integral component of the output. This term dampens the output to prevent overshoot and oscillation.
//	float d_output;	// The derivative component of the output. This term is responsible for accelerating the output if the error is large.
//
//	float output;	// The sum of the proportional, integral and derivative terms.
//
//	// Calculate the three errors.
//	p_error = MOTOR_SPEED_MAX * ENC_RES * m->veloDesired / CTRL_FREQ - m->veloEncoder;
//	i_error = m->integError;
//	d_error = p_error - m->prevError;
//
//	// Calculate the three components of the PID output.
//	p_output = m->kp * p_error;
//	i_output = m->ki * i_error;
//	d_output = m->kd * d_error;
//
//	// Sum the three components of the PID output.
//	output = m->command - p_output + i_output + d_output;
//
//	// Update the previous error and the integral error.
//	m->prevError   = p_error;
//	m->integError += p_error;

//  	command_update(m, (int) output);


}

void drive_OL(motor *m){
	command_update(m, (int) (m->veloDesired * MOTOR_COMMAND_MAX));
}

void command_update(motor *m, int newCommand){
	if (newCommand < 0){
		m->command = MAX(newCommand,-MOTOR_COMMAND_MAX);
	}
	else{
		m->command = MIN(newCommand, MOTOR_COMMAND_MAX);
	}
	
}
