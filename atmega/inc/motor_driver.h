/*
 * motor_driver.h
 *
 * Created: 11/10/2016 1:17:59 PM
 *  Author: jdcap
 */ 


#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

#include "m_general.h"
#include "macros.h"

#define MOTOR_COMMAND_MAX 1000
#define PWM_FREQ 100		// PWM Frequency (only set freq or max not both)
#define PWM_MAX (F_CPU/(TIMER_1_PRSCL*PWM_FREQ))	// PWM Duty Cycle max (inversely proportional with frequency)
#define MOTOR_SPEED_MAX 8.3   // Max attempted Rotations per second
#define ENC_RES 32*18.75	//Number of edges per rotation of output shaft in the encoders (using one interrupt)
#define OL_MOTOR_MATCH 0    // Open loop motor matching calibration
#define CL_VEL_KP 2
#define CL_VEL_KI 0
#define CL_VEL_KD 0
#define ALPHA_EN_LPF .7

// Pin class contains register and bit for set or clearing
typedef struct pin	 {
	uint8_t *reg;	//register
	uint8_t bit;	//bit
} pin;


//Motor
//Contains relavent information for the motor including the following
typedef struct motor {

	int    command; // Latest command for motors to acheive
	uint16_t *dutyCycleRegister; // Register for changing the duty cycle of PWM pin 
	pin direct1; 
	pin encA;
	pin encB;
	//float dirEncoder;
	float veloEncoder;
	float countEncoder;
	float veloDesired;
	float prevError;
	float integError;
	float kp;
	float ki;
	float kd;

} motor;

void drive_OL(motor *m);
void motor_update(motor *m);
void encoder_update(motor *m, int dir);
void encoder_velocity(motor *m);
void drive_CL(motor *m);

#endif /* MOTOR_DRIVER_H_ */
