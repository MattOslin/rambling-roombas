/*
 * motor_driver.h
 *
 * Created: 11/10/2016 1:17:59 PM
 *  Author: jdcap
 */ 


#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_
#define MOTOR_COMMAND_MAX 1000
#define PWM_FREQ 5000		// PWM Frequency (only set freq or max not both)
#define PWM_MAX (F_CPU/(TIMER_1_PRSCL*PWM_FREQ))	// PWM Duty Cycle max (inversely proportional with frequency)
#define OL_MOTOR_MATCH 0    // Open loop motor matching calibration
#define CL_VEL_KP 3
#define CL_VEL_KI 0
#define CL_VEL_KD 0
#define ALPHA_EN_LPF .05

// Pin class contains register and bit for set or clearing
typedef struct pin	 {
	uint8_t *reg;	//register
	uint8_t bit;	//bit
} pin;

// Position,
// contains x, y, and theta values 

typedef struct position {
	double x;		//
	double y;
	double th;
} pos;

//Motor
//Contains relavent information for the motor including the following
typedef struct motor {

	int    command; // Latest command for motors to acheive
	uint16_t *dutyCycleRegister; // Register for chaning the duty cycle of PWM pin 
	uint8_t dirControl1; //  
	uint8_t dirControl2;
	pin encA;
	pin encB;
	double dirEncoder;
	double veloEncoder;
	double countEncoder;
	double veloDesired;
	double prevError;
	double integError;
	double kp;
	double ki;
	double kd;

} motor;

typedef struct diffDrive {
	motor M1;
	motor M2;
	pin enable;
	pin SF;
	uint8_t fault;
	pos global;
	pos velocity;
	pos goalLoc;
	pos goalVel;
} dd;

void motor_drive(int a, int b);
void motor_update(motor *m);
void encoder_update(motor *m);
void encoder_velocity(motor *m);
void drive_CL(motor *m);
void command_update(motor *m, int newCommand);

void dd_toggle(pin *pinToToggle);
void dd_clear(pin *pinToToggle);
void dd_set(pin *pinToToggle);
bool dd_check(pin *pinToToggle);
void dd_enable(dd *rob);
void dd_disable(dd *rob);
void dd_comm_test(dd *rob);


#endif /* MOTOR_DRIVER_H_ */