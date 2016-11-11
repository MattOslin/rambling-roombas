/*
 * motor_driver.h
 *
 * Created: 11/10/2016 1:17:59 PM
 *  Author: jdcap
 */ 


#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_
#define MOTOR_COMMAND_MAX 1000

typedef struct pin	 {
	uint8_t *reg;
	uint8_t bit;
} pin;

typedef struct motor {
	int    command;

	uint16_t *dutyCycleRegister;
	uint8_t dirControl1;
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
void timer1_init();
void motor_init();
void motor_drive(int a, int b);
void motor_enable();
void motors_disable();
void motor_update(motor *m);
void encoder_update(motor *m);
void encoder_velocity(motor *m);
void drive_CL(motor *m);

#endif /* MOTOR_DRIVER_H_ */