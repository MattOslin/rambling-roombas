
#include "init.h"


void dd_drive(dd *rob){
	// Updates desired motor velocities based on 
	//	unicycle commands of velocity and omega
	float leftMotorV, rightMotorV;

	leftMotorV = rob->veloDesired - rob->omegaDesired * WHEEL_RADIAL_LOC;
	rightMotorV = rob->veloDesired + rob->omegaDesired * WHEEL_RADIAL_LOC;

	// Normalize the velocities to the highest velocities if greater than 1

	if( (ABS(leftMotorV) > 1.0) && (ABS(leftMotorV) > ABS(rightMotorV)) ) {
		rightMotorV /= ABS(leftMotorV);
		leftMotorV /= ABS(leftMotorV);
	}
	else if((ABS(rightMotorV) > 1.0) && (ABS(rightMotorV) > ABS(leftMotorV))){
		leftMotorV /= ABS(rightMotorV);
		rightMotorV /= ABS(rightMotorV);
	}

	//Set desired velocity for motors
	rob->M1.veloDesired = leftMotorV;
	rob->M2.veloDesired = rightMotorV;
	
}

void dd_goto_loc(dd *rob){
	//Simple go to location that needs to be made more intelligent
	double deltaX  = rob->desLoc.x  - rob->global.x;
	double deltaY  = rob->desLoc.y  - rob->global.y;
	//double deltaTh = rob->desLoc.th - rob->global.th;

	if (dd_is_loc(rob)){
		rob->veloDesired  = 0;
		rob->omegaDesired = 0; //deltaTh/(2*PI)
	}

	// else{
	// 	rob->veloDesired = 
	// }
	
}

bool dd_is_loc(dd*rob){
	double deltaX  = rob->desLoc.x  - rob->global.x;
	double deltaY  = rob->desLoc.y  - rob->global.y;
	if (deltaX < POS_THRESH && deltaY < POS_THRESH){
		return TRUE;
	}
	return FALSE;

}

void dd_update(dd *rob) {
 	// Update the state of the diff drive robot
 	dd_drive(rob);
 	motor_update( &(rob->M1) );
 	motor_update( &(rob->M2) );
 }

// ENABLES MOTOR DRIVER OPERATION
void dd_enable(dd *rob) {
	set(*(rob->enable.reg),rob->enable.bit);
}
// STOPS MOTOR DRIVER OPERATION
void dd_disable(dd *rob) {
	clr(*(rob->enable.reg),rob->enable.bit);
}

void dd_set(pin *pinToToggle) {
	set(*(pinToToggle->reg),pinToToggle->bit);
}
void dd_clear(pin *pinToToggle) {
	clr(*(pinToToggle->reg),pinToToggle->bit);
}
void dd_toggle(pin *pinToToggle) {
	toggle(*(pinToToggle->reg),pinToToggle->bit);
}
bool dd_check(pin *pinToToggle) {
	return check(*(pinToToggle->reg),pinToToggle->bit);
}

// void get_fault_status(dd *rob) {
// 	rob->fault = check(*(rob->SF.reg),rob->SF.bit);
// }

void dd_comm_test(dd *rob) {
	//
}

