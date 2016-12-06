
#include "diff_drive.h"

void dd_drive(dd *rob);
void dd_toggle(pin *pinToToggle);
void dd_clear(pin *pinToToggle);
void dd_set(pin *pinToToggle);
bool dd_check(pin *pinToToggle);

void dd_drive(dd *rob){
	if(rob->enable){

		// Updates desired motor velocities based on 
		//	unicycle commands of velocity and omega
		float leftMotorV, rightMotorV;

		leftMotorV = rob->veloDesired - rob->omegaDesired ;//* WHEEL_RADIAL_LOC;
		rightMotorV = rob->veloDesired + rob->omegaDesired ;//* WHEEL_RADIAL_LOC;

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
		rob->M1.veloDesired = rightMotorV;
		rob->M2.veloDesired = leftMotorV;
	}
	else{

		rob->M1.veloDesired = 0;
		rob->M2.veloDesired = 0;
	}
	
}

bool dd_goto_rot_trans(dd *rob, float veloDes){

	float deltaX  = rob->desLoc.x  - rob->global.x;
	float deltaY  = rob->desLoc.y  - rob->global.y;
	float deltaTh = ANG_REMAP(rob->desLoc.th - rob->global.th);
	float distFromGoal = sqrt(deltaX * deltaX + deltaY * deltaY);

	float posThresh = 10.0;
	float thThresh = 5 * PI / 180;
	if ( distFromGoal < posThresh ) {
		if ( ABS(deltaTh) < thThresh ) {
			rob->veloDesired  = 0;
			rob->omegaDesired = 0;
			return TRUE;
		}
		rob->veloDesired  = 0;
		rob->omegaDesired = ABS(deltaTh/(PI/2)) > 1 ? veloDes * deltaTh/ABS(deltaTh) : veloDes * deltaTh/(PI/2);
		return FALSE;
	}
	else {
		float thTemp = atan2(deltaY,deltaX);
		float deltaTh = thTemp - rob->global.th;
		if ( ABS(deltaTh) < thThresh ){
			rob->veloDesired  = veloDes;
			rob->omegaDesired = 0;
			return FALSE;
		}
		rob->veloDesired  = 0;
		rob->omegaDesired = ABS(deltaTh/(PI/2)) > 1 ? veloDes * deltaTh/ABS(deltaTh) : veloDes * deltaTh/(PI/2);
		return FALSE;
	}
}

void dd_goto_spiral(dd *rob, float veloDes){
	//Using J.J. Park and B Kuipers paper for smooth diff drive control
	//Unclear whether smooth is good or too slow or what
	//Simple go to location that needs to be made more intelligent
	float K1 = 0.5;
	float K2 = 5;

	float deltaX  = rob->desLoc.x  - rob->global.x;
	float deltaY  = rob->desLoc.y  - rob->global.y;
	//float deltaTh = rob->desLoc.th - rob->global.th;
	float gamma = atan2( deltaY, deltaX);
	float del = ANG_REMAP(rob->global.th - gamma);
	float theta = ANG_REMAP(rob->desLoc.th - gamma);
	float r = sqrt( deltaX * deltaX + deltaY * deltaY);
	
	float posThresh = 20.0;
	if( r < posThresh){
		// rob->veloDesired  = veloDes * r / posThresh;
		// rob->omegaDesired = - (veloDes / posThresh) * (K2 * (del - atan(-K1 * theta))
	 //                    + ( 1 + K1 / (1 + K1 * K1 * theta * theta)) * sin(del));
		rob->veloDesired = 0;
	    return;
	}

	rob->veloDesired = veloDes;
	rob->omegaDesired = - (veloDes / r) *(K2 * (del - atan(-K1 * theta))
	                    + ( 1 + K1 / (1 + K1 * K1 * theta * theta)) * sin(del));
	
}

bool dd_is_loc(dd*rob , float posThresh, float thThresh){
	float deltaX  = rob->desLoc.x  - rob->global.x;
	float deltaY  = rob->desLoc.y  - rob->global.y;
	float deltaTh  =rob->desLoc.th  - rob->global.th;


	if (deltaX < posThresh && deltaY < posThresh && ANG_REMAP(deltaTh) < ANG_REMAP(thThresh)){
		return TRUE;
	}
	return FALSE;

}

void dd_update(dd *rob) {
 	// Update the state of the diff drive robot
 	dd_drive( rob );
	encoder_velocity( &(rob->M1) );
	encoder_velocity( &(rob->M2) );
 	drive_OL( &(rob->M1) );
 	drive_OL( &(rob->M2) );
 	motor_update( &(rob->M1) );
 	motor_update( &(rob->M2) );
 }

// ENABLES MOTOR DRIVER OPERATION
void dd_enable(dd *rob) {
	rob->enable = TRUE;
}
// STOPS MOTOR DRIVER OPERATION
void dd_disable(dd *rob) {
	rob->enable = FALSE;
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
float get_radius_curv(dd *rob){
	if (rob->omegaDesired <.001){
		return 1000.0;
	}
	return rob->veloDesired/rob->omegaDesired;// actual equation Vdes/omDes = -R/(2*Rr)
}
