
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

		leftMotorV = -rob->veloDesired - rob->omegaDesired ;//* WHEEL_RADIAL_LOC;
		rightMotorV = -rob->veloDesired + rob->omegaDesired ;//* WHEEL_RADIAL_LOC;

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
    rob->M1.veloDesired = leftMotorV*.8;
    rob->M2.veloDesired = rightMotorV*.8;
	}
	else{

		rob->M1.veloDesired = 0;
		rob->M2.veloDesired = 0;
	}
	
}


bool dd_goto_rot_trans(dd *rob, float veloDes){
	static float thPrev = 0;
	static float thInt = 0;

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
	}
	
	float thTemp = atan2(deltaY,deltaX);
	deltaTh = ANG_REMAP(thTemp - rob->global.th);
	rob->veloDesired  = 0;
	float kp = 1;
	float ki = 0;
	float kd = 1;

	// float k1 = .4;
	// float k2 = 2;
	rob->omegaDesired = kp * deltaTh + ki * thInt + kd * 100 * (deltaTh - thPrev);
	thPrev = deltaTh;
	thInt = MIN(.1/ki,thInt + deltaTh);
	return FALSE;
}


void dd_norm(dd *rob, float maxV){
	if( (ABS(rob->veloDesired) > ABS(rob->omegaDesired)) ) {
		rob->omegaDesired /= ABS(rob->veloDesired);
		rob->veloDesired /= ABS(rob->veloDesired);
	}
	else {
		rob->omegaDesired /= ABS(rob->omegaDesired);
		rob->veloDesired /= ABS(rob->omegaDesired);
	}
	rob->veloDesired = maxV*rob->veloDesired;
	rob->omegaDesired = maxV*rob->omegaDesired;
}

void dd_goto(dd *rob, pk *puck, float veloDes){
	static float prevAlpha = 0;
    static float prevPhi = 0;
    float kp = 2;
    float kd = 6;
    float k1 = veloDes;//.4;
    float k2 = 2;
    float kap = 0;//1.5;
    float kad = CTRL_FREQ * 0;//;.1;
    float gamma = atan2(rob->desLoc.y - rob->global.y, rob->desLoc.x - rob->global.x);
    float phi = ANG_REMAP(gamma - rob->global.th);
    float alpha = ANG_REMAP(gamma - rob->desLoc.th);
    rob->omegaDesired = kp * phi  + kd * (phi - prevPhi)+ kap * alpha - kad * (alpha - prevAlpha);
    rob->veloDesired = k1 / (k2 * ABS(rob->omegaDesired) + 1);
    if (puck->isHave){
    	rob->veloDesired = MAX(rob->veloDesired, 2 * MIN_PUCK_TURN_RAD * ABS(rob->omegaDesired) / WHEEL_RADIAL_LOC);
    }
    // dd_norm(rob,veloDes);
    prevAlpha = alpha;
    prevPhi = phi;  
}
bool dd_in_goal(dd*rob , float posThresh){
	if (ABS(rob->global.x) < posThresh && ABS(rob->global.y + rob->direction * 250) < posThresh){
		return TRUE;
	}
	
	return FALSE;
	
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

bool dd_theta_control(dd *rob, float theta_desired){

	static float theta_prev = 0;
	static float theta_int = 0;
	float kpa = 1;
	float kia = .03;
	float kda = -.05;

	float w = ANG_REMAP(rob->global.th - theta_prev) * CTRL_FREQ;
	float d_th = ANG_REMAP(theta_desired-rob->global.th);

	rob->veloDesired  = 0;
	rob->omegaDesired = kpa * d_th + theta_int + kda * w;
	theta_int += kia * d_th;
	theta_prev = rob->global.th;
	//  m_usb_tx_int(100*d_th);
	//  m_usb_tx_string(" ");
	//  m_usb_tx_int(100*w);
	//  m_usb_tx_string(" ");
	if(ABS(d_th) < .03 && ABS(w) < .1) {
		theta_int = 0;
		return true;
	} 
	else {
		return false;
	}

}

void dd_update(dd *rob) {
 	// Update the state of the diff drive robot
 	dd_drive( rob );
	encoder_velocity( &(rob->M1) );
	encoder_velocity( &(rob->M2) );
 	drive_CL( &(rob->M1) );
 	drive_CL( &(rob->M2) );
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






// void dd_goto_spiral(dd *rob, float veloDes){
// 	//DOESNT WORK
// 	//Using J.J. Park and B Kuipers paper for smooth diff drive control
// 	//Unclear whether smooth is good or too slow or what
// 	//Simple go to location that needs to be made more intelligent
// 	float K1 = 2;
// 	float K2 = 5;

// 	float deltaX  = rob->desLoc.x  - rob->global.x;
// 	float deltaY  = rob->desLoc.y  - rob->global.y;
// 	//float deltaTh = rob->desLoc.th - rob->global.th;
// 	float gamma = atan2( deltaY, deltaX);
// 	float del = ANG_REMAP(rob->global.th - gamma);
// 	float theta = ANG_REMAP(rob->desLoc.th - gamma);
// 	float r = sqrt( deltaX * deltaX + deltaY * deltaY);
	
// 	float posThresh = 50.0;
// 	if( r < posThresh){
// 		// rob->veloDesired  = veloDes * r / posThresh;
// 		// rob->omegaDesired = - (veloDes / posThresh) * (K2 * (del - atan(-K1 * theta))
// 	 //                    + ( 1 + K1 / (1 + K1 * K1 * theta * theta)) * sin(del));
// 		rob->veloDesired = 0;
// 		rob->omegaDesired = 0;
// 	    return;
// 	}

// 	rob->veloDesired = veloDes;
// 	rob->omegaDesired = - (veloDes / r) *(K2 * (del - atan(-K1 * theta))
// 	                    + ( 1 + K1 / (1 + K1 * K1 * theta * theta)) * sin(del));
	
// }

// void dd_goto_spiral2(dd *robot, float veloDes){
// 	//DOESNT WORK
//    // printf("puck_to_goal\t");

//     float goalTh = PI/2;

//     static float prevAlpha = 0;
//     static float prevPhi = 0;
//     float kp = 2;
//     float kd = 6;
//     float k1 = .4;
//     float k2 = 2;
//     float kap = 0;//1.5;
//     float kad = 0;//;.1;
//     float gamma = atan2(robot->desLoc.y - robot->global.y, robot->desLoc.x - robot->global.x);
//     float phi = ANG_REMAP(gamma - robot->global.th);
//     float alpha = ANG_REMAP(gamma - goalTh);
//     robot->omegaDesired = kp * phi  + kd * (phi - prevPhi)+ kap * alpha - kad * prevAlpha;
//     robot->veloDesired = k1 / (k2 * ABS(robot->omegaDesired) + 1);
//     robot->veloDesired = MAX(robot->veloDesired, 2 * MIN_PUCK_TURN_RAD * robot->omegaDesired / WHEEL_RADIAL_LOC);
//     dd_norm(robot,.3);
//     prevAlpha = alpha;
//     prevPhi = phi;
// }
// 	//Using J.J. Park and B Kuipers paper for smooth diff drive control
// 	//Unclear whether smooth is good or too slow or what
// 	//Simple go to location that needs to be made more intelligent
// 	float Krho = .01;
// 	float Kalpha = .5;
// 	float Kbeta = 1;

// 	float deltaX  = rob->desLoc.x  - rob->global.x;
// 	float deltaY  = rob->desLoc.y  - rob->global.y;
// 	//float deltaTh = rob->desLoc.th - rob->global.th;
// 	float gamma = atan2( deltaY, deltaX);
// 	float alpha = ANG_REMAP(gamma - rob->global.th);
// 	float beta = ANG_REMAP(-alpha - rob->global.th + rob->desLoc.th);
// 	float rho = sqrt( deltaX * deltaX + deltaY * deltaY);
	
// 	// m_usb_tx_string(" alpha: ");
// 	// m_usb_tx_int(100*alpha);

// 	// m_usb_tx_string(" beta: ");
// 	// m_usb_tx_int(100*beta);	

// 	// m_usb_tx_string(" gamma: ");
// 	// m_usb_tx_int(100*gamma);

// 	// m_usb_tx_string(" rho: ");
// 	// m_usb_tx_int(rho);

// 	// m_usb_tx_string("\n");

// 	float posThresh = 50.0;
// 	if( rho < posThresh){
// 		// rob->veloDesired  = veloDes * r / posThresh;
// 		// rob->omegaDesired = - (veloDes / posThresh) * (K2 * (del - atan(-K1 * theta))
// 	 //                    + ( 1 + K1 / (1 + K1 * K1 * theta * theta)) * sin(del));
// 		rob->veloDesired = 0;
// 		rob->omegaDesired = 0;
// 	    return;
// 	}

// 	rob->veloDesired = Krho * rho;
// 	rob->omegaDesired = (Kalpha * alpha - Kbeta * beta);

// 	if( (ABS(rob->veloDesired) > 1.0) && (ABS(rob->veloDesired) > ABS(rob->omegaDesired)) ) {
// 		rob->omegaDesired /= ABS(rob->veloDesired);
// 		rob->veloDesired /= ABS(rob->veloDesired);
// 	}
// 	else if((ABS(rob->omegaDesired) > 1.0) && (ABS(rob->omegaDesired) > ABS(rob->veloDesired))){
// 		rob->omegaDesired /= ABS(rob->omegaDesired);
// 		rob->veloDesired /= ABS(rob->omegaDesired);
// 	}
// 	rob->veloDesired = veloDes*rob->veloDesired;
// 	rob->omegaDesired = veloDes*rob->omegaDesired;

// }
