#ifndef DIFF_DRIVE_H_
#define DIFF_DRIVE_H_

#include "m_general.h"
#include "motor_driver.h"
#include "macros.h"
#include "m_usb.h"
// Position,
// contains x, y, and theta values 
#define WHEEL_RADIAL_LOC 52 //mm
#define WHEEL_RADIUS 60.3 //mm
#define MIN_PUCK_TURN_RAD 100

typedef struct position {
	double x;
	double y;
	double th;
} pos;

typedef struct diffDrive {
	motor M1;
	motor M2;
	bool enable;
	pos global;
	pos velocity;
	pos desLoc;
	float veloDesired;
	float omegaDesired;
	int ping;
	int ev;
	int nxtSt;
	int8_t direction;
	uint8_t team;
	uint8_t solenoid;
	uint8_t goalMade;
} dd;

void dd_enable(dd *rob);
void dd_disable(dd *rob);
void dd_update(dd *rob);
void dd_norm(dd *rob, float maxV);


bool dd_is_loc(dd *rob, float posThresh,float thThresh);
void dd_goto(dd *rob, pk *puck, float veloDes);
void dd_goto_spiral(dd *rob, float veloDes);
void dd_goto_spiral2(dd *rob, float veloDes);
bool dd_goto_rot_trans(dd *rob, float veloDes);

#endif /* DIFF_DRIVE_H_ */