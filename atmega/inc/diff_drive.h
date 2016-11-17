#ifndef DIFF_DRIVE_H_
#define DIFF_DRIVE_H_
// Position,
// contains x, y, and theta values 

#define WHEEL_RADIAL_LOC 52 //mm
#define WHEEL_RADIUS 60.3 //mm

typedef struct position {
	double x;		//
	double y;
	double th;
} pos;

typedef struct diffDrive {
	motor M1;
	motor M2;
	pin enable;
	pin SF;
	uint8_t fault;
	pos global;
	pos velocity;
	pos goalLoc;
	float veloDesired;
	float omegaDesired;
} dd;

void dd_toggle(pin *pinToToggle);
void dd_clear(pin *pinToToggle);
void dd_set(pin *pinToToggle);
bool dd_check(pin *pinToToggle);
void dd_enable(dd *rob);
void dd_disable(dd *rob);
void dd_comm_test(dd *rob);

void dd_drive(dd *rob);
void get_fault_status(dd *rob);
void dd_update(dd *rob);
#endif /* DIFF_DRIVE_H_ */