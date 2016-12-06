/*
 * motor_driver.h
 *
 * Created: 11/10/2016 1:17:59 PM
 *  Author: jdcap
 */ 


#ifndef COMMS_H_
#define COMMS_H_

#include "m_bus.h"
#include "m_usb.h"
#include "m_rf.h"
#include "diff_drive.h"
#include "eep_locations.h"
#include "macros.h"

enum rf_command {
	COMM_TEST = 0xA0, PLAY, GOAL_R, GOAL_B, PAUSE, HALFTIME, GAME_OVER, CONTROLLER, CALIBRATE, COACH
};

void rf_parse(unsigned char *buffer, dd *robot);
void rf_diagnostics(dd *robot);
// void usb_read_command(void);

#endif /* COMMS_H_ */