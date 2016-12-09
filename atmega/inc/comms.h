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
#include "localize.h"
#include "macros.h"

enum rf_command {
	COMM_TEST = 0xA0, PLAY, GOAL_R, GOAL_B, PAUSE, SKIP, HALFTIME, GAME_OVER, CALIBRATE, COACH, CONTROLLER
};
void set_led(uint8_t team, uint8_t state);
bool rf_parse(unsigned char *buffer, dd *robot);
void rf_diagnostics(dd *robot);
// void usb_read_command(void);

#endif /* COMMS_H_ */