#include "comms.h"
//extern unsigned char buffer[PACKET_LENGTH];

void flash_led(dd *robot);

bool rf_parse(unsigned char *buffer, dd *robot) {
	switch(buffer[0]) {
		case COMM_TEST:
			//FLASH LEDS
			dd_disable(robot);
			flash_led(robot);
			break;
		
		case PLAY:
			// SET STATE TO PLAY 
			set_led(robot->team);
			dd_enable(robot);
			break;
			
		case GOAL_R:
			// PAUSE
			// GET SCORE A AND SCORE B FROM BUFFER
			if(robot->team == RED) {
				robot->goalMade = 1;
			}
			else {
				dd_disable(robot);
			}

			break;

		case GOAL_B:
			// PAUSE
			// GET SCORE A AND SCORE B FROM BUFFER
			if(robot->team == BLUE) {
				robot->goalMade = 1;
			}
			else {
				dd_disable(robot);
			}
			break;
		
		case PAUSE:
			//PAUSE
			set_led(NONE);
			dd_disable(robot);
			break;
		
		case HALFTIME:
			//PAUSE
			dd_disable(robot);
			if (robot->direction == POS_Y) {
			// if (eeprom_read_byte(&eepDirection) == POS_Y) {
				eeprom_write_byte(&eepDirection, (uint8_t) NEG_Y);
				robot->direction = NEG_Y;
			} else {
				eeprom_write_byte(&eepDirection, (uint8_t) POS_Y);
				robot->direction = POS_Y;
			}
			flash_led(robot);
			break;

		case GAME_OVER:
			//PAUSE
			set_led(NONE);
			dd_disable(robot);
			break;

		case CONTROLLER:
			dd_enable(robot);
			robot->veloDesired  = .5 * (buffer[2]-131) / 128.0;
			robot->omegaDesired = .5 * (buffer[1]-124) / 128.0;
			break;

		case CALIBRATE:
			//calibrate_routine();

			set_led(NONE);
			m_wait(50);
			set_led(PURPLE);
			m_wait(50);
			set_led(NONE);
			m_wait(50);
			set_led(PURPLE);
			m_wait(50);
			set_led(NONE);
			m_wait(50);
			set_led(PURPLE);
			m_wait(50);
			set_led(NONE);


			// if(localize_cal(&(robot->global))) {
			// 	set_led(PURPLE);
   //    		} else {
   //      		set_led(BLUE,ON);
   //    		}

			return true;
			break;

		case COACH:
			eeprom_write_byte(&eepDirection, (uint8_t) buffer[2]);
			eeprom_write_byte(&eepTeam, buffer[3]);
			robot->direction = (int8_t) eeprom_read_byte(&eepDirection);
			robot->team = eeprom_read_byte(&eepTeam);
			buffer[4] = eeprom_read_byte(&eepAddress);
			buffer[5] = (uint8_t) robot->direction;
			buffer[6] = robot->team;
			m_rf_send(buffer[1], buffer, PACKET_LENGTH);
			flash_led(robot);

			break;

		default:
			//TEAM COMMANDS GO IN THEIR OWN CASES
			dd_disable(robot);
			break;
	}
	return false;
}

void set_led(uint8_t led_command) {
	if (led_command == PURPLE) {
		set(PORTD, PIN_RED);
		set(PORTD, PIN_BLUE);
	} else if (led_command == RED) {
		set(PORTD, PIN_RED);
		clr(PORTD, PIN_BLUE);
	} else if (led_command == BLUE) {
		clr(PORTD, PIN_RED);
		set(PORTD, PIN_BLUE);
	} else {
		clr(PORTD, PIN_RED);
		clr(PORTD, PIN_BLUE);
	}
} 

void flash_led(dd *robot) {
	uint8_t blinkPin;

	if (robot->team == RED) {
		set(PORTD, PIN_RED);
		clr(PORTD, PIN_BLUE);
		blinkPin = PIN_RED;
	} else {
		clr(PORTD, PIN_RED);
		set(PORTD, PIN_BLUE);
		blinkPin = PIN_BLUE;
	}

	int i;
	if (robot->direction == POS_Y) {
		m_wait(250);
		clr(PORTD, blinkPin);
		for (i = 0; i < 2; i++) {
			m_wait(250);
			set(PORTD, blinkPin);
			m_wait(250);
			clr(PORTD, blinkPin);
		}
	} else {
		m_wait(75);
		clr(PORTD, blinkPin);
		for(i = 0; i < 5; i++) {
			m_wait(75);
			set(PORTD, blinkPin);
			m_wait(75);
			clr(PORTD, blinkPin);
		}
	}
}
