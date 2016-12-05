#include "comms.h"
//extern unsigned char buffer[PACKET_LENGTH];

void update_led(dd *robot);

void rf_parse(unsigned char *buffer, dd *robot) {
	switch(buffer[0]) {
		case COMM_TEST:
			//FLASH LEDS
			dd_disable(robot);
			dd_comm_test(robot);
			update_led(robot);
			break;
		
		case PLAY:
			// SET STATE TO PLAY 
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
			dd_disable(robot);
			break;
		
		case HALFTIME:
			//PAUSE
			dd_disable(robot);
			if (robot->direction == POS_Y) {
			// if (eeprom_read_byte(&eepDirection) == POS_Y) {
				eeprom_write_byte(&eepDirection, NEG_Y);
				robot->direction = NEG_Y;
			} else {
				eeprom_write_byte(&eepDirection, POS_Y);
				robot->direction = POS_Y;
			}
			break;

		case GAME_OVER:
			//PAUSE
			dd_disable(robot);
			break;

		case CONTROLLER:
			dd_enable(robot);
			robot->veloDesired  = .5 * (buffer[2]-131) / 128.0;
			robot->omegaDesired = .5 * (buffer[1]-124) / 128.0;
			break;

		case CALIBRATE:
			//calibrate_routine();

			dd_enable(robot);
			robot->veloDesired = 0;
			robot->omegaDesired = .2;
			break;

		case COACH:
			eeprom_write_byte(&eepDirection, buffer[2]);
			eeprom_write_byte(&eepTeam, buffer[3]);
			robot->direction = eeprom_read_byte(&eepDirection);
			robot->team = eeprom_read_byte(&eepTeam);
			buffer[4] = eeprom_read_byte(&eepAddress);
			buffer[5] = robot->direction;
			buffer[6] = robot->team;
			m_rf_send(buffer[1], buffer, PACKET_LENGTH);
			update_led(robot);

			break;

		default:
			//TEAM COMMANDS GO IN THEIR OWN CASES
			dd_disable(robot);
			break;
	}
}

void rf_diagnostics(dd *robot) {
	;
}

void usb_read_command() {	
	char buff[8];
	unsigned int indx = 0;
	int val = 0;
	int i;

	while(m_usb_rx_available()&&indx<8){
		buff[indx] = m_usb_rx_char();
		indx++;
	}
	
	for(i=indx-1; i > 0; i--){
		val += ((int)buff[i]-'0')*pow(10, indx-i-1);//Introduces mistakes in integer math (rounds down)
		m_usb_tx_int((int)buff[i]-'0');
		m_usb_tx_string("\n");
	}
	switch(buff[0]){
		case 'P':
			//kpth = val;
			m_usb_tx_string("\n");
			m_usb_tx_string("KP: ");
			//m_usb_tx_int(kpth);
			m_usb_tx_string("\n");
			break;
		case 'D':
			//kdth = val;
			m_usb_tx_string("\n");
			m_usb_tx_string("KD: ");
			//m_usb_tx_int(kdth);
			m_usb_tx_string("\n");
			break;
		case 'B':
			//beta = val;
			m_usb_tx_string("\n");
			m_usb_tx_string("1/Beta: ");
			//m_usb_tx_int(1.0/beta);
			m_usb_tx_string("\n");
			break;
		default :
			m_usb_tx_string("NO DATA");
	}
}

void update_led(dd *robot) {
	uint8_t redPin = 5;
	uint8_t bluePin = 6;
	uint8_t blinkPin;

	if (robot->team == RED) {
		set(PORTD, redPin);
		clr(PORTD, bluePin);
		blinkPin = redPin;
	} else {
		clr(PORTD, redPin);
		set(PORTD, bluePin);
		blinkPin = bluePin;
	}

	if (robot->direction == POS_Y) {
		m_wait(250);
		clr(PORTD, blinkPin);
		m_wait(250);
		set(PORTD, blinkPin);
		m_wait(250);
		clr(PORTD, blinkPin);
		m_wait(250);
		set(PORTD, blinkPin);
		m_wait(250);
		clr(PORTD, blinkPin);
	} else {
		m_wait(75);
		clr(PORTD, blinkPin);
		m_wait(75);
		set(PORTD, blinkPin);
		m_wait(75);
		clr(PORTD, blinkPin);
		m_wait(75);
		set(PORTD, blinkPin);
		m_wait(75);
		clr(PORTD, blinkPin);
		m_wait(75);
		set(PORTD, blinkPin);
		m_wait(75);
		clr(PORTD, blinkPin);
		m_wait(75);
		set(PORTD, blinkPin);
		m_wait(75);
		clr(PORTD, blinkPin);
		m_wait(75);
		set(PORTD, blinkPin);
		m_wait(75);
		clr(PORTD, blinkPin);
	}
}