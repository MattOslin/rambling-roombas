#include "init.h"

extern unsigned char buffer[PACKET_LENGTH];

void rf_parse(unsigned char *buffer) {
	switch(buffer[0]) {
		case COMM_TEST:
			//FLASH LEDS
			break;
		
		case PLAY:
			// SET STATE TO PLAY 
			break;
			
		case GOAL_R:
			// PAUSE
			// GET SCORE A AND SCORE B FROM BUFFER
			break;

		case GOAL_B:
			// PAUSE
			// GET SCORE A AND SCORE B FROM BUFFER
			break;
		
		case PAUSE:
			//PAUSE
			break;
		
		case HALFTIME:
			//PAUSE
			break;

		case GAME_OVER:
			//PAUSE
			break;

		default:
			//TEAM COMMANDS GO IN THERE OWN CASES
			break;
	}
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