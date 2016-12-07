/*
 * Robockey
 *
 * Created: 10/25/2016 6:31:06 PM
 * Author : J. Diego Caporale, Matt Oslin, Garrett Wenger, Jake Welde
 */ 
#include "init.h"
#include "behavior_FSM.h"
void usb_debug(dd *rob, pk *puck);

dd robot;
pk puck;

// Global flags for interrupts
volatile bool CTRLreadyFlag = FALSE;	// Frequency control flag for control loop
volatile bool isCommandReady = FALSE; // RF command flag
volatile bool isADCRead = FALSE; 

//Global Variable
uint16_t rawADCCounts[12];	// Array of raw ADC values
volatile uint32_t milliseconds = 0;
unsigned char buffer[PACKET_LENGTH] = {0};
float speed;



int main(void) {

	// Initialize GPIO, ADC, m_bus, interrupts, diffDrive
	m2_init();

	// Initialize diffDrive (motors, encoders, localization)
	dd_init(&robot); 
	// dd_enable(&robot);
	// Initialize Variables
	//const float deltaT = 1.0/CTRL_FREQ;
	uint16_t count = 0; //Used to not bog down processer or terminal with USB Transmissions
	robot.desLoc.x = 0;
	robot.desLoc.y = 0;
	robot.desLoc.th = 0;
	init_fsm();
	m_green(ON); // Ready LED
	// while(1){
	// 	// system_check(&robot);
	// 	// robot.solenoid = TRUE;
	// 	// solenoid_update(&robot);

	// }
	//Main process loop
    while (1) //Stay in this loop forever
    {

		//RF Command inputs
		if (isCommandReady) {	
			m_green(TOGGLE);
			isCommandReady = FALSE;
			m_rf_read(buffer,PACKET_LENGTH);// pull the packet
			rf_parse(buffer, &robot);
		}

		if (isADCRead) {	
			toggle(PORTB,4);
			isADCRead = FALSE;
			adc_read(rawADCCounts);
		}

		if (CTRLreadyFlag) {
			
			CTRLreadyFlag = FALSE; //Reset flag for interrupt	
			localize_wii(&(robot.global));
			puck_update(&puck, rawADCCounts);
			find_state(&robot,&puck);

			// if(puck.isHave){
			// 	set(PORTD,6); // LED Blue
			// 	set(PORTD,5);
			// }
			// else{
			// 	clr(PORTD,6);
			// 	clr(PORTD,5);
			// }

			dd_update(&robot);
			solenoid_update(&robot);
			 //UPDATES THE CONTROLS
			// dd_goto_rot_trans(&robot, .2);
			

			//Iterate count for slower loop
			count++;
		}

		if(count%10 == 0) {
			usb_debug(&robot, &puck); // USB Debug function below
		}
	}
}


//INTERRUPT HANDLER ADC
// Interrupt to inform main loop that ADC is ready to read
// In theory 9.6 kHz (VALIDATED without load)
ISR(ADC_vect)
{
	isADCRead = TRUE;
}

//RF Command Interupt Handler.
ISR(INT2_vect){
	isCommandReady = TRUE;
}

//Interrupt for CTRL_FREQ frequency control loop
ISR(TIMER0_COMPA_vect) {
	CTRLreadyFlag = TRUE;
}

// Interrupt for millisecond timer update
ISR(TIMER4_OVF_vect) {
	milliseconds++;
}

//interrupt for encoders
ISR(INT3_vect){
	encoder_update(&(robot.M1));
}

//interrupt for encoders
ISR(INT6_vect){
	encoder_update(&(robot.M2));
}

// interrupt for input capture on ping sensor
ISR(TIMER3_CAPT_vect) {
	robot.ping = PING_LOWPASS * ICR3 + (1-PING_LOWPASS) * robot.ping;
}


void usb_debug(dd *rob, pk *puck){
	
	// m_usb_tx_string("\n");
//	m_usb_tx_string(" vD: ");
//	m_usb_tx_int(100*rob->veloDesired);
//	m_usb_tx_string(" oD: ");
//	m_usb_tx_int(100*rob->omegaDesired);
	// m_usb_tx_string(" m1 timerpin: ");
	// m_usb_tx_int(*(rob->M1.dutyCycleRegister));
	// m_usb_tx_string(" m2 timerpin: ");
	// m_usb_tx_int(*(rob->M2.dutyCycleRegister));
	// m_usb_tx_string(" m1 command: ");
	// m_usb_tx_int(rob->M1.command);
	// m_usb_tx_string(" m2 command: ");
	// m_usb_tx_int(rob->M2.command);
	// m_usb_tx_string(" vD: ");
	// m_usb_tx_int(100 * robot.veloDesired);
	// m_usb_tx_string(" vD_enc:");
		// m_usb_tx_int(MOTOR_SPEED_MAX * ENC_RES * robot.M1.veloDesired / CTRL_FREQ);
		// m_usb_tx_string(" M1_enc ");//m_usb_tx_int(10);
	// m_usb_tx_int(robot.M1.veloEncoder);
	// m_usb_tx_string(",");
//	m_usb_tx_string("  Location Data:  ");
//	m_usb_tx_int(rob->global.x);
//	m_usb_tx_string(",");
//	m_usb_tx_int(rob->global.y);
//	m_usb_tx_string(",");
//	m_usb_tx_int(100 * rob->global.th);
//	m_usb_tx_string(" Team:");
//	m_usb_tx_int(rob->team);
//	m_usb_tx_string(" Direction:");
//	m_usb_tx_int(rob->direction);

	 m_usb_tx_string(" STATE: ");
	 m_usb_tx_int(rob->nxtSt);
	 m_usb_tx_string(" EVENT: ");
	 m_usb_tx_int(rob->ev);
	 m_usb_tx_string(" enable: ");
	 m_usb_tx_int(rob->enable);
	
//  int i;
//  for(i=0;i<8;i++){
//  m_usb_tx_string(" ADC");
//  m_usb_tx_int(i);
//  m_usb_tx_string(" ");
//    m_usb_tx_int(rawADCCounts[i]);
//  }
//  m_usb_tx_string(" Ping: ");
//  m_usb_tx_int(rob->ping);

	// m_usb_tx_int((uint8_t) eeprom_read_byte(&eepAddress));
	// m_usb_tx_string(" ");
	// m_usb_tx_int((uint8_t) eeprom_read_byte(&eepDirection));
	// m_usb_tx_string(" ");
	// m_usb_tx_int((uint8_t) eeprom_read_byte(&eepTeam));

	// m_usb_tx_string(" ADC L: ");
 // 	m_usb_tx_int(rawADCCounts[3]);
	// m_usb_tx_string(" ADC R: ");
 // 	m_usb_tx_int(rawADCCounts[1]);
 // 	m_usb_tx_string(" Puck TH: ");
 // 	m_usb_tx_int((int)100*puck->th);
 // 	m_usb_tx_string(" isFound: ");
 // 	m_usb_tx_int(puck->isFound);
	// m_usb_tx_string(" puckHave ADC: ");
 // 	m_usb_tx_int(rawADCCounts[4]);
 // 	m_usb_tx_string(" isHave: ");
 // 	m_usb_tx_int(puck->isHave);
	// m_usb_tx_string(" commVelo: ");
 // 	m_usb_tx_int(100*rob->veloDesired);
	// m_usb_tx_string(" commOm: ");
 // 	m_usb_tx_int(100*rob->omegaDesired);
 // 	m_usb_tx_string(" state: ");
 // 	m_usb_tx_int(rob->nxtSt);

   	m_usb_tx_string("\n");


}
