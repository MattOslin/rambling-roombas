/*
 * Robockey
 *
 * Created: 10/25/2016 6:31:06 PM
 * Author : J. Diego Caporale, Matt Oslin, Garrett Wenger, Jake Welde
 */ 
#include "init.h"

dd robot;

// Global flags for interrupts
volatile bool CTRLreadyFlag = FALSE;	// Frequency control flag for control loop
volatile bool isCommandReady = FALSE; // RF command flag

//Global Variable
uint16_t rawADCCounts[12];	// Array of raw ADC values
volatile uint32_t milliseconds = 0;
unsigned char buffer[PACKET_LENGTH] = {0};
float speed;
uint16_t ping;

int main(void) {
	
	// Initialize GPIO, ADC, m_bus, interrupts, diffDrive
	m2_init();

	// Initialize diffDrive (motors, encoders, localization)
	dd_init(&robot);

	// Initialize Variables
	const float deltaT = 1.0/CTRL_FREQ;
	uint16_t countUSB = 0; //Used to not bog down processer or terminal with USB Transmissions
	//unsigned int usbIn, charCount;

	set(DDRC,6);

	//Main process loop
  while (1) //Stay in this loop forever
  {
		dd_enable(&robot);
		//Control Loop at CTRL_FREQ frequency//
		speed = 200; // RPM
		robot.M1.veloDesired = (speed * 19 * 34 / 60.0) / CTRL_FREQ;

		if (CTRLreadyFlag)
		{

			//DEBUG CTRL FREQUENCY TEST//
			toggle(PORTC,6);
			
			CTRLreadyFlag = FALSE; //Reset flag for interupt

			//LOW PASS FILTER FOR ADC//
// 			int i;
// 			for(i = 0; i < NUMADCS; i++) {
// 				oldFiltADCCounts[i] = filtADCCounts[i];
// 				filtADCCounts[i] = alpha*filtADCCounts[i] + (1-alpha)*rawADCCounts[i];
// 			}
			
			drive_CL(&(robot.M1));
			motor_update(&(robot.M1));

			// USB DEBUG//
			// Send USB information for DEBUG every 100 control loop cycles
			if (countUSB%1 == 0) {
			  	//m_red(TOGGLE);
//        m_usb_tx_int(ping);
//		  	m_usb_tx_string("  ");
//		  	m_usb_tx_int( robot.M1.veloDesired);
//		  	m_usb_tx_string("  ");
//				m_usb_tx_int( robot.M1.veloEncoder);
//				m_usb_tx_string("  ");
//				m_usb_tx_int(robot.M1.command);
//				m_usb_tx_string("  ");
//				m_usb_tx_int( robot.M1.prevError);
//				m_usb_tx_string("  ");
//				m_usb_tx_int(1000 * robot.M1.kp);
// 				m_usb_tx_int(rawADCCounts[2]);
// 				m_usb_tx_string("  ");
// 				m_usb_tx_int(rawADCCounts[3]);
// 				m_usb_tx_string("  ");
// 				m_usb_tx_int(*(M1.dutyCycleRegister));
//				m_usb_tx_string("\n");
//	 			m_usb_tx_push();
				//robot.M1.command = /*(M1.command+25)%*/MOTOR_COMMAND_MAX;
				motor_update(&(robot.M1));
			}

			//Iterate countUSB
			countUSB++;

		}
		//RF Command inputs
		if (isCommandReady)
		{	
			isCommandReady = FALSE;
			rf_parse(buffer, &robot);
		}
	}
}


//INTERRUPT HANDLER ADC
// Interrupt to inform main loop that ADC is ready to read
// In theory 9.6 kHz (VALIDATED without load)
ISR(ADC_vect)
{
	adc_read(rawADCCounts);
}

//RF Command Interupt Handler.
ISR(INT2_vect){
	isCommandReady = TRUE;
	m_rf_read(buffer,PACKET_LENGTH);// pull the packet
	
}

//Interrupt for CTRL_FREQ frequency control loop
ISR(TIMER0_COMPA_vect) {
	CTRLreadyFlag = TRUE;
}

// Interrupt for millisecond timer update
ISR(TIMER4_OVF_vect) {
	milliseconds++;
}

// interrupt for encoders
ISR(INT3_vect){
	encoder_update(&(robot.M1));
	m_red(TOGGLE);	
}

// interrupt for input capture on ping sensor
ISR(TIMER3_CAPT_vect) {
  ping = ICR3;
}
