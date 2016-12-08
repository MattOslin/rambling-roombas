/*
 * Robockey Coach
 *
 * Created: 10/25/2016 6:31:06 PM
 * Author : J. Diego Caporale, Matt Oslin, Garrett Wenger, Jake Welde
 */ 
#include "m_general.h"
#include "m_usb.h"
#include "m_bus.h"
#include "m_rf.h"

#define NEG_Y -1
#define POS_Y 1

#define DIR_PIN 0
#define TEAM_PIN 1

#define RED 0
#define BLUE 1

#define RED_PIN 4
#define BLUE_PIN 6

#define CHANNEL 1
#define MY_ADDRESS 23 
#define PACKET_LENGTH 10

#define BOT20ADDR 20
#define BOT21ADDR 21
#define BOT22ADDR 22

#define BOT20PIN 2
#define BOT21PIN 7
#define BOT22PIN 3

#define BOT20LED 4
#define BOT21LED 5
#define BOT22LED 6

volatile bool packetReceived = FALSE; // RF command flag
bool configed20 = FALSE;
bool configed21 = FALSE;
bool configed22 = FALSE;

//Global Variable
volatile uint32_t milliseconds = 0;
char buffer[PACKET_LENGTH] = {0};

enum rf_command {
	COMM_TEST = 0xA0, PLAY, GOAL_R, GOAL_B, PAUSE, SKIP, HALFTIME, GAME_OVER, CALIBRATE, COACH, CONTROLLER
};

void timer4_init(void); // Millisecond timer
uint32_t millis(void); // Returns current milliseconds count
void reset_configs();
bool send_config(uint8_t botAddr, uint8_t botPin, uint8_t ledPin);


int main(void) {
	// SET CLOCK TO 16MHz
	m_clockdivide(0);

    timer4_init(); // Timer4 Used for millis clock

	m_bus_init();
	m_usb_init(); // USB COMs for debug

	
	m_disableJTAG(); //Allows use of some of the portF

	m_rf_open(CHANNEL, MY_ADDRESS, PACKET_LENGTH);// For RF comms 
	// buffer[0] = COACH;
	// buffer[1] = MY_ADDRESS;

	set(DDRD,4); // LED Red
	set(DDRD,6); // LED Blue
	set(DDRF,4);
	set(DDRF,5);
	set(DDRF,6);
	set(DDRB,6);
	set(DDRC,7);


	clr(DDRB,0);
	clr(DDRB,1);
	clr(DDRB,2);
	clr(DDRB,3);
	clr(DDRB,7);

	set(PORTB, 0);
	set(PORTB, 1);
	set(PORTB, 2);
	set(PORTB, 3);
	set(PORTB, 7);
	

	// Enable global interrupts
	sei();
	// uint16_t count = 0; //Used to not bog down processer or terminal with USB Transmissions
	// robot.desLoc.x = 0;
	// robot.desLoc.y = -280;
	// robot.desLoc.th = PI/2;

	// while (!localize_wii(&(robot.global)));

	uint8_t lastTeam = 0xFF;
	int8_t lastDir = 0;
	//Main process loop
    while (1) {
    	if(!check(PINB,DIR_PIN) == POS_Y) {
    		if (lastDir != POS_Y) {
    			reset_configs();
    		}

    		lastDir = POS_Y;
    		set(PORTB, 6);
    		clr(PORTC, 7);
    	} else {
    		if (lastDir != NEG_Y) {
    			reset_configs();
    		}

    		lastDir = NEG_Y;
    		clr(PORTB, 6);
    		set(PORTC, 7);
    	}

    	if(check(PINB,TEAM_PIN) == RED) {
    		if (lastTeam != RED) {
    			reset_configs();
    		}

    		lastTeam = RED;
    		set(PORTD, RED_PIN);
    		clr(PORTD, BLUE_PIN);
    	} else {
    		if (lastTeam != BLUE) {
    			reset_configs();
    		}

    		lastTeam = BLUE;
    		clr(PORTD, RED_PIN);
    		set(PORTD, BLUE_PIN);
    	}

    	bool button1 = !check(PINB, BOT20PIN);
    	bool button2 = !check(PINB, BOT21PIN);
    	bool button3 = !check(PINB, BOT22PIN);

    	if(button1 && button2 && button3) {
    		toggle(PORTF,BOT20LED);
			toggle(PORTF,BOT21LED);
			toggle(PORTF,BOT22LED);
    		buffer[0] = (unsigned char) GAME_OVER;
    		m_rf_send(BOT20ADDR, buffer, PACKET_LENGTH);
    		m_rf_send(BOT21ADDR, buffer, PACKET_LENGTH);
    		m_rf_send(BOT22ADDR, buffer, PACKET_LENGTH);
    	} else if(button2 && button3) {
    		clr(PORTF,BOT20LED);
			set(PORTF,BOT21LED);
			set(PORTF,BOT22LED);
    		buffer[0] = (unsigned char) CALIBRATE;
    		m_rf_send(BOT20ADDR, buffer, PACKET_LENGTH);
    	} else if(button1 && button3) {
    		set(PORTF,BOT20LED);
			clr(PORTF,BOT21LED);
			set(PORTF,BOT22LED);
    		buffer[0] = (unsigned char) CALIBRATE;
    		m_rf_send(BOT21ADDR, buffer, PACKET_LENGTH);
    	} else if(button1 && button2) {
    		set(PORTF,BOT20LED);
			set(PORTF,BOT21LED);
			clr(PORTF,BOT22LED);
    		buffer[0] = (unsigned char) CALIBRATE;
    		m_rf_send(BOT22ADDR, buffer, PACKET_LENGTH);
    	} else if(button1 && !configed20) {
    		configed20 = send_config(BOT20ADDR, BOT20PIN, BOT20LED);
    	} else if(button2 && !configed21) {
    		configed21 = send_config(BOT21ADDR, BOT21PIN, BOT21LED);
    	} else if(button3 && !configed22) {
    		configed22 = send_config(BOT22ADDR, BOT22PIN, BOT22LED);
    	} else {
    		if(!configed20) {
    			clr(PORTF,BOT20LED);
    		} else {
    			set(PORTF,BOT20LED);
    		}
    		if(!configed21) {
    			clr(PORTF,BOT21LED);
    		} else {
    			set(PORTF,BOT21LED);
    		}
    		if(!configed22) {
    			clr(PORTF,BOT22LED);
    		} else {
    			set(PORTF,BOT22LED);
    		}
    	}

		//RF Command inputs
		if (packetReceived) {	
			m_green(TOGGLE);
			packetReceived = FALSE;
		}

		m_wait(250);

	}
}

void reset_configs() {
	configed20 = FALSE;
	configed21 = FALSE;
	configed22 = FALSE;

	clr(PORTF,BOT20LED);
	clr(PORTF,BOT21LED);
	clr(PORTF,BOT22LED);
}

bool send_config(uint8_t botAddr, uint8_t botPin, uint8_t ledPin) {
	bool configured = FALSE;
	int8_t direction;
	uint8_t team;

	if (!check(PINB,DIR_PIN) == POS_Y) {
		direction = POS_Y;
	} else {
		direction = NEG_Y;
	}

	if (check(PINB,TEAM_PIN) == RED) {
		team = RED;
	} else {
		team = BLUE;
	}


	while(!configured && !check(PINB, botPin)) {
		buffer[0] = (unsigned char) COACH;
		buffer[1] = MY_ADDRESS;
		buffer[2] = direction;
		buffer[3] = team;

		m_rf_send(botAddr, buffer, PACKET_LENGTH);

		uint32_t startTime = millis();
		while(!packetReceived) {
			if(millis()-startTime > 1000) {
				break;
			}
			if(millis() % 100 > 90) {
				set(PORTF, ledPin);
			} else {
				clear(PORTF, ledPin);
			}
		}

		clear(PORTF, ledPin);

		if (packetReceived) {
			configured = (botAddr == buffer[4]) && (direction == buffer[5]) && (team == buffer[6]);
		}
		packetReceived = FALSE;
	}

	if(configured) {
		set(PORTF, ledPin);
	}
	return configured;
}

void timer4_init(void) {
  // Count up to OCR4C, then reset
  clr(TCCR4D,WGM41);
  clr(TCCR4D,WGM40);

  OCR4C = 250; // (F_CPU/1000)/TIMER_4_PRSCL; // (16 MHz / 64) * (1 ms) = 250

  set(TIMSK4,TOIE4); // enable overflow interrupt

  // Timer on, prescaler to 128
  clr(TCCR4B,CS43);
  set(TCCR4B,CS42);
  set(TCCR4B,CS41);
  set(TCCR4B,CS40);

}

uint32_t millis(void) {
	return milliseconds;
}

//RF Command Interupt Handler.
ISR(INT2_vect){
	packetReceived = TRUE;
	m_rf_read(buffer,PACKET_LENGTH);// pull the packet
}

// Interrupt for millisecond timer update
ISR(TIMER4_OVF_vect) {
	milliseconds++;
}
