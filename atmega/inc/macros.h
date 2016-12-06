#ifndef MACROS_H_
#define MACROS_H_

#define	F_CPU 16000000	    // CPU Clock Freq [Hz]
#define PI 3.14159265       // The constant that defines the ratio between diameter and circumference
#define TIMER_0_PRSCL 1024  // Timer Prescalers 
#define TIMER_1_PRSCL 8

#define TIMER_3_PRSCL 1024
#define TIMER_4_PRSCL 128
#define CTRL_FREQ 100		// Control loop frequency w/1024 timer0 prscl 62 -> 5kHz
#define POS_THRESH 2 //cm threshold for success

#define GOAL_X 40// xdimension of the top of the goal (negated for bottom)
#define GOAL_Y 300// y dimension of goal in the rink
#define SHT_THRSH_NEAR 65
#define SHT_THRSH_FAR 100


#define CHANNEL 1
#define MY_ADDRESS 21 /*21*/ /*22*/
#define PACKET_LENGTH 10
#define USE_EEP_ADDRESS

#define NEG_Y -1
#define POS_Y 1

#define RED 0
#define BLUE 1

#define PIN_RED 5
#define PIN_BLUE 6

#define SOL_ON_TIME 200  //in milliseconds
#define SOL_COOLDOWN_TIME 500 //in milliseconds
//Helper Macros
#define ABS(X)				(X < 0 ?-X : X)
#define MIN(X,Y)			(X < Y ? X : Y)
#define MAX(X,Y)			(X > Y ? X : Y)
#define clr(reg,bit)		reg &= ~(1<<(bit))
#define ANG_REMAP(TH)		(TH > PI ? TH - 2 * PI : (TH < -PI ? TH + 2 * PI : TH))

#define SOLENOID(val)		set(DDRB,1); if(val==OFF){clr(PORTB,1); }else if(val==ON){set(PORTB,1);}else if(val==TOGGLE){toggle(PORTB,1);}

typedef struct puckInfo {
	float r;		//
	float th;
	float thPrev;
	bool isFound;
	bool isBehind;
	bool isHave;
} pk;

#endif