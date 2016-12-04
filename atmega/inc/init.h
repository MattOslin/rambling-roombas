
#ifndef INIT_H_
#define INIT_H_

#include "m_general.h"
#include "motor_driver.h"
#include "ADC_driver.h"
#include "diff_drive.h"
#include "comms.h"
#include "localize.h"
#include "eep_locations.h"

#define	F_CPU 16000000	    // CPU Clock Freq [Hz]
#define PI 3.14159265       // The constant that defines the ratio between diameter and circumference
#define TIMER_0_PRSCL 1024  // Timer Prescalers 
#define TIMER_1_PRSCL 8

#define TIMER_3_PRSCL 1024
#define TIMER_4_PRSCL 128
#define CTRL_FREQ 100		// Control loop frequency w/1024 timer0 prscl 62 -> 5kHz
#define POS_THRESH 2 //cm threshold for success


#define CHANNEL 1
#define MY_ADDRESS 20 /*21*/ /*22*/
#define PACKET_LENGTH 10
// #define USE_EEP_ADDRESS

//Helper Macros
#define ABS(X)				(X < 0 ?-X : X)
#define MIN(X,Y)			(X < Y ? X : Y)
#define MAX(X,Y)			(X > Y ? X : Y)
#define clr(reg,bit)		reg &= ~(1<<(bit))
#define ANG_REMAP(TH)		(TH > PI ? TH - 2 * PI : (TH < -PI ? TH + 2 * PI : TH))

#define NEG_Y 0
#define POS_Y 1

typedef struct puckInfo {
	float r;		//
	float th;
	float thPrev;
	bool isFound;
	bool isBehind;
	bool isHave;
} pk;

// Initialize helper functions

void m2_init(void);
void timer0_init(void);
void timer1_init(void);
void timer3_init(void); // Ping sensor timer
void timer4_init(void); // Millisecond timer
uint32_t millis(void); // Returns current milliseconds count
void usb_read_command(void);
void motor_GPIO_setup(void);
void dd_init(dd *rob);
void puck_update(pk *puck, uint16_t* ADCs);
float atan2_aprox(float x, float y);
bool system_check(dd*rob);
#endif /* INIT_H_ */
