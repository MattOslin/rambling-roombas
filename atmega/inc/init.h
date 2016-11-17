
#ifndef INIT_H_
#define INIT_H_

#include "m_general.h"
#include "motor_driver.h"
#include "ADC_driver.h"
#include "diff_drive.h"
#include "comms.h"


#define	F_CPU 16000000	    // CPU Clock Freq [Hz]
#define PI 3.14159265       // The constant that defines the ratio between diameter and circumference
#define TIMER_0_PRSCL 1024  // Timer Prescalers 
#define TIMER_1_PRSCL 1
#define TIMER_3_PRSCL 64
#define CTRL_FREQ 100		// Control loop frequency w/1024 timer0 prscl 62 -> 5kHz

#define CHANNEL 1
#define MY_ADDRESS 20 /*21*/ /*22*/
#define PACKET_LENGTH 10

//Helper Macros
#define ABS(X)				(X < 0 ?-X : X)
#define MIN(X,Y)			(X < Y ? X : Y)
#define MAX(X,Y)			(X > Y ? X : Y)
#define clr(reg,bit)		reg &= ~(1<<(bit))



// Initialize helper functions

void m2_init(void);
void timer0_init(void);
void timer1_init(void);
void timer3_init(void); // Millisecond timer
uint32_t millis(void); // Returns current milliseconds count
void usb_read_command(void);
void motor_GPIO_setup(void);
void dd_init(dd *rob);

#endif /* INIT_H_ */