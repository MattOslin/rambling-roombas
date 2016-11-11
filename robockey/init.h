
#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

#include "m_general.h"
#include "m_bus.h"
#include "m_usb.h"
#include "m_rf.h"
#include "motor_driver.h"
#include "ADC_driver.h"

#define	F_CPU 16000000	    // CPU Clock Freq [Hz]
#define PI 3.14159265       // The constant that defines the ratio between diameter and circumference
#define TIMER_0_PRSCL 1024
#define TIMER_1_PRSCL 1
#define TIMER_3_PRSCL 64

#define ABS(X)				(X < 0 ?-X : X)
#define MIN(X,Y)			(X < Y ? X : Y)
#define clr(reg,bit)		reg &= ~(1<<(bit))

// when we want RF
// #define CHANNEL 1
// #define MY_ADDRESS 0x5D
// #define PACKET_LENGTH 3

// Initialize helper functions

void m2_init(void);
void timer0_init(void);
void timer1_init(void);
void timer3_init(void); // Millisecond timer
uint32_t millis(void); // Returns current milliseconds count

void usb_read_command(void);
void motor_GPIO_setup(void);
void dd_init(void);

#endif /* MOTOR_DRIVER_H_ */