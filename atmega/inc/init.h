#ifndef INIT_H_
#define INIT_H_

#include "m_general.h"
#include "macros.h"
#include "localize.h"
#include "ADC_driver.h"
#include "diff_drive.h"
#include "comms.h"
#include "eep_locations.h"
#include "puck_sense.h"

// typedef struct puckInfo {
// 	float r;		//
// 	float th;
// 	float thPrev;
// 	bool isFound;
// 	bool isBehind;
// 	bool isHave;
// } pk;

// Initialize helper functions

void m2_init(void);
uint32_t millis(void); // Returns current milliseconds count
void dd_init(dd *rob);

void puck_update(pk *puck, uint16_t* ADCs);
void shoot_puck(dd *rob, pk *puck);

void solenoid_update(dd *rob);
void wall_adjust(dd *rob);
// float atan2_aprox(float x, float y);
bool system_check(dd*rob);

#endif /* INIT_H_ */
