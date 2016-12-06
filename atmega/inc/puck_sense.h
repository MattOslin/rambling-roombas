#ifndef PUCK_SENSE_H_
#define PUCK_SENSE_H_

#include "init.h"

#define P0 ADCs[6]
#define P1 ADCs[5]
#define P2 ADCs[3]
#define P3 ADCs[1]
#define P4 ADCs[0]
#define P5 ADCs[7]
#define PBACK ADCs[2]
#define PPUCK ADCs[4]

#define NOISE 30
#define HAVE_PUCK 900

void puck_update(pk *puck, uint16_t *ADCs);

#endif /* PUCK_SENSE_H_ */
