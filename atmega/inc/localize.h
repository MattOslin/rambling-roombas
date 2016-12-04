#ifndef LOCALIZE_H_
#define LOCALIZE_H_

#include "init.h"
#include "m_wii.h"

void localize_init(void);
void localize_enc(pos* posStruct, double encCountsL, double encCountsR);
bool localize_wii(pos* posStruct);
bool localize_cal(void);

#endif /* LOCALIZE_H_ */