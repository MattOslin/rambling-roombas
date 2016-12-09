#ifndef LOCALIZE_H_
#define LOCALIZE_H_

#include "m_general.h"
#include "diff_drive.h"
#include "eep_locations.h"
#include "m_wii.h"
#include "init.h"

void localize_init(void);
void localize_enc(pos* posStruct, float encCountsL, float encCountsR);
bool localize_wii(pos* posStruct);
// bool localize_cal(pos* posStruct);
bool localize_blob(pos* posStruct, uint16_t* newBlob);
void localize_set_cals(float calXnew, float calYnew);

#endif /* LOCALIZE_H_ */