#include "puck_sense.h"

void puck_update(pk *puck, uint16_t *ADCs);

void puck_update(pk *puck, uint16_t *ADCs) {

  // Update the pucks information
  puck->thPrev = puck->th;

  if (P0>NOISE || P1>NOISE || P2>NOISE || P3>NOISE || P4>NOISE || P5>NOISE){
    puck->isFound = TRUE;
    puck->isBehind = FALSE;

    puck->th = (PI/15)*((int)P2-(int)P3+3*(int)P1-3*(int)P4+5*(int)P0-5*(int)P5)
    /(float)(P0+P1+P2+P3+P4+P5);
  }
  else if(PBACK>30){
   puck->isBehind = TRUE;
   puck->isFound = TRUE;
   }
  else {
    puck->isFound = FALSE;
    puck->isBehind = FALSE;
    puck->th = 0;
  }
  puck->r = 0;
  puck->isHave = PPUCK<HAVE_PUCK;
}

//void puck_update(pk *puck, uint16_t *ADCs) {
//
//  // Update the pucks information
//  puck->thPrev = puck->th;
//
//  if (ADCs[3]>30 || ADCs[1]>30 || ADCs[5]>30 || ADCs[0]>30 || ADCs[6]>30 || ADCs[7]>30  ){
//    puck->isFound = TRUE;
//    puck->isBehind = FALSE;
//
//    puck->th = (PI/15)*((int)ADCs[3]-(int)ADCs[1]+3*(int)ADCs[5]-3*(int)ADCs[0]+5*(int)ADCs[6]-5*(int)ADCs[7])
//    /(float)(ADCs[3]+ADCs[1]+ADCs[5]+ADCs[0]+ADCs[6]+ADCs[7]);
//  }
//  // else if(ADCs[2]>30){
//  // puck->isBehind = TRUE;
//  // puck->isFound = TRUE;
//  // }
//  else {
//    puck->isFound = FALSE;
//    puck->isBehind = FALSE;
//    puck->th = 0;
//  }
//  puck->r = 0;
//  puck->isHave = ADCs[4]<900;
//}
//
