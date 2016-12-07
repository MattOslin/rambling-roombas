#include "puck_sense.h"
#define PK_ALPHA_LPF .5
void puck_update(pk *puck, uint16_t *P) {

  // Update the pucks information
  puck->thPrev = puck->th;

  if (P[0]>NOISE || P[1]>NOISE || P[2]>NOISE || P[3]>NOISE || P[4]>NOISE || P[5]>NOISE){
    puck->isFound = TRUE;
    puck->isBehind = FALSE;

    float temp = ((int)P[2]-(int)P[3]+3*(int)P[1]-3*(int)P[4]+5*(int)P[0]-5*(int)P[5])
    /(float)(P[0]+P[1]+P[2]+P[3]+P[4]+P[5]);
    // puck->th =(PI/15)*temp;

    puck->th = PK_ALPHA_LPF * (PI/15) * temp + ( 1 - PK_ALPHA_LPF ) * puck->thPrev;
    // m_usb_tx_int(puck->th*100);
    // m_usb_tx_string(" ")
    // int i;
    // for(i = 0; i < 6; i++) {
    //   m_usb_tx_int(P[i]);
    //   m_usb_tx_string(" ");
    // }
    // m_usb_tx_string("\n");
  }
  else if(P[6]>30){
      puck->isBehind = FALSE;//TRUE;
      puck->isFound = FALSE;
      puck->th = -PI;
   }
  else {
      puck->isFound = FALSE;
      puck->isBehind = FALSE;
      puck->th = -PI;
  }
  puck->r = 0;
  puck->maxADC = MAX(MAX(MAX(MAX(MAX(P[0],P[1]),P[2]),P[3]),P[4]),P[5]);
  puck->isHave = P[7]<HAVE_PUCK;
}
