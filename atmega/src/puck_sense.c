#include "puck_sense.h"

void puck_update(pk *puck, uint16_t *P);

void puck_update(pk *puck, uint16_t *P) {

  // Update the pucks information
  puck->thPrev = puck->th;

  if (P[0]>NOISE || P[1]>NOISE || P[2]>NOISE || P[3]>NOISE || P[4]>NOISE || P[5]>NOISE){
    puck->isFound = TRUE;
    puck->isBehind = FALSE;

    float temp = ((int)P[2]-(int)P[3]+3*(int)P[1]-3*(int)P[4]+5*(int)P[0]-5*(int)P[5])
    /(float)(P[0]+P[1]+P[2]+P[3]+P[4]+P[5]);
    puck->th = (PI/15)*temp;
    m_usb_tx_int(puck->th*100);
    m_usb_tx_string(" ");
    int i;
    for(i = 0; i < 6; i++) {
      m_usb_tx_int(P[i]);
      m_usb_tx_string(" ");
    }
    m_usb_tx_string("\n");
  }
  else if(P[6] > NOISE){
   puck->isBehind = TRUE;
   puck->isFound = TRUE;
   }
  else {
    puck->isFound = FALSE;
    puck->isBehind = FALSE;
    puck->th = 0;
  }
  puck->r = 0;
  puck->isHave = P[7]<HAVE_PUCK;


}


//    int sensor_index = 5-((int)(temp+.5) + 5) / 2;
//    float snapped_temp = 5 - 2 * sensor_index;
//    float delta = (temp - snapped_temp);
////
////    int adjusted = P[sensor_index] + 50 * fabs(delta);
////    m_usb_tx_int(P[sensor_index]);
////    m_usb_tx_string(" ");
////    m_usb_tx_int(delta*100);
////    m_usb_tx_string(" ");
////    m_usb_tx_int(adjusted);
////    m_usb_tx_string(" ");
//
////    m_usb_tx_string(" ");
////    m_usb_tx_int(snapped_temp);
////    m_usb_tx_string(" ");
////    m_usb_tx_int(P[peak_index]);
////    m_usb_tx_string(" ");
////    m_usb_tx_int(100*(peak_index-temp_peak));
//
//
////    int max_val = 0;
////    int max_index = 0;
//    int i;
//    for(i = 0; i < 6; i++) {
//      m_usb_tx_int(P[i]);
//      m_usb_tx_string(" ");
//    }
//
//
////    m_usb_tx_int(temp * 100);
////    m_usb_tx_string(" ");
////    m_usb_tx_int(5 - max_index * 2 );
////    m_usb_tx_string(" ");
////    m_usb_tx_int(max_val);
////    m_usb_tx_string("\n");

//    m_usb_tx_string("\n");
