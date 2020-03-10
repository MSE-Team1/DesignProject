#ifndef IRSENSOR_H
#define IRSENSOR_H

#include <SoftwareSerial.h>

const int ci_NO_BEACON = 0;
const int ci_A_BEACON = 1;
const int ci_B_BEACON = 2;
SoftwareSerial mySerial(7, 11); // RX, TX






void CheckBeacon() {
  boolean bt_Flag = 0;
  int i_Beacon;
  char ir_char;
  
  while (Serial.available() > 0) {

    bt_Flag = 1;

    //read the incoming serial
    ir_char = mySerial.read();
    


    if (ir_char == 48) {
      i_Beacon = ci_A_BEACON;
    }
    else if (ir_char == 53) {
      i_Beacon = ci_B_BEACON;
    }
  }

  if(!bt_Flag){
    i_Beacon = ci_NO_BEACON;
  }

  return i_Beacon;
}

#endif
