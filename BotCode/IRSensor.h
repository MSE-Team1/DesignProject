#ifndef IRSENSOR_H
#define IRSENSOR_H

#include <SoftwareSerial.h>

const int ci_NO_BEACON = 0;
const int ci_A_BEACON = 1;
const int ci_B_BEACON = 2;

unsigned int ui_Beacon_Seen;

SoftwareSerial mySerial(10, 11); // RX, TX


//returns type of beacon seen
int CheckBeacon() {
  boolean bt_Flag = 0;
  int i_Beacon;
  int ir_int;

  while (mySerial.available() > 0) {

    bt_Flag = 1;

    //read the incoming serial
    ir_int = mySerial.read();


#ifdef DEBUG_IRSensor
    Serial.print("IR Sensor Value: ");
    Serial.println(!ir_int);
#endif


    if (ir_int == 48) {
      i_Beacon = ci_A_BEACON;
    }
    else if (ir_int == 53) {
      i_Beacon = ci_B_BEACON;
    }
  }

  if (!bt_Flag) {
    i_Beacon = ci_NO_BEACON;
  }
#ifdef DEBUG_IRSensor
  Serial.print("IR Beacon seen: ");
  Serial.println(i_Beacon);
#endif
  return i_Beacon;
}

#endif
