#ifndef IRSENSOR_H
#define IRSENSOR_H

#include <SoftwareSerial.h>

bool bt_A_BEACON = 0; //1
bool bt_B_BEACON = 0; //5

unsigned long ul_No_Beacon_Timer = 0;

unsigned int ui_Beacon_Seen;

SoftwareSerial mySerial(ci_Light_Sensor, 11); // RX, TX

//TODO: add timer functionality
//returns type of beacon seen
int CheckBeacon() {
  boolean bt_Flag = 0;
  int i_Beacon;
  int ir_int;

  while (mySerial.available() > 0) {

    bt_Flag = 0;

    //read the incoming serial
    ir_int = mySerial.read();


#ifdef DEBUG_IRSensor
    Serial.print("IR Sensor Value: ");
    Serial.println(ir_int);
#endif


    if (ir_int == 48) {
      bt_A_BEACON = 1;
      bt_Flag = 1;
    }
    else if (ir_int == '5') {
      bt_B_BEACON = 1;
      bt_Flag = 1;
    }
  }

  if (!bt_Flag && (millis() - ul_No_Beacon_Timer >= 50)) {
    ul_No_Beacon_Timer = millis(); //reset timer
    bt_A_BEACON = 0;
    bt_B_BEACON = 0;
  }
#ifdef DEBUG_IRSensor
  Serial.print("SEES A: ");
  Serial.print(bt_A_BEACON);
  Serial.print(",  SEES B: ");
  Serial.println(bt_B_BEACON);
#endif
  return i_Beacon;
}

#endif
