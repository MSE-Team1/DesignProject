/*
  Software serial for beacon finding IR Sensor

 The circuit:

* RX is digital pin 7 (connect to TX of other device)
* TX is digital pin 11 (connect to RX of other device)

*/

#include <SoftwareSerial.h>

const int ci_NO_BEACON = 0;
const int ci_A_BEACON = 1; 
const int ci_B_BEACON = 2;
unsigned int ui_flag = 0;
SoftwareSerial mySerial(7, 11); // RX, TX



  // set the data rate for the SoftwareSerial port
  mySerial.begin(2400);
  

  while (Serial.available()> ci_NO_BEACON) {
    //read the incoming serial

    char ir_char = mySerial.read();

    if( ir_char == ci_A_BEACON){ 
    return ci_A_BEACON;
    }
    else if (ir_char == ci_B_BEACON) {
      return ci_B_BEACON;
    }

else flag =1

  }

return ci_NO_BEACON;
