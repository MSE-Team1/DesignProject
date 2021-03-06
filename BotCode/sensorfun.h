#ifndef SENSORFUN_H
#define SENSORFUN_H

unsigned long ul_Echo_Time;
unsigned long ul_Ping_Timer = 0; 

unsigned int ui_Ultrasonic_Distance = 0; //cm

// measure distance to target using ultrasonic sensor
void Ping()
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(ci_Ultrasonic_Ping, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ci_Ultrasonic_Ping, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW
  ul_Echo_Time = pulseIn(ci_Ultrasonic_Data, HIGH, 10000);

  // Print Sensor Readings
#ifdef DEBUG_ULTRASONIC
  Serial.print("Time (microseconds): ");
  Serial.print(ul_Echo_Time, DEC);
  Serial.print(", Inches: ");
  Serial.print(ul_Echo_Time / 148); //divide time by 148 to get distance in inches
  Serial.print(", cm: ");
  Serial.println(ul_Echo_Time / 58); //divide time by 58 to get distance in cm
#endif
}

//returns distance seen by ultrasonic in cm
void UpdateUltrasonicDistance(){
  //if time has passed
  if(millis() - ul_Ping_Timer >= 60){
    Ping();
    ui_Ultrasonic_Distance = ul_Echo_Time / 58;

    ul_Ping_Timer = millis(); //reset timer
  }
}


#endif
