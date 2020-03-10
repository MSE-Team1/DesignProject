/*

  MSE 2202 BotCode for the design project
  Language: Arduino
  Authors: Michael Naish, Eugen Porter, Stewart Wing, and Kyle Inzunza
  Date: 03-03-20

  Rev 1 - Initial version
  Rev 2 - Updated for MSEduino
  Rev 3 - New chassis, integrated encoder (I2C) motors and charlieplexing
  Rev 4 - Update for MSEduino v. 2
  Rev 5 - Fix for restoring EEPROM line tracker calibration values
  Rev 6 - Added encoder follow functionality

*/

#include "includes.h"


boolean bt_Motors_Enabled = true;

//constants
const int ci_Grip_Motor_Open = 140;         // Experiment to determine appropriate value
const int ci_Grip_Motor_Closed = 90;        //  "
const int ci_Arm_Servo_Retracted = 55;      //  "
const int ci_Arm_Servo_Extended = 120;      //  "
const int ci_Display_Time = 500;

const int ci_Motor_Calibration_Cycles = 3;
const int ci_Motor_Calibration_Time = 5000;


//variables
byte b_LowByte;
byte b_HighByte;

unsigned long ul_3_Second_timer = 0;
unsigned long ul_Display_Time;
unsigned long ul_Calibration_Time;
unsigned long ui_Left_Motor_Offset;
unsigned long ui_Right_Motor_Offset;

unsigned int ui_Cal_Count;
unsigned int ui_Cal_Cycle;


unsigned int ui_Robot_State_Index = 0;
unsigned int ui_Course_State_Index = 0; //what part of the showcase program should be running

//flags
boolean bt_Go_To_Next_Stage;

//0123456789ABCDEF
unsigned int  ui_Mode_Indicator[6] = {
  0x00,    //B0000000000000000,  //Stop
  0x00FF,  //B0000000011111111,  //Run
  0x0F0F,  //B0000111100001111,  //Calibrate line tracker light level
  0x3333,  //B0011001100110011,  //Calibrate line tracker dark level
  0xAAAA,  //B1010101010101010,  //Calibrate motors
  0xFFFF   //B1111111111111111   //Unused
};

unsigned int  ui_Mode_Indicator_Index = 0;

//display Bits 0,1,2,3, 4, 5, 6,  7,  8,  9,  10,  11,  12,  13,   14,   15
int  iArray[16] = {
  1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 65536
};
int  iArrayIndex = 0;

boolean bt_Heartbeat = true;
boolean bt_3_S_Time_Up = false;
boolean bt_Do_Once = false;
boolean bt_Cal_Initialized = false;

boolean bt_North_Corner; //if true: robot starts in north corner, if false: robot starts in south corner

void setup() {
  Wire.begin();        // Wire library required for I2CEncoder library
  Serial.begin(9600);
  // set the data rate for the SoftwareSerial port
  mySerial.begin(2400);

  CharliePlexM::setBtn(ci_Charlieplex_LED1, ci_Charlieplex_LED2,
                       ci_Charlieplex_LED3, ci_Charlieplex_LED4, ci_Mode_Button);

  // set up ultrasonic
  pinMode(ci_Ultrasonic_Ping, OUTPUT);
  pinMode(ci_Ultrasonic_Data, INPUT);

  // set up drive motors
  pinMode(ci_Right_Motor, OUTPUT);
  servo_RightMotor.attach(ci_Right_Motor);
  pinMode(ci_Left_Motor, OUTPUT);
  servo_LeftMotor.attach(ci_Left_Motor);

  // set up arm motors
  pinMode(ci_Arm_Motor, OUTPUT);
  servo_ArmMotor.attach(ci_Arm_Motor);

  // set up motor enable and corner switch
  pinMode(ci_Motor_Enable_Switch, INPUT);
  pinMode(ci_Corner_Select_Switch, INPUT);

  // set up encoders. Must be initialized in order that they are chained together,
  // starting with the encoder directly connected to the Arduino. See I2CEncoder docs
  // for more information
  encoder_LeftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_RightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_RightMotor.setReversed(true);  // adjust for positive count when moving forward



  ui_Left_Motor_Offset = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Right_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(ci_Right_Motor_Offset_Address_H);
  ui_Right_Motor_Offset = word(b_HighByte, b_LowByte);

}

void loop()
{
  if ((millis() - ul_3_Second_timer) > 3000)
  {
    bt_3_S_Time_Up = true;
  }

  // button-based mode selection
  if (CharliePlexM::ui_Btn)
  {
    if (bt_Do_Once == false)
    {
      bt_Do_Once = true;
      ui_Robot_State_Index++;
      ui_Robot_State_Index = ui_Robot_State_Index & 7;
      ul_3_Second_timer = millis();
      bt_3_S_Time_Up = false;
      bt_Cal_Initialized = false;
      ui_Cal_Cycle = 0;
    }
  }
  else
  {
    bt_Do_Once = LOW;
  }

  // check if drive motors should be powered
  bt_Motors_Enabled = digitalRead(ci_Motor_Enable_Switch);
  // check what corner robot is in
  bt_North_Corner = digitalRead(ci_Corner_Select_Switch);


  // modes
  // 0 = default after power up/reset
  // 1 = Press mode button once to enter. Run robot.
  // 2 = Press mode button twice to enter. FREE SPACE
  // 3 = Press mode button three times to enter. FREE SPACE
  // 4 = Press mode button four times to enter. Calibrate motor speeds to drive straight.
  switch (ui_Robot_State_Index)
  {
    case 0:    //Robot stopped
      {
        //Ping();
        servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
        servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
        //servo_ArmMotor.write(ci_Arm_Servo_Retracted);
        //servo_GripMotor.write(ci_Grip_Motor_Closed);
        encoder_LeftMotor.zero();
        encoder_RightMotor.zero();
        ui_Mode_Indicator_Index = 0;


#ifdef DEBUG_MOTOR_CALIBRATION
        Serial.print("Motor Offsets: Left = ");
        Serial.print(ui_Left_Motor_Offset);
        Serial.print(", Right = ");
        Serial.println(ui_Right_Motor_Offset);
#endif

        break;
      }

    case 1:    //Robot Run after 3 seconds
      {
        if (bt_3_S_Time_Up)
        {

#ifdef DEBUG_ENCODERS
          l_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
          l_Right_Motor_Position = encoder_RightMotor.getRawPosition();

          Serial.print("Encoders L: ");
          Serial.print(l_Left_Motor_Position);
          Serial.print(", R: ");
          Serial.println(l_Right_Motor_Position);
#endif

          /* FROM EUGEN'S CODE BUT SEEMS TO BREAK THINGS
            // set motor speeds
            ui_Left_Motor_Speed = constrain(ui_Motors_Speed + ui_Left_Motor_Offset, 1600, 2100);
            ui_Right_Motor_Speed = constrain(ui_Motors_Speed + ui_Right_Motor_Offset, 1600, 2100);
          */

          /***************************************************************************************
            Add line tracking code here.
            Adjust motor speed according to information from line tracking sensors and
            possibly encoder counts.
            /*************************************************************************************/

#ifdef DEBUG_COURSE_STAGE
          Serial.print("COURSE STAGE BEING RUN: ");
          Serial.println(ui_Course_State_Index);
#endif

          switch (ui_Course_State_Index) {
            case 0:
              {
                //zero encoders
                encoder_RightMotor.zero();
                encoder_LeftMotor.zero();
                
                //make sure motors are stopped
                ui_Left_Motor_Speed = ci_Left_Motor_Stop;
                ui_Right_Motor_Speed = ci_Right_Motor_Stop;
                
                ui_Course_State_Index++;

                break;
              }
              case 1:
              {
                bt_Go_To_Next_Stage = 1; //default to true, encoder statem
                
                //run encoders forward certain distance
                //equiv of right&&left
                //do not run in if statement becasue only the first function will run
                bt_Go_To_Next_Stage &= EncoderDriveForward(100, LEFT_MOTOR); //L
                bt_Go_To_Next_Stage &= EncoderDriveForward(100, RIGHT_MOTOR); //R
               
                
                //when both functions return true then they have reached the desired count
                if(bt_Go_To_Next_Stage){
                  //zero encoders
                  encoder_LeftMotor.zero();
                  encoder_RightMotor.zero();
                  
                  ui_Course_State_Index++;
                }
                break;
              }
              case 2:
              {
                bt_Go_To_Next_Stage = ZeroPoint(90,CLOCKWISE);

                if(bt_Go_To_Next_Stage){
                  //zero encoders
                  encoder_LeftMotor.zero();
                  encoder_RightMotor.zero();
                  
                  ui_Course_State_Index++;
                }
                break;
              }


          }


          if (ui_Left_Motor_Speed != ci_Motor_Speed_Brake) {
            ui_Left_Motor_Speed = constrain(ui_Left_Motor_Speed, 900, 2100);
          }
          if (ui_Right_Motor_Speed != ci_Motor_Speed_Brake) {
            ui_Right_Motor_Speed = constrain(ui_Right_Motor_Speed, 900, 2100);
          }

#ifdef DEBUG_DRIVE
          Serial.print("SENT TO L: ");
          Serial.print(ui_Left_Motor_Speed);
          Serial.print("  SENT TO R: ");
          Serial.println(ui_Right_Motor_Speed);
#endif

          if (bt_Motors_Enabled)
          {
            servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
            servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
          }
          else
          {
            servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
            servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
          }
#ifdef DEBUG_MOTORS
          Serial.print("Motors enabled: ");
          Serial.print(bt_Motors_Enabled);
          Serial.print(", Default: ");
          Serial.print(ui_Motors_Speed);
          Serial.print(", Left = ");
          Serial.print(ui_Left_Motor_Speed);
          Serial.print(", Right = ");
          Serial.println(ui_Right_Motor_Speed);
#endif
          ui_Mode_Indicator_Index = 1;
        }
        break;
      }

    case 2:    //FREE SPACE
      {
        if (bt_3_S_Time_Up) {
          ui_Mode_Indicator_Index = 3;
        }
      }

    case 3:    //FREE SPACE
      {
        if (bt_3_S_Time_Up)
        {
          ui_Mode_Indicator_Index = 3;
        }
        break;
      }

    case 4:    //Calibrate motor straightness after 3 seconds
      {
        if (bt_3_S_Time_Up)
        {
          if (!bt_Cal_Initialized)
          {
            bt_Cal_Initialized = true;
            encoder_LeftMotor.zero();
            encoder_RightMotor.zero();
            ul_Calibration_Time = millis();
            servo_LeftMotor.writeMicroseconds(ui_Motors_Speed);
            servo_RightMotor.writeMicroseconds(ui_Motors_Speed);
          }
          else if ((millis() - ul_Calibration_Time) > ci_Motor_Calibration_Time)
          {
            servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
            servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
            l_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
            l_Right_Motor_Position = encoder_RightMotor.getRawPosition();
            if (l_Left_Motor_Position > l_Right_Motor_Position)
            {
              // May have to update this if different calibration time is used
              ui_Right_Motor_Offset = 0;
              ui_Left_Motor_Offset = (l_Left_Motor_Position - l_Right_Motor_Position) / 4;
            }
            else
            {
              // May have to update this if different calibration time is used
              ui_Right_Motor_Offset = (l_Right_Motor_Position - l_Left_Motor_Position) / 4;
              ui_Left_Motor_Offset = 0;
            }

#ifdef DEBUG_MOTOR_CALIBRATION
            Serial.print("Motor Offsets: Left = ");
            Serial.print(ui_Left_Motor_Offset);
            Serial.print(", Right = ");
            Serial.println(ui_Right_Motor_Offset);
#endif
            EEPROM.write(ci_Right_Motor_Offset_Address_L, lowByte(ui_Right_Motor_Offset));
            EEPROM.write(ci_Right_Motor_Offset_Address_H, highByte(ui_Right_Motor_Offset));
            EEPROM.write(ci_Left_Motor_Offset_Address_L, lowByte(ui_Left_Motor_Offset));
            EEPROM.write(ci_Left_Motor_Offset_Address_H, highByte(ui_Left_Motor_Offset));

            ui_Robot_State_Index = 0;    // go back to Mode 0
          }
#ifdef DEBUG_MOTOR_CALIBRATION
          Serial.print("Encoders L: ");
          Serial.print(encoder_LeftMotor.getRawPosition());
          Serial.print(", R: ");
          Serial.println(encoder_RightMotor.getRawPosition());
#endif
          ui_Mode_Indicator_Index = 4;
        }
        break;
      }
  }

  if ((millis() - ul_Display_Time) > ci_Display_Time)
  {
    ul_Display_Time = millis();

#ifdef DEBUG_MODE_DISPLAY
    Serial.print("Mode: ");
    Serial.println(ui_Mode_Indicator[ui_Mode_Indicator_Index], DEC);
#endif
    bt_Heartbeat = !bt_Heartbeat;
    CharliePlexM::Write(ci_Heartbeat_LED, bt_Heartbeat);
    digitalWrite(13, bt_Heartbeat);
    Indicator();
  }
}

// set mode indicator LED state
void Indicator()
{
  //display routine, if true turn on led
  CharliePlexM::Write(ci_Indicator_LED, !(ui_Mode_Indicator[ui_Mode_Indicator_Index] &
                                          (iArray[iArrayIndex])));
  iArrayIndex++;
  iArrayIndex = iArrayIndex & 15;
}
