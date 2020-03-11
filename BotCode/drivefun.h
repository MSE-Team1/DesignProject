#ifndef DRIVEFUN_H
#define DRIVEFUN_H

//constants
const int ci_Left_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Right_Motor_Stop = 1500;
const int ci_Motor_Speed_Brake = 200;
const int ci_Motor_Speed_Forward_Encoder = 1700;
const int ci_Motor_Speed_Reverse_Encoder = 1300;

//arrays of speeds
int i_Motor_Speed_Forward[] = {1700, 1575, 1600, 1650}; //default, 1, 2, 3...etc
int i_Motor_Speed_Reverse[] = {1300, 1450, 1400, 1350}; //default, 1, 2, 3...etc

const float cf_Counts_Per_Rotation = 627.2; //how many encoder ticks in one rotation of the wheel
//TODO: measure robot and get actual values
const float cf_Rotation_Factor = cf_Counts_Per_Rotation*(18.5/7.0)/(2.0*PI); //to convert degrees of rotation of robot to encoder counts

//literals
const int RIGHT_MOTOR = 0;
const int LEFT_MOTOR = 1;

const int CLOCKWISE = 0;
const int COUNTERCLOCKWISE = 1;

const int SPEED_DEFAULT = 0; //selects default speed
const int SPEED_1 = 1;
const int SPEED_2 = 2;
const int SPEED_3 = 3;
const int SPEED_4 = 4;

//variables
unsigned int ui_Left_Motor_Speed;
unsigned int ui_Right_Motor_Speed;
unsigned int ui_Motors_Speed = 1900;        // Default run speed
unsigned int ui_Left_Saved_Encoder_Position;
unsigned int ui_Right_Saved_Encoder_Position; 
long l_Left_Motor_Position;
long l_Right_Motor_Position;


//if encoder count is less than the required count, forward speed is sent to motor
//encoders must be zeroed in previous stage
bool EncoderDriveForward(int i_Count, int i_Side, int i_Speed_Index) {

  bool b_State;

  //RIGHT MOTOR
  if (i_Side == RIGHT_MOTOR) {
    if (i_Count >= encoder_RightMotor.getRawPosition()) {
      ui_Right_Motor_Speed = i_Motor_Speed_Forward[i_Speed_Index];
      b_State = 0;
    }
    else {
      ui_Right_Motor_Speed = ci_Motor_Speed_Brake;
      b_State = 1;
    }

#ifdef DEBUG_ENCODER_DRIVE
    Serial.print("RIGHT STATE:  ");
#endif
  }
  //LEFT MOTOR
  else {
    if (i_Count >= encoder_LeftMotor.getRawPosition()) {
      ui_Left_Motor_Speed = i_Motor_Speed_Forward[i_Speed_Index];
      b_State = 0;
    }
    else {
      ui_Left_Motor_Speed = ci_Motor_Speed_Brake;
      b_State = 1;
    }
#ifdef DEBUG_ENCODER_DRIVE
    Serial.print("LEFT STATE:  ");
#endif
  }

#ifdef DEBUG_ENCODER_DRIVE
  Serial.println(b_State);
#endif

  return b_State;
}

//if encoder count is greater than the required count, Reverse speed is sent to motor
//encoders must be zeroed in previous stage
bool EncoderDriveReverse(int i_Count, int i_Side, int i_Speed_Index) {

  bool b_State;

  i_Count *= -1; //make value negative

  //RIGHT MOTOR
  if (i_Side == RIGHT_MOTOR) {
    if (i_Count <= encoder_RightMotor.getRawPosition()) {
      ui_Right_Motor_Speed = i_Motor_Speed_Reverse[i_Speed_Index];
      b_State = 0;
    }
    else {
      ui_Right_Motor_Speed = ci_Motor_Speed_Brake;
      b_State = 1;
    }

#ifdef DEBUG_ENCODER_DRIVE
    Serial.print("RIGHT STATE:  ");
#endif
  }
  //LEFT MOTOR
  else {
    if (i_Count <= encoder_LeftMotor.getRawPosition()) {
      ui_Left_Motor_Speed = i_Motor_Speed_Reverse[i_Speed_Index];
      b_State = 0;
    }
    else {
      ui_Left_Motor_Speed = ci_Motor_Speed_Brake;
      b_State = 1;
    }
#ifdef DEBUG_ENCODER_DRIVE
    Serial.print("LEFT STATE:  ");
#endif
  }

#ifdef DEBUG_ENCODER_DRIVE
  Serial.println(b_State);
#endif

  return b_State;
}

//zero point turn left a certain amount of degrees
//encoders must be zeroed in previous stage
bool ZeroPoint(int i_Degrees, int i_Direction, int i_Speed_Index) {
  boolean bt_State = 1;
  int i_Count = (((float)i_Degrees*71.0)/4068.0) * cf_Rotation_Factor; //calculate the number of encoder counts needed to turn (need to convert to rads)

#ifdef DEBUG_ZERO_POINT_TURN
  Serial.print("DEGREES: ");
  Serial.print(i_Degrees);
  Serial.print("  COUNTS: ");
  Serial.println(i_Count);
#endif

  //clockwise direction
  if (i_Direction == CLOCKWISE) {
    bt_State &= EncoderDriveForward(i_Count, LEFT_MOTOR, i_Speed_Index);
    bt_State &= EncoderDriveReverse(i_Count, RIGHT_MOTOR, i_Speed_Index);
  }
  //counterclockwise direction
  else {
    bt_State &= EncoderDriveReverse(i_Count, LEFT_MOTOR, i_Speed_Index);
    bt_State &= EncoderDriveForward(i_Count, RIGHT_MOTOR, i_Speed_Index);
  }
  return bt_State;
}

#endif
