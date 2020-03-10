#ifndef DRIVEFUN_H
#define DRIVEFUN_H

//constants
const int ci_Left_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Right_Motor_Stop = 1500;
const int ci_Motor_Speed_Brake = 200;
const int ci_Motor_Speed_Forward_Encoder = 1700;
const int ci_Motor_Speed_Reverse_Encoder = 1300;
const int ci_Counts_Per_Rotation = 619; //how many encoder ticks in one rotation of the wheel
//TODO: measure robot and get actual values
const float cf_Rotation_Factor = 50.0/100.0; //to convert degrees of rotation of robot to degrees of rotation of wheels -- distance between wheels/diameter of wheel 

//identifiers for left and right motor in functions
const int RIGHT_MOTOR = 0;
const int LEFT_MOTOR = 1;

const int CLOCKWISE = 0;
const int COUNTERCLOCKWISE = 1;

//variables
unsigned int ui_Left_Motor_Speed;
unsigned int ui_Right_Motor_Speed;
unsigned int ui_Motors_Speed = 1900;        // Default run speed
long l_Left_Motor_Position;
long l_Right_Motor_Position;

//if encoder count is less than the required count, forward speed is sent to motor
//encoders must be zeroed in previous stage
bool EncoderDriveForward(int i_Count, int i_Side) {

  bool b_State;

  //RIGHT MOTOR
  if (i_Side == RIGHT_MOTOR) {
    if (i_Count >= encoder_RightMotor.getRawPosition()) {
      ui_Right_Motor_Speed = ci_Motor_Speed_Forward_Encoder;
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
      ui_Left_Motor_Speed = ci_Motor_Speed_Forward_Encoder;
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
bool EncoderDriveReverse(int i_Count, int i_Side) {

  bool b_State;

  i_Count *= -1; //make value negative

  //RIGHT MOTOR
  if (i_Side == RIGHT_MOTOR) {
    if (i_Count <= encoder_RightMotor.getRawPosition()) {
      ui_Right_Motor_Speed = ci_Motor_Speed_Reverse_Encoder;
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
      ui_Left_Motor_Speed = ci_Motor_Speed_Reverse_Encoder;
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
bool ZeroPoint(int i_Degrees, int i_Direction){
  boolean bt_State = 1;
  int i_Count = cf_Rotation_Factor*i_Degrees*ci_Counts_Per_Rotation/360; //calculate the number of encoder counts needed to turn

  //clockwise direction
  if(i_Direction == CLOCKWISE){
    bt_State &= EncoderDriveForward(i_Count, LEFT_MOTOR);
    bt_State &= EncoderDriveReverse(i_Count, RIGHT_MOTOR);
  }
  //counterclockwise direction
  else{
    bt_State &= EncoderDriveReverse(i_Count, LEFT_MOTOR);
    bt_State &= EncoderDriveForward(i_Count, RIGHT_MOTOR);
  }
  return bt_State;
}

#endif
