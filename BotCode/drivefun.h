#ifndef DRIVEFUN_H
#define DRIVEFUN_H

//constants
const int ci_Left_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Right_Motor_Stop = 1500;
const int ci_Motor_Speed_Brake = 200;
const int ci_Motor_Speed_Forward_Encoder = 1600;

const int RIGHT_MOTOR = 0;
const int LEFT_MOTOR = 1;

//variables
unsigned int ui_Left_Motor_Speed;
unsigned int ui_Right_Motor_Speed;
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
      ui_Right_Motor_Speed = ci_Motor_Speed_Forward_Encoder;
      b_State = 0;
    }
    else {
      ui_Right_Motor_Speed = ci_Motor_Speed_Brake;
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

#endif
