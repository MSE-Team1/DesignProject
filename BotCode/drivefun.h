#ifndef DRIVEFUN_H


//constants

const int ci_Left_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Right_Motor_Stop = 1500;
const int ci_Motor_Speed_Brake = 200; 
const int ci_Motor_Speed_Forward_Encoder = 1600;

const int RIGHT_MOTOR
const int LEFT_MOTOR

//if encoder count is less than the required count, forward speed is sent to motor
//encoders must be zeroed in previous stage
bool EncoderDriveForward(int i_Count, int i_Side){

  bool b_State;

  //RIGHT MOTOR
  if(i_Side == RIGHT_MOTOR){
    if(i_Count <= encoder_RightMotor){
      ui_Right_Motor_Speed = ci_Motor_Speed_Forward_Encoder;
      b_State = false;
    }
    else{
      ui_Right_Motor_Speed = ci_Motor_Speed_Brake;
      b_State = true;
    }
  }
  //LEFT MOTOR
  else{
    if(i_Count <= encoder_LeftMotor){
      ui_Right_Motor_Speed = ci_Motor_Speed_Forward_Encoder;
      b_State = false;
    }
    else{
      ui_Right_Motor_Speed = ci_Motor_Speed_Brake;
      b_State = true;
    }
  }

  return b_State;
}

#endif
