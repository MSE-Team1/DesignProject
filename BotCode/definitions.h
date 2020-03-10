#ifndef DEFINITIONS_H
#define DEFINITIONS_H
// Uncomment keywords to enable debugging output

//#define DEBUG_MODE_DISPLAY
//#define DEBUG_MOTORS
//#define DEBUG_DRIVE
//#define DEBUG_LINE_TRACKERS
//#define DEBUG_ENCODERS
//#define DEBUG_ENCODER_DRIVE
//#define DEBUG_MOTOR_CALIBRATION
//#define DEBUG_ULTRASONIC
//#define DEBUG_LINE_TRACKER_CALIBRATION
#define DEBUG_IRSensor

//motor objects
Servo servo_RightMotor;
Servo servo_LeftMotor;
Servo servo_ArmMotor;    
Servo servo_GripMotor;

//encoder objects
I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;

#endif
