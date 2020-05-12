#include <PID_v1.h>
#include <Servo.h>

Servo xServo, yServo;

int UR = A0;  // upper right
int LR = A1;  // lower right
int S  = A2;  // signal
int UL = A3;  // upper left
int LL = A4;  // lower left

// X and Y coordinates
double Xpos, Ypos;
// Servo angles for X and Y direction 
double xServoAngle, yServoAngle;

double prevAngle = 90;

// PID Control Variables
double xSetpoint, xInput, xOutput;
int xKp = 2, xKi = 0.5, xKd = 1;
PID xPID(&xInput, &xOutput, &xSetpoint, xKp, xKi, xKd, DIRECT);
double ySetpoint, yInput, yOutput;
int yKp = 1.1, yKi = 0.5, yKd = 1.5;
PID yPID(&yInput, &yOutput, &ySetpoint, yKp, yKi, yKd, DIRECT);

void setup() {
  xPID.SetSampleTime(50);
  yPID.SetSampleTime(50);
  
  xServo.attach(9);
  yServo.attach(10);
  xServo.write(90);
  yServo.write(90);

  pinMode(UR, OUTPUT);
  pinMode(LR, OUTPUT);
  pinMode(UL, OUTPUT);
  pinMode(LL, OUTPUT);
  pinMode(S, INPUT);

  Serial.begin(9600);

  // PID Initialization
  xInput = analogRead(S);
  xSetpoint = 230; // 230 is setpoint for center of platform
  xPID.SetMode(AUTOMATIC);
  yInput = analogRead(S);
  ySetpoint = 230; // 230 is setpoint for center of platform
  yPID.SetMode(AUTOMATIC);
}

// Get the current X coordinate of the ball
int Get_Xposition() {
  digitalWrite(UL, LOW);
  digitalWrite(UR, HIGH);
  digitalWrite(LL, LOW);
  digitalWrite(LR, HIGH);

  Xpos = analogRead(S);
  Xpos = Xpos - 262; //offset

  return Xpos;
}

// Get the current Y coordinate of the ball
int Get_Yposition() {

  digitalWrite(UL, LOW);
  digitalWrite(UR, HIGH);
  digitalWrite(LL, HIGH);
  digitalWrite(LR, LOW);

  Ypos = analogRead(S);
  Ypos = Ypos - 262; //offset

  return Ypos;
}

// ***ADD TOUCH DETECTION???***
void loop() {
  Xpos = Get_Xposition();
  xInput = Xpos;
  xPID.Compute();
  
  Ypos = Get_Yposition();
  yInput = Ypos;
  yPID.Compute();

  xOutput = xOutput*0.1;
  yOutput = yOutput*0.1;

  if(Xpos > 230) {
    xPID.SetControllerDirection(REVERSE);
    xServoAngle = (90 - xOutput);
  }
  else if(Xpos <= 230) {
    xPID.SetControllerDirection(DIRECT);
    xServoAngle = (90 + xOutput);
  }
  
  if(Ypos > 230) {
    yPID.SetControllerDirection(REVERSE);
    yServoAngle = (90 + yOutput);
  }
  else if(Ypos <= 230) {
    yPID.SetControllerDirection(DIRECT);
    yServoAngle = (90 - yOutput);
  }

  Serial.print("X Position: "); Serial.print(Xpos); Serial.print(" ");
  Serial.print("Servo Angle: "); Serial.println(xServoAngle);
  xServo.write(xServoAngle);

  Serial.print("Y Position: "); Serial.print(Ypos); Serial.print(" ");
  Serial.print("Servo Angle: "); Serial.println(yServoAngle);
  yServo.write(yServoAngle);

  delay(50);
}
