#include <Wire.h>
#include <I2Cdev.h>
//#include <MPU6050_6Axis_MotionApps20.h>
#include <IRremote.h>

IRrecv irReceiver(A0);
decode_results irInput;

#include <PID_v2.h>
float kp = 12, kp2 = 0.1;                    //15
int pControlSpeedR, pControlSpeedL, pControlSpeed;

int initialSpeed, initialSpeedRotation = 74; 
float runTime, runTimeF = 1550, runTimeB = 400;
float currentAngle;
float targetAngle;
int CW = 0, CCW = 0, stopRotate = 0;
int orientation = 0;

int maxSpeed = 255;
int minSpeed = 0;
int maxSpeed2 = 85;
int minSpeed2 = 0;

#include <AFMotor.h>
AF_DCMotor motorL(3);
AF_DCMotor motorR(4);

void setup() {
  Serial.begin(9600); 
// IMU Stuff
  Wire.begin();
  //Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  delay(20);
// IR Stuff
  irReceiver.enableIRIn();

  motorR.setSpeed(maxSpeed);
  motorL.setSpeed(maxSpeed);
}

void loop() {
  Serial.println(irInput.value);
  if (irReceiver.decode(&irInput))
  {
  int irReading = irInput.value;
switch (irReading)
    {
      case 16736925:                            // volume up - go straight
        motorL.run(FORWARD);
        motorR.run(FORWARD);
        delay(100);
        break;
      case 16754775:                           // volume down - go backward
        motorL.run(BACKWARD);
        motorR.run(BACKWARD);
        delay(100);
        break;
      case 16720605:                            // skip backward - face left
        motorL.run(BACKWARD);
        motorR.run(FORWARD);
        delay(100);
        break;
      case 16761405:                           // skip forward - face right
        motorL.run(FORWARD);
        motorR.run(BACKWARD);
        delay(100);
        break;
      case 16769055:                           // channel down - turn left (5 degrees)
          motorL.run(BACKWARD);
          motorR.run(FORWARD);
          delay(100);
        break;
      case 16756815:                           // channel up - turn right (5 degrees)
          motorL.run(FORWARD);
          motorR.run(BACKWARD);
          delay(100);
        break;
      case 16726215:                           // button 5 - forward adjustment
          motorL.run(FORWARD);
          motorR.run(FORWARD);
          delay(100);
        break;
      case 16712445:
          motorL.run(RELEASE);
          motorR.run(RELEASE);
          delay(100);
    }
    irReceiver.resume();
  }
}
