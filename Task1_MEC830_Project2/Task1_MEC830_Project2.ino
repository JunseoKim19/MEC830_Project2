#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include <utility/imumaths.h>
float angle_offset;
float angle;
MPU6050 mpu;
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity
vector
// IR receiver-----------------------------
#include <IRremote.hpp>
#define irPin 11
IRrecv irReceive(irPin);
decode_results irInput;
// motor driver----------------------------
#define enA 9 // right wheel
#define enB 6 // left wheel
#define in1 12 // forward
#define in2 13 // backward
#define in3 7 // forward
#define in4 8 // backward
int maxSpeed = 255;
int minSpeed = 0;
int maxSpeed2 = 85;
int minSpeed2 = 0;
// proportional control--------------------
float kp = 12, kp2 = 0.1; //15
78
int pControlSpeedR, pControlSpeedL, pControlSpeed;
int initialSpeed, initialSpeedRotation = 74; //68 or 72
float runTime, runTimeF = 1550, runTimeB = 400;
float currentAngle;
float targetAngle;
int CW = 0, CCW = 0, stopRotate = 0;
int orientation = 0;
void TurnOff () //
Motor Off
{
digitalWrite(in1, LOW);
digitalWrite(in2, LOW);
digitalWrite(in3, LOW);
digitalWrite(in4, LOW);
}
void forward () //
Forward
{
digitalWrite(in1, HIGH);
digitalWrite(in2, LOW);
digitalWrite(in3, HIGH);
digitalWrite(in4, LOW);
}
void backward () //
Backward
{
digitalWrite(in1, LOW);
digitalWrite(in2, HIGH);
digitalWrite(in3, LOW);
digitalWrite(in4, HIGH);
}
void left () //
Turn Left
{
digitalWrite(in1, HIGH);
digitalWrite(in2, LOW);
79
digitalWrite(in3, LOW);
digitalWrite(in4, HIGH);
}
void right () //
Turn Right
{
digitalWrite(in1, LOW);
digitalWrite(in2, HIGH);
digitalWrite(in3, HIGH);
digitalWrite(in4, LOW);
}
float measure_angle(void) //
IMU Angle Measurement
{
float z_angle;
sensors_event_t event;
bno.getEvent(&event);
z_angle = event.orientation.x;
//Serial.println(z_angle);
return z_angle;
}
void SpeedControlF () //
Speed Control Forward
{
currentAngle = measure_angle();
if (targetAngle == 0)
{
if (currentAngle > 180)
{
currentAngle -= 360;
}
}
pControlSpeedR = initialSpeed + (currentAngle - targetAngle) * kp;
if (pControlSpeedR > maxSpeed)
{
pControlSpeedR = maxSpeed;
}
80
if (pControlSpeedR < minSpeed)
{
pControlSpeedR = minSpeed;
}
pControlSpeedL = initialSpeed - (currentAngle - targetAngle) * kp;
if (pControlSpeedL > maxSpeed)
{
pControlSpeedL = maxSpeed;
}
if (pControlSpeedL < minSpeed)
{
pControlSpeedL = minSpeed;
}
}
void SpeedControlB () //
Speed Control Backward
{
currentAngle = measure_angle();
if (targetAngle == 0)
{
if (currentAngle > 180)
{
currentAngle -= 360;
}
}
//Serial.println(currentAngle);
pControlSpeedR = initialSpeed - (currentAngle - targetAngle) * kp;
if (pControlSpeedR > maxSpeed)
{
pControlSpeedR = maxSpeed;
}
if (pControlSpeedR < minSpeed)
{
pControlSpeedR = minSpeed;
}
pControlSpeedL = initialSpeed + (currentAngle - targetAngle) * kp;
if (pControlSpeedL > maxSpeed)
81
{
pControlSpeedL = maxSpeed;
}
if (pControlSpeedL < minSpeed)
{
pControlSpeedL = minSpeed;
}
}
void DriveForward () //
Drive Forward
{
forward();
float previousTime = millis();
while (1)
{
float currentTime = millis();
SpeedControlF();
analogWrite(enA, pControlSpeedR);
analogWrite(enB, pControlSpeedL);
if ((currentTime - previousTime) > runTime)
{
TurnOff();
break;
}
}
}
void DriveBackward () //
Drive Backward
{
backward();
float previousTime = millis();
while (1)
{
float currentTime = millis();
SpeedControlB();
analogWrite(enA, pControlSpeedR);
analogWrite(enB, pControlSpeedL);
if ((currentTime - previousTime) > runTime)
82
{
TurnOff();
break;
}
}
}
void SpeedControlRotation () //
Speed Control Rotation
{
currentAngle = measure_angle();
if ((targetAngle == 0) | (targetAngle == 90))
{
if (currentAngle > 180)
{
currentAngle -= 360;
}
}
float error = currentAngle - targetAngle;
if (error < -1)
{
pControlSpeed = initialSpeed - (currentAngle - targetAngle) * kp2;
//pControlSpeedR = initialSpeed - (currentAngle - targetAngle) * kp2;
//pControlSpeedL = initialSpeed - (currentAngle - targetAngle) * kp2;
CW = 1; CCW = 0; stopRotate = 0;
}
else if (error > 1)
{
pControlSpeed = initialSpeed + (currentAngle - targetAngle) * kp2;
//pControlSpeedR = initialSpeed + (currentAngle - targetAngle) * kp2;
//pControlSpeedL = initialSpeed + (currentAngle - targetAngle) * kp2;
CW = 0; CCW = 1; stopRotate = 0;
}
else
{
pControlSpeed = 0;
CW = 0; CCW = 0; stopRotate = 1;
}
if (pControlSpeed > maxSpeed2)
{
83
pControlSpeed = maxSpeed2;
}
if (pControlSpeed < minSpeed2)
{
pControlSpeed = minSpeed2;
}
}
void RotateCW () //
Rotate Clockwise
{
currentAngle = measure_angle();
if ((targetAngle == 0) | (targetAngle == 90))
{
if (currentAngle > 180)
{
currentAngle -= 360;
}
}
//Serial.print("current angle: ");
//Serial.println(currentAngle);
while (1)
{
Serial.print("error: ");
Serial.println((currentAngle - targetAngle));
SpeedControlRotation();
if (CW)
{
right();
}
if (CCW)
{
left();
}
if (stopRotate)
{
TurnOff();
break;
}
Serial.print(targetAngle);
84
Serial.print(" ");
Serial.println(currentAngle);
analogWrite(enA, pControlSpeed);
analogWrite(enB, pControlSpeed);
//Serial.print("control speed: ");
//Serial.println(pControlSpeed);
}
}
void RotateCCW () //
Rotate Counterclockwise
{
currentAngle = measure_angle();
if ((targetAngle == 0) | (targetAngle == 270))
{
if (currentAngle > 180)
{
currentAngle -= 360;
}
}
//Serial.print("current angle: ");
//Serial.println(currentAngle);
while (1)
{
SpeedControlRotation();
if (CW)
{
right();
}
if (CCW)
{
left();
}
if (stopRotate)
{
TurnOff();
break;
}
Serial.print(targetAngle);
Serial.print(" ");
85
Serial.println(currentAngle);
analogWrite(enA, pControlSpeed);
analogWrite(enB, pControlSpeed);
//Serial.print("control speed: ");
//Serial.println(pControlSpeed);
}
}
void setup() //
Setup
{
Serial.begin(9600);
irReceive.enableIRIn();
Serial.println();
Serial.println("Calibrating IMU");
if (!bno.begin())
{
Serial.print("no imu sensor detected");
while (1);
}
delay(2000);
bno.setExtCrystalUse(true);
Serial.println("Done Calibrating");
Serial.println("Starting...");
sensors_event_t event;
bno.getEvent(&event);
angle_offset = event.orientation.x;
Serial.println("Offset: ");
Serial.println(angle_offset);
}
void loop() //
Main Loop
{
if (irReceive.decode(&irInput))
{
int irReading = irInput.value;
switch (irReading)
{
case -26521: // face North
86
Serial.println("Facing +Y");
initialSpeed = initialSpeedRotation;
orientation = 0;
targetAngle = orientation;
RotateCCW();
break;
case 6375: // button 2 - go straight
Serial.println("Going forward for 1.55s");
initialSpeed = 120;
targetAngle = orientation;
runTime = runTimeF;
DriveForward();
break;
case 19125: // button 8 - go backward
Serial.println("Going backward for 1.55s");
initialSpeed = 120;
targetAngle = orientation;
runTime = runTimeB;
DriveBackward();
break;
case 4335: // button 4 - face left
Serial.println("Rotating CCW by 90 degrees");
initialSpeed = initialSpeedRotation;
orientation -= 90;
if (orientation < 0)
{
orientation += 360;
}
targetAngle = orientation;
RotateCCW();
break;
case 23205: // button 6 - face right
Serial.println("Rotating CW by 90 degrees");
initialSpeed = initialSpeedRotation;
orientation += 90;
targetAngle = orientation;
RotateCW();
break;
case 12495: // button 1 - turn left (5
degrees)
87
Serial.println("Rotating CCW by 5 degrees");
initialSpeed = initialSpeedRotation;
orientation =- 5;
if (orientation < 0)
{
orientation += 360;
}
targetAngle = orientation;
RotateCCW();
break;
case 31365: // button 3 - turn right (5
degrees)
Serial.println("Rotating CW by 90 degrees");
initialSpeed = initialSpeedRotation;
orientation += 5;
targetAngle = orientation;
RotateCW();
break;
case 17085: // button 7 - face West
Serial.println("Facing -X");
initialSpeed = initialSpeedRotation;
orientation = 270;
targetAngle = orientation;
RotateCCW();
break;
case 21165: // button 9 - face East
Serial.println("Facing +X");
initialSpeed = initialSpeedRotation;
orientation = 90;
targetAngle = orientation;
RotateCW();
break;
case 14535: // button 5 - forward
adjustment
Serial.println("Going forward for 0.4s");
initialSpeed = 120;
targetAngle = orientation;
runTime = runTimeB;
DriveForward();
break;
88
}
irReceive.resume();
}
