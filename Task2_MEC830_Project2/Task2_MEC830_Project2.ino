#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include <Smoothed.h>
#include <Ewma.h>
#include <PID_v2.h>
#include <SR04.h>
Smoothed <float> average_rot_speedL;
Smoothed <float> average_rot_speedR;
float angle_offset;
double angle;
double angleR;
MPU6050 mpu;
VectorFloat gravity; 
float ypr[3]; 
vector
//path checkpoints
bool start = 1;
bool checkpoint_1 =0;
bool checkpoint_2 =0;
bool checkpoint_3 = 0;
bool rotateR = 1;
// ultrasonic sensor-----------------------
#define trigPin 4
#define echoPin 5
SR04 sr04 = SR04(TRIG_PIN, ECHO_PIN);
89
float distance;
//Time variables
float previousTimeR = 0;
float currentTimeR = 0;
float previousTimeL = 0;
float currentTimeL = 0;
float loopTimeCurrent =0;
float loopTimePrevious =0;
// motor driver----------------------------
#define enA 9 // right wheel
#define enB 6 // left wheel
#define in1 12 // forward
#define in2 13 // backward
#define in3 7 // backward
#define in4 8 // forward
//Drive functions
float set_speed;
// PID Controller
double Kp_A = 8;
double Ki_A = 0.3;
double Kd_A = 0.4;
double Kp_R = 2;
double Ki_R = 0.5;
double Kd_R = 0;
double setpoint_rotate = 90.0;
double setpoint_drive = 0.0;
double output_rotate;
double output_drive;
//Proportional Control
int pControlSpeedR;
90
int pControlSpeedL;
float kp = 15.0;
int initialSpeed;
float targetAngle =0;
int maxSpeed = 255;
int minSpeed = 0;
PID
myPID_drive(&angle,&output_drive,&setpoint_drive,Kp_A,Ki_A,Kd_A,DIRECT);
PID
myPID_rotate(&angleR,&output_rotate,&setpoint_rotate,Kp_R,Ki_R,Kd_R,DIRECT
);
void stop_car(void){
setMotorL(0);
setMotorR(0);
}
void drive_straight(double set_speed){
myPID_drive.Compute();
double rightSpeed = set_speed - output_drive;
double leftSpeed = set_speed + output_drive;
if (rightSpeed> 255){
rightSpeed = 255;
}
else if (rightSpeed<0){
rightSpeed = 0;
}
if (leftSpeed>255){
leftSpeed = 255;
}
else if (leftSpeed<0){
leftSpeed = 0;
}
setMotorR(rightSpeed);
91
setMotorL(leftSpeed);
}
void rotate(){
delay(1000);
while (1){
angleR = measure_angle();
float error = abs(((angleR - setpoint_rotate)/(90.0))*100);
myPID_rotate.Compute();
double rightRotate = -output_rotate;
double leftRotate = output_rotate;
setMotorR(rightRotate);
setMotorL(leftRotate);
if (error<1)
{
stop_car();
break;
}
}
}
void setMotorR(int pwmR) {
if (pwmR>0)
{
digitalWrite(in1, HIGH);
digitalWrite(in2, LOW);
}
else if (pwmR<0){
digitalWrite(in1, LOW);
digitalWrite(in2, HIGH);
}
else if (pwmR ==0){
digitalWrite(in1, LOW);
digitalWrite(in2, LOW);
}
analogWrite(enA,abs(pwmR));
}
92
void setMotorL(int pwmL) {
if (pwmL>0)
{
digitalWrite(in3, HIGH);
digitalWrite(in4, LOW);
}
else if (pwmL<0){
digitalWrite(in3, LOW);
digitalWrite(in4, HIGH);
}
else if (pwmL ==0){
digitalWrite(in3, LOW);
digitalWrite(in4, LOW);
}
analogWrite(enB,abs(pwmL));
}
double measure_angle(void)
{
mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
double z_angle;// to determine absolute orientation
/* Get a new sensor event */
z_angle = ypr[0];
if (z_angle > 180){
z_angle = z_angle - 360;
}
return z_angle;
}
float obstacle_detection(void)
{
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
93
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
float duration = pulseIn(echoPin, HIGH);
float obstacle_distance = (duration * 0.034 / 2.0) * 1.0227 + 0.0031;
return obstacle_distance;
}
void setup() {
myPID_drive.SetMode(MANUAL); //initalize PID Controller
myPID_drive.SetSampleTime(10);
myPID_drive.SetOutputLimits(-200, 200);
myPID_rotate.SetMode(MANUAL); //initalize PID Controller
myPID_rotate.SetSampleTime(10);
myPID_rotate.SetOutputLimits(-110, 110);
//Initalize Ultrasonic Sensor
pinMode(trigPin, OUTPUT);
pinMode(echoPin,INPUT);
}
void loop() {
angle = measure_angle();
if (start){
delay(1000);
myPID_drive.SetMode(AUTOMATIC);
start = 0;
checkpoint_1 =1;
}
if (checkpoint_1){
distance = obstacle_detection();
if (distance <= 22){
myPID_drive.SetMode(MANUAL);
stop_car();
myPID_rotate.SetMode(AUTOMATIC);
rotate();
94
myPID_rotate.SetMode(MANUAL);
checkpoint_1=0;
checkpoint_2=1;
setpoint_drive = 90.0;
myPID_drive.SetMode(AUTOMATIC);
}
else
{
drive_straight(180);
//previous22
}
}
if (checkpoint_2){
distance = obstacle_detection();
if (distance <= 22){
myPID_drive.SetMode(MANUAL);
stop_car();
setpoint_rotate = 0.0;
myPID_rotate.SetMode(AUTOMATIC);
rotate();
myPID_rotate.SetMode(MANUAL);
checkpoint_2=0;
checkpoint_3=1;
targetAngle = 0.0;
setpoint_drive = 0.0;
myPID_drive.SetMode(AUTOMATIC);
}
else
{
drive_straight(180);
}
}
if (checkpoint_3){
distance = obstacle_detection();
if (distance <= 22){
myPID_drive.SetMode(MANUAL);
95
stop_car();
setpoint_rotate = 0.0;
myPID_rotate.SetMode(AUTOMATIC);
rotate();
myPID_rotate.SetMode(MANUAL);
checkpoint_3 =0.0;
}
else
{
drive_straight(180);
}
}
}