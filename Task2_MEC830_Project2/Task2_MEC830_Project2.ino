#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <PID_v1.h>

MPU6050 mpu;
bool dmpReady = false;  
uint8_t mpuIntStatus;   
uint8_t devStatus;      
uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64]; 
Quaternion q;           
VectorFloat gravity;    
float ypr[3];          

#define enA 9       // right wheel
#define enB 6       // left wheel
#define in1 12      // forward
#define in2 13      // backward
#define in3 7       // backward
#define in4 8       // forward

#define trigPin 4
#define echoPin 5
float distance;

float set_speed;
double Kp_A = 8;
double Ki_A = 0.3;
double Kd_A = 0.4;
double Kp_R = 2;
double Ki_R = 0.5;
double Kd_R = 0;
double setpoint_rotate = 90.0;
double setpoint_drive = 1.0;
double output_rotate;
double output_drive;

int pControlSpeedR;
int pControlSpeedL;
float kp = 15.0;
int initialSpeed;
float targetAngle = 0;
int maxSpeed = 255;
int minSpeed = 0;

PID myPID_drive(&ypr[0], &output_drive, &setpoint_drive, Kp_A, Ki_A, Kd_A, DIRECT);
PID myPID_rotate(&ypr[0], &output_rotate, &setpoint_rotate, Kp_R, Ki_R, Kd_R, DIRECT);

void setup() {
    Wire.begin();
    Serial.begin(9600);
    mpu.initialize();

    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    // Initialize PID controllers
    myPID_drive.SetMode(AUTOMATIC);
    myPID_drive.SetOutputLimits(-200, 200);
    myPID_drive.SetSampleTime(10);

    myPID_rotate.SetMode(AUTOMATIC);
    myPID_rotate.SetOutputLimits(-110, 110); 
    myPID_rotate.SetSampleTime(10); 
}


void loop() {
  angle = measure_angle();
  if (start){
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
      myPID_rotate.SetMode(MANUAL);
      checkpoint_1=0;
      checkpoint_2=1;
      setpoint_drive = 90.0;
      myPID_drive.SetMode(AUTOMATIC);
    }
    else
    { 
      drive_straight(120);
     
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
      
      drive_straight(120);
    }
  }

  if (checkpoint_3){
    distance = obstacle_detection();
    if (distance <= 22){
      myPID_drive.SetMode(MANUAL);
      stop_car();
      setpoint_rotate = 0.0;
      myPID_rotate.SetMode(AUTOMATIC);
      rotate();
      myPID_rotate.SetMode(MANUAL);
      checkpoint_3 =0.0;
    }
    else
    {  
       drive_straight(120);
    }
  }

}

double measure_angle() {
    if (!dmpReady) return 0;
    fifoCount = mpu.getFIFOCount();

    if (fifoCount >= packetSize) {
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        return ypr[0] * 180/M_PI; // yaw angle in degrees
    }
    return 0;
}

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
  setMotorL(leftSpeed);
}

void SpeedControlF (int initialSpeed)                                                   // Speed Control Forward
{
  int currentAngle = measure_angle();

  pControlSpeedR = initialSpeed + (currentAngle - targetAngle)*kp;
  if (pControlSpeedR > maxSpeed)
  {
    pControlSpeedR = maxSpeed;
  }
  if (pControlSpeedR < minSpeed)
  {
    pControlSpeedR = minSpeed;
  }
  
  pControlSpeedL = initialSpeed - (currentAngle - targetAngle)*kp;
  if (pControlSpeedL > maxSpeed)
  {
    pControlSpeedL = maxSpeed;
  }
  if (pControlSpeedL < minSpeed)
  {
    pControlSpeedL = minSpeed;
  }

  setMotorR(pControlSpeedR);
  setMotorL(pControlSpeedL);
}

void rotate() {
    unsigned long startTime = millis();
    while (millis() - startTime < 5000) { // 5-second timeout
        angleR = measure_angle();
        float error = abs(((angleR - setpoint_rotate) / 90.0) * 100);
        myPID_rotate.Compute();
        setMotorR(-output_rotate);
        setMotorL(output_rotate);
        if (error < 1) {
            stop_car();
            return; // Exit if within error threshold
        }
    }
    stop_car(); // Stop if timeout reached
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


float obstacle_detection() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    float duration = pulseIn(echoPin, HIGH);
    distance = (duration / 2.0) * 0.0343;
    return distance;
}
