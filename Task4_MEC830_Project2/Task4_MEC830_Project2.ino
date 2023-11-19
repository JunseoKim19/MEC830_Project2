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

float set_speed;
double Kp_A = 8;
double Ki_A = 0.3;
double Kd_A = 0.4;
double Kp_R = 1.9;
double Ki_R = 0.5;
double Kd_R = 0;

double setpoint_rotate = 90.0;
double setpoint_drive = 0.0;
double output_rotate;
double output_drive;

int pControlSpeedR;
int pControlSpeedL;
float kp = 15.0;
int initialSpeed = 100; 
float targetAngle = 0;
int maxSpeed = 255;
int minSpeed = 0;
int maxSpeed2 = 200; 
int minSpeed2 = -200; 
float kp2 = 0.1; 
int CW = 0, CCW = 0, stopRotate = 0;
unsigned long runTime = 2000;

PID myPID_drive(&ypr[0], &output_drive, &setpoint_drive, Kp_A, Ki_A, Kd_A, DIRECT);
PID myPID_rotate(&ypr[0], &output_rotate, &setpoint_rotate, Kp_R, Ki_R, Kd_R, DIRECT);

void setup() {
    Serial.begin(9600);
    Wire.begin();
    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    pinMode(enA, OUTPUT); // Right wheel speed control
    pinMode(enB, OUTPUT); // Left wheel speed control
    pinMode(in1, OUTPUT); // Right wheel forward
    pinMode(in2, OUTPUT); // Right wheel backward
    pinMode(in3, OUTPUT); // Left wheel forward
    pinMode(in4, OUTPUT); // Left wheel backward
}

void loop() {

}

float measure_angle() {
    if (!dmpReady) return 0;

    fifoCount = mpu.getFIFOCount();
    if (fifoCount >= packetSize) {
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        float yaw_angle = ypr[0] * 180/M_PI;
        Serial.println(yaw_angle);
        return yaw_angle; // return yaw angle in degrees
    }
    return 0;
}

void forward ()                                                         // Forward
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void backward ()                                                        // Backward
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void left ()                                                            // Turn Left
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void SpeedControlF ()                                                   // Speed Control Forward
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

void DriveForward ()                                                    // Drive Forward
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

void SpeedControlRotation ()                                            // Speed Control Rotation
{
  currentAngle = measure_angle();
  if ((targetAngle == 0) | (targetAngle == 90))
  {
    if ((currentAngle > 270) && (currentAngle < 360))
    {
      currentAngle -= 360;
    }
    Serial.print("current: ");
    Serial.println(currentAngle);
  }
  float error = currentAngle - targetAngle;
  if (error < -1)
  {
    pControlSpeed = initialSpeed - (currentAngle - targetAngle) * kp2;

    CW = 1; CCW = 0; stopRotate = 0;
    Serial.println(CW);
  }
  else if (error > 1)
  {
    pControlSpeed = initialSpeed + (currentAngle - targetAngle) * kp2;
    CW = 0; CCW = 1; stopRotate = 0;
    Serial.println(CCW);
  }
  else
  {
    pControlSpeed = 0;
    CW = 0; CCW = 0; stopRotate = 1;
    Serial.println(stopRotate);
  }
  if (pControlSpeed > maxSpeed2)
  {
    pControlSpeed = maxSpeed2;
  }
  if (pControlSpeed < minSpeed2)
  {
    pControlSpeed = minSpeed2;
  }
}

