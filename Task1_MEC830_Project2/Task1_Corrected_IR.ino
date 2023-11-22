#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <IRremote.h>
#include <PID_v2.h>


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
#define in3 7       // forward
#define in4 8       // backward

#define irPin A0    // IR receiver pin
IRrecv irReceiver(irPin);
decode_results irInput;

int maxSpeed = 255;
int minSpeed = 0;
int maxSpeed2 = 85;
int minSpeed2 = 0;

float kp = 12, kp2 = 0.1;                    //15
int pControlSpeedR, pControlSpeedL, pControlSpeed;
int initialSpeed, initialSpeedRotation = 74; 
float runTime, runTimeF = 1550, runTimeB = 400;
float currentAngle;
float targetAngle;
int CW = 0, CCW = 0, stopRotate = 0;
int orientation = 0;

void setup() {
    Wire.begin();
    Serial.begin(9600);
    mpu.initialize();
    irReceiver.enableIRIn();

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
    irReceiver.enableIRIn();
}

void loop()                                                             // Main Loop
{
  Serial.println(irInput.value);
  if (irReceiver.decode(&irInput))
  {
    int irReading = irInput.value;
    switch (irReading)
    {

      case 16736925:                            // volume up - go straight
        Serial.println("Going forward for 1.55s");
        initialSpeed = 120;
        targetAngle = orientation;
        runTime = runTimeF;
        DriveForward();
        break;
      case 16754775:                           // volume down - go backward
        Serial.println("Going backward for 1.55s");
        initialSpeed = 120;
        targetAngle = orientation;
        runTime = runTimeB;
        DriveBackward();
        break;
      case 16761405:                           // skip forward - face right
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
      case 23205:                           // button 6 - face right
        Serial.println("Rotating CW by 90 degrees");
        initialSpeed = initialSpeedRotation;
        orientation += 90;
        targetAngle = orientation;
        RotateCW();
        break;
      case 16769055:                           // channel down - turn left (5 degrees)
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
      case 16756815:                           // channel up - turn right (5 degrees)
        Serial.println("Rotating CW by 90 degrees");
        initialSpeed = initialSpeedRotation;
        orientation += 5;
        targetAngle = orientation;
        RotateCW();
        break;
        case -26521:                          // face North
        Serial.println("Facing +Y");
        initialSpeed = initialSpeedRotation;
        orientation = 0;
        targetAngle = orientation;
        RotateCCW();
        break;
      case 17085:                           // button 7 - face West
        Serial.println("Facing -X");
        initialSpeed = initialSpeedRotation;
        orientation = 270;
        targetAngle = orientation;
        RotateCCW();
        break;
      case 21165:                           // button 9 - face East
        Serial.println("Facing +X");
        initialSpeed = initialSpeedRotation;
        orientation = 90;
        targetAngle = orientation;
        RotateCW();
        break;
      case 16726215:                           // button 5 - forward adjustment
        Serial.println("Going forward for 0.4s");
        initialSpeed = 120;
        targetAngle = orientation;
        runTime = runTimeB;
        DriveForward();
        break;
    }
    irReceiver.resume();
  }
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
        return ypr[0] * 180/M_PI;
    }
    return 0;
}

void TurnOff ()                                                         // Motor Off
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW); 
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

void right ()                                                           // Turn Right
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW); 
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

void SpeedControlB ()                                                   // Speed Control Backward
{
  currentAngle = measure_angle();
  if (targetAngle == 0)
  {
    if (currentAngle > 180)
    {
      currentAngle -= 360;
    }
  }
  Serial.println(currentAngle);
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

void DriveBackward ()                                                   // Drive Backward
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
    if (currentAngle > 180)
    {
      currentAngle -= 360;
    }
  }
  float error = currentAngle - targetAngle;
  if (error < -1)
  {
    pControlSpeed = initialSpeed - (currentAngle - targetAngle) * kp2;
    pControlSpeedR = initialSpeed - (currentAngle - targetAngle) * kp2;
    pControlSpeedL = initialSpeed - (currentAngle - targetAngle) * kp2;
    CW = 1; CCW = 0; stopRotate = 0; 
  }
  else if (error > 1)
  {
    pControlSpeed = initialSpeed + (currentAngle - targetAngle) * kp2;
    pControlSpeedR = initialSpeed + (currentAngle - targetAngle) * kp2;
    pControlSpeedL = initialSpeed + (currentAngle - targetAngle) * kp2;
    CW = 0; CCW = 1; stopRotate = 0;
  }
  else 
  {
    pControlSpeed = 0;
    CW = 0; CCW = 0; stopRotate = 1;
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

void RotateCW ()                                                        // Rotate Clockwise
{
  currentAngle = measure_angle();
  if ((targetAngle == 0) | (targetAngle == 90))
  {
    if (currentAngle > 180)
    {
      currentAngle -= 360;
    }
  }
  Serial.print("current angle: ");
  Serial.println(currentAngle);
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
    Serial.print("  ");
    Serial.println(currentAngle);
    analogWrite(enA, pControlSpeed);
    analogWrite(enB, pControlSpeed);
    //Serial.print("control speed: "); 
    //Serial.println(pControlSpeed);
  }
}

void RotateCCW ()                                                       // Rotate Counterclockwise
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
    Serial.print("  ");
    Serial.println(currentAngle);
    analogWrite(enA, pControlSpeed);
    analogWrite(enB, pControlSpeed);
    //Serial.print("control speed: "); 
    //Serial.println(pControlSpeed);
    }
}

