#include <Wire.h>
#include <I2Cdev.h>

//IR stuff
#include <IRremote.h>
#define irPin A0    // IR receiver pin
IRrecv irReceiver(irPin);
decode_results irInput;
float start = 0;

//Motor stuff
#define enA 9       // right wheel
#define enB 6       // left wheel
#define in1 12      // forward left side
#define in2 13      // backward left side
#define in3 7       // backward right side
#define in4 8       // forward right side
int speed = 100;
int time = 350;
int pControlSpeed;

//HC-SR04 stuff
#define trigPin A3
#define echoPin A2
float distance;

//Checkpoints
int Task1, Task2, Task2C1, Task2C2, Task2C3, Task3, Task4, count = 0;

void setup() {
   Wire.begin();
    Serial.begin(9600);
    //IR stuff
    irReceiver.enableIRIn();

    //Motor stuff
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

}

void loop() {
  
  if (irReceiver.decode(&irInput))
  {
    Serial.println(irInput.value);
    int irReading = irInput.value;
    switch (irReading)
    {
      case 16736925:                            // volume up - go straight
        pControlSpeed = 120;
        analogWrite(enA, pControlSpeed);
        analogWrite(enB, pControlSpeed);
        forward();
        delay(300);
        TurnOff();
        break;
      case 16754775:                           // volume down - go backward
        pControlSpeed = 120;
        analogWrite(enA, pControlSpeed);
        analogWrite(enB, pControlSpeed);
        backward();
        delay(300);
        TurnOff();
        break;
      case 16720605:                            // skip backward - face left
        pControlSpeed = 120;
        analogWrite(enA, pControlSpeed);
        analogWrite(enB, pControlSpeed);
        left();
        delay(300);
        TurnOff();
        break;
      case 16761405:                           // skip forward - face right
        pControlSpeed = 120;
        analogWrite(enA, pControlSpeed);
        analogWrite(enB, pControlSpeed);
        right();
        delay(300);
        TurnOff();
        break;
      case 16716015:                           // button 4 - turn left (5 degrees)
        pControlSpeed = 120;
        analogWrite(enA, pControlSpeed);
        analogWrite(enB, pControlSpeed);
        left();
        delay(100);
        TurnOff();
        break;
      case 16734885:                           // button 6 - turn right (5 degrees)
        pControlSpeed = 120;
        analogWrite(enA, pControlSpeed);
        analogWrite(enB, pControlSpeed);
        right();
        delay(100);
        TurnOff();
        break;
      case 16718055:                           // button 2 - forward adjustment
        pControlSpeed = 120;
        analogWrite(enA, pControlSpeed);
        analogWrite(enB, pControlSpeed);
        forward();
        delay(100);
        TurnOff();
        break;
      case 16730805:                           // button 8 - backward adjustment
        pControlSpeed = 120;
        analogWrite(enA, pControlSpeed);
        analogWrite(enB, pControlSpeed);
        backward();
        delay(100);
        TurnOff();
        break;
      case 16738455:
      Task1 = 1;
      break;
      case 16750695:
      Task2 = 1;
      break;
      case 16756815: // play/pause button
      Task3 = 1;
      break;
      case 16743045:
      Task4 = 1;
      count = 0;
      break;
    }
    irReceiver.resume();
  }

  if (Task2) {
      Task2 = 0;
      Task2C1 = 1;
    }
  if (Task2C1) {
    distance = obstacle_detection();
    Serial.println(distance);
    delay(50);
    if (distance <= 10){
      TurnOff();
      delay(300);
      analogWrite(enA, speed);
      analogWrite(enB, speed);
      right();
      delay(time*1.5);
      TurnOff();
      delay(200);
      Task2C1=0;
      Task2C2=1;
    }
    else {
      analogWrite(enA, speed);
      analogWrite(enB, speed);  
      forward();
      distance = obstacle_detection();
    }
  }

  if (Task2C2) {
    distance = obstacle_detection();
    delay(50);
    if (distance <= 10){
      TurnOff();
      delay(300);
      analogWrite(enA, speed);
      analogWrite(enB, speed);
      left();
      delay(time*1.5);
      TurnOff();
      delay(200);
      Task2C2=0;
      Task2C3=1;
    }
    else { 
      analogWrite(enA, speed);
      analogWrite(enB, speed);  
      forward();
      distance = obstacle_detection();
    }
  }

  if (Task2C3) {
    distance = obstacle_detection();
    delay(50);
    if (distance <= 10){
      TurnOff();
      delay(300);
      analogWrite(enA, speed);
      analogWrite(enB, speed);
      for(int i=0;i<6;i++){
      right();
      delay(100);
      left();
      delay(100);
      }
      TurnOff();
      Task2C3 = 0;
      start = 0;
    }
    else {
      analogWrite(enA, speed);
      analogWrite(enB, speed);  
      forward();
      distance = obstacle_detection();
    }
  }
  if (Task3) {
    distance = obstacle_detection();
    Serial.println(distance);
    delay(50);
    if (distance <= 20){
      TurnOff();
      delay(300);
      analogWrite(enA, speed);
      analogWrite(enB, speed);
      left();
      delay(time);
      TurnOff();
      delay(200);
      forward();
      delay(time*3);
      TurnOff();
      delay(200);
      right();
      delay(time*1.5);
      TurnOff();
      delay(200);
      forward();
      delay(time*3);
      delay(200);
      left();
      delay(time);
      TurnOff();
      delay(200);
      forward();
      delay(2000);
      TurnOff();
      Task3 = 0;
    }
    else {
      analogWrite(enA, speed);
      analogWrite(enB, speed);  
      forward();
      distance = obstacle_detection();
    }
  }
  if(Task4)
  {
    while(count<4){
      distance = obstacle_detection();
      Serial.println(distance);
      delay(50);
      if (distance <= 10){
        TurnOff();
        delay(300);
        analogWrite(enA, speed);
        analogWrite(enB, speed);
        right();
        delay(time*2);
        TurnOff();
        delay(200);
        count++;
      }
      else {
        analogWrite(enA, speed);
        analogWrite(enB, speed);  
        forward();
        distance = obstacle_detection();
      }
    }
  }
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
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void right ()                                                           // Turn Right
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH); 
}

void TurnOff ()                                                         // Motor Off
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW); 
}

float obstacle_detection() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    float duration = pulseIn(echoPin, HIGH);
    distance = (duration / 2.0) * 0.0343;
    Serial.println(distance);
    return distance;
}