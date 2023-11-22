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
int time = 700;

//HC-SR04 stuff
#define trigPin A3
#define echoPin A2
float distance;

//Checkpoints
int checkpoint_1, checkpoint_2, checkpoint_3 = 0;
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
      case 16712445: // play/pause button
      start = 1;
      checkpoint_1 = 0;
      checkpoint_2 = 0;
      checkpoint_3 = 0;
      break;
    }
    irReceiver.resume();
  }
  if (start) {
      start = 0;
      checkpoint_1 = 1;
    }

  if (checkpoint_1) {
    distance = obstacle_detection();
    Serial.println(distance);
    delay(50);
    if (distance <= 10){
      TurnOff();
      delay(300);
      analogWrite(enA, speed);
      analogWrite(enB, speed);
      right();
      delay(time);
      TurnOff();
      delay(200);
      checkpoint_1=0;
      checkpoint_2=1;
    }
    else {
      analogWrite(enA, speed);
      analogWrite(enB, speed);  
      forward();
      distance = obstacle_detection();
    }
  }

  if (checkpoint_2) {
    distance = obstacle_detection();
    delay(50);
    if (distance <= 10){
      TurnOff();
      delay(300);
      analogWrite(enA, speed);
      analogWrite(enB, speed);
      left();
      delay(time);
      TurnOff();
      delay(200);
      checkpoint_2=0;
      checkpoint_3=1;
    }
    else { 
      analogWrite(enA, speed);
      analogWrite(enB, speed);  
      forward();
      distance = obstacle_detection();
    }
  }

  if (checkpoint_3) {
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
      checkpoint_3 = 0;
      start = 0;
    }
    else {
      analogWrite(enA, speed);
      analogWrite(enB, speed);  
      forward();
      distance = obstacle_detection();
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