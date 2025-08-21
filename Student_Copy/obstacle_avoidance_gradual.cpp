#include <Arduino.h>
#include <ServoTimer2.h>

#define ENA 5   // Enable pin for Motor A
#define IN1 6   // Control pin 1 for Motor A (Right)
#define IN2 7   // Control pin 2 for Motor A (Right)
#define IN3 8   // Control pin 1 for Motor B (Left)
#define IN4 9   // Control pin 2 for Motor B (Left)
#define ENB 10  // Enable pin for Motor B

// Pins for Servo Motor and Ultrasonic Sensor
#define TRIG_PIN 12
#define ECHO_PIN 13
#define SERVO_PIN 11

ServoTimer2 myservo; // Create Servo object to control a servo

// Speed range (0 - 255)
int travelSpeed = 200;  
int turningSpeed = 140;
int maximumAllowedSpeed = 200;
int leastAllowedSpeed = 70;

int travelDuration = 500;
int turningDuration = 700;
int time = 500;

int obstacleThresholdMin = 30;
int obstacleThresholdMax = 200;
int distance_left, distance_right;

// Function Prototypes
void moveForward(int duration);
void turnRight(int duration);
void turnLeft(int duration);
void moveBackward(int duration);
void stopCar();

int lookRight();

void changeSpeedWithDistance(int distance);
int lookLeft();
void rotateServo(int angle);
int getDistance();
void findAndRotateToFreePath();

void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  myservo.attach(SERVO_PIN);
  rotateServo(90);

  Serial.println("Robot starting...");
  delay(2000);
}


void loop() {
  //////////// WRITE YOUR CODE BETWEEN HERE


  
  //////////// END
}

// Move Forward
void moveForward(int duration) {
  Serial.println("Moving Forward...");
  analogWrite(ENA, travelSpeed);
  analogWrite(ENB, travelSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  delay(duration);
}

// Turn Right
void turnRight(int duration) {
  Serial.println("Turning Right...");
  analogWrite(ENA, turningSpeed);
  analogWrite(ENB, turningSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  delay(duration);
}

// Turn Left
void turnLeft(int duration) {
  Serial.println("Turning Left...");
  analogWrite(ENA, turningSpeed);
  analogWrite(ENB, turningSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);


  delay(duration);
}


// Move Backward
void moveBackward(int duration) {
  Serial.println("Moving Backward...");
  analogWrite(ENA, travelSpeed);
  analogWrite(ENB, travelSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  delay(duration);
}

void stopCar() {
  Serial.println("Stopping Car...");
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// Moves at maximum speed if no obstacle and stops if there is an obstacle
void changeSpeedWithDistance(int distance) {
  int mappedSpeed;
  
  distance = constrain(distance, obstacleThresholdMin, obstacleThresholdMax);
  mappedSpeed = map(distance, obstacleThresholdMin, obstacleThresholdMax, leastAllowedSpeed, maximumAllowedSpeed);

  if (mappedSpeed <= leastAllowedSpeed) {
    mappedSpeed = 0;
  }

  travelSpeed = mappedSpeed;
}

// Look Right
int lookRight() {
  Serial.println("Looking Right...");
  rotateServo(0);
  delay(500);
  int distance = getDistance();
  rotateServo(90);
  delay(100);
  return distance;
}

// Look Left
int lookLeft() {
  Serial.println("Looking Left...");
  rotateServo(180);
  delay(500);
  int distance = getDistance();
  rotateServo(90);
  delay(100);
  return distance;
}

// Get Distance
int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  int distance = (pulseIn(ECHO_PIN, HIGH)/2) / 29.1;

  return distance;
}


void findAndRotateToFreePath()
{
  int distance_left, distance_right;
  
  // Look right
  distance_right = lookRight();

  // Look left
  distance_left = lookLeft();

  if (distance_right > obstacleThresholdMin) {
    Serial.println("Turning Right...");
    turnRight(turningDuration);
  } else if (distance_left > obstacleThresholdMin) {
    Serial.println("Turning Left...");
    turnLeft(turningDuration);
  }
}


int val;
// Rotate Servo Motor
void rotateServo(int angle)
{

  if (angle == 0) {
    val = 750;
  } else if (angle == 90) {
    val = 1500;
  } else if (angle == 180) {
    val = 2250;
  } else {
    val = 2250;
  }

  myservo.write(val);
  delay(2000);// waits for the servo to finish rotation
}