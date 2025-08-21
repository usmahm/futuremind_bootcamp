#include <Arduino.h>

#define ENA 5   // Enable pin for Motor A
#define IN1 6   // Control pin 1 for Motor A (Right)
#define IN2 7   // Control pin 2 for Motor A (Right)
#define IN3 8   // Control pin 1 for Motor B (Left)
#define IN4 9   // Control pin 2 for Motor B (Left)
#define ENB 10  // Enable pin for Motor B

#define R_S 2 // Right sensor
#define L_S 4 // Left sensor

// Speed range (0 - 255)
int travelSpeed = 100;  
int turningSpeed = 80;
int travelDuration = 1000;
int turningDuration = 300;
int time = 2000;

int distance_left, distance_right;

// Function Prototypes
void moveForward(int duration);
void turnRight(int duration);
void turnLeft(int duration);
void moveBackward(int duration);
void stopCar();
String getPositionOfBlackLine();

void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(R_S, INPUT);
  pinMode(L_S, INPUT);

  Serial.println("Robot starting...");
  delay(2000);
}


void loop() {
  
}

// Returns "CENTER" or "LEFT" or "RIGHT"
String getPositionOfBlackLine()
{
  String blackLinePosition = "CENTER";

  if ((digitalRead(R_S) == 1) && (digitalRead(L_S) == 1))
  {
    blackLinePosition = "CENTER";
  }
  else if ((digitalRead(R_S) == 1) && (digitalRead(L_S) == 0))
  {
    blackLinePosition = "LEFT";
  }
  else if ((digitalRead(R_S) == 0) && (digitalRead(L_S) == 1))
  {
    blackLinePosition = "RIGHT";
  }

  return blackLinePosition;
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
