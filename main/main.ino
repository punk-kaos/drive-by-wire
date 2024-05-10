#include <PID_v1.h>

// Define the pins
const int potPin = A0;                 // Potentiometer pin
const int positionSensorPin = A1;      // Position sensor pin
const int motorSpeedPin = 9;           // PWM pin connected to H-bridge input for speed control
const int motorDirectionPin1 = 8;      // Digital pin connected to H-bridge input for direction control 1
const int motorDirectionPin2 = 7;      // Digital pin connected to H-bridge input for direction control 2
const int fastidle = 6;

// Define PID variables
double Setpoint, Input, Output;
double Kp = 2, Ki = 5, Kd = 1;         // PID constants
PID pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  pinMode(potPin, INPUT);               // Set potentiometer pin as input
  pinMode(positionSensorPin, INPUT);    // Set position sensor pin as input
  pinMode(fastidle, INPUT);
  pinMode(motorSpeedPin, OUTPUT);       // Set motor speed control pin as output
  pinMode(motorDirectionPin1, OUTPUT);  // Set motor direction control pin 1 as output
  pinMode(motorDirectionPin2, OUTPUT);  // Set motor direction control pin 2 as output

  // Set prescaler to 8 for Timer1 (associated with pins 9 and 10) for a PWM frequency of 1 kHz
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;

  // Initialize serial communication at 9600 baud rate
  Serial.begin(9600);

  // Set PID parameters
  Setpoint = 512;  // Middle position
  pid.SetMode(AUTOMATIC);
}

void loop() {
  // Read the potentiometer value
  int potValue = analogRead(potPin);

  // Read the position sensor value
  int sensorValue = analogRead(positionSensorPin);

  // Check if potentiometer value is below 250 for idle condition
    if (potValue < 250) {
    // Set motor direction for reverse
    digitalWrite(motorDirectionPin1, HIGH);
    digitalWrite(motorDirectionPin2, LOW);
    int idle = digitalRead(fastidle);
    if (idle == LOW) {
      analogWrite(motorSpeedPin, 170);
    } else {
      analogWrite(motorSpeedPin, 160);
    }
  }

  // Map potentiometer value to match sensor range
  Setpoint = map(potValue, 0, 1023, 0, 1023);

  // Map sensor value to match potentiometer range
  Input = map(sensorValue, 0, 1023, 0, 1023);

  // Compute PID output
  pid.Compute();

  // Limit output to PWM range
  int motorSpeed = constrain(Output, 0, 255);

  // Set motor direction based on the error between setpoint and actual position
  if (sensorValue < Setpoint) {
    // Set motor direction for reverse
    digitalWrite(motorDirectionPin1, HIGH);
    digitalWrite(motorDirectionPin2, LOW);
  } else {
    // Set motor direction for forward
    digitalWrite(motorDirectionPin1, LOW);
    digitalWrite(motorDirectionPin2, HIGH);
  }
  Serial.println(motorSpeed);
  // Write the motor speed to the motor speed pin using PWM
  if (potValue > 250) {
    analogWrite(motorSpeedPin, motorSpeed);
  }
  // Add a small delay to stabilize readings
  delay(10);
}
