// Define the pins
const int potPin = A0;          // Potentiometer pin
const int motorSpeedPin = 9;    // PWM pin connected to H-bridge input for speed control
const int motorDirectionPin1 = 8; // Digital pin connected to H-bridge input for direction control 1
const int motorDirectionPin2 = 7; // Digital pin connected to H-bridge input for direction control 2
const int fastidle = 6;

void setup() {
  pinMode(potPin, INPUT);            // Set potentiometer pin as input
  pinMode(fastidle, INPUT);
  pinMode(motorSpeedPin, OUTPUT);    // Set motor speed control pin as output
  pinMode(motorDirectionPin1, OUTPUT); // Set motor direction control pin 1 as output
  pinMode(motorDirectionPin2, OUTPUT); // Set motor direction control pin 2 as output
  
  // Set prescaler to 8 for Timer1 (associated with pins 9 and 10) for a PWM frequency of 1 kHz
TCCR1B = (TCCR1B & 0b11111000) | 0x01;  
  // Initialize serial communication at 9600 baud rate
  Serial.begin(9600);
}

void loop() {
  // Read the potentiometer value
  int potValue = analogRead(potPin);

  //if(potValue < 200 and potValue > 45){potValue=potValue*3;} 
  Serial.print("Potentiometer Value: ");
  Serial.println(potValue);

  // Adjust mapping to focus on the range 600-1023
    
  int motorSpeed = map(potValue, 260, 950, 100, 255);
  //motorSpeed = motorSpeed * motorSpeed / 255;

  // Set motor direction based on the potentiometer value
  if (potValue < 275) {
    // Set motor direction for reverse
    digitalWrite(motorDirectionPin1, HIGH);
    digitalWrite(motorDirectionPin2, LOW);
    int idle=digitalRead(fastidle);
    if (idle==LOW) {
      analogWrite(motorSpeedPin,175);
    }
    else{
    analogWrite(motorSpeedPin,150);
    }
  }
  else{
    // Set motor direction for forward
    digitalWrite(motorDirectionPin1, LOW);
    digitalWrite(motorDirectionPin2, HIGH);
  
  // Write the motor speed to the motor speed pin using PWM
  analogWrite(motorSpeedPin, motorSpeed);
  }
  // Add a small delay to stabilize readings
  //delay(10);
}
