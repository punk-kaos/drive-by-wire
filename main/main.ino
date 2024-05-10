void loop() {
  // Read the potentiometer value
  int potValue = analogRead(potPin);

  Serial.print("Potentiometer Value: ");
  Serial.println(potValue);

  // Adjust mapping to focus on the range 600-1023
  int mappedPotValue = map(potValue, 260, 950, 0, 1000); // Adjust the mapping range

  // Apply quadratic mapping for better sensitivity in the mid-range
  int motorSpeed = map(mappedPotValue * mappedPotValue, 0, 1000000, 100, 255);

  // Set motor direction based on the potentiometer value
  if (potValue < 275) {
    // Set motor direction for reverse
    digitalWrite(motorDirectionPin1, HIGH);
    digitalWrite(motorDirectionPin2, LOW);
    int idle = digitalRead(fastidle);
    if (idle == LOW) {
      analogWrite(motorSpeedPin, 175);
    } else {
      analogWrite(motorSpeedPin, 150);
    }
  } else {
    // Set motor direction for forward
    digitalWrite(motorDirectionPin1, LOW);
    digitalWrite(motorDirectionPin2, HIGH);
  }

  // Write the motor speed to the motor speed pin using PWM
  analogWrite(motorSpeedPin, motorSpeed);

  // Add a small delay to stabilize readings
  // delay(10);
}
