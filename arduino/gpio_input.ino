void setup() {
  Serial.begin(300);
  DDRD &= ~(1 << 2);  // PD2 = input
  // No internal pull-up enabled
}

void loop() {
  uint8_t buttonState = (PIND & (1 << 2)); // Read PD2
 Serial.println(PIND);
  if (buttonState == 0) {
     Serial.println("Button Pressed");
  } else {
    Serial.println("Button Released");
  }

  delay(300);
}
