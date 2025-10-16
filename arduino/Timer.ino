// Generate square wave on Pin 9 using Timer1 in CTC mode
void setup() {
  // Set Pin 9 (OC1A) as output
  pinMode(9, OUTPUT);

  // Stop Timer1
  TCCR1A = 0;
  TCCR1B = 0;

  // Set CTC mode (Clear Timer on Compare Match)
  // WGM12 = 1 (Mode 4)
  TCCR1B |= (1 << WGM12);

  // Toggle OC1A on Compare Match
  // COM1A0 = 1, COM1A1 = 0
  TCCR1A |= (1 << COM1A0);

  // Set prescaler to 8 → CS11 = 1
  TCCR1B |= (1 << CS11);

  // Initial compare value → sets frequency
  OCR1A = 999; // Adjust to set output frequency
}

void loop() {
  // Change frequency dynamically
  delay(1000);
  OCR1A = 1999; // Lower frequency
  delay(1000);
  OCR1A = 499;  // Higher frequency
}
