void setup() {
  pinMode(9, OUTPUT);
  setupPWM();
}

void loop() {
}

void setupPWM() {
  TCCR1A = 0;
  TCCR1B = 0;

  TCCR1A |= (1 << WGM10) | (1 << WGM11);
  TCCR1B |= (1 << WGM12);

  TCCR1A |= (1 << COM1A1);

  TCCR1B |= (1 << CS11);

  OCR1A = 512;
}
