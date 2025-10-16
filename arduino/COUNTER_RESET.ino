void setup() {
  pinMode(9, OUTPUT);
  pinMode(4, INPUT);
  pinMode(13, OUTPUT);

  setupTimer1();
  setupTimer0();
}

void loop() {
  if (TCNT0 >= 100) {
    TCNT0 = 0;
    PORTB ^= (1 << 5);
  }
}

void setupTimer1(void) {
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << WGM12);
  TCCR1A |= (1 << COM1A0);
  TCCR1B |= (1 << CS11);
  OCR1A = 999;
}

void setupTimer0(void) {
  TCCR0A = 0;
  TCCR0B = 0;
  TCCR0B |= (1 << CS02) | (1 << CS00);
  TCNT0 = 0;
}
