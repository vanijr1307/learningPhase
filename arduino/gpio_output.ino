void setup() {
DDRD |=(1<<3);

}

void loop() {
PORTD |=(1<<3);
delay(1000);

PORTD &= ~(1<<3);
delay(1000);
}
