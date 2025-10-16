//attachInterrupt(interruptNumber, ISR_function, mode);
//interruptNumber → usually digitalPinToInterrupt(pin)
// ISR_function → your function to run on interrupt
// mode → RISING, FALLING, CHANGE, LOW, HIGH

bool volatile buttonPressed =false;

void handleButtonPress(){
  buttonPressed=true;
}
void setup() {
Serial.begin(9600);
DDRD &= ~(1<<2);
PORTD |=(1<<2);
attachInterrupt(digitalPinToInterrupt(2),handleButtonPress,FALLING);
// attachInterrupt(digitalPinToInterrupt(2),handleButtonPress,RISING);

}

void loop() {
if(buttonPressed){
  Serial.println("Inteerupt Triggerd");
  buttonPressed=false;
}
}
