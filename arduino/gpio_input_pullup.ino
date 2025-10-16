void setup() {
Serial.begin(9600);
DDRB &= ~(1<<4); //set pb4 pin 4 as input
PORTB |=(1<<4);// Enable internal pull-up resistor
}


void loop() {
  
  uint8_t buttonstate = (PINB &(1<<4));
  if(buttonstate==0){
    Serial.println("Pressed");

  }
  else{
        Serial.println(" Not Pressed");

  }

delay(1000);
}
