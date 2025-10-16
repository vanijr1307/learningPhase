// Pin Definitions
const int ledPin = 13;
const int buttonPin = 2;



void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP); 
  Serial.begin(9600);
}

void loop() {
  // Read the button (LOW = pressed)
    bool val= digitalRead(buttonPin);
     if(val){
      digitalWrite(ledPin,LOW);
      
     }else {
      digitalWrite(ledPin,HIGH);
     }

  }

