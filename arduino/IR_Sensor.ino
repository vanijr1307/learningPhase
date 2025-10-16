void setup() {
pinMode(13,OUTPUT);
pinMode(3,INPUT);
Serial.begin(9600);

}

void loop() {
 if(digitalRead(3)==LOW){
  digitalWrite(13,LOW);
  delay(1000);
 }
 else{
  digitalWrite(13,LOW);
  delay(1000);
 }

}
