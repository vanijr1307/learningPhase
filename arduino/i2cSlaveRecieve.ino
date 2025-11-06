#include <Wire.h>

#define SLAVE_ADDR 0x61
uint8_t rcv_buf[32];
int data_len = 0;

void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  Wire.begin(); // join I2C bus as master
}

void loop() {
  
Serial.println("Arduino Master");
  Serial.println("Send character 's' to begin");
  Serial.println("-----------------------------");
  // Wait until something is typed in serial monitor
  while (Serial.available() == 0);

  char in_read = Serial.read();

  if (in_read == 's') {
    Serial.println("Starting..");

    // Step 1: Ask STM32 for length
    Wire.beginTransmission(SLAVE_ADDR);
    Wire.write(0x51); 
    Wire.endTransmission();

    Wire.requestFrom(SLAVE_ADDR, 1);
    if (Wire.available()) {
      data_len = Wire.read();
    }

    Serial.print("Data Length: ");
    Serial.println(data_len);

    // Step 2: Ask STM32 for data
    Wire.beginTransmission(SLAVE_ADDR);
    Wire.write(0x52);
    Wire.endTransmission();

    Wire.requestFrom(SLAVE_ADDR, data_len);

    uint32_t i = 0;
    for (i = 0; i < data_len; i++) {
      if (Wire.available()) {
        rcv_buf[i] = Wire.read();
      }
    }
    rcv_buf[data_len] = '\0';

    Serial.print("Data: ");
    Serial.println((char*)rcv_buf);
    Serial.println("*********************END*********************");
  }

  // Optional small delay before next prompt
  delay(300);
}
