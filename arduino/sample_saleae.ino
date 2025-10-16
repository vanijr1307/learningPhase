// Sample UART sketch for Arduino with Saleae probing

void setup() {
  // Initialize UART at 9600 baud
  Serial.begin(9600);

  // Optional: indicate setup done via onboard LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  // Send a sample message over UART
  Serial.println("Hello Saleae! UART test data.");

  // Optional: blink LED to show loop is running
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);

  // Delay to space out messages
  delay(2000); // Total 1 second between messages
}
