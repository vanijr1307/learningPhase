const int trigPin = 11;
const int echoPin = 10;
long duration;
float distance;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  // Send trigger pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read echo pulse
  duration = pulseIn(echoPin, HIGH);  // Measure HIGH pulse duration in µs

  // Convert to distance (cm)
  distance = duration * 0.034 / 2;    // speed of sound ~ 0.034 cm/µs

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  delay(500);
}
