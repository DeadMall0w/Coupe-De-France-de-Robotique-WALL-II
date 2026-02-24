void setup() {
  pinMode(13, OUTPUT);
}

void loop() {
  digitalWrite(13, HIGH); // Allume la LED
  delay(1000);            // Attend 1 seconde
  digitalWrite(13, LOW);  // Éteint la LED
  delay(1000);            // Attend 1 seconde
}