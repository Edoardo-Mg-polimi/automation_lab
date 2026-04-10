void setup() {
  Serial.begin (115200);
  pinMode(11, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(2, INPUT_PULLUP);
  digitalWrite (11, LOW);
  digitalWrite (8, LOW);
}

void loop() {
  for (int thisPin = 0; thisPin < 100; thisPin++) {
    Serial.println ();
  }

  Serial.println ("LEVITRON SELF TEST.");
  Serial.println("\n\n");
  delay(1000);              // wait for a second
  Serial.print ("PHOTORESISTOR:IR LED LOW:\t");
  Serial.println (analogRead(1)-50);
  digitalWrite (8, HIGH);
  delay(100);
  Serial.print ("PHOTORESISTOR:IR LED HIGH:\t");
  Serial.println (analogRead(1));
  digitalWrite (8, LOW);
  delay(100);
  Serial.print ("SWITCH 1:\t\t\t");
  Serial.println (digitalRead(2));
  Serial.println ("LEVITRON SELF TEST COMPLETE");
  int count = 0;
  while (count < 6 && millis() > 120000) {
    count = count + 1;
    digitalWrite (8, HIGH);
    delay (100);
    digitalWrite (8, LOW);
    delay (300);
  }
  delay (4000);

}
