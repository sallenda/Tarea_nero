void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0){
    String cmd = Serial.readStringUntil('\n');
  }
  Serial.println("12.24");
  delay(50);
}
