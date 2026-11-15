#define Measure 1
int URECHO = 3;
int URTRIG = 5;
int sensorPin = A0;
int sensorValue = 0;

unsigned int DistanceMeasured = 0;

void setup() {
  Serial.begin(115200);
  pinMode(URTRIG, OUTPUT); 
  digitalWrite(URTRIG, HIGH);
  pinMode(URECHO, INPUT);
  
  delay(500);
//  Serial.println("Init the sensor");
}

void loop() {
  PWM_Mode();
  delay(100);
}

void PWM_Mode() {
  //Serial.print("Distance Measured=");
  digitalWrite(URTRIG, LOW);
  delayMicroseconds(50);
  digitalWrite(URTRIG, HIGH);

  if (Measure) {
    unsigned long LowLevelTime = pulseIn(URECHO, LOW, 30000);
    if (LowLevelTime == 0 || LowLevelTime >= 4500) {
      Serial.println(-1.0);
    }
    else {
      DistanceMeasured = LowLevelTime / 50;
      //Serial.print(DistanceMeasured);
      //Serial.println("cm");
    }
  } 
  else { 
    sensorValue = analogRead(sensorPin);
    sensorValue = sensorValue * 0.718;
    //Serial.print(sensorValue);
    //Serial.println("cm");
  }
}