void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(24,INPUT);
  digitalWrite(24,LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
    if(digitalRead(24)==HIGH)  {
      Serial.println("Somebody is here.");
    }
    else  {
      Serial.println("Nobody.");
    }
    delay(1000);
}
