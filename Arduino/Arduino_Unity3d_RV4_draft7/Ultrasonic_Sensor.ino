void getUltrasonicSensorDistance(){
  digitalWrite(Pin_USS_Trig, LOW);  
  delayMicroseconds(2);
  digitalWrite(Pin_USS_Trig, HIGH);
  delayMicroseconds(10); 
  digitalWrite(Pin_USS_Trig, LOW);
  USS_duration = pulseIn(Pin_USS_Echo, HIGH);
  USS_distance = (USS_duration/2) / 29.1;
}
