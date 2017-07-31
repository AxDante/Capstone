void toggleServoMicro1(){
  if (isServoMicro1){
    for (pos = 10; pos <= 170; pos += 1) { 
    ServoMicro1.write(pos);  
    }
  }else{
    for (pos = 170; pos >= 10; pos -= 1) { 
    ServoMicro1.write(pos);              
    }
  }
  delay(15); 
  isServoMicro1 = !isServoMicro1;
}

void toggleServoMicro2(){
  if (isServoMicro2){
    for (pos = 10; pos <= 170; pos += 1) { 
    ServoMicro2.write(pos);              
    }
  }else{
    for (pos = 170; pos >= 10; pos -= 1) { 
    ServoMicro2.write(pos);              
    }
  }
  delay(15);
  isServoMicro2 = !isServoMicro2;
}
void toggleServoCameraIncrease1(){
  if (ServoCamera1pos < 180){
    ServoCamera1pos += 10;
    ServoCamera1.attach(Pin_Servo_Camera_1);
    ServoCamera1.write(ServoCamera1pos);
    Serial.print(ServoCamera1pos);
  }
  delay(100);
  ServoCamera1.detach();
}

void toggleServoCameraDecrease1(){
  if (ServoCamera1pos > 0){
    ServoCamera1pos -= 10;
    ServoCamera1.attach(Pin_Servo_Camera_1);
    ServoCamera1.write(ServoCamera1pos);
    Serial.print(ServoCamera1pos);
  }
  delay(100);
  ServoCamera1.detach();
}

void toggleServoCameraIncrease2(){
  if (ServoCamera2pos < 160){
    ServoCamera2pos += 10;
    ServoCamera2.attach(Pin_Servo_Camera_2);
    ServoCamera2.write(ServoCamera2pos);
    Serial.print(ServoCamera2pos);
  }
  delay(100);
  ServoCamera2.detach();
}

void toggleServoCameraDecrease2(){
  if (ServoCamera2pos > 60){
    ServoCamera2pos -= 10;
    ServoCamera2.attach(Pin_Servo_Camera_2);
    ServoCamera2.write(ServoCamera2pos);
    Serial.print(ServoCamera2pos);
  }
  delay(100);
  ServoCamera2.detach();
}
