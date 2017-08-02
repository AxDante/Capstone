void toggleServoMicro1(){
  ServoMicro1.attach(Pin_Servo_Micro_1);
  if (isServoMicro1){
    for (pos = 30; pos <= 180; pos += 10) { 
    ServoMicro1.write(pos);
    delay(30);  
    }
  }else{
    for (pos = 180; pos >= 30; pos -= 10) { 
    ServoMicro1.write(pos);
    delay(30);               
    }
  }
  delay(100); 
  ServoMicro1.detach();
  isServoMicro1 = !isServoMicro1;
}

void toggleServoMicro2(){
  ServoMicro2.attach(Pin_Servo_Micro_2);
  if (isServoMicro2){
    for (pos = 60; pos <= 180; pos += 10) { 
    ServoMicro2.write(pos);
    delay(30);   
    }
  }else{
    for (pos = 180; pos >= 60; pos -= 10) { 
    ServoMicro2.write(pos);
    delay(30);              
    }
  }
  delay(100);
  ServoMicro2.detach();
  isServoMicro2 = !isServoMicro2;
}

void toggleServoMicroIncrease1(){
  if (ServoMicro1pos < 180){
    ServoMicro1pos += 30;
    ServoMicro1.attach(Pin_Servo_Micro_1);
    ServoMicro1.write(ServoMicro1pos);
    Serial.print(ServoMicro1pos);
  }
  delay(50);
  ServoMicro1.detach();
}
void toggleServoMicroDecrease1(){
  if (ServoMicro1pos > 0){
    ServoMicro1pos -= 30;
    ServoMicro1.attach(Pin_Servo_Micro_1);
    ServoMicro1.write(ServoMicro1pos);
    Serial.print(ServoMicro1pos);
  }
  delay(10);
  ServoMicro1.detach();
}
void toggleServoMicroIncrease2(){
  if (ServoMicro2pos < 180){
    ServoMicro2pos += 30;
    ServoMicro2.attach(Pin_Servo_Micro_2);
    ServoMicro2.write(ServoMicro2pos);
    Serial.print(ServoMicro2pos);
  }
  delay(50);
  ServoMicro2.detach();
}
void toggleServoMicroDecrease2(){
  if (ServoMicro2pos > 0){
    ServoMicro2pos -= 30;
    ServoMicro2.attach(Pin_Servo_Micro_2);
    ServoMicro2.write(ServoMicro2pos);
    Serial.print(ServoMicro2pos);
  }
  delay(50);
  ServoMicro2.detach();
}
void toggleServoCameraIncrease1(){
  if (ServoCamera1pos < 180){
    ServoCamera1pos += 10;
    ServoCamera1.attach(Pin_Servo_Camera_1);
    ServoCamera1.write(ServoCamera1pos);
    Serial.print(ServoCamera1pos);
  }
  delay(50);
  ServoCamera1.detach();
}

void toggleServoCameraDecrease1(){
  if (ServoCamera1pos > 0){
    ServoCamera1pos -= 10;
    ServoCamera1.attach(Pin_Servo_Camera_1);
    ServoCamera1.write(ServoCamera1pos);
    Serial.print(ServoCamera1pos);
  }
  delay(50);
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
