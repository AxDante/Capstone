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
void toggleServoCamera1(){
  if (isServoCamera1){
    for (pos = 10; pos <= 170; pos += 1) { 
    ServoCamera1.write(pos);              
    }
  }else{
    for (pos = 170; pos >= 10; pos -= 1) { 
    ServoCamera1.write(pos);              
    }
  }
  delay(15);
  isServoCamera1 = !isServoCamera1;
}
void toggleServoCamera2(){
  if (isServoCamera2){
    for (pos = 10; pos <= 170; pos += 1) { 
    ServoCamera2.write(pos);              
    }
  }else{
    for (pos = 170; pos >= 10; pos -= 1) { 
    ServoCamera2.write(pos);              
    }
  }
  delay(15);
  isServoCamera2 = !isServoCamera2;
}
