void threadSerial(){
  
  sysTime = millis() / 1000;
  lcd.setCursor(0, 1);
  lcd.print(sysTime);
  lcd.setCursor(5, 1);
  lcd.print(incomingByte);

  if (Serial2.available()>0) {
    
    incomingByte = Serial2.read();
    Serial.print("Serial Input: ");
    Serial.println(incomingByte);
    
    //Input = 1   -- Display Hello World
    if (incomingByte == 49){    
        lcd.clear();
        lcd.setCursor(0, 0);
        char toPrint[16] = "Hello World!";
        inputTime = millis();
        lcd.print(toPrint);
    }

    //Input = a   -- Right Servo Micro Motor
    else if (incomingByte == 97){
        lcd.clear();
        lcd.setCursor(0, 0);
        char toPrint[16] = "Micro Servo R";
        lcd.print(toPrint);
        toggleServoMicro1();
        inputTime = millis();
    }
    
    //Input = b   -- Left Servo Micro Motor
    else if (incomingByte == 98){
        lcd.clear();
        lcd.setCursor(0, 0);
        char toPrint[16] = "Micro Servo L";
        lcd.print(toPrint);
        toggleServoMicro2();
        inputTime = millis();
    }

    //Input = c   -- Temperature Sensor
    else if (incomingByte == 99 || (is_LCD_autoDisplay && (sysTime % 20 < 10))){
        getTemperature();
        lcd.clear();
        lcd.setCursor(0, 0);
        if (currentTemp > -10){
          char toPrint[16] = "Temp";
          lcd.print(toPrint);
          lcd.setCursor(6, 0);
          
          lcd.print(currentTemp);
          lcd.setCursor(13, 0);
          char toPrint2[16] = "deg";
          lcd.print(toPrint2);
        }
        else{
          char toPrint[16] = "TS not detected";
          lcd.print(toPrint);
        }
        inputTime = millis();
    }

    //Input = d   -- Ultrasonic sensor
    else if (incomingByte == 100 || (is_LCD_autoDisplay && (sysTime % 20 < 5))){
        lcd.clear();
        lcd.setCursor(0, 0);
        getUltrasonicSensorDistance();
        if (USS_distance >= 200 || USS_distance <= 0){
          Serial.println("Out of range");
          char toPrint[16] = "Dis";
          lcd.print("USS not detected");
        }
        else {
          Serial.print(USS_distance);
          Serial.println(" cm");
          char toPrint[16] = "Dis";
          lcd.print(toPrint);
          lcd.setCursor(4, 0);
          lcd.print(USS_distance);
          lcd.setCursor(9, 0);
          char toPrint2[16] = "cm";
          lcd.print(toPrint2);
        }
        inputTime = millis();
    }

    // Input = e   -- Light Sensor
    else if (incomingByte == 101 || (is_LCD_autoDisplay && (sysTime % 20 < 5))){
        lcd.clear();
        lcd.setCursor(0, 0);
        
        char toPrint[16] = "LS not detected";
        lcd.print(toPrint);
        
        inputTime = millis();
    }

    // Input = f   -- Motion Sensor
    else if (incomingByte == 102 || (is_LCD_autoDisplay && (sysTime % 20 < 5))){
        lcd.clear();
        lcd.setCursor(0, 0);
        char toPrint[16] = "MS not detected";
        lcd.print(toPrint);
        
        inputTime = millis();
       
    }
    // Input = g   -- IMU Sensor
    else if (incomingByte == 103 || (is_LCD_autoDisplay && (sysTime % 20 < 5))){
        lcd.clear();
        lcd.setCursor(0, 0);
        printIMUoutput();
        char toPrint1[16] = "IMU R:";
        lcd.print(toPrint1);
        lcd.setCursor(6, 0);
        lcd.print(IMUroll);
        lcd.setCursor(0, 1);
        char toPrint2[16] = "P:";
        lcd.print(toPrint2);
        lcd.setCursor(2, 1);
        lcd.print(IMUpitch);
        char toPrint3[16] = " H:";
        lcd.setCursor(7, 1);
        lcd.print(toPrint3);
        lcd.setCursor(10, 1);
        lcd.print(IMUheading);
        inputTime = millis();
    }
    
    // Input = h   -- LED red
    else if (incomingByte == 104){
        lcd.clear();
        lcd.setCursor(0, 0);
        LEDmode = "Danger";
        char toPrint[16] = "LED Red";
        lcd.print(toPrint);
        inputTime = millis();
    }

    // Input = i   -- LED yellow
    else if (incomingByte == 105){
        lcd.clear();
        lcd.setCursor(0, 0);
        LEDmode = "Warning";
        char toPrint[16] = "LED Yellow";
        lcd.print(toPrint);
        inputTime = millis();
    }   
    
    // Input = j   -- LED green
    else if (incomingByte == 106){
        lcd.clear();
        lcd.setCursor(0, 0);
        LEDmode = "Safe";
        char toPrint[16] = "LED Green";
        lcd.print(toPrint);
        inputTime = millis();
    }
    
    // Input = k   -- Camera Servo R_+
    else if (incomingByte == 107){
        lcd.clear();
        lcd.setCursor(0, 0);
        char toPrint[16] = "Cam Servo R+";
        lcd.print(toPrint);
        toggleServoCameraIncrease1();
        inputTime = millis();
    }

    //Input = l   -- Camera Servo R_-
    else if (incomingByte == 108){
        lcd.clear();
        lcd.setCursor(0, 0);
        char toPrint[16] = "Cam Servo R-";
        lcd.print(toPrint);
        toggleServoCameraDecrease1();
        inputTime = millis();
    }
    //Input = m   -- Camera Servo L_+
    else if (incomingByte == 109){
        lcd.clear();
        lcd.setCursor(0, 0);
        char toPrint[16] = "Cam Servo L+";
        lcd.print(toPrint);
        toggleServoCameraIncrease2();
        inputTime = millis();
    }
    //Input = n   -- Camera Servo L_-
    else if (incomingByte == 110){
        lcd.clear();
        lcd.setCursor(0, 0);
        char toPrint[16] = "Cam Servo L-";
        lcd.print(toPrint);
        toggleServoCameraDecrease2();
        inputTime = millis();
    }
    /*
    //Input = o   -- Camera TakeSnapshot
    else if (incomingByte == 111){
        lcd.clear();
        lcd.setCursor(0, 0);
        char toPrint[16] = "Take Sanpshot";
        lcd.print(toPrint);
        cameraTakePicture();
        inputTime = millis();
    }
*/
    //Input = p   -- Mciro Servo L_+
    else if (incomingByte == 112){
        lcd.clear();
        lcd.setCursor(0, 0);
        char toPrint[16] = "Mic Servo L+";
        lcd.print(toPrint);
        toggleServoMicroIncrease1();
        inputTime = millis();
    }
    //Input = q   -- Mciro Servo L_-
    else if (incomingByte == 113){
        lcd.clear();
        lcd.setCursor(0, 0);
        char toPrint[16] = "Mic Servo L-";
        lcd.print(toPrint);
        toggleServoMicroDecrease1();
        inputTime = millis();
    }
    //Input = r   -- Mciro Servo R_+
    else if (incomingByte == 114){
        lcd.clear();
        lcd.setCursor(0, 0);
        char toPrint[16] = "Mic Servo R+";
        lcd.print(toPrint);
        toggleServoMicroIncrease2();
        inputTime = millis();
    }
    //Input = s   -- Mciro Servo R_-
    else if (incomingByte == 115){
        lcd.clear();
        lcd.setCursor(0, 0);
        char toPrint[16] = "Mic Servo R-";
        lcd.print(toPrint);
        toggleServoMicroDecrease2();
        inputTime = millis();
    }
    
  }else if (millis() - inputTime > 2000){
      lcd.clear();
      lcd.setCursor(0, 0);
      char toPrint[16] = "CubeSat System";
      inputTime = millis();
      lcd.print(toPrint);

      getTemperature();
      getUltrasonicSensorDistance();
      SerialSend();
  }
}

void SerialSend(){
  int cubeSatVoltage = 0;
  
  Serial2.println(  "sysT," + String(sysTime)         
                  + ",sysV," + String(cubeSatVoltage) 
                  + ",imuR," + String(IMUroll)
                  + ",imuP," + String(IMUpitch)
                  + ",imuH," + String(IMUheading) 
                  + ",temp," + String(currentTemp)    
                  + ",ussD," + String(USS_distance)   
                  + ",cs_S," + String(ServoCamera1pos) 
                  + ",cs_E," + String(ServoCamera2pos)                
                  + ",jpgN," + String(filename)        
                  + ",jpgS," + String(jpglen)    
                  + ",tfrT," + String(elapsedTime));
  
  Serial.println("message Sent");
}

