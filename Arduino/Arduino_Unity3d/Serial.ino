void threadSerial(){
  
  int sysTime = millis() / 1000;
  lcd.setCursor(0, 1);
  lcd.print(sysTime);

  if (Serial.available()>0) {
    
  int sysTime = millis() / 1000;
  lcd.setCursor(0, 1);
  lcd.print(sysTime);
  lcd.setCursor(5,1 );
  lcd.print(incomingByte);
    
    //Input = 1
    if (incomingByte == 47){    
        lcd.clear();
        lcd.setCursor(0, 0);
        char toPrint[16] = "Hello World!";
        inputTime = millis();
        lcd.print(toPrint);
    }

    //Input = a
    else if (incomingByte == 97){
        lcd.clear();
        lcd.setCursor(0, 0);
        char toPrint[16] = "Right Arm ON";
        inputTime = millis();
        lcd.print(toPrint);
    }
    
    //Input = b
    else if (incomingByte == 98){
        lcd.clear();
        lcd.setCursor(0, 0);
        char toPrint[16] = "Left Arm ON";
        inputTime = millis();
        lcd.print(toPrint);
    }

    //Input = c
    else if (incomingByte == 101 || (AutoDisplay && (sysTime % 20 < 10))){
        lcd.clear();
        lcd.setCursor(0, 0);
        Serial.print(" Requesting temperatures..."); 
        sensors.requestTemperatures(); // Send the command to get temperature readings 
        Serial.println("DONE"); 

        Serial.print("Temperature is: "); 
        Serial.println(sensors.getTempCByIndex(0)); 
        char toPrint[16] = "Temp";
        lcd.print(toPrint);
        lcd.setCursor(6, 0);
        inputTime = millis();
        lcd.print(sensors.getTempCByIndex(0));
        lcd.setCursor(13, 0);
        char toPrint2[16] = "deg";
        lcd.print(toPrint2);
    }

    //Input = d
    else if (incomingByte == 100 || (AutoDisplay && (sysTime % 20 < 5))){
        lcd.clear();
        lcd.setCursor(0, 0);
        long duration, distance;
        digitalWrite(Pin_USS_Trig, LOW);  
        delayMicroseconds(2); 
        digitalWrite(Pin_USS_Trig, HIGH);
        delayMicroseconds(10); 
        digitalWrite(Pin_USS_Trig, LOW);
        duration = pulseIn(Pin_USS_Echo, HIGH);
        distance = (duration/2) / 29.1;
        if (distance >= 200 || distance <= 0){
          Serial.println("Out of range");
        }
        else {
          Serial.print(distance);
          Serial.println(" cm");
        }
        
        char toPrint[16] = "Dis";
        lcd.print(toPrint);
        lcd.setCursor(4, 0);
        lcd.print(distance);
        lcd.setCursor(9, 0);
        char toPrint2[16] = "cm";
        lcd.print(toPrint2);
        inputTime = millis();
    }

    // Input = e
    else if (incomingByte == 101 || (AutoDisplay && (sysTime % 20 < 5))){
        lcd.clear();
        lcd.setCursor(0, 0);
        
        char toPrint[16] = "Light Sensor";
        lcd.print(toPrint);
        inputTime = millis();
    }

    // Input = f
    else if (incomingByte == 102 || (AutoDisplay && (sysTime % 20 < 5))){
        lcd.clear();
        lcd.setCursor(0, 0);
        
        char toPrint[16] = "IMU Sensor";
        lcd.print(toPrint);
        inputTime = millis();
    }
    
    // Input = g
    else if (incomingByte == 103){
        lcd.clear();
        lcd.setCursor(0, 0);
        
        char toPrint[16] = "LED Red";
        lcd.print(toPrint);
        inputTime = millis();
    }

    // Input = h
    else if (incomingByte == 104){
        lcd.clear();
        lcd.setCursor(0, 0);
        
        char toPrint[16] = "LED Green";
        lcd.print(toPrint);
        inputTime = millis();
    }

    // Input = i
    else if (incomingByte == 105){
        lcd.clear();
        lcd.setCursor(0, 0);
        
        char toPrint[16] = "LED Yellow";
        lcd.print(toPrint);
        inputTime = millis();
    }


    
  }else if (millis() - inputTime > 2000){
      lcd.clear();
      lcd.setCursor(0, 0);
      char toPrint[16] = "CubeSat System";
      inputTime = millis();
      lcd.print(toPrint);
  }



  if (millis() - LEDTime > 50){
    LEDcolorWipe(strip.Color(255, 0, 0), LEDcounter);
    LEDcounter = (LEDcounter >= strip.numPixels()-1)? 0:LEDcounter+1;
    LEDTime = millis();
    //Serial.print(LEDcounter);
  }
   if (millis() - LEDTime > 100){
    LEDcolorWipe(strip.Color(255, 0, 0), LEDcounter);
    LEDcounter = (LEDcounter >= strip.numPixels()-1)? 0:LEDcounter+1;
    LEDTime = millis();
    //Serial.print(LEDcounter);
  }
   if (millis() - LEDTime > 150){
    LEDcolorWipe(strip.Color(255, 0, 0), LEDcounter);
    LEDcounter = (LEDcounter >= strip.numPixels()-1)? 0:LEDcounter+1;
    LEDTime = millis();
    //Serial.print(LEDcounter);
  }

      
    /*
    int inByte = Serial.read();
    Serial.println(inByte);
    // do something different depending on the character received.
    // The switch statement expects single number values for each case;
    // in this exmaple, though, you're using single quotes to tell
    // the controller to get the ASCII value for the character.  For
    // example 'a' = 97, 'b' = 98, and so forth:
    switch (inByte) {
      case 'a':
        switchLED = !switchLED;
        switchServoRight = !switchServoRight;
        break;
      case 'b':
        switchServoLeft = !switchServoLeft;
        break;
      case 'c':
      break;
    }
  }*/
}
