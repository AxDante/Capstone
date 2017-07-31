void getTemperature(){
  Serial.print(" Requesting temperatures..."); 
  sensors.requestTemperatures(); // Send the command to get temperature readings 
  Serial.println("DONE"); 

  Serial.print("Temperature is: "); 
  Serial.println(sensors.getTempCByIndex(0)); 
  currentTemp = sensors.getTempCByIndex(0);
}

