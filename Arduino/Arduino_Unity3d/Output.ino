void printdata(void)
{   
      #if PRINT_EULER == 1
      Serial.print("a ");
      Serial.print(ToDeg(roll));
      Serial.print(" ");
      Serial.print(ToDeg(pitch));
      Serial.print(" ");
      Serial.println(ToDeg(yaw));
      Serial.flush();
      #endif      
      /*
      #if PRINT_ANALOGS==1
      Serial.print(",AN:");
      Serial.print(AN[0]);  //(int)read_adc(0)
      Serial.print(",");
      Serial.print(AN[1]);
      Serial.print(",");
      Serial.print(AN[2]);  
      Serial.print(",");
      Serial.print(AN[3]);
      Serial.print (",");
      Serial.print(AN[4]);
      Serial.print (",");
      Serial.print(AN[5]);
      Serial.print(",");
      Serial.print(c_magnetom_x);
      Serial.print (",");
      Serial.print(c_magnetom_y);
      Serial.print (",");
      Serial.print(c_magnetom_z);
      #endif
      #if PRINT_DCM == 1
      Serial.print (",DCM:");
      Serial.print(DCM_Matrix[0][0]);
      Serial.print (",");
      Serial.print(DCM_Matrix[0][1]);
      Serial.print (",");
      Serial.print(DCM_Matrix[0][2]);
      Serial.print (",");
      Serial.print(DCM_Matrix[1][0]);
      Serial.print (",");
      Serial.print(DCM_Matrix[1][1]);
      Serial.print (",");
      Serial.print(DCM_Matrix[1][2]);
      Serial.print (",");
      Serial.print(DCM_Matrix[2][0]);
      Serial.print (",");
      Serial.print(DCM_Matrix[2][1]);
      Serial.print (",");
      Serial.print(DCM_Matrix[2][2]);
      #endif
      */
}

/*long convert_to_dec(float x)
{
  return x*10000000;
}*/

