void ALU_management()
 {
 ALU_SS.listen(); // This instruction is to ensure the ucontroller to listen ONLY to this port, ignoring all the data coming from the others 
 
 // Once the updated data from the unit has been received, there is no need for the function to run further so the loop is interrupted to save 
 // time for the execution of the other tasks
 if(ALU_SS.available()) 
  {
  ALU_get_serial_data();
  
  if(ALU_debug_out == HIGH)
   {
   Serial.print("GNSS fix:  ");
   Serial.println(ALU_data_str_rec.GNSS_fix);
   Serial.print("GNSS altitude [m]:  ");
   Serial.println(ALU_data_str_rec.GNSS_alt);
   Serial.print("GNSS latitude [deg]:  ");
   Serial.println(ALU_data_str_rec.GNSS_lat, 6);
   Serial.print("GNSS longitude [deg]:  ");
   Serial.println(ALU_data_str_rec.GNSS_lon, 6);
   Serial.print("GNSS speed [m/s]:  ");
   Serial.println(ALU_data_str_rec.GNSS_speed);

   Serial.print("IMU pitch angle [deg]:  ");
   Serial.println(ALU_data_str_rec.IMU_pitch);
   Serial.print("IMU pitch angle [deg]:  ");
   Serial.println(ALU_data_str_rec.IMU_roll);
   Serial.print("IMU load factor [g]:  ");
   Serial.println(ALU_data_str_rec.IMU_LF); 
   }
  task_6_stop = HIGH; // The task can be stopped 
  }    
 }
