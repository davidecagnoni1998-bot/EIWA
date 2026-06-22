void EPU_management()
 {
 EPU_SS.listen(); // This instruction is to ensure the ucontroller to listen ONLY to this port, ignoring all the data coming from the others
 
 // Once the updated data from the unit has been received, there is no need for the function to run further so the loop is interrupted to save 
 // time for the execution of the other tasks
 if(EPU_SS.available()) 
  {
  EPU_get_serial_data();   
  
  if(EPU_debug_out == HIGH)
   {
   Serial.print("EPU RPM:  ");
   Serial.println(EPU_data_str_rec.EPU_rpm);
   Serial.print("EPU temp. [°C]:  ");
   Serial.println(EPU_data_str_rec.EPU_temp);
   Serial.print("EPU battery volt. [V*100]:  ");
   Serial.println(EPU_data_str_rec.EPU_batt_volt);
   Serial.print("Engine is ON:  ");
   Serial.println(EPU_data_str_rec.EPU_eng_ON);
   }
  task_4_stop = HIGH; // The task can be stopped
  }
 }
