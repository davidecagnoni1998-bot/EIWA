void ADU_management()
 {
 ADU_SS.listen(); // This instruction is to ensure the ucontroller to listen ONLY to this port, ignoring all the data coming from the others 

 // Once the updated data from the unit has been received, there is no need for the function to run further so the loop is interrupted to save 
 // time for the execution of the other tasks
 if(ADU_SS.available()) 
  {
  ADU_get_serial_data();
 
  if(ADU_debug_out == HIGH)
   {
   Serial.print("ADU IAS [m/s]:  ");
   Serial.println(ADU_data_str_rec.ADU_ias);
   Serial.print("ADU barometric height [m]:  ");
   Serial.println(ADU_data_str_rec.ADU_bar_height);
   Serial.print("ADU static temp. [°K]:  ");
   Serial.println(ADU_data_str_rec.ADU_sta_temp);
   }
  task_3_stop = HIGH; // The task can be stopped
  }   
 }
