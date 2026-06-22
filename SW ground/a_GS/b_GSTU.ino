// The GSTU module is designed to be handled as a normal UART port
void GSTU_management_RX()
 { 
 const byte FAP_ID = 1; // Flight Attitude Panel
 const byte ESP_ID = 2; // Electric System Panel
 const byte EP_ID = 3;  // Engine Panel
 const byte LP_ID = 4;  // Location Panel
 
 //GSTU_SS.listen();
 if(GSTU_SS.available())
  {
  GSTU_get_serial_data(); 

  // Check sum algorythm
  // To prevent the saturation of the MatLab buffer sending continuously the same data this algorythm is used
  // It sums all the data of the received struct (a part for the "identifier") and send the corrisponding string to the MatLab interface ONLY of the sum
  // of all the data contained in a specific panel structure is different from the last one received, meaning that a change in the string data has occoured
  StructSum();
  switch(GSTU_RX_data_str_rec.identifier)
   {
   case FAP_ID:
    if(GSTU_str_sum != GSTU_last_str_sum_FAP)
     {
     GSTU_last_str_sum_FAP = GSTU_str_sum;
     MIDS();
     }
   break; 

   case ESP_ID:
    if(GSTU_str_sum != GSTU_last_str_sum_ESP)
     {
     GSTU_last_str_sum_ESP = GSTU_str_sum;
     MIDS();
     }
   break; 

   case EP_ID:
    if(GSTU_str_sum != GSTU_last_str_sum_EP)
     {
     GSTU_last_str_sum_EP = GSTU_str_sum;
     MIDS();
     }
   break; 

   case LP_ID:
    if(GSTU_str_sum != GSTU_last_str_sum_LP)
     {
     GSTU_last_str_sum_LP = GSTU_str_sum;
     MIDS();
     }
   break; 

   default:
   break; 
   }
  
  if(GTSU_debug_out == HIGH)
   {
   Serial.print("Identifier:  ");
   Serial.println(GSTU_RX_data_str_rec.identifier);
   Serial.print("ui16_data_1:  ");
   Serial.println(GSTU_RX_data_str_rec.f_data_1);
   Serial.print("ui16_data_2:  ");
   Serial.println(GSTU_RX_data_str_rec.f_data_2);
   Serial.print("ui16_data_3:  ");
   Serial.println(GSTU_RX_data_str_rec.f_data_3);
   Serial.print("ui16_data_4:  ");
   Serial.println(GSTU_RX_data_str_rec.f_data_4);
   Serial.print("ui16_data_5:  ");
   Serial.println(GSTU_RX_data_str_rec.f_data_5);
   Serial.print("ui16_data_6:  ");
   Serial.println(GSTU_RX_data_str_rec.f_data_6);
   Serial.print("ui16_data_7:  ");
   Serial.println(GSTU_RX_data_str_rec.f_data_7);
   Serial.print("ui16_data_8:  ");
   Serial.println(GSTU_RX_data_str_rec.f_data_8);
   Serial.print("ui16_data_9:  ");
   Serial.println(GSTU_RX_data_str_rec.f_data_9);
   } 
  }
 //task_1_stop = HIGH; // The task can be stopped 
 }

void StructSum()
 {
 GSTU_str_sum = GSTU_RX_data_str_rec.f_data_1 + GSTU_RX_data_str_rec.f_data_2 + GSTU_RX_data_str_rec.f_data_3 + GSTU_RX_data_str_rec.f_data_4 + GSTU_RX_data_str_rec.f_data_5 + GSTU_RX_data_str_rec.f_data_6 + GSTU_RX_data_str_rec.f_data_7 + GSTU_RX_data_str_rec.f_data_8 + GSTU_RX_data_str_rec.f_data_9; 
 }
