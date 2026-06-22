// The MIDS (MatLab Interface Data Sender) create a text string where the data are divided 
// by a " ", the first "cell" of each string sent is the string identifier, used by MatLab to sort out each string to the 
// right cockpit panel, each string identifier correspond to single cockpit panel 

void MIDS()
 {  
 // If the struct identifier is 0, the struct has not been written correctly thus the data contained are invalid and the MatLab string must be zeroized
 if(GSTU_RX_data_str_rec.identifier != 0) 
  MatLab_string = GSTU_RX_data_str_rec.identifier + STR_DIV + GSTU_RX_data_str_rec.f_data_1 + STR_DIV + GSTU_RX_data_str_rec.f_data_2 + STR_DIV + GSTU_RX_data_str_rec.f_data_3 + STR_DIV + GSTU_RX_data_str_rec.f_data_4 + STR_DIV + GSTU_RX_data_str_rec.f_data_5 + STR_DIV + GSTU_RX_data_str_rec.f_data_6 + STR_DIV + GSTU_RX_data_str_rec.f_data_7 + STR_DIV + GSTU_RX_data_str_rec.f_data_8 + STR_DIV + GSTU_RX_data_str_rec.f_data_9;
 else
  MatLab_string = 0 + STR_DIV + 0 + STR_DIV + 0 + STR_DIV + 0 + STR_DIV + 0 + STR_DIV + 0 + STR_DIV + 0 + STR_DIV + 0 + STR_DIV + 0 + STR_DIV + 0;

 // String sending to MatLab app 
 Serial.print(MatLab_string);
 Serial.print("\n"); 
 //task_2_stop = HIGH; // The task can be stopped 
 }
 
