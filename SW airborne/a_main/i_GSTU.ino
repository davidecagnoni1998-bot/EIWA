// This function is in charge of sending to the ground station the data structure for the MatLab app,
// The structs here are prepared to suit with the current MatLab panel structure and layout.
// Each structure contains data coming from different units, in case on unavailable or completely absent units in a specifi configuration,
// the missing data in the struct cells will be zeroized ot allow the MatLab app to work with the remainig values.
// For the structs composition refer to the document "GSTU structs and MatLab string"

// The GSTU module is designed to be handled as a normal UART port, therefore the same protocol used for the other modules is applied
void GSTU_management_TX()
 {
 // MatLab app panels (strings) identifiers
 const byte FAP_ID = 1; // Flight Attitude Panel
 const byte ESP_ID = 2; // Electric System Panel
 const byte EP_ID = 3;  // Engine Panel
 const byte LP_ID = 4;  // Location Panel
 
 // Check sum algorythm
 // To prevent the saturation of the MatLab buffer sending continuously the same data this algorythm is used
 // It sums all the data of the each struct that has to be sent (a part for the "identifier") and send the corrisponding 
 // string to the GS ONLY of the sum of all the data contained in a specific panel structure is different from the last one sent,
 // meaning that a change in the string data has occoured
 
 // flight attitude panel struct composition and sending
 GSTU_TX_data_str_loc.identifier = FAP_ID; 
 GSTU_TX_data_str_loc.f_data_1 = ADU_data_str_rec.ADU_ias;
 GSTU_TX_data_str_loc.f_data_2 = ALU_data_str_rec.IMU_roll;
 GSTU_TX_data_str_loc.f_data_3 = ALU_data_str_rec.IMU_pitch;
 GSTU_TX_data_str_loc.f_data_4 = ALU_data_str_rec.GNSS_alt;
 GSTU_TX_data_str_loc.f_data_5 = 0;
 GSTU_TX_data_str_loc.f_data_6 = 0;
 GSTU_TX_data_str_loc.f_data_7 = 0;
 GSTU_TX_data_str_loc.f_data_8 = 0;
 GSTU_TX_data_str_loc.f_data_9 = 0;
 if(GSTU_str_sum != GSTU_last_str_sum_FAP)
  {
  GSTU_last_str_sum_EP = GSTU_str_sum;
  GSTU_send_serial_data();
  }

 // electric system panel struct composition and sending
 GSTU_TX_data_str_loc.identifier = ESP_ID; 
 GSTU_TX_data_str_loc.f_data_1 = EPU_data_str_rec.EPU_batt_volt;
 GSTU_TX_data_str_loc.f_data_2 = 0;
 GSTU_TX_data_str_loc.f_data_3 = 0;
 GSTU_TX_data_str_loc.f_data_4 = 0;
 GSTU_TX_data_str_loc.f_data_5 = 0;
 GSTU_TX_data_str_loc.f_data_6 = 0;
 GSTU_TX_data_str_loc.f_data_7 = 0;
 GSTU_TX_data_str_loc.f_data_8 = 0;
 GSTU_TX_data_str_loc.f_data_9 = 0;
 StructSum();
 if(GSTU_str_sum != GSTU_last_str_sum_ESP)
  {
  GSTU_last_str_sum_ESP = GSTU_str_sum;
  GSTU_send_serial_data();
  }

 // engine panel struct composition and sending
 GSTU_TX_data_str_loc.identifier = EP_ID; 
 GSTU_TX_data_str_loc.f_data_1 = EPU_data_str_rec.EPU_rpm;
 GSTU_TX_data_str_loc.f_data_2 = EPU_data_str_rec.EPU_temp;
 GSTU_TX_data_str_loc.f_data_3 = 0;
 GSTU_TX_data_str_loc.f_data_4 = 0;
 GSTU_TX_data_str_loc.f_data_5 = 0;
 GSTU_TX_data_str_loc.f_data_6 = 0;
 GSTU_TX_data_str_loc.f_data_7 = 0;
 GSTU_TX_data_str_loc.f_data_8 = 0;
 GSTU_TX_data_str_loc.f_data_9 = 0;
 StructSum();
 if(GSTU_str_sum != GSTU_last_str_sum_EP)
  {
  GSTU_last_str_sum_EP = GSTU_str_sum;
  GSTU_send_serial_data();
  }

 // location panel struct composition and sending
 GSTU_TX_data_str_loc.identifier = LP_ID; 
 GSTU_TX_data_str_loc.f_data_1 = ALU_data_str_rec.GNSS_lat;
 GSTU_TX_data_str_loc.f_data_2 = ALU_data_str_rec.GNSS_lon;
 GSTU_TX_data_str_loc.f_data_3 = ALU_data_str_rec.GNSS_alt;
 GSTU_TX_data_str_loc.f_data_4 = ALU_data_str_rec.GNSS_speed;
 GSTU_TX_data_str_loc.f_data_5 = 0;
 GSTU_TX_data_str_loc.f_data_6 = 0;
 GSTU_TX_data_str_loc.f_data_7 = 0;
 GSTU_TX_data_str_loc.f_data_8 = 0;
 GSTU_TX_data_str_loc.f_data_9 = 0;
 StructSum();
 if(GSTU_str_sum != GSTU_last_str_sum_LP)
  {
  GSTU_last_str_sum_LP = GSTU_str_sum;
  GSTU_send_serial_data();
  }
  
 task_7_stop = HIGH; // The task can be stopped
 }

void StructSum()
 {
 GSTU_str_sum = GSTU_TX_data_str_loc.f_data_1 + GSTU_TX_data_str_loc.f_data_2 + GSTU_TX_data_str_loc.f_data_3 + GSTU_TX_data_str_loc.f_data_4 + GSTU_TX_data_str_loc.f_data_5 + GSTU_TX_data_str_loc.f_data_6 + GSTU_TX_data_str_loc.f_data_7 + GSTU_TX_data_str_loc.f_data_8 + GSTU_TX_data_str_loc.f_data_9; 
 }
