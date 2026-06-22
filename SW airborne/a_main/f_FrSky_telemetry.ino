void FrSky_telemetry()
 {
 telem_EPU_batt_volt.value = EPU_data_str_rec.EPU_batt_volt;
 telem_flight_time.value = FT_ms;  
 telem_ADU_ias.value = ADU_data_str_rec.ADU_ias;
 telem_EPU_temp.value = EPU_data_str_rec.EPU_temp;
 telem_EPU_rpm.value = EPU_data_str_rec.EPU_rpm;
 telem_GNSS_altitude.value = ALU_data_str_rec.GNSS_alt;
 S_port.handle(); 

 //task_1_stop = HIGH; // The task can be stopped 
 }
