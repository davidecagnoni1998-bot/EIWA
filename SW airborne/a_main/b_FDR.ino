// Since most of the data are recorded every second this function is design to run once every second
// A higher repetition rate is allowed but must be considered during the on ground data analysis since
// some data will have repeated values, check also the SCH external units refresh rate before increase the 
// FDR recording frequency

void FDR_management()
 {
 GNSS_data = SD.open("GNSS_data.txt", FILE_WRITE);
 GNSS_data.print(ALU_data_str_rec.GNSS_lat);
 GNSS_data.print("  ");
 GNSS_data.print(ALU_data_str_rec.GNSS_lon);
 GNSS_data.print("  ");
 GNSS_data.print(ALU_data_str_rec.GNSS_alt);
 GNSS_data.print("  ");
 GNSS_data.print(ALU_data_str_rec.GNSS_speed);
 GNSS_data.println("  ");
 GNSS_data.close();
 
 IMU_data = SD.open("IMU_data.txt", FILE_WRITE);
 IMU_data.print(ALU_data_str_rec.IMU_pitch);
 IMU_data.print("  ");
 IMU_data.print(ALU_data_str_rec.IMU_roll);
 IMU_data.print("  ");
 IMU_data.print(ALU_data_str_rec.IMU_LF);
 IMU_data.println("  ");
 IMU_data.close();

 sys_data = SD.open("sys_data.txt", FILE_WRITE);
 sys_data.print(FT_s);
 sys_data.println("  ");
 sys_data.close();

 EPU_data = SD.open("EPU_data.txt", FILE_WRITE);
 EPU_data.print(EPU_data_str_rec.EPU_rpm);
 EPU_data.print("  ");
 EPU_data.print(EPU_data_str_rec.EPU_temp);
 EPU_data.print("  ");
 EPU_data.print(EPU_data_str_rec.EPU_batt_volt);
 EPU_data.println("  ");
 EPU_data.print(EPU_data_str_rec.EPU_eng_ON);
 EPU_data.println("  ");
 EPU_data.close();

 ADU_data = SD.open("ADU_data.txt", FILE_WRITE);
 ADU_data.print(ADU_data_str_rec.ADU_ias);
 ADU_data.print("  ");
 ADU_data.print(ADU_data_str_rec.ADU_bar_height);
 ADU_data.print("  ");
 ADU_data.print(ADU_data_str_rec.ADU_sta_temp);
 ADU_data.println("  ");
 ADU_data.close();
 
 task_5_stop = HIGH; // The task can be stopped
 }
