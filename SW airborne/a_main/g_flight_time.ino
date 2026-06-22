void flight_time()
 {
 const int ARMING_DET_THESHOLD = 1500;        // Over this value in [ms] the SW interprets the arming signal as HIGH
 if(sys_debug_out == HIGH)
  {
  Serial.println("flight time func. debug data:");
  Serial.print("FT_s:   ");
  Serial.println(FT_s);
  Serial.print("FT_m:   ");
  Serial.println(FT_m);
  Serial.print("FT_ms:   ");
  Serial.println(FT_ms); 
  Serial.print("Arming sig. ms:   ");
  Serial.println(pulseIn(ARMED_SIG_PIN, HIGH)); 
  }
  
 if(millis()-FT_last_time > 1000)
  { 
  if(pulseIn(ARMED_SIG_PIN, HIGH) > ARMING_DET_THESHOLD) // This counter increase every 5 seconds
   { 
   FT_s = FT_s + 1;
   FT_last_time = millis();
  
   if(FT_s > 59)
    {
    FT_m = FT_m + 100;
    FT_s = 0;
    }
   FT_ms = FT_m + FT_s; // This value is composed by minutes (the first two numbers) and seconds (the last two numbers), the radio telemetry will interpret it as a [mm:ss] time
   }
  } 
 task_2_stop = HIGH; // The task can be stopped 
 }
