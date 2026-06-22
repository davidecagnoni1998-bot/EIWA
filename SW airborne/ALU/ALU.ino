 //SW version: ALU 1 
//Functions: Attitude and Localization Unit, data computation and sending to the SCH
//Programmer/s: Davide Cagnoni
//Last edit: 
//Note: 

//________________________________________________________________________________________________________________________________________________________________________________________________________________________________________

// Libraries 

#include <SoftwareSerial.h>
#include "Wire.h"
#include <TinyGPS++.h>
#include <MPU6050_light.h>

#include <stdlib.h>
#include "transport_PPP.h"
#include "Arduino.h"

//______________________________________________________________________Variables and constant declaration________________________________________________________________________________________

// Debug variables
const int debug_serial_boud_rate = 9600;    // Boud rate for the output serial data
const int debug_out = HIGH;                  // If set to HIGH the program will provide as serial output the debug data

// ALU variables (Attitude and Localization Unit)
typedef struct __attribute__((__packed__))
 {
 bool GNSS_fix = LOW;       // When HIGH this flag indicates that the GNSS is fixed
 uint16_t GNSS_alt = 0;     // GNSS altidude [m]
 
 // IMPORTANT NOTE! The GNSS latitude and longitude are reliable ONLY in the northen emisphere!!
 float GNSS_lat = 0;       // GNSS latidìtude [deg]
 float GNSS_lon = 0;       // GNSS longitude [deg]
 uint16_t GNSS_speed = 0;   // GNSS speed [m/s]

 float IMU_pitch = 0;   // IMU pitch angle [deg]
 float IMU_roll = 0;    // IMU roll angle [deg]                
 float IMU_LF = 0;      // Load Factor in [g]
 }

ALU_data_str;
ALU_data_str ALU_data_str_loc;  //Local data struct  
ALU_data_str ALU_data_str_rec;  //Remote (UART received) data struct

volatile uint8_t tx_buffer[sizeof(ALU_data_str) * 2];  //Structure textrure with adding the "point to point protocol"


const byte GNSS_pos_prec = 6;  // Number of digit for the GNSS latitude and longitude
bool GNSS_TO_alt_acq = LOW; // This flag is set to HIGH as once the take-off altitude has been acquired, it remains HIGH even in case of a temporary GNSS "de-fixing" and "re-fixing"

// IMU variables
float IMU_raw_accX = 0, IMU_raw_accY = 0, IMU_raw_accZ = 0;     // MPU6050 raw accelerations raw in [g]
float IMU_raw_gyroX = 0, IMU_raw_gyroY = 0, IMU_raw_gyroZ = 0;  // MPU6050 raw angular velocities in [rad/s]
float IMU_raw_pitch = 0, IMU_raw_roll = 0, IMU_raw_yaw = 0;     // MPU6050 raw attitude in [deg]
float IMU_raw_LF = 0;                                           // MPU6050 raw load factor in [g]

// Dummy scheduler, it assigns a specific time to each task (function) before switching to another one.
// If a task takes more time to be executed compared to assigned one, the next task will start whiele the last task is still running,
// Be sure to set a proper time for each function

// Assigned time for each task in [ms]
const int TASK_1_TIME = 100;
const int TASK_2_TIME = 100;
const int TASK_3_TIME = 100;

long int last_task_1_time = 0;
long int last_task_2_time = 0;
long int last_task_3_time = 0;

bool task_1_done = LOW;
bool task_2_done = LOW;
bool task_3_done = LOW;



//___________________________________________________________________________Pins numbers declaration_______________________________________________________________________________________________________________________

// Debug serial port, this is a unidirectional port
const byte DEBUG_TX_PIN = 3;

// GNSS serial port
const byte GNSS_TX_PIN = 5;
const byte GNSS_RX_PIN = 4;

// The IMU is a MPU6050 pressure sensor working with an I2C bus
const byte IMU_SDA_PIN = A4;
const byte IMU_SCL_PIN = A5;

//_______________________________________________________________________________________Objects inizialization___________________________________________________________________________

// Software Serial ports
SoftwareSerial DEBUG_SS(-1, DEBUG_TX_PIN);   // The ADU debug serial port is configured as a "transmitting only" using a "-1" to inhibit the receiving pin, 
                                              // an UART-USB converter with a terminal on PC is required for debug
                                          
SoftwareSerial GNSS_SS(GNSS_RX_PIN, GNSS_TX_PIN); 
TinyGPSPlus GNSS;
MPU6050 IMU(Wire);

//________________________________________________________________________________________Functions declaration_____________________________________________________________________________

void GNSS_management();
//________________________________________________________________________________________Secondary functions declaration_____________________________________________________________________________


//___________________________________________________________________________________________________Setup and Loop__________________________________________________________________________________________________________________________________________________

void setup() 
 {
 // Setting pins mode
 
 // Debug function initializing
 if(debug_out == HIGH)
  {
  DEBUG_SS.begin(debug_serial_boud_rate); 
  DEBUG_SS.println("Debug out active");
  }
  
 Serial.begin(9600);

 // Serial initialization for the GNSS
 GNSS_SS.begin(9600);
 
 // IMU initialization, range and filter setting
 Wire.begin();
 IMU.begin();
 IMU.calcOffsets(true,true); // Zeroizing IMU offsets
 }
 
void loop() 
 { 
 while(millis()-last_task_1_time < TASK_1_TIME || task_1_done == LOW)
  {
  if(task_1_done == LOW)
   {
   task_1_done = HIGH;
   last_task_1_time = millis();
   }
  GNSS_management();    
  }  

 while(millis()-last_task_2_time < TASK_2_TIME || task_2_done == LOW)
  {
  if(task_2_done == LOW)
   {
   task_2_done = HIGH;
   last_task_2_time = millis();
   }
  IMU_management();    
  }  

 while(millis()-last_task_3_time < TASK_3_TIME || task_3_done == LOW)
  {
  if(task_3_done == LOW)
   {
   task_3_done = HIGH;
   last_task_3_time = millis();
   }
  serial_send();    
  } 

 // When all the tasks has been completed the task_done flags are zeroized to start again the tasks schedule
 task_1_done = LOW;
 task_2_done = LOW;
 task_3_done = LOW;
 }

void GNSS_management()
 {
 GNSS_SS.listen(); 
 if(GNSS_SS.available()) 
  {  
  if(GNSS.encode(GNSS_SS.read()))
   {
   if(GNSS.location.isValid())
    {
    // NOTE: in this SW the GNSS provides correct data ONLY in the north emisphere!
    if(debug_out == HIGH)
     {
     DEBUG_SS.print("GNSS latitude:   ");
     DEBUG_SS.println(ALU_data_str_loc.GNSS_lat, GNSS_pos_prec);
     DEBUG_SS.print("GNSS longitude:   ");
     DEBUG_SS.println(ALU_data_str_loc.GNSS_lon, GNSS_pos_prec); 

     DEBUG_SS.print("GNSS speed in m/s:   ");
     DEBUG_SS.println(ALU_data_str_loc.GNSS_speed); 
     DEBUG_SS.print("GNSS fix:   ");
     DEBUG_SS.println(ALU_data_str_loc.GNSS_fix);
     DEBUG_SS.print("GNSS altitude:   ");
     DEBUG_SS.println(ALU_data_str_loc.GNSS_alt);
     }
    ALU_data_str_loc.GNSS_alt = GNSS.altitude.meters();   
    ALU_data_str_loc.GNSS_lat = GNSS.location.lat();   
    ALU_data_str_loc.GNSS_lon = GNSS.location.lng();
    ALU_data_str_loc.GNSS_speed = GNSS.speed.kmph();
    ALU_data_str_loc.GNSS_fix = HIGH;
    }
   else
    {
    ALU_data_str_loc.GNSS_fix = LOW; 
    }    
   }
  } 
 }

void IMU_management()
 {
 const float rad2deg = 57.3; // Conversion factor from radiant to deg

 if(debug_out == HIGH)
  {
  DEBUG_SS.print("IMU_raw_accX:   ");
  DEBUG_SS.println(IMU_raw_accX);
  DEBUG_SS.print("IMU_raw_accY:   ");
  DEBUG_SS.println(IMU_raw_accY);
  DEBUG_SS.print("IMU_raw_accZ:   ");
  DEBUG_SS.println(IMU_raw_accZ);
  DEBUG_SS.print("IMU_raw_gyroX:   ");
  DEBUG_SS.println(IMU_raw_gyroX);
  DEBUG_SS.print("IMU_raw_gyroY:   ");
  DEBUG_SS.println(IMU_raw_gyroY);
  DEBUG_SS.print("IMU_raw_gyroZ:   ");
  DEBUG_SS.println(IMU_raw_gyroZ);
  DEBUG_SS.print("IMU_raw_pitch:   ");
  DEBUG_SS.println(IMU_raw_pitch);
  DEBUG_SS.print("IMU_raw_roll:   ");
  DEBUG_SS.println(IMU_raw_roll);
  DEBUG_SS.print("IMU_raw_LF:   ");
  DEBUG_SS.println(IMU_raw_LF);
  }
 IMU.update();
 IMU_raw_accX = IMU.getAccX();
 IMU_raw_accY = IMU.getAccY();
 IMU_raw_accZ = IMU.getAccZ();

 IMU_raw_gyroX = IMU.getGyroX();
 IMU_raw_gyroY = IMU.getGyroY();
 IMU_raw_gyroZ = IMU.getGyroZ();

 IMU_raw_pitch = IMU.getAngleY();  
 IMU_raw_roll = IMU.getAngleX();                   
 IMU_raw_LF = IMU.getAccZ();
 
 ALU_data_str_loc.IMU_pitch = IMU_raw_pitch;
 ALU_data_str_loc.IMU_roll = IMU_raw_roll;
 ALU_data_str_loc.IMU_LF = IMU_raw_LF;
 }

void serial_send()
 {
 uint16_t size_out = 0u;
 // This function takes the structure "dummy_struct_loc" as an input buffer, insert it in the "point to point" protocol and make the output available in the tx_buffer
 if (PPP_insert((const uint8_t*)&ALU_data_str_loc, (const uint16_t)sizeof(ALU_data_str_loc), (uint8_t*)tx_buffer, &size_out) == true)
  {
  Serial.write((uint8_t *)tx_buffer, size_out);
  }
 }
