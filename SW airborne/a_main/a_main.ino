// SW version: Spitfire 1
// Functions: plane management
// Programmer/s: Davide Cagnoni
// Last edit: 30-04-2023
// Note: the code integrates some section wroten by Andrea Zaffino for the proper UART exchange of the data structure, refer to the
// "UART_struct_receive" for further information and the original code version

// Aircraft reference system   
// X axis: roll axis with arrow to the nose of the aircraft
// Y axis: pitch axis with outgoing arrow towards the aircraft right side  
// Z axis: yaw axis axis with outgoing arrow towards the aircraft bottom side
                                                   

//________________________________________________________________________________________________________________________________________________________________________________________________________________________________________

// Libraries 

#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <SPort.h>
#include "heltec.h"  
#include <stdlib.h>
#include "transport_PPP.h"
#include "Arduino.h"
#include "EBYTE.h"

//______________________________________________________________________OLED images and variables________________________________________________________________________________________

const unsigned char CAGNONI[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x3F, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x3E, 0xF2, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x03, 0x00, 0x07, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x00, 0x00, 
  0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x70, 0x00, 0x00, 0xB8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x30, 0x02, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 
  0x60, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x0C, 0x00, 0x00, 0xC0, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x80, 0x61, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 
  0x80, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x03, 0x20, 0x00, 0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xE0, 0x07, 0x00, 0x03, 0x06, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x20, 0x20, 
  0x00, 0x03, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x03, 0x40, 0x00, 0x00, 0x03, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x40, 0x00, 0x00, 0x03, 0x18, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x40, 0x00, 
  0x00, 0x03, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x03, 0x00, 0x00, 0x00, 0x03, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x01, 0x60, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 
  0x80, 0x01, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x36, 0x00, 0x00, 0x80, 0x01, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x6C, 0x00, 0x00, 0xC0, 0x00, 0x80, 0x01, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x98, 0x01, 0x00, 
  0x60, 0x00, 0x80, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x18, 0x1E, 0x00, 0x30, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0xF0, 0x00, 0x18, 0x18, 0x00, 0x03, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x01, 0x1F, 
  0x0E, 0xC0, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x80, 0x03, 0xE0, 0x07, 0xC0, 0x01, 0x06, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFD, 0xFA, 0xFF, 0x00, 0x06, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x1F, 
  0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x10, 0x0E, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x20, 0x04, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0xC0, 0x03, 0xC0, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x80, 0x07, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x06, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0xC0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x7F, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x80, 0x0F, 0x00, 0x68, 0x00, 0xE0, 0x03, 0x60, 0x01, 0x06, 0xC0, 0x03, 
  0x80, 0x02, 0x19, 0x1C, 0xF0, 0x7F, 0x00, 0xFC, 0x00, 0xFC, 0x1F, 0xE0, 
  0x83, 0x0F, 0xF8, 0x1F, 0xE0, 0x07, 0x0F, 0x7C, 0xF8, 0xFF, 0x00, 0xFC, 
  0x00, 0xFE, 0x3F, 0xF0, 0x87, 0x07, 0xFE, 0x3F, 0xE0, 0x87, 0x0F, 0x3C, 
  0xFC, 0xFF, 0x01, 0xFE, 0x01, 0xFF, 0x3F, 0xF0, 0x87, 0x07, 0xFF, 0x7F, 
  0xE0, 0x8F, 0x0F, 0x3E, 0x7C, 0xF0, 0x01, 0xFF, 0x81, 0x1F, 0x7C, 0xF0, 
  0xCF, 0x07, 0x1F, 0x7C, 0xE0, 0x8F, 0x0F, 0x3E, 0x3E, 0x50, 0x00, 0xEF, 
  0x81, 0x0F, 0x00, 0xF0, 0xCF, 0x87, 0x0F, 0x7C, 0xF0, 0x9F, 0x07, 0x3E, 
  0x3E, 0x00, 0x80, 0xEF, 0x81, 0x8F, 0x3F, 0xF8, 0xDF, 0x83, 0x0F, 0x7C, 
  0xF0, 0x9F, 0x07, 0x1E, 0x1E, 0x00, 0xC0, 0xE7, 0xC1, 0xC7, 0x3F, 0xF8, 
  0xFF, 0x83, 0x0F, 0x7C, 0xF0, 0xFE, 0x07, 0x1F, 0x3F, 0xA0, 0xC0, 0xE3, 
  0xC1, 0xC7, 0x3F, 0x78, 0xFE, 0x83, 0x0F, 0x7C, 0xF0, 0xFC, 0x07, 0x1F, 
  0x3E, 0xF8, 0xE0, 0xFF, 0x83, 0x4F, 0x3E, 0x7C, 0xFE, 0x83, 0x0F, 0x3E, 
  0xF8, 0xFC, 0x03, 0x1F, 0x7E, 0x7E, 0xF0, 0xFF, 0x83, 0x1F, 0x1F, 0x7C, 
  0xFC, 0x81, 0x1F, 0x3F, 0xF8, 0xF8, 0x83, 0x0F, 0xFC, 0x7F, 0xF0, 0xFF, 
  0x83, 0xFF, 0x0F, 0x7C, 0xFC, 0x01, 0xFF, 0x1F, 0x78, 0xF8, 0x83, 0x0F, 
  0xFC, 0x3F, 0xF8, 0xE0, 0x03, 0xFF, 0x07, 0x3C, 0xF8, 0x01, 0xFE, 0x0F, 
  0x78, 0xF0, 0x81, 0x0F, 0xF0, 0x0F, 0x7C, 0xE0, 0x03, 0xFC, 0x03, 0x3E, 
  0xF0, 0x00, 0xFC, 0x03, 0x7C, 0xF0, 0x81, 0x0F, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x54, 0x12, 0x89, 0x21, 0xA6, 0x02, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x4D, 0x96, 0x08, 0x92, 0xA0, 0x04, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x20, 
  0x88, 0x24, 0x05, 0x02, };

const byte ROW_SPACE = 12; // Number of separation pixels between two text rows on the OLED

//______________________________________________________________________Variables and constant declaration________________________________________________________________________________________

// Debug variables
const int debug_serial_boud_rate = 9600;    // Boud rate for the output serial data
// If set to HIGH the program will provide as serial output the debug data
// IMPORTANT NOTE: When the debug is active the script won't run correctly due to temporization issues
const bool sys_debug_out = LOW;  // System variables debug data
const bool FDR_debug_out = LOW;  // FDR variables debug data 
const bool ALU_debug_out = LOW;  // ALU variables debug data    
const bool ADU_debug_out = LOW;  // ADU variables debug data 
const bool EPU_debug_out = LOW;  // EPU variables debug data 
const bool MCDU_debug_out = LOW; // MCDU variables debug data
const bool GTSU_debug_out = LOW; // GTSU variables debug data 

// This flag is used to initialize the debug serial port, it turns on if at least one "function debug flag" has been raised
const bool debug_out = sys_debug_out || FDR_debug_out || ALU_debug_out || ADU_debug_out || EPU_debug_out || MCDU_debug_out || GTSU_debug_out;
                                            
// OLED characteristics
const byte OLED_H = 64;
const byte OLED_W = 128;

// SD variables
bool SD_active = LOW;            // If LOW the SD isn't ready
int FDR_delta_time = 1000;       // Time in [ms] between two log on the FDR 
long int FDR_last_time = 0;      // Last time the FDR has recorded a log in [ms]  
 
// ALU variables (Attitude and Localization Unit)
typedef struct __attribute__((__packed__))
 {
 bool GNSS_fix = LOW;      // When HIGH this flag indicates that the GNSS is fixed
 uint16_t GNSS_alt = 0;     // GNSS altidude [m]
 
 // IMPORTANT NOTE! The GNSS latitude and longitude are reliable ONLY in the northen emisphere!!
 float GNSS_lat = 0;       // GNSS latidìtude [deg]
 float GNSS_lon = 0;       // GNSS longitude [deg]
 uint16_t GNSS_speed = 0;  // GNSS speed [m/s]

 float IMU_pitch = 0;   // IMU pitch angle [deg]
 float IMU_roll = 0;    // IMU roll angle [deg]                
 float IMU_LF = 0;      // Load Factor in [g]
 }

ALU_data_str;
ALU_data_str ALU_data_str_loc;  //Local data struct  
ALU_data_str ALU_data_str_rec;  //Remote (UART received) data struct

// ADU variables (Air Data Unit)
typedef struct __attribute__((__packed__))
 {
 uint16_t ADU_ias = 0;                     // IAS in [m/s]
 uint16_t ADU_bar_height = 0;              // ADU barometric height [100*m], the height is computed from the departure point
 uint16_t ADU_sta_temp = 0;                // ADU static temp [°K]          
 }
 
ADU_data_str;
ADU_data_str ADU_data_str_loc;  //Local data struct  
ADU_data_str ADU_data_str_rec;  //Remote (UART received) data struct

// EPU variables (Engine and Power Unit)
typedef struct __attribute__((__packed__))
 {
 uint16_t EPU_rpm = 0;                     // Engine rpm
 uint16_t EPU_temp = 0;                    // Engine temperature in [°C]
 uint16_t EPU_batt_volt = 0;               // Battery voltage [V*100]
 bool EPU_eng_ON = 0;                      // Engine is ON flag
 }
 
EPU_data_str;
EPU_data_str EPU_data_str_loc;  //Local data struct  
EPU_data_str EPU_data_str_rec;  //Remote (UART received) data struct

// Flight_time variables
uint8_t FT_s = 0;                   // Flight time in [s]
uint8_t FT_m = 0;                   // Flight time in ["]
uint8_t FT_ms = 0;                  // Flight time in [mm ss]
uint16_t FT_last_time = 0; 


// GSTU variables (Ground Station Telemetry Unit)
/*  
Each one of the struct "cells" can contain a float variable, the struct is defined with enough cells to host all the largest string data.
For some of them, not all the available cells will be used, in such cases the unused cells contain the variable 0 or, 
if they have been already used to store another data, the last value.
They won't be read whether the string corresponding to that specific header does't foresee information in that specific cell  
*/

/*  
Two struct are used for the GSTU, a TX structure from the SCH to the ground station (GSTU_TX_data_str) designed to send telemetry data,
and a RX structure from the ground station to the SCH (GSTU_RX_data_str) to receive ground commands
*/

typedef struct __attribute__((__packed__))
 {
 byte identifier = 0; // This first variable contains the header for the struct identification
 float  f_data_1 = 0; 
 float  f_data_2 = 0;
 float  f_data_3 = 0; 
 float  f_data_4 = 0;
 float  f_data_5 = 0;
 float  f_data_6 = 0;
 float  f_data_7 = 0; 
 float  f_data_8 = 0; 
 float  f_data_9 = 0;
 }

GSTU_TX_data_str;
GSTU_TX_data_str GSTU_TX_data_str_loc;  //Local data struct  
GSTU_TX_data_str GSTU_TX_data_str_rec;  //Remote (UART received) data struct

// Variables for the checksum of the sent structs
uint16_t GSTU_last_str_sum_FAP  = 0;
uint16_t GSTU_last_str_sum_ESP  = 0;
uint16_t GSTU_last_str_sum_EP  = 0;
uint32_t GSTU_last_str_sum_LP  = 0; // This is the only value requiring a 32 bit space to be stored due to the GNSS coordinates
uint16_t GSTU_str_sum = 0;

// Structure textrure with adding the "point to point protocol", being this buffer used to store the data coming from the several peripheral units, 
// it must be sized to receive the largest struct used (ALU_data_str)
volatile uint8_t tx_buffer[sizeof(ALU_data_str) * 2];

// Dummy scheduler, it assigns a specific time to each task (function) before switching to another one.
// If a task takes more time to be executed compared to assigned one, the next task will start whiele the last task is still running,
// Be sure to set a proper time for each function
 
// Assigned time for each task in [ms], these are the maximum times dedicated to each task, the several task could take less time to be executed
const int TASK_1_TIME = 70;
const int TASK_2_TIME = 50;
const int TASK_3_TIME = 50;
const int TASK_4_TIME = 50;
const int TASK_5_TIME = 50;
const int TASK_6_TIME = 50;
const int TASK_7_TIME = 70;

long int last_task_1_time = 0;
long int last_task_2_time = 0;
long int last_task_3_time = 0;
long int last_task_4_time = 0;
long int last_task_5_time = 0;
long int last_task_6_time = 0;
long int last_task_7_time = 0;

bool task_1_done = LOW;
bool task_2_done = LOW;
bool task_3_done = LOW;
bool task_4_done = LOW;
bool task_5_done = LOW;
bool task_6_done = LOW;
bool task_7_done = LOW;

// When one of these flag is raised, the task scheduler will stop the corresponding task earlyer, executing the next one scheduled to save time
bool task_1_stop = LOW;
bool task_2_stop = LOW;
bool task_3_stop = LOW;
bool task_4_stop = LOW;
bool task_5_stop = LOW;
bool task_6_stop = LOW;
bool task_7_stop = LOW;

//___________________________________________________________________________Pins numbers declaration_______________________________________________________________________________________________________________________

// Micro SD blizzard pins
const byte FDR_CS_PIN = 2;
const byte FDR_SCK_PIN = 18;
const byte FDR_MISO_PIN = 19;
const byte FDR_MOSI_PIN = 23;

// ALU pins
const byte ALU_RX_PIN = 38;

// ADU pins
const byte ADU_RX_PIN = 34; 

// EPU pins
const byte EPU_RX_PIN = 35; 

// MCDU pins
const byte MCDU_TX_PIN = 12;
const byte MCDU_RX_PIN = 14;

// GTSU pins
const byte GSTU_TX_PIN = 26;
const byte GSTU_RX_PIN = 25;
const byte GSTU_M0_PIN = 17;
const byte GSTU_M1_PIN = 27;
const byte GSTU_AX_PIN = 22;

// FrSky telemetry pins
const byte FrSky_SPORT_OUT = 5;

// Input servo pins
const byte ARMED_SIG_PIN = 21;


//_______________________________________________________________________________________Objects inizialization___________________________________________________________________________

// FDR files

/* The FDR has a .txt file for each peripheral installed, plus one for general system data (sys_data).
   Due to the deep difference between the two data types provided by the ALU, two files are reserved to this peripheral, IMU_data and GNSS_data 
   for the respective sensor installed on the ALU. */

File GNSS_data; 
File IMU_data;
File ADU_data;
File EPU_data;
File sys_data;

// S_port objects
// To visualize the sensors on the radio select in the telemetry menu, "Create sensor DIY" an then press "autodetecting" the available ID will compare on the screen
SPortHub S_port(0x12, FrSky_SPORT_OUT);         //Hardware ID 0x12, this pin provide in output the telemetry
SimpleSPortSensor telem_EPU_batt_volt(0x5900);  //Sensor with custom ID 0x4105
SimpleSPortSensor telem_flight_time(0x5901);    //Sensor with custom ID 0x4106
SimpleSPortSensor telem_GNSS_altitude(0x5902);  //Sensor with custom ID 0x4107
SimpleSPortSensor telem_ADU_ias(0x5903);        //Sensor with custom ID 0x4108
SimpleSPortSensor telem_EPU_temp(0x5904);       //Sensor with custom ID 0x4109
SimpleSPortSensor telem_EPU_rpm(0x5905);        //Sensor with custom ID 0x4109


// Software Serial ports
SoftwareSerial ALU_SS (ALU_RX_PIN, -1);          // The ALU serial port is configured as a "receiving only" using a "-1" to inhibit the tranmitting pin
SoftwareSerial ADU_SS (ADU_RX_PIN, -1);          // The ADU serial port is configured as a "receiving only" using a "-1" to inhibit the tranmitting pin
SoftwareSerial EPU_SS (EPU_RX_PIN, -1);          // The EPU serial port is configured as a "receiving only" using a "-1" to inhibit the tranmitting pin
SoftwareSerial MCDU_SS (MCDU_RX_PIN, MCDU_TX_PIN);
SoftwareSerial GSTU_SS (GSTU_RX_PIN, GSTU_TX_PIN);   

// GSTU object creation
EBYTE GSTU(&GSTU_SS, GSTU_M0_PIN, GSTU_M1_PIN, GSTU_AX_PIN);

// Serial ports baud rate
const int ALU_SS_BR = 9600;
const int ADU_SS_BR = 9600;
const int EPU_SS_BR = 9600;
const int MCDU_SS_BR = 9600;
const int GSTU_SS_BR = 9600;
//________________________________________________________________________________________Functions declaration_____________________________________________________________________________

// Peripherals management functions
void FDR_management();                          
void ALU_management();                         
void ADU_management();
void EPU_management();    
void FrSky_telemetry(); 
// Since the GSTU can work both as transmiter and receiver, two separate functions are used
void GSTU_management_TX();   
void GSTU_management_RX();                             
void flight_time();                     
void MCDU_management();
void StructSum(); // Function used to sum all the data in the structs prior to their sending to the GS

//________________________________________________________________________________________Secondary functions declaration_____________________________________________________________________________



//___________________________________________________________________________________________________Setup and Loop__________________________________________________________________________________________________________________________________________________

void setup() 
 {
 // Leave the "Heltec.begin()" in the first position of the setup.
 // This function set baudrate and pinmode not compliant with the ones needed by the code

 // OLED initializing
 Heltec.begin(true, false, true);
 Heltec.display->setContrast(255);
 Heltec.display->setFont(ArialMT_Plain_10);

 Heltec.display->clear();
 Heltec.display->drawXbm(0, 0, OLED_W, OLED_H, CAGNONI);
 Heltec.display->display();
 delay(1500);
 Heltec.display->clear(); 
 Heltec.display->display();

 Heltec.display->drawString(0, ROW_SPACE, "SW version: 1.0");
 Heltec.display->drawString(0, ROW_SPACE*2, "Perform sys. checks:");
 Heltec.display->display();
 delay(500);
 
 // Setting pins mode
 // Input servo pins
 pinMode(ARMED_SIG_PIN, INPUT); 

 // Debug function initializing
 if(debug_out == HIGH)
  {
  Serial.begin(debug_serial_boud_rate);
  delay(100); 
  Serial.println("Debug out active");
  }
 
 // SD card inizializing
 if(!SD.begin(FDR_CS_PIN))
  {
  Heltec.display->drawString(0, ROW_SPACE*3, "FDR not ready");
  Heltec.display->display();
  SD_active = LOW;
  }
 else
  {
  SD_active = HIGH;
  if(SD.exists("GNSS_data.txt"))
   SD.remove("GNSS_data.txt");
   
  if(SD.exists("IMU_data.txt"))
   SD.remove("IMU_data.txt");

  if(SD.exists("ADU_data.txt"))
   SD.remove("ADU_data.txt");
   
  if(SD.exists("EPU_data.txt"))
   SD.remove("EPU_data.txt");
   
  if(SD.exists("Sys_data.txt"))
   SD.remove("Sys_data.txt");

  Heltec.display->drawString(0, ROW_SPACE*3, "FDR ready");
  Heltec.display->display();
  }

 // S_port configuration
 S_port.registerSensor(telem_EPU_batt_volt);
 S_port.registerSensor(telem_flight_time);
 S_port.registerSensor(telem_GNSS_altitude);
 S_port.registerSensor(telem_ADU_ias);
 S_port.registerSensor(telem_EPU_temp);
 S_port.registerSensor(telem_EPU_rpm);
 S_port.begin();
  
 // ALU serial port initialization
 ALU_SS.begin(ALU_SS_BR);
 delay(1000);
 if(ALU_SS.available())
  {
  Heltec.display->clear();
  Heltec.display->drawString(0, ROW_SPACE, "SW version: 1.0");
  Heltec.display->drawString(0, ROW_SPACE*2, "Perform sys. checks:");
  Heltec.display->drawString(0, ROW_SPACE*3, "ALU active");
  Heltec.display->display(); 
  delay(1000);
  
  // Check for GNSS fixing, this is not a blocking check, if the GNSS is not already fixed the code will go on just allerting about the 
  // temporary GNSS unavailability
  ALU_management();
  if(ALU_data_str_rec.GNSS_fix == HIGH)
   {
   Heltec.display->clear();
   Heltec.display->drawString(0, ROW_SPACE, "SW version: 1.0");
   Heltec.display->drawString(0, ROW_SPACE*2, "Perform sys. checks:");
   Heltec.display->drawString(0, ROW_SPACE*3, "ALU active");
   Heltec.display->drawString(0, ROW_SPACE*4, "GNSS fixed");
   Heltec.display->display(); 
   }
  else
   {
   Heltec.display->clear();
   Heltec.display->drawString(0, ROW_SPACE, "SW version: 1.0");
   Heltec.display->drawString(0, ROW_SPACE*2, "Perform sys. checks:");
   Heltec.display->drawString(0, ROW_SPACE*3, "ALU active");
   Heltec.display->drawString(0, ROW_SPACE*4, "GNSS not fixed");
   Heltec.display->display(); 
   }  
  }
 else
  {
  Heltec.display->clear();
  Heltec.display->drawString(0, ROW_SPACE, "SW version: 1.0");
  Heltec.display->drawString(0, ROW_SPACE*2, "Perform sys. checks:");
  Heltec.display->display();
  Heltec.display->drawString(0, ROW_SPACE*3, "ALU not ready");
  Heltec.display->display();  
  }
 memset(&ALU_data_str_loc, 0, sizeof(ALU_data_str_loc));
 memset(&ALU_data_str_rec, 0, sizeof(ALU_data_str_rec));
  
 // ADU serial port initialization 
 ADU_SS.begin(ADU_SS_BR);
 delay(1000);
 
 if(ADU_SS.available())
  {
  Heltec.display->clear();
  Heltec.display->drawString(0, ROW_SPACE, "SW version: 1.0");
  Heltec.display->drawString(0, ROW_SPACE*2, "Perform sys. checks:");
  Heltec.display->display();
  Heltec.display->drawString(0, ROW_SPACE*3, "ADU active");
  Heltec.display->display();  
  }
 else
  {
  Heltec.display->clear();
  Heltec.display->drawString(0, ROW_SPACE, "SW version: 1.0");
  Heltec.display->drawString(0, ROW_SPACE*2, "Perform sys. checks:");
  Heltec.display->display();
  Heltec.display->drawString(0, ROW_SPACE*3, "ADU not ready");
  Heltec.display->display();  
  } 
 memset(&ADU_data_str_loc, 0, sizeof(ADU_data_str_loc));
 memset(&ADU_data_str_rec, 0, sizeof(ADU_data_str_rec));

 // EPU serial port initialization
 EPU_SS.begin(EPU_SS_BR);
 delay(1000);
 
 if(EPU_SS.available())
  {
  Heltec.display->clear();
  Heltec.display->drawString(0, ROW_SPACE, "SW version: 1.0");
  Heltec.display->drawString(0, ROW_SPACE*2, "Perform sys. checks:");
  Heltec.display->display();
  Heltec.display->drawString(0, ROW_SPACE*3, "EPU active");
  Heltec.display->display();  
  }
 else
  {
  Heltec.display->clear();
  Heltec.display->drawString(0, ROW_SPACE, "SW version: 1.0");
  Heltec.display->drawString(0, ROW_SPACE*2, "Perform sys. checks:");
  Heltec.display->display();
  Heltec.display->drawString(0, ROW_SPACE*3, "EPU not ready");
  Heltec.display->display();  
  }
 memset(&EPU_data_str_loc, 0, sizeof(EPU_data_str_loc));
 memset(&EPU_data_str_rec, 0, sizeof(EPU_data_str_rec));
 
 // MCDU serial port initialization
 //MCDU_SS.begin(MCDU_SS_BR);
 delay(1000);

 // GSTU serial port initialization
 GSTU_SS.begin(GSTU_SS_BR);
 GSTU.init();
 Heltec.display->clear();
 Heltec.display->drawString(0, ROW_SPACE, "SW version: 1.0");
 Heltec.display->drawString(0, ROW_SPACE*2, "Perform sys. checks:");
 Heltec.display->display();
 Heltec.display->drawString(0, ROW_SPACE*3, "GSTU configured");
 Heltec.display->display(); 
 delay(1000);
 Heltec.display->clear();
 Heltec.display->display();
 memset(&GSTU_TX_data_str_loc, 0, sizeof(GSTU_TX_data_str_loc));
 memset(&GSTU_TX_data_str_rec, 0, sizeof(GSTU_TX_data_str_rec));
 }

void loop() 
 {
 while((millis()-last_task_1_time < TASK_1_TIME || task_1_done == LOW) && task_1_stop == LOW)
  {
  if(task_1_done == LOW)
   {
   task_1_done = HIGH;
   last_task_1_time = millis();
   }
  FrSky_telemetry();    
  }

 while((millis()-last_task_2_time < TASK_2_TIME || task_2_done == LOW) && task_2_stop == LOW)
  {
  if(task_2_done == LOW)
   {
   task_2_done = HIGH;
   last_task_2_time = millis();
   }
  flight_time();    
  }  

 while((millis()-last_task_3_time < TASK_3_TIME || task_3_done == LOW) && task_3_stop == LOW)
  {
  if(task_3_done == LOW)
   {
   task_3_done = HIGH;
   last_task_3_time = millis();
   }
  ADU_management();    
  }  

 while((millis()-last_task_4_time < TASK_4_TIME || task_4_done == LOW) && task_4_stop == LOW)
  {
  if(task_4_done == LOW)
   {
   task_4_done = HIGH;
   last_task_4_time = millis();
   }
  EPU_management();    
  }

 while((millis()-last_task_5_time < TASK_5_TIME || task_5_done == LOW) && task_5_stop == LOW)
  {
  if(task_5_done == LOW)
   {
   task_5_done = HIGH;
   last_task_5_time = millis();
   }
  // This if condition with a further timer has been inserted to ensure a single log every time the "FDR_management" is called, it also keeo the coherence with the 
  // other task scheduler's items
  if(millis()-FDR_last_time > FDR_delta_time)
   {
   FDR_last_time = millis();
   FDR_management(); 
   }
  }
  
 while((millis()-last_task_6_time < TASK_6_TIME || task_6_done == LOW) && task_6_stop == LOW)
  {
  if(task_6_done == LOW)
   {
   task_6_done = HIGH;
   last_task_6_time = millis();
   }
  ALU_management();    
  }

 while((millis()-last_task_7_time < TASK_7_TIME || task_7_done == LOW) && task_7_stop == LOW)
  {
  if(task_7_done == LOW)
   {
   task_7_done = HIGH;
   last_task_7_time = millis();
   }
  GSTU_management_TX();    
  }
  
 // When all the tasks has been completed the task_done flags are zeroized to start again the tasks schedule
 task_1_done = LOW;
 task_2_done = LOW;
 task_3_done = LOW;
 task_4_done = LOW;
 task_5_done = LOW;
 task_6_done = LOW;
 task_7_done = LOW;

 task_1_stop = LOW;
 task_2_stop = LOW;
 task_3_stop = LOW;
 task_4_stop = LOW;
 task_5_stop = LOW;
 task_6_stop = LOW;
 task_7_stop = LOW;
 }

//___________________________________________________________________________________________________________Functions________________________________________


// Be sure to use MAX_PPP_BUF_SIZE larger at least the double of the transmitted data, 
// this is useful in case of all the sent bytes are 0x5D o 0x5E, they will be doubled sent with an "escape"
uint8_t buf_in[MAX_PPP_BUF_SIZE];       // Received texture with the "point to point" protocol
uint8_t buf_out[MAX_PPP_BUF_SIZE / 2];  // Texture without the "point to point protocol"

//_____________________________________________________________________________________Functions to call to get the data structs coming from the peripheral units________________________________________

void ALU_get_serial_data() 
 {
 e_PPP_extract_result ppp_res = DATA_INCOMPLETE;
 uint16_t buf_in_len = 0u;
 uint16_t buf_out_len = 0u;

 while (ALU_SS.available() && (buf_in_len < sizeof(buf_in)))
  {
  buf_in[buf_in_len++] = ALU_SS.read();
  }
  
 do  // The do-while cycle is used to extract possible multiple textures from the received buffer
  {
  buf_out_len = (uint16_t)(sizeof(buf_out));  
  // It is necessary to be recalled because it is used to indicate to PPP_extract() how big is the "output buffer" 
  // It reads the input buffer from the serial, extract it from the "point ot point protocol" and make it available in the output in "buf_out"
  ppp_res = PPP_extract(buf_in, buf_in_len, buf_out, &buf_out_len);
  // This to avoid a multiple buffer passage, it is usefule in case of SUCCESS_DATA_PENDING, since passing 0, 
  // the function is commanded to not read the input buffer but to analyze the data present in the last texture received 
  buf_in_len = 0u;    

  if ((ppp_res == SUCCESS_END) || (ppp_res == SUCCESS_DATA_PENDING))
   {
   // Texture has been correctly received
   if (buf_out_len == sizeof(ALU_data_str_rec)) // The reeived data are checked to verify the compliancy with the expected structure
    {
    memcpy(&ALU_data_str_rec, buf_out, (size_t)buf_out_len); // The remote struct is populated with the incoming data
    }
   }
  } 
 while (ppp_res == SUCCESS_DATA_PENDING);
 }
 

void ADU_get_serial_data() 
 {
 e_PPP_extract_result ppp_res = DATA_INCOMPLETE;
 uint16_t buf_in_len = 0u;
 uint16_t buf_out_len = 0u;

 while (ADU_SS.available() && (buf_in_len < sizeof(buf_in)))
  {
  buf_in[buf_in_len++] = ADU_SS.read();
  }
  
 do  // The do-while cycle is used to extract possible multiple textures from the received buffer
  {
  buf_out_len = (uint16_t)(sizeof(buf_out));  
  // It is necessary to be recalled because it is used to indicate to PPP_extract() how big is the "output buffer" 
  // It reads the input buffer from the serial, extract it from the "point ot point protocol" and make it available in the output in "buf_out"
  ppp_res = PPP_extract(buf_in, buf_in_len, buf_out, &buf_out_len);
  // This to avoid a multiple buffer passage, it is usefule in case of SUCCESS_DATA_PENDING, since passing 0, 
  // the function is commanded to not read the input buffer but to analyze the data present in the last texture received 
  buf_in_len = 0u;    

  if ((ppp_res == SUCCESS_END) || (ppp_res == SUCCESS_DATA_PENDING))
   {
   // Texture has been correctly received
   if (buf_out_len == sizeof(ADU_data_str_rec)) // The reeived data are checked to verify the compliancy with the expected structure
    {
    memcpy(&ADU_data_str_rec, buf_out, (size_t)buf_out_len); // The remote struct is populated with the incoming data
    }
   }
  } 
 while (ppp_res == SUCCESS_DATA_PENDING);
 }


void EPU_get_serial_data() 
 {
 e_PPP_extract_result ppp_res = DATA_INCOMPLETE;
 uint16_t buf_in_len = 0u;
 uint16_t buf_out_len = 0u;

 while (EPU_SS.available() && (buf_in_len < sizeof(buf_in)))
  {
  buf_in[buf_in_len++] = EPU_SS.read();
  }
  
 do  // The do-while cycle is used to extract possible multiple textures from the received buffer
  {
  buf_out_len = (uint16_t)(sizeof(buf_out));  
  // It is necessary to be recalled because it is used to indicate to PPP_extract() how big is the "output buffer" 
  // It reads the input buffer from the serial, extract it from the "point ot point protocol" and make it available in the output in "buf_out"
  ppp_res = PPP_extract(buf_in, buf_in_len, buf_out, &buf_out_len);
  // This to avoid a multiple buffer passage, it is usefule in case of SUCCESS_DATA_PENDING, since passing 0, 
  // the function is commanded to not read the input buffer but to analyze the data present in the last texture received 
  buf_in_len = 0u;    

  if ((ppp_res == SUCCESS_END) || (ppp_res == SUCCESS_DATA_PENDING))
   {
   // Texture has been correctly received
   if (buf_out_len == sizeof(EPU_data_str_rec)) // The reeived data are checked to verify the compliancy with the expected structure
    {
    memcpy(&EPU_data_str_rec, buf_out, (size_t)buf_out_len); // The remote struct is populated with the incoming data
    }
   }
  } 
 while (ppp_res == SUCCESS_DATA_PENDING);
 }

void GSTU_send_serial_data()
 {
 uint16_t size_out = 0u;
 // This function takes the structure "dummy_struct_loc" as an input buffer, insert it in the "point to point" protocol and make the output available in the tx_buffer
 if (PPP_insert((const uint8_t*)&GSTU_TX_data_str_loc, (const uint16_t)sizeof(GSTU_TX_data_str_loc), (uint8_t*)tx_buffer, &size_out) == true)
  {
  GSTU_SS.write((uint8_t *)tx_buffer, size_out);
  }
 }
