// SW version: ADU 1 
// Functions: Air Data Unit, data computation and sending to the SCH
// Programmer/s: Davide Cagnoni
// Last edit: 
// Note: the code integrates some section wroten by Andrea Zaffino for the proper UART exchange of the data structure, refer to the
// "UART_struct_send" for further information and the original code version

//________________________________________________________________________________________________________________________________________________________________________________________________________________________________________

// Libraries 

#include <SoftwareSerial.h>
#include <Q2HX711.h>

#include <stdlib.h>
#include "transport_PPP.h"
#include "Arduino.h"
 
//______________________________________________________________________Variables and constant declaration________________________________________________________________________________________

// Debug variables
const int debug_serial_boud_rate = 9600;    // Boud rate for the output serial data
const int debug_out = LOW;                  // If set to HIGH the program will provide as serial output the debug data

// ADU data struct
typedef struct __attribute__((__packed__))
 {
 uint16_t ADU_ias = 0;                     // IAS in [m/s]
 uint16_t ADU_bar_height = 0;              // ADU barometric height [100*m], the height is computed from the departure point
 uint16_t ADU_sta_temp = 0;                // ADU static temp [°K]          
 }
 
ADU_data_str;
ADU_data_str ADU_data_str_loc;  //Local data struct  
ADU_data_str ADU_data_str_rec;  //Remote (UART received) data struct

volatile uint8_t tx_buffer[sizeof(ADU_data_str) * 2];  //Structure textrure with adding the "point to point protocol"

// The IAS measurement is carryed out using the S.L. air density, no TAS is provided 
int sta_temp_an_in = 0;
long int sta_press_an_in = 0;
int dyn_press_an_in = 0;
long int sta_press = 0;         // Static pressure in [hPa]
int dyn_press = 0;              // Dynamic pressure in [Pa]
long int qfe = 0;               // QFE in [Pa]

const int STA_TEMP_COUNT2K = 33;    // Conversion factor from count to K for the temperature sensor
const int DYN_PRESS_COUNT2K = 2330; // Conversion factor from count to K for the pressure sensor
const float VBG = 823;              // Vertical Baric Gradient [100*m/Pa] 
const int KRHO = 1225;              // Air density in [1000*Kg/m3], to avoid float variables
const int STA_PRESS_SENS2Pa = 72;   // Conversion factor from sensor unit to Pa for the static pressure sensor        

// Dummy scheduler, it assigns a specific time to each task (function) before switching to another one.
// If a task takes more time to be executed compared to assigned one, the next task will start whiele the last task is still running,
// Be sure to set a proper time for each function
 
// Assigned time for each task in [ms]
const int TASK_1_TIME = 100;
const int TASK_2_TIME = 200;

long int last_task_1_time = 0;
long int last_task_2_time = 0;

bool task_1_done = LOW;
bool task_2_done = LOW;

//___________________________________________________________________________Pins numbers declaration_______________________________________________________________________________________________________________________

// Debug serial port, this is a unidirectional port
const byte DEBUG_TX = 3;

const byte STA_TEMP_AN_IN_PIN = A0;

// The static pressure sensor is a HX710B pressure sensor working with an I2C bus
const byte STA_PRESS_SDA_PIN = A4;
const byte STA_PRESS_SCL_PIN = A5;

//_______________________________________________________________________________________Objects inizialization___________________________________________________________________________

// Software Serial ports
SoftwareSerial DEBUG_SS (-1, DEBUG_TX);   // The ADU debug serial port is configured as a "transmitting only" using a "-1" to inhibit the receiving pin, 
                                          // an UART-USB converter with a terminal on PC is required for debug

Q2HX711 sta_press_sens(STA_PRESS_SDA_PIN, STA_PRESS_SCL_PIN); // Static pressure sensor
                                          
//________________________________________________________________________________________Functions declaration_____________________________________________________________________________


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

 // Serial initialization
 Serial.begin(9600);

 // Instructions send the struct through the UART
 memset(&ADU_data_str_loc, 0, sizeof(ADU_data_str_loc));
 memset(&ADU_data_str_rec, 0, sizeof(ADU_data_str_rec));
 
 // QFE computation, the departure level is set as the "zero"
 qfe = sta_press_sens.read()/STA_PRESS_SENS2Pa;
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
  sta_temp_an_in = analogRead(STA_TEMP_AN_IN_PIN);
  sta_press = sta_press_sens.read()/STA_PRESS_SENS2Pa;
 
  ADU_data_str_loc.ADU_ias = 123;//sqrt(2000*dyn_press/KRHO);                                // IAS computation from dynamic pressure
  ADU_data_str_loc.ADU_bar_height = 456;//abs(qfe - sta_press)/VBG;
  ADU_data_str_loc.ADU_sta_temp = 555; //map(sta_temp_an_in, 0, 1024, 0, STA_TEMP_COUNT2K); 

  if(debug_out == HIGH)
   {
   delay(200);
   DEBUG_SS.print("Static temp. [°K]:  ");
   DEBUG_SS.println(ADU_data_str_loc.ADU_sta_temp);
   DEBUG_SS.print("Static pressure. [Pa]:  ");
   DEBUG_SS.println(sta_press);
   DEBUG_SS.print("Barometric height [m]:  ");
   DEBUG_SS.println(ADU_data_str_loc.ADU_bar_height);
   DEBUG_SS.print("IAS [m/s]:  ");
   DEBUG_SS.println(ADU_data_str_loc.ADU_ias);
   }
  }  
  
 while(millis()-last_task_2_time < TASK_1_TIME || task_2_done == LOW)
  {
  if(task_2_done == LOW)
   {
   task_2_done = HIGH;
   last_task_2_time = millis();
   }
  serial_send();    
  }  

 // When all the tasks has been completed the task_done flags are zeroized to start again the tasks schedule
 task_1_done = LOW;
 task_2_done = LOW;
 }
 
//___________________________________________________________________________________________________________Functions________________________________________

void serial_send()
 {
 uint16_t size_out = 0u;
 // This function takes the structure "dummy_struct_loc" as an input buffer, insert it in the "point to point" protocol and make the output available in the tx_buffer
 if (PPP_insert((const uint8_t*)&ADU_data_str_loc, (const uint16_t)sizeof(ADU_data_str_loc), (uint8_t*)tx_buffer, &size_out) == true)
  {
  Serial.write((uint8_t *)tx_buffer, size_out);
  }
 }
