//SW version: EPU 1 
//Functions: Engine and Power Unit, data computation and sending to the SCH
//Programmer/s: Davide Cagnoni
//Last edit: 
//Note: 

//________________________________________________________________________________________________________________________________________________________________________________________________________________________________________

// Libraries 

#include <SoftwareSerial.h> // SW serial used for debug, the only available HW UART port on Arduino in used for the communication with the SCH

#include <stdlib.h>
#include "transport_PPP.h"
#include "Arduino.h"
//______________________________________________________________________Variables and constant declaration________________________________________________________________________________________

// Debug variables
const int debug_serial_boud_rate = 9600;    // Boud rate for the output serial data
const int debug_out = LOW;                 // If set to HIGH the program will provide as serial output the debug data

// EPU data struct
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

volatile uint8_t tx_buffer[sizeof(EPU_data_str) * 2];  //Structure textrure with adding the "point to point protocol"

int eng_temp_an_in = 0;
int batt_volt_an_in = 0; 

long int tachometer_last_time = 0;
int tachometer_delta_time = 0;
volatile int tachometer_rev = 0;

long int loop_last_time = 0;
const int LOOP_EXE_TIME = 200;       // Delta time between a loop execution and another 

const int ENG_TEMP_COUNT2K = 52;     // Conversion factor from count to °C for the engine temperature sensor
const int BATT_VOLT_COUNT2K = 2646;  // Conversion factor from count to V*100 for the voltage sensor


//___________________________________________________________________________Pins numbers declaration_______________________________________________________________________________________________________________________

// Debug serial port, this is a unidirectional port
const byte DEBUG_TX_PIN = 3;

const byte RPM_IN_PIN = 2; // RPM sensor input pin, on Arduino Pro Mini, only the bit 2 and 3 can provide interrupt
const byte BATT_VOLT_AN_IN_PIN = A1;
const byte ENG_TEMP_AN_IN_PIN = A2;

//_______________________________________________________________________________________Objects inizialization___________________________________________________________________________

// Software Serial ports
SoftwareSerial DEBUG_SS (-1, DEBUG_TX_PIN);   // The EPU debug serial port is configured as a "transmitting only" using a "-1" to inhibit the receiving pin, 
                                          // an UART-USB converter with a terminal on PC is required for debug

//________________________________________________________________________________________Functions declaration_____________________________________________________________________________


//____________________________________________________________________________________Secondary functions declaration_____________________________________________________________________________


//___________________________________________________________________________________________________Setup and Loop__________________________________________________________________________________________________________________________________________________

void setup() 
 {
 // Debug function initializing
 if(debug_out == HIGH)
  {
  DEBUG_SS.begin(debug_serial_boud_rate); 
  DEBUG_SS.println("Debug out active");
  delay(100);
  }

 // Serial initialization
 Serial.begin(9600);
 
 pinMode(RPM_IN_PIN, INPUT);
 attachInterrupt(digitalPinToInterrupt(RPM_IN_PIN), Tachometer, FALLING); // Interrupt pin set for tachometer
 }
 
void loop() 
 {
 if(millis() - loop_last_time > LOOP_EXE_TIME)
  {
  detachInterrupt(RPM_IN_PIN);
  EPU_data_str_loc.EPU_rpm = (tachometer_rev*60000/(millis()-loop_last_time)); 
  loop_last_time = millis();
  tachometer_rev = 0;
  eng_temp_an_in = analogRead(ENG_TEMP_AN_IN_PIN);
  batt_volt_an_in = analogRead(BATT_VOLT_AN_IN_PIN); 
  EPU_data_str_loc.EPU_temp =  map(eng_temp_an_in, 0, 1024, 0, ENG_TEMP_COUNT2K);
  EPU_data_str_loc.EPU_batt_volt = map(batt_volt_an_in, 0, 1024, 0, BATT_VOLT_COUNT2K); 

  if(EPU_data_str_loc.EPU_rpm > 30)
   EPU_data_str_loc.EPU_eng_ON = HIGH;
  else
   EPU_data_str_loc.EPU_eng_ON = LOW;

  if(debug_out == HIGH)
   {
   delay(200);
   DEBUG_SS.print("Engine RPM:  ");
   DEBUG_SS.println(EPU_data_str_loc.EPU_rpm);
   DEBUG_SS.print("Engine temp. [°C]:  ");
   DEBUG_SS.println(EPU_data_str_loc.EPU_temp);
   DEBUG_SS.print("Battery voltage [V*100]:  ");
   DEBUG_SS.println(EPU_data_str_loc.EPU_batt_volt);
   }
  serial_send(); 
  attachInterrupt(digitalPinToInterrupt(RPM_IN_PIN), Tachometer, FALLING); 
  }
 }

void Tachometer()
 {
 tachometer_rev++;
 }

void serial_send()
 {
 uint16_t size_out = 0u;
 // This function takes the structure "dummy_struct_loc" as an input buffer, insert it in the "point to point" protocol and make the output available in the tx_buffer
 if (PPP_insert((const uint8_t*)&EPU_data_str_loc, (const uint16_t)sizeof(EPU_data_str_loc), (uint8_t*)tx_buffer, &size_out) == true)
  {
  Serial.write((uint8_t *)tx_buffer, size_out);
  }
 }
