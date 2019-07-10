/*

  Authors: Henrik von Coler, Anton Schmied, Yrkkö Äkkijyrkkä
  Date: 2019-06-10

*/

#if defined(ARDUINO)
SYSTEM_MODE(SEMI_AUTOMATIC);
#endif

/************************************************************************************************************
   libraries
 ************************************************************************************************************/

#include "util.h"
#include "imu.h"
#include "valves.h"
#include "pad.h"
#include "ribbon.h"

/************************************************************************************************************
   variables

 ************************************************************************************************************/

//float const PI_F = 3.14159265F;

/************************************************************************************************************
  Setup Function for Wifi Connection and Module Initialization
************************************************************************************************************/
void setup()
{
  Serial.begin(9600);
  /****************************************** Establish WiFi Connection *************************************/
  setupWifi();
  delay(1000);
  /******************************************* Establish UDP Connection *************************************/
  udpConnection.begin(LOCALPORT);
  /**************************************** Initialize ads1015 and bno055 **********************************/
  delay(1000);
  /* Initialise the sensor */
  setup_bno();
  delay(1000);
  /**************************************** calibrate modules **********************************/
  calibrate_pad();
  delay(1000);
  calibrate_valves();
}

/************************************************************************************************************
   Main Loop Function
 ************************************************************************************************************/
void loop()
{
  msg = "/id/" + IP;
  msg.toCharArray(copy, 50);
  fsr_bndl.add(copy).add(deviceID);

  // Prepare all data
  prepare_ribbon_data();
  prepare_valve_data();
  prepare_imu_data();

  // Send data
  send_bundle();
}
