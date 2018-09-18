 

/*
 * Author: Anton Schmied (24.01.2018)
 * 
  This is the WiFi connection setup and comunnication
  between the RedBear Duo and the A Wifi Dongle based
  on the 'ConnectWithWPA' example last modified by
  Jackson Lv (1 DEC 2015).

  TODOs:
  - Hidden network!?!?!?
*/

/*
   SYSTEM_MODE:
       - AUTOMATIC: Automatically try to connect to Wi-Fi and the Particle Cloud and handle the cloud messages.
       - SEMI_AUTOMATIC: Manually connect to Wi-Fi and the Particle Cloud, but automatically handle the cloud messages.
       - MANUAL: Manually connect to Wi-Fi and the Particle Cloud and handle the cloud messages.

   SYSTEM_MODE(AUTOMATIC) does not need to be called, because it is the default state.
   However the user can invoke this method to make the mode explicit.
   Learn more about system modes: https://docs.particle.io/reference/firmware/photon/#system-modes .
*/

#if defined(ARDUINO)
SYSTEM_MODE(SEMI_AUTOMATIC);
#endif

/************************************************************************************************************
   libraries
 ************************************************************************************************************/
#include "particle_osc.h"               // the particle header includes all OSC relevant headers and cpps

#include <Adafruit_ADS1015.h>           // Analog-Digital-Converter ADS1015 on the I2C bus
#include <Adafruit_LSM9DS0.h>           // Accelerometer (+ Gyro + Mag +Temp) LSM 9DS0

#include "captouch.h"                   // Capacitive Sensors
#include "application.h"      


/************************************************************************************************************
   global variables
 ************************************************************************************************************/
char ssid[] = "pfeffer_host";           // SSID of the network
char password[] = "pfeffer123";         // network password
IPAddress hostIpAddress;                // IP Address of the host, obtaind in setup()

UDP udpConnection;                      // UDP Instance
const int LOCALPORT = 8888;              // port of the RedBEar DUO, which can receive the OSC Messages
const int HOSTPORT = 9999;               // port on the host, which the OSC messages will be sent to

OSCBundle pfefferBndl;                  // OSC bundle for reading the piezos for octave selection

Adafruit_ADS1015 ads = Adafruit_ADS1015();    // create an instance for the ADC
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();    // create an instance for the accelerometer
sensors_event_t accel, mag, gyro, temp;

#define pSensorPin_1 D5
#define pSensorPin_2 D6
#define pSensorPin_3 D7

#define pDriverPin_1 D2
#define pDriverPin_2 D3
#define pDriverPin_3 D4

CapTouch Touch[3] = {CapTouch(pSensorPin_1, pDriverPin_1), CapTouch(pSensorPin_2, pDriverPin_2), CapTouch(pSensorPin_3, pDriverPin_3)};
CapTouch::Event event;

int octLayout[7] = {0,0,2,1,4,0,3};      // Ocave Selection Array Layout
int activeOctSense = 2;
bool changeOctave = false;
int sensor_1 = 0x0000;
int sensor_2 = 0x0000;
int octCurrentMillis = 0;
int octDiffMillis = 0;

const int VELOCITYTHR = 400;
const int NOTETHR = 3000;
int noteCurrentMillis = 0;
int noteDiffMillis = 0;
int holdPitch = 0;


/************************************************************************************************************
 Setup Function for Wifi Connection and Module Initialization
************************************************************************************************************/
void setup() 
{
  //Initialize serial and wait for port to open:
  Serial.begin(115200);

  /****************************************** Establish WiFi Connection *************************************/
  WiFi.on();
  WiFi.setCredentials(ssid, password);
  WiFi.connect();
  while (!WiFi.ready())
  {
    Serial.print("Connecting ... \n");
    delay(300);
  }
  if (WiFi.ready())
  {
    Serial.println("Connection to pfeffer_host established! \n\n");
  }
  
  // get IP Address
  IPAddress localIP = WiFi.localIP();
  while (localIP[0] == 0)
  {
    localIP = WiFi.localIP();
    Serial.println("Waiting for an IP address ...");
    delay(1000);
  }
  if (localIP[0] != 0)
  {
    Serial.println("IP Address obtained! \n\n");
  }

  hostIpAddress = WiFi.gatewayIP();
  printWiFiInfo();                                      // print WiFi Connection Info
  

  /******************************************* Establish UDP Connection *************************************/
  udpConnection.begin(LOCALPORT);

  /**************************************** Initialize lsm9ds0 and ads1015 **********************************/
  lsm.begin();
  ads.begin();

  
  /***************************************** Setup the Capacitve Sensors ************************************/
  for (uint32_t ind = 0; ind < 3; ind ++)
  {
    Touch[ind].setup(); 
  }
}



/************************************************************************************************************
   Main Loop Function
 ************************************************************************************************************/
void loop() 
{
  /******************************************** Temporal Variables *****************************************/
  int velocity = 0;
  int pressureSum = 0;
  int dirX_amnt = 0;
  int dirY_amnt = 0;
  int dirZ_amnt = 0;

                         
  /****************************************** Get Continous Velocity ***************************************/
  /************************************* and Excitation Button Directions **********************************/
  
  int fsrPressure_pos = ads.readADC_SingleEnded(0);
  int fsrPressure_neg = ads.readADC_SingleEnded(2);
  
  dirX_amnt = map(fsrPressure_pos - fsrPressure_neg, -1023, 1023, 0, 127);
  dirX_amnt = constrain(dirX_amnt, 0 ,127);
  
  pressureSum = fsrPressure_pos + fsrPressure_neg;

  fsrPressure_pos = ads.readADC_SingleEnded(1);
  fsrPressure_neg = ads.readADC_SingleEnded(3);

  dirY_amnt = map(fsrPressure_pos - fsrPressure_neg, -1023, 1023, 0, 127);
  dirX_amnt = constrain(dirY_amnt, 0 ,127);
  
  pressureSum = pressureSum + fsrPressure_pos + fsrPressure_neg;
  pressureSum = int(pressureSum) / 4;
  
  velocity = map(pressureSum, VELOCITYTHR, 975, 0, 127);
  velocity = constrain(velocity, 0, 127);
  pfefferBndl.add("/velocity").add(velocity);

  pfefferBndl.add("/dirX_push").add(dirX_amnt);
  pfefferBndl.add("/dirY_push").add(dirY_amnt);


  /********************************************** Read Pitch Sensors ****************************************/
  int fsrPressed = 0;
  int currentPitch = 0;
  pressureSum = 0;

  fsrPressure_pos = analogRead(A3);
  if (fsrPressure_pos > NOTETHR)
  {
    currentPitch |= 0x0001;                                   // set first Bit to 1
    pressureSum += (fsrPressure_pos - NOTETHR);
    fsrPressed++;
  }

  fsrPressure_pos = analogRead(A2);
  if (fsrPressure_pos > NOTETHR)
  {
    currentPitch |= 0x0002;                                   // set second Bit to 1
    pressureSum += (fsrPressure_pos - NOTETHR);
    fsrPressed++;
  }

  fsrPressure_pos = analogRead(A1);
  if (fsrPressure_pos > NOTETHR)
  {
    currentPitch |= 0x0004;                                   // set third Bit to 1
    pressureSum += (fsrPressure_pos - NOTETHR);
    fsrPressed++;
  }

  fsrPressure_pos = analogRead(A0);
  if (fsrPressure_pos > NOTETHR)
  {
    currentPitch |= 0x0008;                                   // set fourth Bit to 1
    pressureSum += (fsrPressure_pos - NOTETHR);
    fsrPressed++;
  }

  if (currentPitch != holdPitch)
  {
    noteCurrentMillis = millis();
    noteDiffMillis = 0;
    holdPitch = currentPitch;
  }

  /********************************************* Read Ocatve Sensors ****************************************/
  event = Touch[0].getEvent();
  event = Touch[1].getEvent();
  event = Touch[2].getEvent();

  if (!changeOctave)
  {
    for (int ind = 0; ind < 3; ind++)
    {
      sensor_1 |= (Touch[ind].getState() << ind);
    }
    if (sensor_1 > 0)
    {
      changeOctave = true;
      octCurrentMillis = millis();
      octDiffMillis = 0;
    }
  }
  else if (changeOctave && octDiffMillis < 5)
  {
    for (int ind = 0; ind < 3; ind++)
    {
      sensor_2 |= (Touch[ind].getState() << ind);
    }
    octDiffMillis = millis() - octCurrentMillis;
  }
  else if (changeOctave && octDiffMillis > 5)
  {
    activeOctSense = sensor_1 | sensor_2;
    sensor_1 = 0x0000;
    sensor_2 = 0x0000;
    changeOctave = false;
  }

  /***************************************** Calculate MIDI Pitch Value *************************************/
  int notePitch = 0;
  
  if (fsrPressed > 0)
  {
    pressureSum = pressureSum / fsrPressed;
    pressureSum = map(pressureSum, 0, 4095-NOTETHR, 0, 127);
    pressureSum = constrain(pressureSum, 0 ,127);
    
    if (noteDiffMillis > 5)
    {
      notePitch = (holdPitch + 12 * octLayout[activeOctSense] + 35);
    }
    else if (noteDiffMillis < 5)
    {
      noteDiffMillis = millis() - noteCurrentMillis;
    }
  }
  else 
  {
    notePitch = 0;
    pressureSum = 0;
    holdPitch = 0;
  }

  pfefferBndl.add("/note_pitch").add(notePitch);
  pfefferBndl.add("/note_pressure").add(pressureSum);


  /**************************************** Get Tilt Direction Amounts ************************************/
  lsm.getEvent(&accel, &mag, &gyro, &temp);
  dirX_amnt = int(accel.acceleration.x);
  dirY_amnt = int(accel.acceleration.y);
  dirZ_amnt = int(accel.acceleration.z);

  pfefferBndl.add("/dirX_tilt").add(dirX_amnt);
  pfefferBndl.add("/dirY_tilt").add(dirY_amnt);
  pfefferBndl.add("/dirZ_tilt").add(dirZ_amnt);


  /************************************************ Send Bundle *******************************************/
  udpConnection.beginPacket(hostIpAddress, HOSTPORT);
  pfefferBndl.send(udpConnection);
  udpConnection.endPacket();
  pfefferBndl.empty();
}



/************************************************************************************************************
   Once the RedBear Duo is successfully connected to the WiFi Host
   this Function should provide all Information about the Connection
 ************************************************************************************************************/
void printWiFiInfo()
{
  Serial.print("SSID: ");                       // print the SSID of the network you're attached to
  Serial.println(WiFi.SSID());

  long rssi = WiFi.RSSI();                      // print the received signal strength
  Serial.print("signal strength (RSSI):");
  Serial.println(rssi);

  Serial.print("Encryption Type:");             // print the encryption type
  Serial.println(WPA, HEX);
  Serial.println();

  Serial.print("DUO IP Address: ");             // print your WiFi IP address
  Serial.println(WiFi.localIP());

  byte mac[6];                                  // print your MAC address:
  WiFi.macAddress(mac);
  Serial.print("DUO MAC address: ");
  Serial.print(mac[5], HEX);
  Serial.print(":");
  Serial.print(mac[4], HEX);
  Serial.print(":");
  Serial.print(mac[3], HEX);
  Serial.print(":");
  Serial.print(mac[2], HEX);
  Serial.print(":");
  Serial.print(mac[1], HEX);
  Serial.print(":");
  Serial.println(mac[0], HEX);

  Serial.print("HOST IP Address: ");              // print host IP address:
  Serial.println(WiFi.gatewayIP());
}
