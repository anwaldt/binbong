/*
 * Author: Anton Schmied (26.03.2018)
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
#include <cmath>
#include "particle_osc.h"               // the particle header includes all OSC relevant headers and cpps

#include <Adafruit_ADS1015.h>           // Analog-Digital-Converter ADS1015 on the I2C bus
#include <Adafruit_LSM9DS0.h>           // Accelerometer (+ Gyro + Mag +Temp) LSM 9DS0
#include <Adafruit_Simple_AHRS.h>
#include <Adafruit_Sensor.h>

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
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());

// Function to configure the sensors on the LSM9DS0 board.
// You don't need to change anything here, but have the option to select different
// range and gain values.
void configureLSM9DS0(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

sensors_event_t accel, mag, gyro, temp;

/* Capacitive Sensors -> new! */
#define fslpDriveLine1_DIG  D12
#define fslpDriveLine2_DIG  D13
#define fslpSenseLine_DIG   D14
#define fslpBotR0_DIG       D15

#define fslpDriveLine1_ANAL  A4               // unused
#define fslpDriveLine2_ANAL  A5
#define fslpSenseLine_ANAL   A6
#define fslpBotR0_ANAL       A7

int currentOctave = 2;

const int VELOCITYTHR = 400;
const int NOTETHR = 3000;
int noteCurrentMillis = 0;
int noteDiffMillis = 0;
int holdPitch = 0;

float const PI_F = 3.14159265F;


/************************************************************************************************************
 Setup Function for Wifi Connection and Module Initialization
************************************************************************************************************/
void setup() 
{
  //Initialize serial and wait for port to open:
  Serial.begin(9600);

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
  delay(2000);
  ads.begin();
  
  delay(2000);
  lsm.begin();
//  configureLSM9DS0();
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
  dirY_amnt = constrain(dirY_amnt, 0 ,127);
  
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

  /********************************************* Read Ribbon Sensor *****************************************/
  
  /*** Read Pressure from FSLP Strip ***/
  pinMode(fslpDriveLine1_DIG, OUTPUT);        // Configure fslpDriveLine1 as an output High
  digitalWrite(fslpDriveLine1_DIG, HIGH);

  pinMode(fslpBotR0_DIG, OUTPUT);             // Configure fslpBotR0 as output low
  digitalWrite(fslpBotR0_DIG, LOW);

  pinMode(fslpDriveLine2_ANAL, INPUT);
  int v1 = analogRead(fslpDriveLine2_ANAL);   // take ADC Reading from fslpDriveLine2

  pinMode(fslpSenseLine_ANAL, INPUT);  
  int v2 = analogRead(fslpSenseLine_ANAL);    // take ADC Reading from SenseLine

  int ribb_pressure;
  if (v1 == v2)
  {
    ribb_pressure = 32 * 1023;                     // Avoid dividing by zero, and return maximum reading.
  }
  else
  {
    ribb_pressure = 32 * v2 / (v1 - v2); 
  }

  /*** Read Pressure from FSLP Strip ***/
  int ribb_position = 0;
  if (ribb_pressure > 0)
  {
    // Step 1 - Clear the charge on the sensor.
    pinMode(fslpDriveLine1_DIG, OUTPUT);      // fslpDriveLine1
    digitalWrite(fslpDriveLine1_DIG, LOW);

    pinMode(fslpDriveLine2_DIG, OUTPUT);      // fslpDriveLine2
    digitalWrite(fslpDriveLine2_DIG, LOW);
    
    pinMode(fslpSenseLine_DIG, OUTPUT);       // fslpSenseLine     
    digitalWrite(fslpSenseLine_DIG, LOW);
    
    pinMode(fslpBotR0_DIG, OUTPUT);           // fslpBotR0
    digitalWrite(fslpBotR0_DIG, LOW);

    // Step 2 - Set up appropriate drive line voltages.
    digitalWrite(fslpDriveLine1_DIG, HIGH);         // Configute fslpDriveLine1 as output high
  
    pinMode(fslpSenseLine_ANAL, INPUT);            // Configure fslpSenseLine as an ADC input
    pinMode(fslpBotR0_ANAL, INPUT);            // Configure fslpBotR0 as a high impedance input

    // Step 3 - Wait for the voltage to stabilize 5 - 10 microseconds.
    delayMicroseconds(10);

    ribb_position = analogRead(fslpSenseLine_ANAL);

    /* 3 Octave Solution, so three sectors on the Ribbon*/
    if (ribb_position < 1365)
    {
       currentOctave = 1;
    }
    else if (ribb_position > 1365 && ribb_position < 2730)
    {
      currentOctave = 2;
    }
    else if (ribb_position > 2730)
    {
      currentOctave = 3;
    }
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
        notePitch = (holdPitch + 12 * currentOctave + 35);
        Serial.print("notepitch: ");
        Serial.println(notePitch);
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
//  sensors_vec_t orientation;
//
//  // Use the simple AHRS function to get the current orientation.
//  ahrs.getOrientation(&orientation);
//  pfefferBndl.add("/roll").add(orientation.roll);
//  pfefferBndl.add("/pitch").add(orientation.pitch);
//  pfefferBndl.add("/heading").add(orientation.heading);

  lsm.getEvent(&accel, &mag, &gyro, &temp);
  float roll_x = (float)atan2(accel.acceleration.y, accel.acceleration.z);
  float roll_y = (float)atan2(accel.acceleration.z, accel.acceleration.x);
  
  pfefferBndl.add("/roll_x").add(roll_x * 180 / PI_F);
  pfefferBndl.add("/roll_y").add(roll_y * 180 / PI_F);
  
  pfefferBndl.add("/mag_x").add(mag.magnetic.x);
  pfefferBndl.add("/mag_y").add(mag.magnetic.y);
  pfefferBndl.add("/mag_z").add(mag.magnetic.z);

  pfefferBndl.add("/accel_x").add(accel.acceleration.x);
  pfefferBndl.add("/accel_y").add(accel.acceleration.y);
  pfefferBndl.add("/accel_z").add(accel.acceleration.z);
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
