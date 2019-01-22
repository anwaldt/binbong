/*
  
  Authors: Henrik von Coler, Anton Schmied
  Date: 2018-11-05
   
*/

#if defined(ARDUINO)
SYSTEM_MODE(SEMI_AUTOMATIC);
#endif

/************************************************************************************************************
   libraries
 ************************************************************************************************************/
 
#include <cmath>
#include "particle_osc.h"               // the particle header includes all OSC relevant headers and cpps
#include <Adafruit_Sensor.h>
#include <Adafruit_ADS1015.h>           // Analog-Digital-Converter ADS1015 on the I2C bus
#include <Adafruit_LSM9DS0.h>           // Accelerometer (+ Gyro + Mag +Temp) LSM 9DS0
//#include <Adafruit_Simple_AHRS.h>       // IMU conversion Library to calculate roll//pitch//heading !!!includes <cmath>!!!

//#include "application.h"      

/************************************************************************************************************
   variables
   
 ************************************************************************************************************/
int currentOctave = 2;

const int VELOCITYTHR = 400;
const float NOTETHR   = 0.1;
const int valveMAX    = 4000;
const int padMAX      = 1024;

int noteCurrentMillis = 0;
int noteDiffMillis    = 0;
int holdPitch         = 0;

float const PI_F = 3.14159265F;





/************************************************************************************************************
   NETWORK STUFF
************************************************************************************************************/


// this will be the last octet of the IP address
int deviceID = 0;
char copy[50];
String IP;



char ssid[]     = "BINBONG_NET";           // SSID of the network
char password[] = "03396025";         // network password

IPAddress hostIpAddress(10,10,10,100);                // IP Address of the host

UDP udpConnection;                       // UDP Instance
const int LOCALPORT = 8888;              // port of the RedBEar DUO, which can receive the OSC Messages
const int HOSTPORT  = 9999;              // port on the host, which the OSC messages will be sent to

OSCBundle fsr_bndl;                  // OSC bundle for reading the FSRs
OSCBundle imu_bndl;                  // OSC bundle for reading the imu



/************************************************************************************************************
   SENSOR STUFF
************************************************************************************************************/

Adafruit_ADS1015 ads = Adafruit_ADS1015();    // create an instance for the ADC
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();    // create an instance for the accelerometer
 


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



/******************************************** LOOP Variables *****************************************/

  int velocity    = 0;
  int pressureSum = 0;  //!< ACID
  int dirX_amnt   = 0;
  int dirY_amnt   = 0;
  int dirZ_amnt   = 0;



/* for FSR offsets: */
/* set once before main loop 
 *  
 */
int valve_offset[4] = {0, 0, 0, 0};
int pad_offset[4]   = {0, 0, 0, 0};

/************************************************************************************************************
 Setup Function for Wifi Connection and Module Initialization
************************************************************************************************************/

void setup() 
{
 
   
  // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
//  Serial.print("MAC address: ");
//  Serial.print(mac[5], HEX);
//  Serial.print(":");
//  Serial.print(mac[4], HEX);
//  Serial.print(":");
//  Serial.print(mac[3], HEX);
//  Serial.print(":");
//  Serial.print(mac[2], HEX);
//  Serial.print(":");
//  Serial.print(mac[1], HEX);
//  Serial.print(":");
//  Serial.println(mac[0], HEX);
// 
//  //Initialize serial and wait for port to open:
//  Serial.begin(9600);

  /****************************************** Establish WiFi Connection *************************************/
  
  WiFi.on();
  WiFi.setCredentials(ssid, password);
  WiFi.connect();
  
//  while (!WiFi.ready())
//  {
//    Serial.print("Connecting ... \n");
//    delay(300);
//  }
  
//  if (WiFi.ready())
//  {
//    Serial.println("Connection to host established! \n\n");
//  }
  
  // get IP Address
  IPAddress localIP = WiFi.localIP();
  while (localIP[0] == 0)
  {
    localIP = WiFi.localIP();
   // Serial.println("Waiting for an IP address ...");
    delay(1000);
  }
  if (localIP[0] != 0)
  {
    // Serial.println("IP Address obtained! \n\n");
    deviceID = localIP[3];
  }


  IP = String(deviceID);



//  hostIpAddress = '192.168.1.187'; 
 // hostIpAddress = WiFi.gatewayIP();
  printWiFiInfo();                                      // print WiFi Connection Info
  

  /******************************************* Establish UDP Connection *************************************/
  udpConnection.begin(LOCALPORT);


  
  /**************************************** Initialize lsm9ds0 and ads1015 **********************************/
  delay(2000);
  ads.begin();
  
  delay(2000);
  lsm.begin();
//  configureLSM9DS0();

  /**************************************** calibrate FSR **********************************/

  
  valve_offset[0] = analogRead(A3);
  valve_offset[1] = analogRead(A2);
  valve_offset[2] = analogRead(A1);
  valve_offset[3] = analogRead(A0);
  
  
  pad_offset[0]  = ads.readADC_SingleEnded(0);
  pad_offset[1]  = ads.readADC_SingleEnded(1);
  pad_offset[2]  = ads.readADC_SingleEnded(2);
  pad_offset[3]  = ads.readADC_SingleEnded(3);


}



/************************************************************************************************************
   Main Loop Function
 ************************************************************************************************************/
void loop() 
{


  String msg = "/id/" + IP;
  msg.toCharArray(copy, 50);
  
  fsr_bndl.add(copy).add(deviceID);
                    
  /****************************************** Get Continous Velocity ***************************************/
  /************************************* and Excitation Button Directions **********************************/
  
  float fsrPressure_0 = (float) (ads.readADC_SingleEnded(0)-pad_offset[0]) / (float) (padMAX - pad_offset[0] );
  float fsrPressure_1 = (float) (ads.readADC_SingleEnded(1)-pad_offset[1]) / (float) (padMAX - pad_offset[1] );
  float fsrPressure_2 = (float) (ads.readADC_SingleEnded(2)-pad_offset[2]) / (float) (padMAX - pad_offset[2] );
  float fsrPressure_3 = (float) (ads.readADC_SingleEnded(3)-pad_offset[3]) / (float) (padMAX - pad_offset[3] );

  msg = "/bong/" + IP + "/pad/1";
  msg.toCharArray(copy, 50);
  fsr_bndl.add(copy).add(fsrPressure_0);

  msg = "/bong/" + IP + "/pad/2";
  msg.toCharArray(copy, 50);
  fsr_bndl.add(copy).add(fsrPressure_1);

  msg = "/bong/" + IP + "/pad/3";
  msg.toCharArray(copy, 50);
  fsr_bndl.add(copy).add(fsrPressure_2);
  
  msg = "/bong/" + IP + "/pad/4";
  msg.toCharArray(copy, 50);
  fsr_bndl.add(copy).add(fsrPressure_3);

  
  /*
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
  fsr_bndl.add("/velocity").add(velocity);

  fsr_bndl.add("/dirX_push").add(dirX_amnt);
  fsr_bndl.add("/dirY_push").add(dirY_amnt);
 
   fsr_bndl.add("/dirX_push").add(dirX_amnt);
  
  */
  
  /********************************************** Read Pitch Sensors ****************************************/
  int fsrPressed    = 0;
  int currentPitch  = 0;
  pressureSum       = 0;

  float  fsrPressure = (float) (analogRead(A3) -valve_offset[0]) / (float) (valveMAX -valve_offset[0] );     
  msg = "/bong/" + IP + "/valve/1";
  msg.toCharArray(copy, 50);
  fsr_bndl.add(copy).add(fsrPressure);

  
  if (fsrPressure > NOTETHR)
  {
    currentPitch |= 0x0001;                                   // set first Bit to 1
    pressureSum += (fsrPressure - NOTETHR);
    fsrPressed++;
  }

     fsrPressure = (float) (analogRead(A2) -valve_offset[1]) / (float) (valveMAX -valve_offset[1] );     
   msg = "/bong/" + IP + "/valve/2";
  msg.toCharArray(copy, 50);
  fsr_bndl.add(copy).add(fsrPressure);

  if (fsrPressure > NOTETHR)
  {
    currentPitch |= 0x0002;                                   // set second Bit to 1
    pressureSum += (fsrPressure - NOTETHR);
    fsrPressed++;
  }

     fsrPressure = (float) (analogRead(A1) -valve_offset[2]) / (float) (valveMAX -valve_offset[2] );     
   msg = "/bong/" + IP + "/valve/3";
  msg.toCharArray(copy, 50);
  fsr_bndl.add(copy).add(fsrPressure);
  
  if (fsrPressure > NOTETHR)
  {
    currentPitch |= 0x0004;                                   // set third Bit to 1
    pressureSum += (fsrPressure - NOTETHR);
    fsrPressed++;
  }

     fsrPressure = (float) (analogRead(A0) -valve_offset[3]) / (float) (valveMAX -valve_offset[3] );     
   msg = "/bong/" + IP + "/valve/4";
  msg.toCharArray(copy, 50);
  fsr_bndl.add(copy).add(fsrPressure);

  if (fsrPressure > NOTETHR)
  {
    currentPitch |= 0x0008;                                   // set fourth Bit to 1
    pressureSum += (fsrPressure - NOTETHR);
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

 msg = "/bong/" + IP + "/ribbon/position";
  msg.toCharArray(copy, 50);
     fsr_bndl.add(copy).add(ribb_position);

      msg = "/bong/" + IP + "/ribbon/pressure";
  msg.toCharArray(copy, 50);
     fsr_bndl.add(copy).add(ribb_pressure);

  } 

  /***************************************** Calculate MIDI Pitch Value *************************************/
  int notePitch = 0;
  
  if (fsrPressed > 0)
  {
    pressureSum = pressureSum / fsrPressed;
    //pressureSum = map(pressureSum, 0, 4095-NOTETHR, 0, 1);
    //pressureSum = constrain(pressureSum, 0 ,1);

     
    if (noteDiffMillis > 5)
    {
        notePitch = (holdPitch + 12 * currentOctave + 35);
        //Serial.print("notepitch: ");
        //Serial.println(notePitch);
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

  float intensity = (float) pressureSum / 900.0;

  msg = "/bong/" + IP + "/note";
  msg.toCharArray(copy, 50);
  fsr_bndl.add(copy).add(notePitch);

  msg = "/bong/" + IP + "/intensity";
  msg.toCharArray(copy, 50);
  //fsr_bndl.add(copy).add(intensity);


  /**************************************** IMU RAW ************************************/
//  sensors_vec_t orientation;
//
//  // Use the simple AHRS function to get the current orientation.
//  ahrs.getOrientation(&orientation);
//  fsr_bndl.add("/roll").add(orientation.roll);
//  fsr_bndl.add("/pitch").add(orientation.pitch);
//  fsr_bndl.add("/heading").add(orientation.heading);

 
  // get raw imu sensor data
  lsm.getEvent(&accel, &mag, &gyro, &temp);
  
//  fsr_bndl.add("/roll_x").add(roll_x * 180 / PI_F);
//  fsr_bndl.add("/roll_y").add(roll_y * 180 / PI_F);

  msg = "/bong/" + IP + "/gyro/x";
  msg.toCharArray(copy, 50);
  imu_bndl.add(copy).add(gyro.gyro.x);

  msg = "/bong/" + IP + "/gyro/y";
  msg.toCharArray(copy, 50);
  imu_bndl.add(copy).add(gyro.gyro.y);
  
  msg = "/bong/" + IP + "/gyro/z";
  msg.toCharArray(copy, 50);
  imu_bndl.add(copy).add(gyro.gyro.z);
  
  msg = "/bong/" + IP + "/mag/x";
  msg.toCharArray(copy, 50);
  imu_bndl.add(copy).add(mag.magnetic.x);

  msg = "/bong/" + IP + "/mag/y";
  msg.toCharArray(copy, 50);
  imu_bndl.add(copy).add(mag.magnetic.y);

  msg = "/bong/" + IP + "/mag/z";
  msg.toCharArray(copy, 50);
  imu_bndl.add(copy).add(mag.magnetic.z);

  msg = "/bong/" + IP + "/accel/x";
  msg.toCharArray(copy, 50);
  imu_bndl.add(copy).add(accel.acceleration.x);

  msg = "/bong/" + IP + "/accel/y";
  msg.toCharArray(copy, 50);
  imu_bndl.add(copy).add(accel.acceleration.y);

  msg = "/bong/" + IP + "/accel/z";
  msg.toCharArray(copy, 50);
  imu_bndl.add(copy).add(accel.acceleration.z);


  /**************************************** Absolute Orientation ************************************/
  // extracted from 'Adafruit_Simple_AHRS.cpp'


  sensors_vec_t orientation;


  // roll: Rotation around the X-axis. -180 <= roll <= 180                                          
  // a positive roll angle is defined to be a clockwise rotation about the positive X-axis          
  //                                                                                                
  //                    y                                                                           
  //      roll = atan2(---)                                                                         
  //                    z                                                                           
  //                                                                                                
  // where:  y, z are returned value from accelerometer sensor    
  orientation.roll = (float)atan2(accel.acceleration.y, accel.acceleration.z);


  // pitch: Rotation around the Y-axis. -180 <= roll <= 180                                         
  // a positive pitch angle is defined to be a clockwise rotation about the positive Y-axis         
  //                                                                                                
  //                                 -x                                                             
  //      pitch = atan(-------------------------------)                                             
  //                    y * sin(roll) + z * cos(roll)                                               
  //                                                                                                
  // where:  x, y, z are returned value from accelerometer sensor 
  if (accel.acceleration.y * sin(orientation.roll) + accel.acceleration.z * cos(orientation.roll) == 0)
    orientation.pitch = accel.acceleration.x > 0 ? (PI_F / 2) : (-PI_F / 2);
  else
    orientation.pitch = (float)atan(-accel.acceleration.x / (accel.acceleration.y * sin(orientation.roll) + \
    accel.acceleration.z * cos(orientation.roll)));

  // heading: Rotation around the Z-axis. -180 <= roll <= 180                                       
  // a positive heading angle is defined to be a clockwise rotation about the positive Z-axis       
  //                                                                                                
  //                                       z * sin(roll) - y * cos(roll)                            
  //   heading = atan2(--------------------------------------------------------------------------)  
  //                    x * cos(pitch) + y * sin(pitch) * sin(roll) + z * sin(pitch) * cos(roll))   
  //                                                                                                
  // where:  x, y, z are returned value from magnetometer sensor 
  orientation.heading = (float)atan2(mag.magnetic.z * sin(orientation.roll) - mag.magnetic.y * cos(orientation.roll), \
                                      mag.magnetic.x * cos(orientation.pitch) + \
                                      mag.magnetic.y * sin(orientation.pitch) * sin(orientation.roll) + \
mag.magnetic.z * sin(orientation.pitch) * cos(orientation.roll));




  // Convert angular data to degree 
  orientation.roll = orientation.roll * 180 / PI_F;
  orientation.pitch = orientation.pitch * 180 / PI_F;
orientation.heading = orientation.heading * 180 / PI_F;



  msg = "/bong/" + IP + "/orientation/roll";
  msg.toCharArray(copy, 50);
  imu_bndl.add(copy).add(orientation.roll);

  msg = "/bong/" + IP + "/orientation/pitch";
  msg.toCharArray(copy, 50);
  imu_bndl.add(copy).add(orientation.pitch);

  msg = "/bong/" + IP + "/orientation/heading";
  msg.toCharArray(copy, 50);
  imu_bndl.add(copy).add(orientation.heading);


  

  
  /************************************************ Send Bundle *******************************************/

  udpConnection.beginPacket(hostIpAddress, HOSTPORT);
  imu_bndl.send(udpConnection);
  udpConnection.endPacket();

  udpConnection.beginPacket(hostIpAddress, HOSTPORT);
  fsr_bndl.send(udpConnection);
  udpConnection.endPacket();

  
  fsr_bndl.empty();
  imu_bndl.empty();
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

  Serial.print("Sending to IP Address: ");              // print host IP address:
  Serial.println(hostIpAddress);
}
