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

#include <cmath>
#include <Wire.h>
#include "application.h"
#include "particle_osc.h" // the particle header includes all OSC relevant headers and cpps
#include <Adafruit_Sensor.h>
#include <Adafruit_ADS1015.h> // Analog-Digital-Converter ADS1015 on the I2C bus
//#include <Adafruit_Simple_AHRS.h>       // IMU conversion Library to calculate roll//pitch//heading !!!includes <cmath>!!!
#include <Adafruit_BNO055.h> // BN055 Absolute orientation board
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

/************************************************************************************************************
   variables

 ************************************************************************************************************/
int currentOctave = 2;

const int VELOCITYTHR = 400;
const float NOTETHR = 0.1;
const int valveMAX = 4000;
const int padMAX = 1024;

int noteCurrentMillis = 0;
int noteDiffMillis = 0;
int holdPitch = 0;

float const PI_F = 3.14159265F;

/************************************************************************************************************
   NETWORK STUFF
************************************************************************************************************/

// this will be the last octet of the IP address
int deviceID = 0;
char copy[50];
String IP;

char ssid[] = "Wu-Tang LAN";        // SSID of the network
char password[] = "efortheenglish"; // network password

IPAddress hostIpAddress(192, 168, 2, 113); // IP Address of the host

UDP udpConnection;          // UDP Instance
const int LOCALPORT = 8888; // port of the RedBEar DUO, which can receive the OSC Messages
const int HOSTPORT = 9999;  // port on the host, which the OSC messages will be sent to

OSCBundle fsr_bndl; // OSC bundle for reading the FSRs
OSCBundle imu_bndl; // OSC bundle for reading the imu

/************************************************************************************************************
   SENSOR STUFF
************************************************************************************************************/

Adafruit_ADS1015 ads = Adafruit_ADS1015(); // create an instance for the ADC
Adafruit_BNO055 bno = Adafruit_BNO055(55); // create an instance for the accelerometer

void displaySensorDetails(void)
{
    sensor_t sensor;
    bno.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print("Sensor:       ");
    Serial.println(sensor.name);
    Serial.print("Driver Ver:   ");
    Serial.println(sensor.version);
    Serial.print("Unique ID:    ");
    Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    ");
    Serial.print(sensor.max_value);
    Serial.println(" xxx");
    Serial.print("Min Value:    ");
    Serial.print(sensor.min_value);
    Serial.println(" xxx");
    Serial.print("Resolution:   ");
    Serial.print(sensor.resolution);
    Serial.println(" xxx");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
}

void displaySensorStatus(void)
{
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);

    /* Display the results in the Serial Monitor */
    Serial.println("");
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");
    delay(500);
}

void displayCalStatus(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    Serial.print("\t");
    if (!system)
    {
        Serial.print("! ");
    }

    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);
}

/* Capacitive Sensors -> new! */
#define fslpDriveLine1_DIG D12
#define fslpDriveLine2_DIG D13
#define fslpSenseLine_DIG D14
#define fslpBotR0_DIG D15

#define fslpDriveLine1_ANAL A4 // unused
#define fslpDriveLine2_ANAL A5
#define fslpSenseLine_ANAL A6
#define fslpBotR0_ANAL A7

/******************************************** LOOP Variables *****************************************/

int velocity = 0;
int pressureSum = 0; //!< ACID
int dirX_amnt = 0;
int dirY_amnt = 0;
int dirZ_amnt = 0;

/* for FSR offsets: */
/* set once before main loop

*/
int valve_offset[4] = {0, 0, 0, 0};
int pad_offset[4] = {0, 0, 0, 0};

/************************************************************************************************************
  Setup Function for Wifi Connection and Module Initialization
************************************************************************************************************/

void setup()
{

    /****************************************** Establish WiFi Connection *************************************/

    WiFi.on();
    WiFi.setCredentials(ssid, password);
    WiFi.connect();

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
        deviceID = localIP[3];
    }

    IP = String(deviceID);

    printWiFiInfo(); // print WiFi Connection Info

    /******************************************* Establish UDP Connection *************************************/
    udpConnection.begin(LOCALPORT);

    /**************************************** Initialize ads1015 and bno055 **********************************/
    delay(2000);
    ads.begin();

    delay(2000);
    /* Initialise the sensor */
    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    }

    delay(1000);

    /* Display some basic information on this sensor */
    displaySensorDetails();

    /* Optional: Display current status */
    displaySensorStatus();

    bno.setExtCrystalUse(true);

    /**************************************** calibrate FSR **********************************/

    valve_offset[0] = analogRead(A3);
    valve_offset[1] = analogRead(A2);
    valve_offset[2] = analogRead(A1);
    valve_offset[3] = analogRead(A0);

    pad_offset[0] = ads.readADC_SingleEnded(0);
    pad_offset[1] = ads.readADC_SingleEnded(1);
    pad_offset[2] = ads.readADC_SingleEnded(2);
    pad_offset[3] = ads.readADC_SingleEnded(3);
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

    float fsrPressure_0 = (float)(ads.readADC_SingleEnded(0) - pad_offset[0]) / (float)(padMAX - pad_offset[0]);
    float fsrPressure_1 = (float)(ads.readADC_SingleEnded(1) - pad_offset[1]) / (float)(padMAX - pad_offset[1]);
    float fsrPressure_2 = (float)(ads.readADC_SingleEnded(2) - pad_offset[2]) / (float)(padMAX - pad_offset[2]);
    float fsrPressure_3 = (float)(ads.readADC_SingleEnded(3) - pad_offset[3]) / (float)(padMAX - pad_offset[3]);

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
    int fsrPressed = 0;
    int currentPitch = 0;
    pressureSum = 0;

    float fsrPressure = (float)(analogRead(A3) - valve_offset[0]) / (float)(valveMAX - valve_offset[0]);
    msg = "/bong/" + IP + "/valve/1";
    msg.toCharArray(copy, 50);
    fsr_bndl.add(copy).add(fsrPressure);

    if (fsrPressure > NOTETHR)
    {
        currentPitch |= 0x0001; // set first Bit to 1
        pressureSum += (fsrPressure - NOTETHR);
        fsrPressed++;
    }

    fsrPressure = (float)(analogRead(A2) - valve_offset[1]) / (float)(valveMAX - valve_offset[1]);
    msg = "/bong/" + IP + "/valve/2";
    msg.toCharArray(copy, 50);
    fsr_bndl.add(copy).add(fsrPressure);

    if (fsrPressure > NOTETHR)
    {
        currentPitch |= 0x0002; // set second Bit to 1
        pressureSum += (fsrPressure - NOTETHR);
        fsrPressed++;
    }

    fsrPressure = (float)(analogRead(A1) - valve_offset[2]) / (float)(valveMAX - valve_offset[2]);
    msg = "/bong/" + IP + "/valve/3";
    msg.toCharArray(copy, 50);
    fsr_bndl.add(copy).add(fsrPressure);

    if (fsrPressure > NOTETHR)
    {
        currentPitch |= 0x0004; // set third Bit to 1
        pressureSum += (fsrPressure - NOTETHR);
        fsrPressed++;
    }

    fsrPressure = (float)(analogRead(A0) - valve_offset[3]) / (float)(valveMAX - valve_offset[3]);
    msg = "/bong/" + IP + "/valve/4";
    msg.toCharArray(copy, 50);
    fsr_bndl.add(copy).add(fsrPressure);

    if (fsrPressure > NOTETHR)
    {
        currentPitch |= 0x0008; // set fourth Bit to 1
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
    pinMode(fslpDriveLine1_DIG, OUTPUT); // Configure fslpDriveLine1 as an output High
    digitalWrite(fslpDriveLine1_DIG, HIGH);

    pinMode(fslpBotR0_DIG, OUTPUT); // Configure fslpBotR0 as output low
    digitalWrite(fslpBotR0_DIG, LOW);

    pinMode(fslpDriveLine2_ANAL, INPUT);
    int v1 = analogRead(fslpDriveLine2_ANAL); // take ADC Reading from fslpDriveLine2

    pinMode(fslpSenseLine_ANAL, INPUT);
    int v2 = analogRead(fslpSenseLine_ANAL); // take ADC Reading from SenseLine

    int ribb_pressure;
    if (v1 == v2)
    {
        ribb_pressure = 32 * 1023; // Avoid dividing by zero, and return maximum reading.
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
        pinMode(fslpDriveLine1_DIG, OUTPUT); // fslpDriveLine1
        digitalWrite(fslpDriveLine1_DIG, LOW);

        pinMode(fslpDriveLine2_DIG, OUTPUT); // fslpDriveLine2
        digitalWrite(fslpDriveLine2_DIG, LOW);

        pinMode(fslpSenseLine_DIG, OUTPUT); // fslpSenseLine
        digitalWrite(fslpSenseLine_DIG, LOW);

        pinMode(fslpBotR0_DIG, OUTPUT); // fslpBotR0
        digitalWrite(fslpBotR0_DIG, LOW);

        // Step 2 - Set up appropriate drive line voltages.
        digitalWrite(fslpDriveLine1_DIG, HIGH); // Configute fslpDriveLine1 as output high

        pinMode(fslpSenseLine_ANAL, INPUT); // Configure fslpSenseLine as an ADC input
        pinMode(fslpBotR0_ANAL, INPUT);     // Configure fslpBotR0 as a high impedance input

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

    float intensity = (float)pressureSum / 900.0;

    msg = "/bong/" + IP + "/note";
    msg.toCharArray(copy, 50);
    fsr_bndl.add(copy).add(notePitch);

    msg = "/bong/" + IP + "/intensity";
    msg.toCharArray(copy, 50);
    //fsr_bndl.add(copy).add(intensity);

    /**************************************** IMU RAW ************************************/

    // get IMU data

    sensors_event_t event = absolute_orientation();

    msg = "/bong/" + IP + "/orientation/roll";
    msg.toCharArray(copy, 50);
    imu_bndl.add(copy).add(event.orientation.y);

    msg = "/bong/" + IP + "/orientation/pitch";
    msg.toCharArray(copy, 50);
    imu_bndl.add(copy).add(event.orientation.z);

    msg = "/bong/" + IP + "/orientation/heading";
    msg.toCharArray(copy, 50);
    imu_bndl.add(copy).add(event.orientation.x - 180);

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
    Serial.print("SSID: "); // print the SSID of the network you're attached to
    Serial.println(WiFi.SSID());

    long rssi = WiFi.RSSI(); // print the received signal strength
    Serial.print("signal strength (RSSI):");
    Serial.println(rssi);

    Serial.print("Encryption Type:"); // print the encryption type
    Serial.println(WPA, HEX);
    Serial.println();

    Serial.print("DUO IP Address: "); // print your WiFi IP address
    Serial.println(WiFi.localIP());

    byte mac[6]; // print your MAC address:
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

    Serial.print("Sending to IP Address: "); // print host IP address:
    Serial.println(hostIpAddress);
}

/**************************************** Absolute Orientation ************************************/
sensors_event_t absolute_orientation()
{
    sensors_event_t event;
    bno.getEvent(&event);

    /* Display the floating point data */
    Serial.print("X: ");
    Serial.print(event.orientation.x, 4);
    Serial.print("\tY: ");
    Serial.print(event.orientation.y, 4);
    Serial.print("\tZ: ");
    Serial.print(event.orientation.z, 4);

    /* Optional: Display calibration status */
    displayCalStatus();

    /* Optional: Display sensor status (debug only) */
    //displaySensorStatus();

    /* New line for the next sample */
    Serial.println("");

    /* Wait the specified delay before requesting nex data */
    delay(BNO055_SAMPLERATE_DELAY_MS);
    return event;
}
