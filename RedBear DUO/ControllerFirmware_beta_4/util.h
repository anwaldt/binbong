#include "cmath"
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <particle_osc.h> // the particle header includes all OSC relevant headers and cpps

const float NOTETHR = 0.1;
const int LOCALPORT = 8888; // port of the RedBEar DUO, which can receive the OSC Messages
const int HOSTPORT = 9999;  // port on the host, which the OSC messages will be sent to

int currentOctave = 2;
int deviceID = 0;
char copy[50];
String IP;
String msg;

OSCBundle fsr_bndl; // OSC bundle for reading the FSRs
OSCBundle imu_bndl; // OSC bundle for reading the imu
UDP udpConnection;  // UDP Instance

/************************************************************************************************************
   NETWORK STUFF
************************************************************************************************************/

// this will be the last octet of the IP address

// char ssid[] = "Yrkkö's iPhone"; // SSID of the network
// char password[] = "salasana";   // network password

// IPAddress hostIpAddress(172, 20, 10, 13); // IP Address of the host

char ssid[] = "Notlandung";         // SSID of the network
char password[] = "BorisBlacher66"; // network password

IPAddress hostIpAddress(10, 11, 1, 200); // IP Address of the host

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

void setupWifi()
{
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
}

void send_bundle()
{
  udpConnection.beginPacket(hostIpAddress, HOSTPORT);
  imu_bndl.send(udpConnection);
  udpConnection.endPacket();

  udpConnection.beginPacket(hostIpAddress, HOSTPORT);
  fsr_bndl.send(udpConnection);
  udpConnection.endPacket();

  fsr_bndl.empty();
  imu_bndl.empty();
}
