/*
Ribbon_testing
testing with 4.7k Ohm, 5.7k Ohm, 7.5k Ohm and 10k Ohm
*/

#if defined(ARDUINO)
SYSTEM_MODE(MANUAL);
#endif


#define fslpDriveLine1_DIG  D12
#define fslpDriveLine2_DIG  D13
#define fslpSenseLine_DIG   D14
#define fslpBotR0_DIG       D15

#define fslpDriveLine1_ANAL  A4               // unused
#define fslpDriveLine2_ANAL  A5
#define fslpSenseLine_ANAL   A6
#define fslpBotR0_ANAL       A7

void setup() 
{
  Serial.begin(9600);
  delay(5000);
}

void loop() 
{
  // read pressure from FSLP STRIP
  pinMode(fslpDriveLine1_DIG, OUTPUT);        // Configure fslpDriveLine1 as an output High
  digitalWrite(fslpDriveLine1_DIG, HIGH);

  pinMode(fslpBotR0_DIG, OUTPUT);             // Configure fslpBotR0 as output low
  digitalWrite(fslpBotR0_DIG, LOW);

  pinMode(fslpDriveLine2_ANAL, INPUT);
  int v1 = analogRead(fslpDriveLine2_ANAL);   // take ADC Reading from fslpDriveLine2
  Serial.print(v1);
  Serial.print('\t');

  pinMode(fslpSenseLine_ANAL, INPUT);  
  int v2 = analogRead(fslpSenseLine_ANAL);    // take ADC Reading from SenseLine
  Serial.print(v2);
  Serial.print('\t');

  int pressure;
  
  if (v1 == v2)
  {
    pressure = 32 * 1023;                     // Avoid dividing by zero, and return maximum reading.
  }
  else
  {
    pressure = 32 * v2 / (v1 - v2); 
  }
  
  Serial.print(pressure);
  Serial.print('\t');

  // get position
  int rib_position;
  if (pressure > 0)
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

    rib_position = analogRead(fslpSenseLine_ANAL);
  } 
  else
  {
    rib_position = 0;
  }

  Serial.print(rib_position);
  Serial.print('\n');

  delay(400);
}
