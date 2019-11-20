/* Capacitive Sensors -> new! */
#include <CircularBuffer.h>

#define fslpDriveLine1_DIG D12
#define fslpDriveLine2_DIG D13
#define fslpSenseLine_DIG D14
#define fslpBotR0_DIG D15
#define fslpDriveLine1_ANAL A4 // unused
#define fslpDriveLine2_ANAL A5
#define fslpSenseLine_ANAL A6
#define fslpBotR0_ANAL A7
#define DELTA_LIMIT 300       // Amount of movement required to be added into the buffer
#define OCTAVE_THRESHOLD 1500 // Amount of total movement within sampling period to change octave
#define COOLDOWN_TIME 500     // Time in ms until octave can be changed again
#define MAX_OCTAVE 5          // Maximum + or - octave shift from zero

int currentOctave = 0;
int currentDirection = 0;
int previousPos = 0;
unsigned long lastSet = millis();
CircularBuffer<int, 10> buffer;

void prepare_ribbon_data()
{
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
    int ribbonPosition = 0;
    int deltaRibbon = 0;

    /*** Read Pressure from FSLP Strip ***/
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

        ribbonPosition = analogRead(fslpSenseLine_ANAL);
        if (previousPos != 0) // Don't read the first sample after a finger is first placed on the ribbon to avoid value jumps
        {
            deltaRibbon = abs(previousPos) - abs(ribbonPosition);
            if (abs(deltaRibbon) > DELTA_LIMIT)
            {
                buffer.push(previousPos - ribbonPosition);
                for (int i = 0; i < buffer.size(); i++)
                {
                    currentDirection += buffer[i]; // Sum the movement vectors
                }
                if (currentDirection > OCTAVE_THRESHOLD && millis() - lastSet >= COOLDOWN_TIME && currentOctave < MAX_OCTAVE)
                {
                    currentOctave++;
                    currentDirection = 0;
                    previousPos = 0;
                    buffer.clear();
                    lastSet = millis();
                }
                else if (currentDirection < -OCTAVE_THRESHOLD && millis() - lastSet >= COOLDOWN_TIME && -currentOctave < MAX_OCTAVE)
                {
                    currentOctave--;
                    previousPos = 0;
                    currentDirection = 0;
                    buffer.clear();
                    lastSet = millis();
                }
            }
            else if (buffer.size() > 0)
            {
                buffer.shift(); // Dump oldest values from buffer once per cycle if they are not used eg. finger is not moving.
            }
            //Serial.printf("buf_siz:%d, pre_pos:%d, rib_pos:%d, del_rib:%d, cur_dir:%d\n", buffer.size(), previousPos, ribbonPosition, deltaRibbon, currentDirection);
        }
        previousPos = ribbonPosition;
        msg = "/bong/" + IP + "/ribbon/octave";
        msg.toCharArray(copy, 50);
        fsr_bndl.add(copy).add(currentOctave);

        msg = "/bong/" + IP + "/ribbon/position";
        msg.toCharArray(copy, 50);
        fsr_bndl.add(copy).add(ribbonPosition);

        msg = "/bong/" + IP + "/ribbon/pressure";
        msg.toCharArray(copy, 50);
        fsr_bndl.add(copy).add(ribb_pressure);
    }
    else
    {
        previousPos = 0;
        currentDirection = 0;
        buffer.clear();
    }
}