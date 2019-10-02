/* Capacitive Sensors -> new! */

#define fslpDriveLine1_DIG D12
#define fslpDriveLine2_DIG D13
#define fslpSenseLine_DIG D14
#define fslpBotR0_DIG D15
#define fslpDriveLine1_ANAL A4 // unused
#define fslpDriveLine2_ANAL A5
#define fslpSenseLine_ANAL A6
#define fslpBotR0_ANAL A7

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
}