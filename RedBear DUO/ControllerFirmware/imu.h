#include <Adafruit_BNO055.h> // BN055 Absolute orientation board

#define BNO055_SAMPLERATE_DELAY_MS 10
Adafruit_BNO055 bno = Adafruit_BNO055(55); // create an instance for the accelerometer

/**************************************** Absolute Orientation ************************************/
sensors_event_t absolute_orientation()
{
    sensors_event_t event;
    bno.getEvent(&event);

    /* Display the floating point data */
    // Serial.print("X: ");
    // Serial.print(event.orientation.x, 4);
    // Serial.print("\tY: ");
    // Serial.print(event.orientation.y, 4);
    // Serial.print("\tZ: ");
    // Serial.print(event.orientation.z, 4);
    // Serial.print("\n");

    return event;
}

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
    delay(100);
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
    Serial.print("\n");
}

void prepare_imu_data()
{
    /* SEND CALIBRATION STATUS */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    msg = "/bong/" + IP + "/cal/sys";
    msg.toCharArray(copy, 50);
    imu_bndl.add(copy).add((float)system);

    msg = "/bong/" + IP + "/cal/gyro";
    msg.toCharArray(copy, 50);
    imu_bndl.add(copy).add((float)gyro);

    msg = "/bong/" + IP + "/cal/accel";
    msg.toCharArray(copy, 50);
    imu_bndl.add(copy).add((float)accel);

    msg = "/bong/" + IP + "/cal/mag";
    msg.toCharArray(copy, 50);
    imu_bndl.add(copy).add((float)mag);

    /* Optional: Display calibration status */
    //displayCalStatus();

    /* Optional: Display sensor status (debug only) */
    //displaySensorStatus();

    // get IMU data

    /* Wait the specified delay before requesting nex data */
    delay(BNO055_SAMPLERATE_DELAY_MS);
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
}

void setup_bno()
{
    Adafruit_BNO055::adafruit_bno055_opmode_t opmode = Adafruit_BNO055::adafruit_bno055_opmode_t::OPERATION_MODE_NDOF;
    if (!bno.begin(opmode))
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    }
    bno.setExtCrystalUse(true);
    /* Display some basic information on this sensor */
    displaySensorDetails();

    /* Optional: Display current status */
    displaySensorStatus();
}