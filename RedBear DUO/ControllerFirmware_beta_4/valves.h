
const int valveMAX = 4000;

int pressureSum = 0; //!< ACID
int valve_offset[4] = {0, 0, 0, 0};
int noteCurrentMillis = 0;
int noteDiffMillis = 0;
int holdPitch = 0;

void calibrate_valves()
{
    valve_offset[0] = analogRead(A3);
    valve_offset[1] = analogRead(A2);
    valve_offset[2] = analogRead(A1);
    valve_offset[3] = analogRead(A0);
}

void prepare_valve_data()
{
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
}