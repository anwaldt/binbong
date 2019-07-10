#include <Adafruit_ADS1015.h> // Analog-Digital-Converter ADS1015 on the I2C bus

Adafruit_ADS1015 ads = Adafruit_ADS1015(); // create an instance for the ADC

const int VELOCITYTHR = 400;
const int padMAX = 1024;

int pad_offset[4] = {0, 0, 0, 0};
int velocity = 0;

int dirX_amnt = 0;
int dirY_amnt = 0;
int dirZ_amnt = 0;

void calibrate_pad()
{
  ads.begin();
  delay(100);
  pad_offset[0] = ads.readADC_SingleEnded(0);
  pad_offset[1] = ads.readADC_SingleEnded(1);
  pad_offset[2] = ads.readADC_SingleEnded(2);
  pad_offset[3] = ads.readADC_SingleEnded(3);
}

void prepare_pad_data()
{
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
}