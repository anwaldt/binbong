# BinBong technical repository

## Overview

BingBong is a controller interface for the GLOOO Digital Music Instrument (DMI).
This repository contains all the necessary files to develop the firmaware for the controller and the receiving software patch for Pure Data.

## Requirements

### [Arduino IDE](https://www.arduino.cc/en/Main/Software)
  
- **Install through board manager**
  - [Redbear DUO Board package](https://github.com/redbear/Duo/blob/master/docs/arduino_board_package_installation_guide.md) 
    - **Important!** select version **0.3.1** when installing!

- **Install through library manager**
  -  Adafruit BNO055 library **1.1.9**
  -  Adafruit unified sensor library **1.0.3**
  -  Adafruit ADS1015 AD conversion library **1.0.0**
  -  AgileWare CircularBuffer **1.3.1**

- **Install manually**
  - Particle_osc (included in repo)
    - `cp /Arduino Libraries/particle_osc ${YOUR_ARDUINO_HOME}/libraries`


### [Pure Data (vanilla)](https://puredata.info/downloads)
  
- **Install through package manager inside PD** 
  - MrPeach