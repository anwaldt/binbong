# BinBong technical repository

## Overview

BingBong is a controller interface for the GLOOO Digital Music Instrument (DMI).
This repository contains all the necessary files to develop the firmaware for the controller and the receiving software patch for Pure Data.

## Requirements

- [Arduino IDE](https://www.arduino.cc/en/Main/Software)
  
  Install through board manager
  -  0.3.1 [Redbear DUO Board package](https://github.com/redbear/Duo/blob/master/docs/arduino_board_package_installation_guide.md)
  
  Install through library manager
  -  Adafruit BNO055 library
  -  Adafruit unified sensor library
  -  Adafruit ADS1015 AD conversion library
  
  Install manually
  -  Particle_osc (included in repo)
     - `cp /Arduino Libraries/particle_osc ${YOUR_ARDUINO_HOME}/libraries`

- [Pure Data (vanilla)](https://puredata.info/downloads)
  
  Install through package manager inside PD 
  - MrPeach