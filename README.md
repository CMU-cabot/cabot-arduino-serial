[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

# Cabot-Arduino

This repository contains an Arduino project for CaBot, managaing sensors and the handle.

# Usage

```
$ rosrun rosserial_python serial_node.py <parameters>
```

## parameters

- **calibration_params** : `int[22]`
  - BNO055 calibration parameter. Follow instruction when you not specify this parameter.
- **touch_params** : [`touch_base (int)`, `touch_threshold (int)`, `release_threshold (int)`]
  - Touch threshold parameters
  - `touch_base` - base value when you don't touch the touch sensor
  - `touch_threshold` - if the value is below `touch_base - touch_threshold` then changes to touch state
  - `release_threshold` - After transitioning to touch state, the value bigger than `touch_base - release_threshold` then changes to release state
- ~**touch_threshold**~ deprecated (int) - touch threshold
- ~**release_threshold**~ deprecated - release threshold

## Pre-requisites

### Hardware

One example of hardware components

- [3D print parts](https://github.com/CMU-cabot/cabot_design/tree/master/cabot2_e2/handle)
- [Arduino Mega 2560](https://store.arduino.cc/usa/mega-2560-r3) or ESP32 (beta)
- [MPR121](https://www.adafruit.com/product/1982) capacitive touch sensor
- [BNG055](https://www.adafruit.com/product/2472) 9-axis IMU
- [BMP280](https://www.adafruit.com/product/2651) Barometric Pressure & Altitude Sensor
- [PCB shield example](https://github.com/RealCabot/simplePCB.git)
  - This Arduino shield is derived from an earlier project. It includes motor controllers, but here we will use it for IMU and touch sensor (does not include barometric pressure sensor part)
- 4 [push buttons](https://www.adafruit.com/product/4183)
- 3 [mini disc vibrators](https://www.adafruit.com/product/1201)
- Conductive Material (ex. [Copper foil tape](https://www.adafruit.com/product/3483))
- Wires and headers

### Software (docker, arduino-ide)

```
  host $ docker-compose build
  host $ docker-compose run arduino
docker $ ./build.sh all                # build and upload (with -b arduino:avr:mega:cpu=atmega2560 -p /dev/ttyARDUINO_MEGA)
                                       # you can set board by ARDUINO_BOARD, and port by ARDUINO_PORT environment variables

or use arduino-cli
docker $ arduino-cli compile -b arduino:avr:mega:cpu=atmega2560 .
docker $ arduino-cli upload -b arduino:avr:mega:cpu=atmega2560 -p /dev/ttyARDUINO_MEGA .
```
- change `-b <board type> -p <port>` for your environment
- linux is required to upload built sketch (Windows/Mac docker container does not support)


### Software (Arduino IDE)

In order to run ROS on Arduino Mega 2560, you will need both the Arduino IDE as well as ROS Serial Arduino Library. Please follow the instructions on the following page for installing these requirements:
[Installation instructions](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)

<font color = "blue">After installing Arduino IDE, please make sure to open the IDE at least once before proceeding to the next steps.</font>

The following packages are essential for letting the touch sensor, push buttons and vibrator communicate with Arduino Mega 2560. Please open a new terminal and copy-paste the following instructions:
```
cd ~/Arduino/libraries
git clone https://github.com/adafruit/Adafruit_BNO055.git
git clone https://github.com/adafruit/Adafruit_Sensor.git
git clone https://github.com/adafruit/Adafruit_ADXL343.git
git clone https://github.com/adafruit/Adafruit_MPR121.git
git clone https://github.com/adafruit/Adafruit_BMP280.git
git clone https://github.com/contrem/arduino-timer
git clone https://github.com/frankjoshua/rosserial_arduino_lib.git
rosrun rosserial_arduino make_libraries.py ~/Arduino
```

#### tips

- rosserial 0.7.9 works with Arduino Mega 2560
- rosserial 0.9.1 works with EPS32

## Assembly instructions

The following figure explains the manner in which the touch sensor, push buttons and vibrators are supposed to be connected with Arduino Mega 2560.

<p align="center">
  <img src="figures/Arduino_shield.svg">
</p>

## Components description

- The Cabot's handle has three main features namely, touch sensor, push buttons (four of them), and vibrators (three of them).
- The touch sensor is used for giving user feedback to the robot while it is moving, to ensure that the robot moves along with the user without getting lost.
- The three vibrators are useful for giving alerts to the user in response to obstacles encountered in the front and on the sides of the robot while it is moving. They also provide alerts/ warnings to the user about upcoming left turns and right turns.
- The four push butons are useful for giving manual input signals to the robot, to command it to move in preferred directions (forward, backward, left turn, and right turn).
- The touch sensor is usually connected to a conductive material like copper film, or copper plate.
