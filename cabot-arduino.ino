/*******************************************************************************
 * Copyright (c) 2020  Carnegie Mellon University
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *******************************************************************************/

#ifdef ESP32
#undef ESP32
#include <ros.h>
#define ESP32
#else
#include <ros.h>
#endif

#include "Arduino.h"
#include <Timer.h>

#include "BarometerReader.h"
#include "ButtonsReader.h"
#include "Heartbeat.h"
#include "IMUReader.h"
#include "TouchReader.h"
#include "VibratorController.h"

ros::NodeHandle nh;
Timer timer;

// configurations
#define BAUDRATE (115200)

#define HEARTBEAT_DELAY (20)

#ifdef ESP32
// TODO: need to reconfigure
#define BTN1_PIN (13) // up
#define BTN2_PIN (14) // down
#define BTN3_PIN (15) // left
#define BTN4_PIN (16) // right
#define BTN5_PIN (12) // decision

#define VIB1_PIN (19)  //front
#define VIB2_PIN (20)   //back //not using
#define VIB3_PIN (18)  //left
#define VIB4_PIN (17)   //right
#else
#define BTN1_PIN (2) // up
#define BTN2_PIN (3) // down
#define BTN3_PIN (4) // left
#define BTN4_PIN (5) // right
#define BTN5_PIN (7) // decision

#define VIB1_PIN (11)  //front
#define VIB2_PIN (6)   //back //not using
#define VIB3_PIN (10)  //left
#define VIB4_PIN (9)   //right
#endif



#define TOUCH_BASELINE (128)
#define TOUCH_THRESHOLD_DEFAULT (64)
#define RELEASE_THRESHOLD_DEFAULT (24)

// sensors
BarometerReader bmpReader(nh);
ButtonsReader buttonsReader(nh, BTN1_PIN, BTN2_PIN, BTN3_PIN, BTN4_PIN, BTN5_PIN);
IMUReader imuReader(nh);
TouchReader touchReader(nh);

// controllers
VibratorController vibratorController(nh, VIB1_PIN, VIB2_PIN, VIB3_PIN, VIB4_PIN);
Heartbeat heartbeat(LED_BUILTIN, HEARTBEAT_DELAY);


void setup()
{
  // set baud rate
  nh.getHardware()->setBaud(BAUDRATE);

  // connect to rosserial
  nh.initNode();
  while(!nh.connected()) {nh.spinOnce();}
  nh.loginfo("Connected");

  int calibration_params[22];
  if (!nh.getParam("~calibration_params", calibration_params, 22, 500)) {
    nh.logerror("clibration_params is needed to use correctly.");
    nh.logerror("You can check calibration value with /calibration topic.");
    nh.logerror("First 22 byte is calibration data, following 4 byte is calibration status for");
    nh.logerror("System, Gyro, Accel, Magnet, 0 (not configured) <-> 3 (configured)");
    nh.logerror("Specify like calibration_params:=[0, 0, 0, 0 ...]");
    nh.logerror("Visit the following link to check how to calibrate sensoe");
    nh.logerror("https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/device-calibration");
    imuReader.calibration();
    timer.every(100, [](){
      imuReader.update();
    });
    nh.loginfo("Calibration Mode started");
    return;
  }
  uint8_t offsets[22];
  for(int i = 0; i < 22; i++) {
    offsets[i] = calibration_params[i] & 0xFF;
  }
  //nh.loginfo("loglogloglog!!!!!!!!!!!");


  int touch_params[3];
  int touch_baseline;
  int touch_threshold;
  int release_threshold;
  if (!nh.getParam("~touch_params", touch_params, 3, 500)) {
    nh.logwarn("Please use touch_params:=[baseline,touch,release] format to set touch params");
    touch_baseline = TOUCH_BASELINE;
    if (nh.getParam("~touch_threshold", &touch_threshold, 1, 500)) {
      nh.logwarn("touch_threshold is depricated");
    } else {
      touch_threshold = TOUCH_THRESHOLD_DEFAULT;
    }
    if (nh.getParam("~release_threshold", &release_threshold, 1, 500)) {
      nh.logwarn("release_threshold is depricated");
    } else {
      release_threshold = RELEASE_THRESHOLD_DEFAULT;
    }

    nh.logwarn(" touched  if the raw value is less   than touch_params[0] - touch_params[1]");
    nh.logwarn(" released if the raw value is higher than touch_params[0] - touch_params[2]");
  } else {
    touch_baseline = touch_params[0];
    touch_threshold = touch_params[1];
    release_threshold = touch_params[2];
  }
  char default_values[128];
  sprintf(default_values, "Using [%d, %d, %d] for touch_params", touch_baseline, touch_threshold, release_threshold);
  nh.loginfo(default_values);

  // initialize
  nh.loginfo("setting up BMP280");
  bmpReader.init();
  nh.loginfo("setting up Buttons");
  buttonsReader.init();
  nh.loginfo("setting up BNO055");
  imuReader.init(offsets);
  nh.loginfo("setting up MPR121");
  touchReader.init(touch_baseline, touch_threshold, release_threshold);
  nh.loginfo("setting up vibrations");
  vibratorController.init();
  nh.loginfo("setting up heartbeat");
  heartbeat.init();
  
  // wait sensors ready
  delay(100);

  // set timers
  timer.every(500, [](){
      bmpReader.update();
    });

  timer.every(20, [](){
      heartbeat.update();
      buttonsReader.update();
      touchReader.update();
    });

  timer.every(10, [](){
      imuReader.update();
    });
  
  nh.loginfo("Arduino is ready");
}

void loop()
{
  timer.update();
  nh.spinOnce();
}
