/*******************************************************************************
 * Copyright (c) 2020, 2022  Carnegie Mellon University
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
#include "CaBotHandle.h"

#include <arduino-timer.h>
#include "Arduino.h"

#include "BarometerReader.h"
#include "ButtonsReader.h"
#include "Heartbeat.h"
#include "IMUReader.h"
#include "TouchReader.h"
#include "VibratorController.h"

cabot::Handle ch;
Timer<10> timer;

// configurations
#define BAUDRATE (115200UL)

#define HEARTBEAT_DELAY (20)

#ifdef GT
#define BTN1_PIN (2)  // up
#define BTN2_PIN (3)  // down
#define BTN3_PIN (4)  // left
#define BTN4_PIN (5)  // right
#define BTN5_PIN (0)  // decision //not using

#define VIB1_PIN (11)  // front
#define VIB2_PIN (6)   // back //not using
#define VIB3_PIN (10)  // left
#define VIB4_PIN (9)   // right
#endif

#ifdef GTM
#define BTN1_PIN (2)  // up
#define BTN2_PIN (4)  // down
#define BTN3_PIN (3)  // left
#define BTN4_PIN (5)  // right
#define BTN5_PIN (6)  // decision

#define VIB1_PIN (10)  // front
#define VIB2_PIN (12)  // back //not using
#define VIB3_PIN (9)   // left
#define VIB4_PIN (11)  // right
#endif

#define TOUCH_BASELINE (128)
#define TOUCH_THRESHOLD_DEFAULT (64)
#define RELEASE_THRESHOLD_DEFAULT (24)

#define TIMEOUT_DEFAULT (2000)

// sensors
BarometerReader bmpReader(ch);
ButtonsReader buttonsReader(ch, BTN1_PIN, BTN2_PIN, BTN3_PIN, BTN4_PIN,
                            BTN5_PIN);
IMUReader imuReader(ch);
TouchReader touchReader(ch);

// controllers
VibratorController vibratorController(ch, VIB1_PIN, VIB2_PIN, VIB3_PIN,
                                      VIB4_PIN);
Heartbeat heartbeat(LED_BUILTIN, HEARTBEAT_DELAY);

void setup() {
  // set baud rate
  ch.setBaudRate(BAUDRATE);

  // connect to rosserial
  ch.init();

  ch.loginfo("Connected");
  while (!ch.connected()) {
    ch.spinOnce();
  }

  int run_imu_calibration = 0;
  ch.getParam("run_imu_calibration", &run_imu_calibration, 1, TIMEOUT_DEFAULT);
  if (run_imu_calibration != 0) {
    imuReader.calibration();
    timer.every(100, []() {
      imuReader.update();
      imuReader.update_calibration();
    });
    ch.loginfo("Calibration Mode started");
    return;
  }

  int calibration_params[22];
  uint8_t *offsets = NULL;
  if (ch.getParam("calibration_params", calibration_params, 22,
                  TIMEOUT_DEFAULT)) {
    offsets = malloc(sizeof(uint8_t) * 22);
    for (int i = 0; i < 22; i++) {
      offsets[i] = calibration_params[i] & 0xFF;
    }
  } else {
    ch.logwarn("clibration_params is needed to use IMU (BNO055) correctly.");
    ch.logwarn("You can run calibration by setting _run_imu_calibration:=1");
    ch.logwarn("You can check calibration value with /calibration topic.");
    ch.logwarn(
        "First 22 byte is calibration data, following 4 byte is calibration "
        "status for");
    ch.logwarn(
        "System, Gyro, Accel, Magnet, 0 (not configured) <-> 3 (configured)");
    ch.logwarn("Specify like calibration_params:=[0, 0, 0, 0 ...]");
    ch.logwarn("Visit the following link to check how to calibrate sensoe");
    ch.logwarn(
        "https://learn.adafruit.com/"
        "adafruit-bno055-absolute-orientation-sensor/device-calibration");
  }

  int touch_params[3];
  int touch_baseline;
  int touch_threshold;
  int release_threshold;
  if (!ch.getParam("touch_params", touch_params, 3, TIMEOUT_DEFAULT)) {
    ch.logwarn(
        "Please use touch_params:=[baseline,touch,release] format to set touch "
        "params");
    touch_baseline = TOUCH_BASELINE;
    touch_threshold = TOUCH_THRESHOLD_DEFAULT;
    release_threshold = RELEASE_THRESHOLD_DEFAULT;
    ch.logwarn(
        " touched  if the raw value is less   than touch_params[0] - "
        "touch_params[1]");
    ch.logwarn(
        " released if the raw value is higher than touch_params[0] - "
        "touch_params[2]");
  } else {
    touch_baseline = touch_params[0];
    touch_threshold = touch_params[1];
    release_threshold = touch_params[2];
  }
  char default_values[48];
  snprintf(default_values, 48, "Using [%d, %d, %d] for touch_params",
           touch_baseline, touch_threshold, release_threshold);
  ch.loginfo(default_values);

  // initialize
  ch.loginfo("setting up BMP280");
  bmpReader.init();
  ch.loginfo("setting up Buttons");
  buttonsReader.init();
  ch.loginfo("setting up BNO055");
  imuReader.init(offsets);
  ch.loginfo("setting up MPR121");
  touchReader.init(touch_baseline, touch_threshold, release_threshold);
  ch.loginfo("setting up vibrations");
  vibratorController.init();
  ch.loginfo("setting up heartbeat");
  heartbeat.init();

  // wait sensors ready
  delay(100);

  // set timers
  timer.every(1000, []() { ch.sync(); });

  timer.every(500, []() { bmpReader.update(); });

  timer.every(20, []() {
    heartbeat.update();
    buttonsReader.update();
    touchReader.update();
  });

  timer.every(10, []() { imuReader.update(); });

  ch.loginfo("Arduino is ready");
}

void loop() {
  timer.tick<void>();
  ch.spinOnce();
}
