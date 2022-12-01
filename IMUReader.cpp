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

#include "IMUReader.h"

#define D2R 0.0174532925

IMUReader::IMUReader(cabot::Handle &ch) : SensorReader(ch) {}

void IMUReader::calibration() {
  in_calibration_ = true;
  init();
}

void IMUReader::init() { init(NULL); }

void IMUReader::init(uint8_t *offsets) {
  if (!imu_.begin()) {
    ch_.loginfo("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    return;
  }
  initialized_ = true;
  if (offsets != NULL) {
    imu_.setSensorOffsets(offsets);
  }
  imu_.setExtCrystalUse(true);
}

void IMUReader::update() {
  if (!initialized_) {
    return;
  }
  static float data[12];
  // put int32 as float32
  auto timestamp = ch_.now();
  data[0] = *reinterpret_cast<float *>(&timestamp.sec);
  data[1] = *reinterpret_cast<float *>(&timestamp.nsec);

  imu::Quaternion q = imu_.getQuat();

  data[2] = q.x();
  data[3] = q.y();
  data[4] = q.z();
  data[5] = q.w();

  imu::Vector<3> xyz = imu_.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  data[6] = xyz.x() * D2R;
  data[7] = xyz.y() * D2R;
  data[8] = xyz.z() * D2R;

  xyz = imu_.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  data[9] = xyz.x();
  data[10] = xyz.y();
  data[11] = xyz.z();

  // publish
  if (!ch_.is_synchronized()) return;
  ch_.publish(0x13, data, 12);
}

void IMUReader::update_calibration() {
  if (!initialized_) {
    return;
  }

  static uint8_t offsets[26];

  imu_.getSensorOffsets(offsets);
  imu_.getCalibration(offsets + 22, offsets + 23, offsets + 24, offsets + 25);
  ch_.publish(0x14, offsets, 26);
}
