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

#include "TouchReader.h"

TouchReader::TouchReader(cabot::Handle &ch):
  SensorReader(ch)
{
}

void TouchReader::init() {
  if (!cap_.begin(0x5A)) {
    ch_.loginfo("Ooops, no MPR121 detected ... Check your wiring or I2C ADDR!");
    return;
  }
  initialized_ = true;
  set_mode(128);
}

void TouchReader::init(uint8_t touch_baseline, uint8_t touch_threshold, uint8_t release_threshold) {
  if (!cap_.begin(0x5A, &Wire, touch_threshold, release_threshold)){
    ch_.loginfo("Ooops, no MPR121 detected ... Check your wiring or I2C ADDR!");
    return;
  }
  initialized_ = true;
  set_mode(touch_baseline);
}

void TouchReader::set_mode(uint8_t touch_baseline) {
  // stop mode
  cap_.writeRegister(MPR121_ECR, 0b00000000);
  // set baseline to 128 ( do not remove bit shift)
  cap_.writeRegister(MPR121_BASELINE_0, touch_baseline >> 2);
  // use only pin 0
  cap_.writeRegister(MPR121_ECR, 0b01000001);
  
  ch_.loginfo("Touch ready");
}

void TouchReader::update() {
  if (!initialized_) {
    return;
  }
  int touched = cap_.touched();

  // touch
  ch_.publish(0x10, (int16_t) touched);
  // touch_raw
  ch_.publish(0x11, (int16_t) cap_.filteredData(0));
}
