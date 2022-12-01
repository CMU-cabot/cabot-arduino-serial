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

#ifndef ARDUINO_NODE_VIBRATOR_CONTROLLER_H
#define ARDUINO_NODE_VIBRATOR_CONTROLLER_H

#include <Wire.h>
#include "SensorReader.h"
#ifdef ESP32
#include <analogWrite.h>
#endif

class VibratorController : public SensorReader {
  int vib1_pin_;
  int vib2_pin_;
  int vib3_pin_;
  int vib4_pin_;

 public:
  VibratorController(cabot::Handle &ch, int vib1_pin, int vib2_pin,
                     int vib3_pin, int vib4_pin);
  void init();
  void update();
};

#endif  // ARDUINO_NODE_VIBRATOR_CONTROLLER_H
