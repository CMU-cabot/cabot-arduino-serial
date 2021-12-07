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

#ifndef ARDUINO_NODE_HEARTBEAT_H
#define ARDUINO_NODE_HEARTBEAT_H

#include <Arduino.h>
#ifdef ESP32
#include <analogWrite.h>
#endif

class Heartbeat {
  int led_pin_;
  int delay_;
  int status_;
public:
Heartbeat(int led_pin, int delay):
  led_pin_(led_pin),
  delay_(delay),
  status_(0)
  {
  }

  void init() {
    pinMode(led_pin_, OUTPUT);
    analogWrite(led_pin_, 0xff);
  }

  void update() {
    status_ = status_+1;
    analogWrite(led_pin_, sin(6.28 * status_ * delay_ / 1000.0) * 127 + 127);
  }
};

#endif // ARDUINO_NODE_HEARTBEAT_H
