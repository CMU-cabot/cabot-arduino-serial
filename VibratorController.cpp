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

#include "VibratorController.h"

// keep the instance as static for callback
VibratorController *instance;

VibratorController::VibratorController(cabot::Handle &ch,
				       int vib1_pin, int vib2_pin, int vib3_pin, int vib4_pin):
  SensorReader(ch),
  vib1_pin_(vib1_pin),
  vib2_pin_(vib2_pin),
  vib3_pin_(vib3_pin),
  vib4_pin_(vib4_pin)
{
  instance = this;
  ch.subscribe(0x20, [](const uint8_t msg) {analogWrite(instance->vib1_pin_, msg);});
  ch.subscribe(0x21, [](const uint8_t msg) {analogWrite(instance->vib2_pin_, msg);});
  ch.subscribe(0x22, [](const uint8_t msg) {analogWrite(instance->vib3_pin_, msg);});
  ch.subscribe(0x23, [](const uint8_t msg) {analogWrite(instance->vib4_pin_, msg);});
}

void VibratorController::init(){
  pinMode(vib1_pin_, OUTPUT);
  analogWrite(vib1_pin_,0);
  
  pinMode(vib2_pin_, OUTPUT);
  analogWrite(vib2_pin_,0);
  
  pinMode(vib3_pin_, OUTPUT);
  analogWrite(vib3_pin_,0);
  
  pinMode(vib4_pin_, OUTPUT);
  analogWrite(vib4_pin_,0);
}

void VibratorController::update() {
}
