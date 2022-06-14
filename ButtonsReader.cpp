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

#include "ButtonsReader.h"

ButtonsReader::ButtonsReader(ros::NodeHandle &nh, int b1_pin, int b2_pin, int b3_pin, int b4_pin, int b5_pin):
  SensorReader(nh),
  b1_pin_(b1_pin),
  b2_pin_(b2_pin),
  b3_pin_(b3_pin),
  b4_pin_(b4_pin),
  b5_pin_(b5_pin),
  b_pub_("pushed", &b_msg_)
{
  nh.advertise(b_pub_);
}

void ButtonsReader::init(){
  pinMode(b1_pin_, INPUT_PULLUP);
  pinMode(b2_pin_, INPUT_PULLUP);
  pinMode(b3_pin_, INPUT_PULLUP);
  pinMode(b4_pin_, INPUT_PULLUP);
  pinMode(b5_pin_, INPUT_PULLUP);
}

void ButtonsReader::update() {
  int reading_1 = !digitalRead(b1_pin_);
  int reading_2 = !digitalRead(b2_pin_);
  int reading_3 = !digitalRead(b3_pin_);
  int reading_4 = !digitalRead(b4_pin_);
  int reading_5 = 0;
  if (b5_pin_ > 0) {
    reading_5 = !digitalRead(b5_pin_);
  }

  for(int i = 0; i < 10; i++) {
    delayMicroseconds(10);
    reading_1 = reading_1 && !digitalRead(b1_pin_);
    reading_2 = reading_2 && !digitalRead(b2_pin_);
    reading_3 = reading_3 && !digitalRead(b3_pin_);  
    reading_4 = reading_4 && !digitalRead(b4_pin_); 
    if (b5_pin_ > 0) {
      reading_5 = reading_5 && !digitalRead(b5_pin_);
    }
  }
  
  b_msg_.data = reading_1 | reading_2<<1 | reading_3<<2 | reading_4<<3 | reading_5<<4;
        
  b_pub_.publish(&b_msg_);
}
