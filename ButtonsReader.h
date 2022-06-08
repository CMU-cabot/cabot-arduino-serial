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

#ifndef ARDUINO_NODE_BUTTONS_READER_H
#define ARDUINO_NODE_BUTTONS_READER_H

#include <Wire.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include "SensorReader.h"

class ButtonsReader: public SensorReader {
  int b1_pin_;
  int b2_pin_;
  int b3_pin_;
  int b4_pin_;
  int b5_pin_;
//  ros::Publisher b1_pub_;
//  ros::Publisher b2_pub_;
//  ros::Publisher b3_pub_;
//  ros::Publisher b4_pub_;
//  ros::Publisher b5_pub_;
  ros::Publisher b_pub_;
//  std_msgs::Bool b1_msg_;
//  std_msgs::Bool b2_msg_;
//  std_msgs::Bool b3_msg_;
//  std_msgs::Bool b4_msg_;
//  std_msgs::Bool b5_msg_;
  std_msgs::Int8 b_msg_;
public:
  ButtonsReader(ros::NodeHandle &nh, int b1_pin, int b2_pin, int b3_pin, int b4_pin, int b5_pin);
  void init();
  void update();
};

#endif //ARDUINO_NODE_BUTTONS_READER_H
