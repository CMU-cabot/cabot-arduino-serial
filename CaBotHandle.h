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

#ifndef CABOT_HANDLE_H
#define CABOT_HANDLE_H

#include <Arduino.h>

namespace cabot{
typedef struct Time {
  unsigned long sec;
  unsigned long nsec;
} Time;

typedef struct Callback {
  uint8_t cmd;
  void (*callback)(const uint8_t);
} Callback;

class Handle {
 public:
  Handle();
  ~Handle();
  void setBaudRate(unsigned long);
  void init();
  bool connected();
  void spinOnce();
  void subscribe(uint8_t, void (*)(const uint8_t));
  void logdebug(char *);
  void loginfo(char *);
  void logwarn(char *);
  bool getParam(char *, int *, size_t, int);
  void publish(uint8_t, int8_t*, size_t);
  void publish(uint8_t, float*, size_t);
  void publish(uint8_t, int8_t);
  void publish(uint8_t, int16_t);
  void publish(uint8_t, float);
  void sync();
  bool is_synchronized();
  Time now();
  
 private:
  bool sendCommand(uint8_t, uint8_t*, size_t);
  size_t readCommand(uint8_t*, uint8_t**);
  uint8_t checksum(uint8_t *, size_t);
  uint32_t Handle::parseUInt32(uint8_t * ptr);
  void Handle::toBytes(uint32_t v, uint8_t* ptr, size_t num);
  void Handle::toBytes(float, uint8_t* ptr);
    
  unsigned long mBaudRate;
  Time mTime;
  unsigned long mSyncTime;
  float diff;
  unsigned long mTimeOffset;
  bool mConnected;
  bool mConnecting;

  uint8_t state = 0;
  uint8_t header_count = 0;
  size_t size = 0;
  uint8_t size_count = 0;
  uint8_t cmd = 0;
  uint8_t count = 0;
  Callback callbacks[4];
  size_t callback_count = 0;
  
};
}
#endif //CABOT_HANDLE_H
