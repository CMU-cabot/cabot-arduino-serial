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

#include "CaBotHandle.h"

namespace cabot{
Handle::Handle() {
}

Handle::~Handle() {
}

void Handle::setBaudRate(long rate) {
  mBaudRate = rate;
}

void Handle::init() {
  mBaudRate = 0;
  mConnected = false;
  mConnecting = false;
  mTimeOffset = 0;
  diff = 0;
  state = 0;
  header_count = 0;
  size = 0;
  size_count = 0;
  cmd = 0;
  count = 0;
  
  if (mBaudRate == 0) { return; }
  Serial.begin(mBaudRate);
  while (!Serial) {
    ;
  }
  mConnected = true;
}

bool Handle::connected() {
  return mConnected;
}

void Handle::spinOnce() {
  static uint8_t cmd = 0;
  uint8_t *data;
  int count = readCommand(&cmd, &data);
  if (count > 0) {
    if (cmd == 0x01) { // time sync
      if (millis() - mSyncTime < 1000) {
        unsigned long temp = (millis() - mSyncTime) / 2UL;
        diff = diff*0.9f + temp*0.1f;
        
        static uint8_t buff[24];
        sprintf(buff, "diff=");
        dtostrf(diff, 4, 2, buff+5);
        loginfo(buff);

        mTimeOffset = mSyncTime + diff;
    
        mTime.sec = parseUInt32(data);
        mTime.nsec = parseUInt32(data+4);
      }
    }
  }
}

void Handle::subscribe(char * name, void *callback(const uint8_t)) {
}

void Handle::loginfo(char * text) {
  sendCommand(0x03, text, strlen(text));
}

void Handle::logwarn(char * text) {
  sendCommand(0x04, text, strlen(text));
}

bool Handle::getParam(char * name, int * out, size_t num, int timeout_ms) {
  sendCommand(0x08, name, strlen(name));
  uint8_t cmd = 0x08;
  uint8_t *ptr;
  int count = 0;
  while(readCommand(&cmd, &ptr) == 0) {
    count+=1;
    if (count > timeout_ms*10) {
      loginfo("timeout");
      return false;
    }
  }
  for(size_t i = 0; i < num; i++) {
    out[i] = parseUInt32(ptr+i*4);
  }
  
  return true;
}

void Handle::publish(uint8_t cmd, int8_t* buff, size_t num) {
  sendCommand(cmd, buff, num);
}

void Handle::publish(uint8_t cmd, float* buff, size_t num) {
  uint8_t temp[128];
  for(int i = 0; i < num; i++) {
    toBytes(buff[i], temp+i*4);
  }
  sendCommand(cmd, temp, num*4);
}

void Handle::publish(uint8_t cmd, int8_t data) {
  uint8_t buff[1];
  toBytes(data, buff, 1);
  sendCommand(cmd, buff, 1);
}

void Handle::publish(uint8_t cmd, int16_t data) {
  uint8_t buff[2];
  toBytes(data, buff, 2);
  sendCommand(cmd, buff, 2);
}

void Handle::publish(uint8_t cmd, float data) {
  uint8_t buff[2];
  toBytes(data, buff, 2);
  sendCommand(cmd, buff, 2);
}

void Handle::sync() {
  uint8_t buff[8];

  Time current = now();
  toBytes(current.sec, buff, 4);
  toBytes(current.nsec, buff+4, 4);
  mSyncTime = millis();
  sendCommand(0x01, buff, 8);
}

Time Handle::now() {
  Time current;
  unsigned long diff = (millis() - mTimeOffset);
  
  current.sec = mTime.sec + (diff / 1000UL);
  current.nsec = mTime.nsec + (diff % 1000UL) * 1000000UL;
  if (current.nsec > 1000000000UL) {
    current.nsec -= 1000000000UL;
    current.sec += 1;
  }
  return current;
}

/* private */
size_t Handle::readCommand(uint8_t* expect, uint8_t** ptr) {  
  static uint8_t buffer[256];

  if (Serial.available() > 0) {
    int received = Serial.read();
    static uint8_t buff[48];
    sprintf(buff, "%d %d %d %d %d %d", state, header_count, size, size_count, cmd, count);
    loginfo(buff);
    if (state == 0) {
      if (received == 0xAA) {
        header_count += 1;
      }
      else {
        header_count = 0;
      }
      if (header_count == 2) {
        header_count = 0;
        state = 1;
      }
      cmd = 0;
    }
    else if (state == 1) {
      cmd = received;
      if (*expect == 0) {
        *expect = cmd;
      }
      else if (cmd != *expect) {
        state = 0;
        return;
      } 
      state = 2;
      size = 0;
      size_count = 0;
    }
    else if (state == 2) {
      size += (received & 0xFF) << size_count*8;
      size_count += 1;
      if (size_count == 2) {
        if (size < 0 || 256 < size) {
          state = 0;
        }
        else if (size == 0) {
          state = 4;
        }
        else {
          state = 3;
        }
      }
      count = 0;
    }
    else if (state == 3) {
      buffer[count] = received;
      count += 1;
      if (count == size) {
        state = 4;
      }
    }
    else if (state == 4) {
      state = 0;
      if (received == checksum(buffer, size)) {
        *ptr = buffer;
        return size;
      }
    }
  }
  return 0;
}

bool Handle::sendCommand(uint8_t type, uint8_t* data, size_t num) {
  static uint8_t buffer[256+6];
  if (num < 0 || 256 < num) {
    return false;
  }

  buffer[0] = 0xAA;
  buffer[1] = 0xAA;
  buffer[2] = type;
  buffer[3] = num &0xFF;
  buffer[4] = (num >> 8) & 0xFF;
  for (size_t i = 0; i < num; i++) {
    buffer[i+5] = data[i];
  }

  buffer[num+5] = checksum(data, num);

  size_t written = 0;
  while(written < num+6) {
    if (Serial.availableForWrite() > 0) {
      written += Serial.write(buffer+written, num+6-written);
    }
  }
  return true;
}

uint8_t Handle::checksum(uint8_t * data, size_t num) {
  uint8_t temp = 0;
  for (size_t i = 0; i < num; i++) {
    temp += data[i];
  }
  return 0xFF - (0xFF & temp);
}

struct LongChar {
  char c0; char c1; char c2; char c3;
};

// bit shift over 16bit does not work
uint32_t Handle::parseUInt32(uint8_t * ptr) {
  LongChar l_Return = {ptr[0], ptr[1], ptr[2], ptr[3]};
  return *reinterpret_cast<uint32_t*>(&l_Return);
}

void Handle::toBytes(uint32_t v, uint8_t* ptr, size_t num) {
  for(int i = 0; i < num; i++) {
    ptr[i] = (v >> 8*i) & 0xFF;
  }
}

typedef union
{
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;
void Handle::toBytes(float v, uint8_t* ptr) {
  FLOATUNION_t temp;
  temp.number = v;
  ptr[0] = temp.bytes[0];
  ptr[1] = temp.bytes[1];
  ptr[2] = temp.bytes[2];
  ptr[3] = temp.bytes[3];
}

}
