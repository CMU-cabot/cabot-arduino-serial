#!/usr/bin/env python3

import os
import sys
import inspect
import logging
import traceback
import threading
import queue
import struct
import time
import serial
import argparse

from cabot.util import setInterval

DEBUG=False

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG if DEBUG else logging.INFO)

class CaBotArduinoDriverDelegate:
    def __init__(self):
        pass

    def log(self, level, text):
        pass

    def getParam(self, name):
        pass

class CaBotArduinoDriver:
    def __init__(self, port_name, baud, delegate=None, timeout=1):
        self.port_name = port_name
        self.baud = baud
        self.delegate = delegate
        self.timeout = timeout

        self.port = None
        self.write_thread = None
        self.write_queue = queue.Queue()
        self.write_lock = threading.RLock()
        self.read_lock = threading.RLock()
        self.is_alive = True


    def _open_serial(self):
        logger.info("opening serial %s", self.port_name)
        while self.is_alive:
            try:
                self.port = serial.Serial(self.port_name, self.baud, timeout=5, write_timeout=10)
                break
            except serial.SerialException as e:
                logging.error("%s", e)
                time.sleep(3)
        logger.info("serial port opened at %d"%(self.baud));


    def start(self):
        sleep_time = 3
        while self.is_alive:
            try:
                self._open_serial()
                self._run()
            except KeyboardInterrupt as e:
                logger.info("KeyboardInterrupt")
                break
            except serial.SerialException as e:
                error_msg = str(e)
                logger.error(e)
                time.sleep(sleep_time)
            except OSError as e:
                error_msg = str(e)
                logger.error(e)
                traceback.print_exc(file=sys.stdout)
                time.sleep(sleep_time)
            except IOError as e:
                error_msg = str(e)
                logger.error(e)
                time.sleep(sleep_time)
            except termios.error as e:
                error_msg = str(e)
                logger.error("connection disconnected")
                time.sleep(sleep_time)
            except SystemExit as e:
                logger.error(e)
                break
            except:
                logger.error(sys.exc_info()[0])
                traceback.print_exc(file=sys.stdout)
                sys.exit()
            finally:
                self.stop()


    def stop(self):
        self.is_alive = False


    def checksum(data):
        temp = 0
        for d in data:
            temp += d
        return 0xFF - (0xFF & temp)


    def send_command(self, command, arg):
        count = len(arg)
        data = bytearray(count+6)
        data[0] = 0xAA
        data[1] = 0xAA
        data[2] = command
        data[3] = count & 0xFF
        data[4] = (count >> 8) & 0xFF
        for i in range(0, count):
            data[5+i] = arg[i]
        data[count+5] = CaBotArduinoDriver.checksum(arg)
        #logger.info("send %s", data)
        self.write_queue.put(bytes(data))

    def _process_write_queue_once(self):
        if self.write_queue.empty():
            time.sleep(0.01)
        else:
            data = self.write_queue.get()
            while self.is_alive:
                try:
                    if isinstance(data, bytes):
                        self._write(data)
                    else:
                        logger.error("Trying to write invalid data type: %s" % type(data))
                    break
                except serial.SerialTimeoutException as exc:
                    logger.error('Write timeout: %s' % exc)
                    time.sleep(1)
                except RuntimeError as exc:
                    logger.error('Write thread exception: %s' % exc)
                    break
    
    def _process_write_queue(self):
        while self.is_alive:
            self._process_write_queue_once()


    def _write(self, data):
        with self.write_lock:
            self.port.write(data)


    def _tryRead(self, length):
        try:
            bytes_remaining = length
            result = bytearray()
            read_start = time.time()
            while bytes_remaining != 0 and time.time() - read_start < self.timeout:
                with self.read_lock:
                    received = self.port.read(bytes_remaining)
                if len(received) != 0:
                    result.extend(received)
                    bytes_remaining -= len(received)
            if bytes_remaining != 0:
                raise IOError("Returned short (expected %d bytes, received %d instead)." % (length, length - bytes_remaining))
            return bytes(result)
        except Exception as e:
            raise IOError("Serial Port read failure: %s" % e)

    @setInterval(3, times=1)
    def send_start(self):
        data = bytearray(1)
        data[0] = 0x00
        self.write_queue.put(bytes(data))

    def _run(self):
        if self.write_thread is None:
            self.write_thread = threading.Thread(target=self._process_write_queue)
            self.write_thread.daemon = True
            self.write_thread.start()            

        header_count = 0
        state = 0
        while self.is_alive:
            time.sleep(0.00001)
            if self.port.inWaiting() < 1:
                continue
            received = self._tryRead(1)
            if state == 0:  # searching header
                if received[0] == 0xAA:
                    header_count += 1
                else:
                    header_count = 0
                    logger.warning("unknown command {}".format(received[0]))
                if header_count == 2:
                    header_count = 0
                    state = 1
            if state == 1: # read command
                logger.debug("reading command")
                cmd = self._tryRead(1)[0]
                logger.debug("cmd={}".format(cmd))
                size = int.from_bytes(self._tryRead(2), 'little')
                logger.debug("size={}".format(size))
                data = self._tryRead(size)
                logger.debug("data length={}".format(len(data)))
                checksum = int.from_bytes(self._tryRead(1), 'little')
                checksum2 = CaBotArduinoDriver.checksum(data)
                logger.debug("checksum {} {}".format(checksum, checksum2))
                if checksum == checksum2:
                    logger.debug("read data command={} size={}".format(cmd, size))

                if cmd == 0x01:  # time sync
                    remote_sec = int.from_bytes(data[0:4], 'little')
                    remote_nsec = int.from_bytes(data[4:8], 'little')
                    remote_now = remote_sec + remote_nsec / 1000000000.0

                    # send current time
                    temp = bytearray()
                    now = self.delegate.rostime()
                    if abs(now - remote_now) > 0.1:
                        self.delegate.log(logging.WARNING,
                                          "large difference in time %.2f"%(now - remote_now))

                    sec = int(now)
                    nsec = int((now%1)*1000000000)
                    temp.extend(sec.to_bytes(4, 'little'))
                    temp.extend(nsec.to_bytes(4, 'little'))
                    self.send_command(0x01, temp)
                    self.delegate.log_throttle(logging.INFO, 1, "time sync")
                elif cmd == 0x02:  # logdebug
                    self.delegate.log(logging.DEBUG, data.decode('utf-8'))
                elif cmd == 0x03:  # loginfo
                    self.delegate.log(logging.INFO, data.decode('utf-8'))
                elif cmd == 0x04:  # logwarn
                    self.delegate.log(logging.WARNING, data.decode('utf-8'))
                elif cmd == 0x05:  # logerror
                    self.delegate.log(logging.ERROR, data.decode('utf-8'))
                elif cmd == 0x08:  # getParam
                    def send_param(data):
                        temp = bytearray()
                        for d in data:
                            temp.extend(d.to_bytes(4, 'little'))
                        self.send_command(0x08, temp)
                    self.delegate.get_param(data.decode('utf-8'), send_param)
                elif 0x10 <= cmd and cmd < 0x20:
                    self.delegate.publish(cmd, data)
                else:
                    logger.error("unknwon command %x", cmd)
                state = 0
            


    def process_command_once(self):
        self._open_serial()
        self._process_write_queue_once()


class PrintDelegate(CaBotArduinoDriverDelegate):
    def __init__(self):
        self.dummy_param = {
            "~run_imu_calibration": 0,
            "~calibration_params": None,
            "~touch_params": [72, 24, 12]
        }
        self.throttle = {}

    def rostime(self):
        return time.time()

    def log(self, level, text):
        logger.log(level, text)

    def log_throttle(self, level, interval, text):
        key = "%d-%s"%(level, text)
        if key in self.throttle:
            if time.time() - self.throttle[key] > 0:
                del self.throttle[key]
        else:
            logger.log(level, text)
            self.throttle[key] = time.time() + interval

    def get_param(self, name, callback):
        logger.info("getParam {}".format(name))
        if name in self.dummy_param and self.dummy_param[name]:
            logger.info(self.dummy_param[name])
            callback(self.dummy_param[name])

    def publish(self, cmd, data):
        #logger.info("%x: %d", cmd, int.from_bytes(data, "little"))
        # touch       0x10
        # touch_raw   0x11
        # buttons     0x12
        # imu         0x13
        # calibration 0x14
        # pressure    0x15
        # temperature 0x16
        # logger.info("%x: %s", cmd, str(data));
        #self.log_throttle(logging.INFO, 1, "got data %x"%(cmd))
        pass

def main():
    port_name = os.environ['CABOT_ARDUINO_PORT'] if 'CABOT_ARDUINO_PORT' in os.environ else '/dev/ttyARDUINO_MEGA'
    baud = int(os.environ['CABOT_ARDUINO_BAUD']) if 'CABOT_ARDUINO_BAUD' in os.environ else 115200
    
    driver = CaBotArduinoDriver(port_name, baud, PrintDelegate())
    
    parser = argparse.ArgumentParser()
    
    args = parser.parse_args()
    
    driver.start()


if __name__ == "__main__":
    #logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    logging.basicConfig(format='%(levelname)s - %(message)s')
    main()
