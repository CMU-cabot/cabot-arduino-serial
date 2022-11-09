#!/usr/bin/env python3
import abc
from typing import Callable, List
import logging
import threading
import time
import queue

import serial

from cabot.util import setInterval

class CaBotArduinoSerialDelegate(abc.ABC):
    """Delegate definition for CaBotArduinoDriver class"""
    def __init__(self):
        pass

    @abc.abstractmethod
    def system_time(self):
        """
        return system time in linux time
        """

    @abc.abstractmethod
    def log(self, level: int, text: str) -> None:
        """
        implement log output for the system
        @param level: logging level
        @param text: logging text
        """

    @abc.abstractmethod
    def log_throttle(self, level: int, interval: float, text: str) -> None:
        """
        implement log throttle output for the system
        @param level: logging level
        @param interval: logging interval
        @param text: logging text
        """

    @abc.abstractmethod
    def get_param(self, name: str, callback: Callable[[List[int]], None]) -> None:
        """
        get parameter from the system
        @param name: name of the parameter
        @param callback: call this callback with an int array
        """

    @abc.abstractmethod
    def publish(self, cmd: bytes, data: bytearray) -> None:
        """
        publish data according to the cmd
        @param cmd: publish topic type
        @param data: compact data of the message which needs to be converted
        
        defined topics and cmd
        # touch       0x10
        # touch_raw   0x11
        # buttons     0x12
        # imu         0x13
        # calibration 0x14
        # pressure    0x15
        # temperature 0x16
        """


class CaBotArduinoSerial:
    """
    Implementation of serial protocol to communicate with cabot arduino
    """
    def __init__(self, port_name, baud, delegate=None, timeout=1):
        self.port_name = port_name
        self.baud = baud
        self.delegate = delegate
        self.timeout = timeout

        self.port = None
        self.read_thread = None
        self.read_lock = threading.RLock()
        self.write_thread = None
        self.write_queue = queue.Queue()
        self.write_lock = threading.RLock()
        self.is_alive = True
        self.read_count = 0
        self.time_synced = False

    def start(self):
        self._open_serial()
        self._run()

    def _open_serial(self):
        self.delegate.log(logging.INFO, "opening serial %s"%(self.port_name))
        while self.is_alive:
            try:
                self.port = serial.Serial(self.port_name, self.baud,
                                          timeout=5, write_timeout=10)
                # reset Arduino
                self.port.setDTR(False)
                time.sleep(0.1)
                self.port.flushInput()
                self.port.setDTR(True)
                break
            except serial.SerialException as error:
                self.delegate.log(logging.ERROR, "%s"%(error))
                time.sleep(3)
        self.delegate.log(logging.INFO, "serial port opened at %d"%(self.baud));

    def _run(self):
        if self.write_thread is None:
            self.write_thread = threading.Thread(target=self._process_write)
            self.write_thread.daemon = True
            self.write_thread.start()
        if self.read_thread is None:
            self.read_thread = threading.Thread(target=self._process_read)
            self.read_thread.daemon = True
            self.read_thread.start()

    def stop(self):
        self.is_alive = False

    def _process_write(self):
        while self.is_alive:
            if self.write_queue.empty():
                time.sleep(0.000001)
                continue
            data = self.write_queue.get()
            try:
                if isinstance(data, bytes):
                    with self.write_lock:
                        self.port.write(data)
                else:
                    self.delegate.log(logging.ERROR,
                                      'Trying to write invalid data type: %s' % type(data))
            except serial.SerialTimeoutException as exc:
                self.delegate.log(logging.ERROR, 'Write timeout: %s' % exc)
                time.sleep(1)
            except RuntimeError as exc:
                self.delegate.log(logging.ERROR, 'Write thread exception: %s' % exc)

    def _try_read(self, length):
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
                raise IOError("Returned short (expected %d bytes,"
                              "received %d instead)." % (length, length - bytes_remaining))
            self.read_count += length
            return bytes(result)
        except Exception as error:
            raise IOError("Serial Port read failure: %s" % str(error))

    def _process_read(self):
        """
        serial command format:
        \xAA\xAA[cmd,1][size,2][data,size][checksum,1]
        """
        reset_time = time.time()

        while self.is_alive:
            time.sleep(0.000001)
            if self.port.inWaiting() < 1:
                continue

            if time.time() - reset_time > 1.0:
                self.delegate.log(logging.DEBUG, "%d bps"%(self.read_count*8))
                self.read_count = 0
                reset_time = time.time()

            cmd = 0
            received = self._try_read(1)
            if received[0] != 0xAA:
                continue
            received = self._try_read(1)
            if received[0] != 0xAA:
                continue

            self.delegate.log(logging.DEBUG, "reading command")
            cmd = self._try_read(1)[0]
            self.delegate.log(logging.DEBUG, "cmd={}".format(cmd))
            size = int.from_bytes(self._try_read(2), 'little')
            self.delegate.log(logging.DEBUG, "size={}".format(size))
            data = self._try_read(size)
            self.delegate.log(logging.DEBUG, "data length={}".format(len(data)))
            checksum = int.from_bytes(self._try_read(1), 'little')
            checksum2 = self.checksum(data)
            self.delegate.log(logging.DEBUG, "checksum {} {}".format(checksum, checksum2))
            if checksum != checksum2:
                continue
            self.delegate.log(logging.DEBUG, "read data command={} size={}".format(cmd, size))

            if cmd == 0x01:  # time sync
                self.check_time_diff(data)
                self.send_time_sync()
            elif cmd == 0x02:  # logdebug
                self.delegate.log(logging.DEBUG, data.decode('utf-8'))
            elif cmd == 0x03:  # loginfo
                self.delegate.log(logging.INFO, data.decode('utf-8'))
            elif cmd == 0x04:  # logwarn
                self.delegate.log(logging.WARNING, data.decode('utf-8'))
            elif cmd == 0x05:  # logerr
                self.delegate.log(logging.ERROR, data.decode('utf-8'))
            elif cmd == 0x08:  # get param
                def send_param(data):
                    temp = bytearray()
                    for d in data:
                        temp.extend(d.to_bytes(4, 'little'))
                    self.send_command(0x08, temp)
                self.delegate.get_param(data.decode('utf-8'), send_param)
            elif 0x10 <= cmd:
                self.delegate.publish(cmd, data)
            else:
                self.delegate.log(logging.ERROR, "unknwon command %x"%(cmd))

    def check_time_diff(self, data):
        # check time difference
        if not self.time_synced:
            return
        remote_sec = int.from_bytes(data[0:4], 'little')
        remote_nsec = int.from_bytes(data[4:8], 'little')
        remote_now = remote_sec + remote_nsec / 1000000000.0
        now = self.delegate.system_time()
        if abs(now - remote_now) > 0.1:
            self.delegate.log(logging.WARNING,
                              "large difference in time %5.4f"%(now - remote_now))

    def send_time_sync(self):
        # send current time
        temp = bytearray()
        now = self.delegate.system_time()
        sec = int(now)
        nsec = int((now%1)*1000000000)
        temp.extend(sec.to_bytes(4, 'little'))
        temp.extend(nsec.to_bytes(4, 'little'))
        self.send_command(0x01, temp)
        self.time_synced = True

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
        data[count+5] = self.checksum(arg)
        self.delegate.log(logging.DEBUG, "send %s"%(data))
        self.write_queue.put(bytes(data))

    def checksum(self, data):
        temp = 0
        for d in data:
            temp += d
        return 0xFF - (0xFF & temp)
