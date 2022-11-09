#!/usr/bin/env python3

import argparse
import logging
import os
import struct
import time

import rospy
from std_msgs.msg import Int8, UInt8, Int16, UInt8MultiArray
from sensor_msgs.msg import Imu, FluidPressure, Temperature

from cabot.arduino_serial import CaBotArduinoSerialDelegate, CaBotArduinoSerial

class ROSDelegate(CaBotArduinoSerialDelegate):
    def __init__(self, owner):
        self.owner = owner
        self.dummy_param = {
            "~run_imu_calibration": [0],
            "~calibration_params": [231, 255, 234, 255, 8, 0, 52, 1, 247, 255, 140, 0, 0, 0, 255, 255, 1, 0, 232, 3, 107, 4, 3, 3, 3, 3],
            "~touch_params": [72, 24, 12]
        }
        self.throttle = {}
        
        self.touch_raw_pub = rospy.Publisher("/touch_raw", Int16, queue_size=10)
        self.touch_pub = rospy.Publisher("/touch", Int16, queue_size=10)
        self.button_pub = rospy.Publisher("/button", Int8, queue_size=10)
        self.imu_last_topic_time = None
        self.imu_pub = rospy.Publisher("/imu", Imu, queue_size=10)
        self.calibration_pub = rospy.Publisher("/calibration", UInt8MultiArray, queue_size=10)
        self.pressure_pub = rospy.Publisher("/pressure", FluidPressure, queue_size=10)
        self.temperature_pub = rospy.Publisher("/temperature", Temperature, queue_size=10)

        self.vib1_sub = rospy.Subscriber("/vibrator1", UInt8, self.vib1_callback)
        self.vib2_sub = rospy.Subscriber("/vibrator2", UInt8, self.vib2_callback)
        self.vib3_sub = rospy.Subscriber("/vibrator3", UInt8, self.vib3_callback)
        self.vib4_sub = rospy.Subscriber("/vibrator4", UInt8, self.vib4_callback)

    def vib1_callback(self, msg):
        data = bytearray()
        data.append(msg.data)
        self.owner.send_command(0x20, data)
    def vib2_callback(self, msg):
        data = bytearray()
        data.append(msg.data)
        self.owner.send_command(0x21, data)
    def vib3_callback(self, msg):
        data = bytearray()
        data.append(msg.data)
        self.owner.send_command(0x22, data)
    def vib4_callback(self, msg):
        data = bytearray()
        data.append(msg.data)
        self.owner.send_command(0x23, data)
        
    def system_time(self):
        return time.time()

    def spin(self):
        rospy.spin()

    def log(self, level, text):
        if level == logging.INFO:
            rospy.loginfo(text)
        if level == logging.WARN:
            rospy.logwarn(text)
        if level == logging.ERROR:
            rospy.logerr(text)

    def log_throttle(self, level, interval, text):
        key = "%d-%s"%(level, text)
        if key in self.throttle:
            if time.time() - self.throttle[key] > 0:
                del self.throttle[key]
        else:
            self.log(level, text)
            self.throttle[key] = time.time() + interval

    def get_param(self, name, callback):
        rospy.loginfo("get_param {}".format(name))
        if name in self.dummy_param and self.dummy_param[name]:
            rospy.loginfo(self.dummy_param[name])
            callback(self.dummy_param[name])

    def publish(self, cmd, data):
        #rospy.loginfo("%x: %d", cmd, int.from_bytes(data, "little"))
        # rospy.loginfo("%x: %s", cmd, str(data));
        #self.log_throttle(logging.INFO, 1, "got data %x"%(cmd))
        if cmd == 0x10:  # touch
            msg = Int16()
            msg.data = int.from_bytes(data, 'little')
            self.touch_pub.publish(msg)
        if cmd == 0x11:  # touch_raw
            msg = Int16()
            msg.data = int.from_bytes(data, 'little')
            self.touch_raw_pub.publish(msg)
        if cmd == 0x12:  # buttons
            msg = Int8()
            msg.data = int.from_bytes(data, 'little')
            self.button_pub.publish(msg)
        if cmd == 0x13:  # imu
            ## discard possible corrupted data
            count = 0
            data2 = [struct.unpack('f', data[i*4:(i+1)*4])[0] for i in range(0, 12)]

            for i in range(2, 12):
                if data2[i] == 0:
                    count += 1
            if count > 3:
                return

            imu_msg = Imu()
            imu_msg.orientation_covariance[0] = 0.1
            imu_msg.orientation_covariance[4] = 0.1
            imu_msg.orientation_covariance[8] = 0.1

            # convert float(32) to int(32)
            imu_msg.header.stamp.set(struct.unpack('i', struct.pack('f', data2[0]))[0]
                                     ,struct.unpack('i', struct.pack('f', data2[1]))[0])
            if self.imu_last_topic_time is not None:
                if self.imu_last_topic_time > imu_msg.header.stamp:
                    #rospy.logerr("IMU timestamp is not consistent, drop a message\n"+
                    #             "last imu time:%.2f > current imu time:%.2f",
                    #               self.imu_last_topic_time.to_sec(), imu_msg.header.stamp.to_sec())
                    return

            imu_msg.header.frame_id = "imu_frame"
            self.imu_last_topic_time = imu_msg.header.stamp
            imu_msg.orientation.x = data2[2]
            imu_msg.orientation.y = data2[3]
            imu_msg.orientation.z = data2[4]
            imu_msg.orientation.w = data2[5]
            imu_msg.angular_velocity.x = data2[6]
            imu_msg.angular_velocity.y = data2[7]
            imu_msg.angular_velocity.z = data2[8]
            imu_msg.linear_acceleration.x = data2[9]
            imu_msg.linear_acceleration.y = data2[10]
            imu_msg.linear_acceleration.z = data2[11]
            self.imu_pub.publish(imu_msg)
        if cmd == 0x14:  # calibration
            msg = UInt8MultiArray()
            msg.data = data
            self.calibration_pub.publish(msg)
        if cmd == 0x15:  # pressure
            msg = FluidPressure()
            msg.fluid_pressure = struct.unpack('f', data)[0]
            msg.variance = 0;
            msg.header.stamp = rospy.Time.now();
            msg.header.frame_id = "bmp_frame";
            self.pressure_pub.publish(msg)
        if cmd == 0x16:  # temperature
            msg = Temperature()
            msg.temperature = struct.unpack('f', data)[0]
            msg.variance = 0;
            msg.header.stamp = rospy.Time.now();
            msg.header.frame_id = "bmp_frame";
            self.temperature_pub.publish(msg)


def main():
    """main method"""
    port_name = os.environ['CABOT_ARDUINO_PORT'] if 'CABOT_ARDUINO_PORT' in os.environ else '/dev/ttyARDUINO_MEGA'
    baud = int(os.environ['CABOT_ARDUINO_BAUD']) if 'CABOT_ARDUINO_BAUD' in os.environ else 115200
    
    serial = CaBotArduinoSerial(port_name, baud)
    serial.delegate = ROSDelegate(serial)
    serial.start()

if __name__ == "__main__":
    rospy.init_node("cabot_serial_node")
    main()
