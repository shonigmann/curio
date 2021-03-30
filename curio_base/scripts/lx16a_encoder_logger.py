#!/usr/bin/env python
# 
#   Software License Agreement (BSD-3-Clause)
#    
#   Copyright (c) 2019 Rhys Mainwaring
#   All rights reserved
#    
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
# 
#   1.  Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
# 
#   2.  Redistributions in binary form must reproduce the above
#       copyright notice, this list of conditions and the following
#       disclaimer in the documentation and/or other materials provided
#       with the distribution.
# 
#   3.  Neither the name of the copyright holder nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#  
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#   POSSIBILITY OF SUCH DAMAGE.
# 

''' Lewansoul LX-16A encoder logger.

    This module contains a ROS node for controlling a LX-16A servo
    to create labelled encoder data. 

    In this node the servo operates in continuous mode, so  
    must be free to rotate continuously and run without load.

    Usage:

    1. Start roscore node

    $ roscore

    2. Start the rotary encoder rosserial node

    $ roslaunch curio_base rotary_encoder.launch

    3. Start the servo node

    $ rosrun curio_base lx16a_encoder_logger.py 

    Notes:

    It may be necessary to change the device assigned to SERVO_SERIAL_PORT
    depending upon your operating system and what USB peripherals you have
    connected. The python 'serial' package contains a command line tool
    for listing ports; an example usage running on macOS is:

    $ python -m serial.tools.list_ports -v
    /dev/cu.Bluetooth-Incoming-Port
        desc: n/a
        hwid: n/a
    /dev/cu.wchusbserialfd5110
        desc: USB2.0-Serial
        hwid: USB VID:PID=1A86:7523 LOCATION=253-5.1.1
    2 ports found

    On linux the device will usually be on /dev/ttyUSBx, x = 0,1,2,...

    The SERVO_BAUDRATE should not be changed, 115200 is the value required by the
    LX-16A datasheet.

    Set the SERVO_ID to the value assigned to your servo. The factory default is 1.

'''

import pandas as pd
import serial

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Int64
from geometry_msgs.msg import Twist

import curio_base.lx16a_driver
from curio_base.utils import get_time_secs

SERVO_SERIAL_PORT   = '/dev/ttyUSB0'
SERVO_BAUDRATE      = 115200
SERVO_TIMEOUT       = 1.0 # [s]
SERVO_ID            = 1

CONTROL_FREQUENCY   = 50  # [Hz]
# TODO: find a better way to do this ...
ROOT_FOLDER = "/home/manuel/workspace/ros2_rov/src/curio/curio_base/"
OUT_DATA_FILENAME   = ROOT_FOLDER + "data/lx16a_raw_data_08.csv"

REFERENCE_ENCODER_CPR = 4096  # Counts per revolution for the reference logger (publishing to /encoder)

# Convert LX-16A position to angle in deg
def pos_to_deg(pos):
    return pos * 240.0 / 1000.0



class LX16AEncoderLogger(Node):
    DATA_BUFFER_SIZE = 100

    def __init__(self):
        ''' Constructor
        '''

        super().__init__('lx16a_encoder_logger')
        # Properties
        self.filename = OUT_DATA_FILENAME
        self._data = dict()

        self._data["time"] = []
        self._data["duty"] = []
        self._data["pos"] = []
        self._data["count"] = []
        self._data_size = 0

        # Subscriptions
        self._encoder_msg = Int64()
        self._encoder_sub = self.create_subscription(Int64, '/encoder', self.encoder_callback, 10)

        self._cmd_vel_msg = Twist()
        self._cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Initialise servo driver
        self._servo_driver = curio_base.lx16a_driver.LX16ADriver(self)
        self._servo_driver.set_port(SERVO_SERIAL_PORT)
        self._servo_driver.set_baudrate(SERVO_BAUDRATE)
        self._servo_driver.set_timeout(SERVO_TIMEOUT)
        self._servo_driver.open()
        
        self.get_logger().info('Open connection to servo bus board')
        self.get_logger().info('is_open: {}'.format(self._servo_driver.is_open()))
        self.get_logger().info('port: {}'.format(self._servo_driver.get_port()))
        self.get_logger().info('baudrate: {}'.format(self._servo_driver.get_baudrate()))
        self.get_logger().info('timeout: {}'.format(self._servo_driver.get_timeout()))

        # Register shutdown behaviour
        rclpy.get_default_context().on_shutdown(self.shutdown)

    def start_loop(self):
        self.get_logger().info('Starting control loop at {} Hz'.format(CONTROL_FREQUENCY))
        self.control_timer = self.create_timer( 1.0 / CONTROL_FREQUENCY, self.update)

    def shutdown(self):
        self.get_logger().info('Shutdown lx16a_encoder_logger...')

        # Stop servo
        self.get_logger().info('Stop servo')
        self._servo_driver.motor_mode_write(SERVO_ID, 0)
        
        # Write remaining data
        self.write_data()

    def encoder_callback(self, msg):
        self._encoder_msg = msg

    def cmd_vel_callback(self, msg):
        self._cmd_vel_msg = msg

    def update(self):
        # To simplify things we map the linear velocity from [-1.0, 1.0] m/s
        # to [-1000, 1000] duty, and limit it to the range.
        duty = int(1000 * self._cmd_vel_msg.linear.x)
        duty = max(duty, -1000)
        duty = min(duty, +1000)

        # Run servo in motor (continuous) mode
        self._servo_driver.motor_mode_write(SERVO_ID, duty)

        pos = self._servo_driver.pos_read(SERVO_ID)
        count = self._encoder_msg.data
        self.get_logger().info("duty: {}, pos: {}, count: {}".format(duty, pos, count % REFERENCE_ENCODER_CPR))

        # Buffer data
        self._data["time"].append(get_time_secs(self))
        self._data["duty"].append(duty)
        self._data["pos"].append(pos)
        self._data["count"].append(count)

        self._data_size = self._data_size + 1
        if (self._data_size == LX16AEncoderLogger.DATA_BUFFER_SIZE):
            self.write_data()

    def write_data(self):
        # I know... Pandas just for csv formatting?
        dfObj = pd.DataFrame.from_dict(self._data)
        dfObj.to_csv(OUT_DATA_FILENAME, index = False, header=True,sep='\t',mode='a')
        
        # Reset buffer
        self._data["time"] = []
        self._data["duty"] = []
        self._data["pos"] = []
        self._data["count"] = []
        self._data_size = 0

def main(args=None):    
    rclpy.init(args=args)    

    # Servo encoder logger
    encoder_logger = LX16AEncoderLogger()
    encoder_logger.get_logger().info('Starting Lewansoul LX-16A encoder logger')

    # Start the control loop
    encoder_logger.start_loop()
    
    rclpy.spin(encoder_logger)


if __name__ == '__main__':
    main()