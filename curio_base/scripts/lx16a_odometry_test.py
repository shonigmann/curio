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

''' Lewansoul LX-16A odometry test.
'''
import csv
import serial

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64

from curio_base.lx16a_driver import LX16ADriver
from curio_base.lx16a_encoder_filter import LX16AEncoderFilter

# Constants .......................................
CONTROL_FREQUENCY   = 50      # Control loop frequency [Hz]
SERVO_SERIAL_PORT   = '/dev/ttyUSB0'
SERVO_BAUDRATE      = 115200
SERVO_TIMEOUT       = 1.0 # [s]
SERVO_ID            = 11
DATA_DIR  = './data/'
WINDOW    = 10
# Filename for persisted ML model
MODEL_FILENAME = "{0}lx16a_tree_model_all.joblib".format(DATA_DIR)

class LX16AOdometer(Node):

    def __init__(self,name):
        '''
        Constructor
        '''
        super().__init__(name)

        self.control_frequency = CONTROL_FREQUENCY
        # Publisher
        self._encoder_msg = Int64()
        self._encoder_pub = self.create_publisher(Int64, '/encoder', 10)

        # Subscriber
        self._cmd_vel_msg = Twist()
        self._cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel',  self.cmd_vel_callback, 10)

        # Initialise encoder filter
        self._encoder_filter = LX16AEncoderFilter(node = self, classifier_filename = MODEL_FILENAME, window = WINDOW)

        # Initialise servo driver
        self._servo_driver = LX16ADriver(self)
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
        rclpy.get_default_context().on_shutdown(self.shutdown_callback)

    def shutdown_callback(self):
        self.get_logger().info('Shutdown lx16a_odometry_test...')
        # Stop servo
        self._servo_driver.motor_mode_write(SERVO_ID, 0)
        self.get_logger().info('Servo stopped')

    def cmd_vel_callback(self, msg):
        self._cmd_vel_msg = msg

    def start_loop(self):
        self.get_logger().info('Starting control loop at {} Hz'.format(self.control_frequency))
        self.control_timer = self.create_timer( 1.0 / self.control_frequency, self.update)

    def update(self, event):
        # To simplify things we map the linear velocity from [-1.0, 1.0] m/s
        # to [-1000, 1000] duty, and limit it to the range.
        duty = int(1000 * self._cmd_vel_msg.linear.x)
        duty = max(duty, -1000)
        duty = min(duty, +1000)

        # Read odometry
        rostime = get_time_secs(self)    

        # @TODO: PROFILING
        pos = 0
        for i in range(10):
            pos = self._servo_driver.pos_read(SERVO_ID)
        # @TODO: PROFILING

        # pos = self._servo_driver.pos_read(SERVO_ID)
        self._encoder_filter.update(rostime, duty, pos)
        count = self._encoder_filter.get_count()

        # Write commands
        self._servo_driver.motor_mode_write(SERVO_ID, duty)

        # Publish
        self._encoder_msg.data = count
        self._encoder_pub.publish(self._encoder_msg)

        # node.get_logger().info("duty: {}, pos: {}, count: {}".format(duty, pos, count))

def main(args=None):    
    rclpy.init(args=args)   
    
    # Servo odometer Node
    lx_odometry = LX16AOdometer('lx16a_odometry')

    lx_odometry.get_logger().info('Starting Lewansoul LX-16A odometry test')

    # Start the control loop
    lx_odometry.start_loop()

    # And sleep ...
    rclpy.spin(lx_odometry)

if __name__ == '__main__':
    main()
