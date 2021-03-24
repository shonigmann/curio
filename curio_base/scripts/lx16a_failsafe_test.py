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

''' Lewansoul LX-16A failsafe test.

This test node connects to the Lewansoul Servo BusLinker board via USB
and attempts to stop all servos in the WHEEL_SERVO_IDS list at the
rate given by CONTROL_FREQUENCY.

See curio_base/base_failsafe.py for motivation and more detail.
'''

import serial

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from curio_base.lx16a_driver import LX16ADriver
from curio_base.lx16a_encoder_filter import LX16AEncoderFilter

# Constants .......................................
CONTROL_FREQUENCY   = 20  # [Hz]
SERVO_SERIAL_PORT   = '/dev/ttyUSB0'
SERVO_BAUDRATE      = 115200
SERVO_TIMEOUT       = 1.0 # [s]
NUM_WHEELS          = 6
WHEEL_SERVO_IDS     = [11, 12, 13, 21, 22, 23]

class LX16AFailsafe(Node):

    def __init__(self,name):
        '''
        Constructor
        '''        
        super().__init__(name)

        self.control_frequency = CONTROL_FREQUENCY
        
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

    def start_loop(self):
        self.get_logger().info('Starting control loop at {} Hz'.format(self.control_frequency))
        self.control_timer = self.create_timer( 1.0 / self.control_frequency, self.update)

    def update(self):
        ''' 
        Update will stop all wheel servos.
        '''
        for i in range(NUM_WHEELS):
            servo_id = WHEEL_SERVO_IDS[i]
            self._servo_driver.motor_mode_write(servo_id, 0)

def main(args=None):    
    rclpy.init(args=args)  

    # Servo failsafe Node
    lx_failsafe = LX16AFailsafe('lx16a_failsafe')
    
    lx_failsafe.get_logger().info('Starting Lewansoul LX-16A failsafe')

    # Start the control loop
    lx_failsafe.start_loop()

    # And sleep ...
    rclpy.spin(lx_failsafe)

if __name__ == '__main__':
    main()
