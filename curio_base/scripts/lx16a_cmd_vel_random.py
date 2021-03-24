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

''' Generate /cmd_vel samples using a random walk constrained to [-1, 1]
'''

import rclpy
import math
import random

from lx16a_cmd_base import LX16A_CMD_BASE
from lx16a_cmd_base import STARTUP_CMD_VEL, SAMPLE_DURATION, STARTUP_DURATION

class LX16A_CMD_VEL_RANDOM(LX16A_CMD_BASE):

    def __init__(self,name, _sample_dur=SAMPLE_DURATION, _start_dur=STARTUP_DURATION):
        '''
        Constructor
        '''
        super().__init__(name, _sample_dur, _start_dur)
        
    def update(self):

        # Startup
        self.curr_t = self.get_clock().now().to_msg()
        if self.curr_t - self.init_t < self.start_dur:
            self.cmd_vel_msg.linear.x = STARTUP_CMD_VEL
            self.cmd_vel_pub.publish(self.cmd_vel_msg)
            return

        # Random Normal
        z = random.gauss(self.mu, self.sigma)

        # Time increment
        dt = 1.0 / self.control_frequency

        # Brownian increment
        dW = math.sqrt(dt) * z

        # Increments
        dx = self.mu * dt + self.sigma * dW

        # Constrained path 
        self.curr_x = self.prev_x + dx
        if self.curr_x > 1.0 or self.curr_x < -1.0:
            self.curr_x = self.prev_x - dx 
        self.prev_x = self.curr_x

        # Update message and publish
        self.cmd_vel_msg.linear.x = self.curr_x
        self.cmd_vel_pub.publish(self.cmd_vel_msg)

def main(args=None):    
    rclpy.init(args=args)    

    lx_cmd_vel_random = LX16A_CMD_VEL_RANDOM('lx_16a_cmd_vel_random')
    
    lx_cmd_vel_random.get_logger().info('Starting LX-16A CMD vel_random')

    # Start the control loop
    lx_cmd_vel_random.start_loop()

    # And sleep ...
    rclpy.spin(lx_cmd_vel_random)

    lx_cmd_vel_random.get_logger().info('Shutting down LX-16A vel_random')


if __name__ == '__main__':
    main()
