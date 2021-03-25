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

''' Generate /cmd_vel samples using a stepped profile in [-1, 1]
'''

import math
import random
import numpy as np

import rclpy
from rclpy.duration import Duration

from lx16a_cmd_base import LX16A_CMD_BASE, STARTUP_DURATION, SAMPLE_DURATION, STARTUP_CMD_VEL
from curio_base.utils import get_time_secs

STARTUP_DURATION  = 2.0     # startup duration [s]


class LX16A_CMD_VEL_STEP(LX16A_CMD_BASE):

    def __init__(self,name, _sample_dur=SAMPLE_DURATION, _start_dur=STARTUP_DURATION):
        '''
        Constructor
        '''
        super().__init__(name, _sample_dur, _start_dur)

        # Create array of evenly spaced levels in [-1, 1]
        self.level = np.concatenate((np.linspace(-1, -0.15, 18), np.linspace(0.15, 1.0, 18)))
        self.level_idx = 0
        self.level_idx_inc = 1
        self.init_t = get_time_secs(self)
        self.prev_t = get_time_secs(self)


    def update(self):

        # Startup
        self.curr_t = get_time_secs(self)
        if self.curr_t - self.init_t < self.start_dur:
            self.cmd_vel_msg.linear.x = STARTUP_CMD_VEL
            self.cmd_vel_pub.publish(self.cmd_vel_msg)
            return

        # Update amplitude
        if self.curr_t - self.prev_t > self.sample_dur:
            self.prev_t = self.curr_t

            # Constrained path - reverse direction when we hit the boundaries
            self.level_idx = self.level_idx + self.level_idx_inc
            if self.level_idx == len(self.level) or self.level_idx == -1:
                self.level_idx_inc = -1 * self.level_idx_inc
                self.level_idx = self.level_idx + 2 * self.level_idx_inc
                self.curr_x = self.level[self.level_idx]

            self.curr_x = self.level[self.level_idx]


        # Update message and publish
        self.cmd_vel_msg.linear.x = self.curr_x
        self.cmd_vel_pub.publish(self.cmd_vel_msg)

def main(args=None):    
    rclpy.init(args=args)  

    lx_cmd_vel_step = LX16A_CMD_VEL_STEP('lx_16a_cmd_vel_stepped', 20, 2.0)
    lx_cmd_vel_step.get_logger().info('Starting LX-16A cmd_vel stepped')

    # Start the control loop
    lx_cmd_vel_step.start_loop()

    # And sleep ...
    rclpy.spin(lx_cmd_vel_step)

    lx_cmd_vel_step.get_logger().info('Shutting down LX-16A vel_stepped')

if __name__ == '__main__':
    main()
