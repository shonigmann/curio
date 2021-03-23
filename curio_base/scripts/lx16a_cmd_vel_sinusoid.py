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

''' Generate /cmd_vel samples using a sinusoid profile in [-1, 1]
'''

import math
import random
import numpy as np

import rclpy
from rclpy.duration import Duration

from lx16a_cmd_base import LX16A_CMD_BASE, STARTUP_CMD_VEL, SAMPLE_DURATION, STARTUP_DURATION

class LX16A_CMD_VEL_SIN(LX16A_CMD_BASE):

    def __init__(self,name, _sample_dur=SAMPLE_DURATION, _start_dur=STARTUP_DURATION):
        '''
        Constructor
        '''
        super().__init__(name, _sample_dur, _start_dur)
        
        # Create array of evenly spaced amplitudes in [0.15, 1]
        self.amp = np.linspace(0.15, 1.0, 18)
        self.amp_idx = 0

        # Create array of periods in [0.5, 20]
        self.period = np.concatenate((np.linspace(0.2, 1, 5), np.linspace(2, 10, 9)))
        self.period_idx = 0

        self.curr_amp = self.amp[self.amp_idx]
        self.curr_period = self.period[self.period_idx]

    def update(self):

        # Startup
        self.curr_t = self.get_clock().now().to_msg()
        if self.curr_t - self.init_t < self.start_dur:
            self.prev_t = self.curr_t
            self.cmd_vel_msg.linear.x = STARTUP_CMD_VEL
            self.cmd_vel_pub.publish(self.cmd_vel_msg)
            return

        # Update amplitude
        if self.curr_t - self.prev_t > self.sample_dur:
            self.prev_t = self.curr_t

            # Update period whenever the amplitudes cycle round
            if self.amp_idx + 1 == len(self.amp):
                self.period_idx = (self.period_idx + 1) % len(self.period)
                self.curr_period = self.period[self.period_idx]

            # Cycle through the amplitudes
            self.amp_idx = (self.amp_idx + 1) % len(self.amp)
            self.curr_amp = self.amp[self.amp_idx]

            self.get_logger().info('amp_idx: {}, amp: {}, period_idx: {}, period: {}'
                .format(self.amp_idx, self.curr_amp, self.period_idx, self.curr_period))

        # Update message and publish
        t = (self.curr_t - self.init_t).to_sec()
        omega = 2 * math.pi / self.curr_period
        self.vel = self.curr_amp * math.sin(omega * t)

        self.cmd_vel_msg.linear.x = self.vel
        self.cmd_vel_pub.publish(self.cmd_vel_msg)

def main(args=None):    
    rclpy.init(args=args)    

    lx_cmd_vel_sin = LX16A_CMD_VEL_SIN('lx_16a_cmd_vel_sinusoidal', 10.0, 2.0)
    
    lx_cmd_vel_sin.get_logger().info('Starting LX-16A cmd_vel sinusoidal')

    # Start the control loop
    lx_cmd_vel_sin.start_loop()

    # And sleep ...
    rclpy.spin(lx_cmd_vel_sin)

    lx_cmd_vel_sin.get_logger().info('Shutting down LX-16A vel_sinusoidal')


if __name__ == '__main__':
    main()