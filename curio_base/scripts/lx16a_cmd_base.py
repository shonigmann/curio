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
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist

CONTROL_FREQUENCY = 50      # Control loop frequency [Hz]
MU = 0.0                    # Random walk mean
SIGMA = 0.8                 # Random walk stddev

STARTUP_CMD_VEL = 0.25      # velocity during startup [m/s]
STARTUP_DURATION = 10.0     # startup duration [s]
SAMPLE_DURATION = 20.0      # Duration to run at a given level

class LX16A_CMD_BASE(Node):

    def __init__(self,name, _sample_dur=SAMPLE_DURATION, _start_dur=STARTUP_DURATION):
        '''
        Constructor
        '''
        super().__init__(name)

        # Publisher
        self.cmd_vel_msg = Twist()
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Parameters
        self.mu = MU
        self.sigma = SIGMA
        self.control_frequency = CONTROL_FREQUENCY
        self.sample_dur = _sample_dur # Duration(seconds=_sample_dur)
        self.start_dur =  _start_dur # Duration(seconds=_start_dur)

        self.init_t = 0.0
        self.prev_t = 0.0
        self.curr_t = 0.0
        self.prev_x = 0.0
        self.curr_x = 0.0

    def start_loop(self):
        self.get_logger().info('Starting control loop at {} Hz'.format(self.control_frequency))
        self.control_timer = self.create_timer( 1.0 / self.control_frequency, self.update)



    def update(self):
        pass

    
def main(args=None):    
    rclpy.init(args=args)    

    lx_base = LX16A_CMD_BASE('LX16A_CMD_BASE')
    
    lx_base.get_logger().info('Starting LX-16A CMD BASE')

    # Start the control loop
    lx_base.start_loop()

    # And sleep ...
    rclpy.spin(lx_base)

    lx_base.get_logger().info('Shutting down LX-16A CMD BASE')

if __name__ == '__main__':
    main()
