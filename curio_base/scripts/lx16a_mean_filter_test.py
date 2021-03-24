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

''' (Rolling window) mean filter test.
'''

from curio_base.base_controller import MeanWindowFilter

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

def main(args=None):    
    rclpy.init(args=args)  
    node = Node('lx16a_mean_filter_test')
    node.get_logger().info('Starting mean filter test')

    # Create the filter
    filter = MeanWindowFilter(window=5)

    node.get_logger().info('got: {}, expect: {}'.format(filter.get_mean(), 0.0))

    filter.update(1.0)    
    node.get_logger().info('got: {}, expect: {}'.format(filter.get_mean(), 1.0/5.0))

    filter.update(2.0)    
    node.get_logger().info('got: {}, expect: {}'.format(filter.get_mean(), 2.0/5.0))

    filter.update(3.0)    
    node.get_logger().info('got: {}, expect: {}'.format(filter.get_mean(), 6.0/5.0))

    filter.update(4.0)    
    node.get_logger().info('got: {}, expect: {}'.format(filter.get_mean(), 10.0/5.0))

    filter.update(5.0)    
    node.get_logger().info('got: {}, expect: {}'.format(filter.get_mean(), 15.0/5.0))

    filter.update(5.0)    
    node.get_logger().info('got: {}, expect: {}'.format(filter.get_mean(), 19.0/5.0))

    filter.update(5.0)    
    node.get_logger().info('got: {}, expect: {}'.format(filter.get_mean(), 22.0/5.0))

    node.get_logger().info('Finished. Bye')

if __name__ == '__main__':
    main()