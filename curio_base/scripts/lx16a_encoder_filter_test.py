#!/usr/bin/env python
# 
# coding: latin-1
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

''' Lewansoul LX-16A encoder filter.
'''

import math
import pandas as pd

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Int64

from curio_base.lx16a_encoder_filter import LX16AEncoderFilter

# Constants
WINDOW = 10

# Raw data produced by servo and encoder 
RAW_DATA_FILENAME = "./data/lx16a_raw_data_05.csv"

# File locations for persisted ML models
CLASSIFIER_FILENAME = "./data/lx16a_tree_classifier.joblib"
REGRESSOR_FILENAME  = "./data/lx16a_tree_regressor.joblib"

def main(args=None):    
    rclpy.init(args=args) 
    ''' A test node for the LX-16A encoder filter.

    This example replays a raw data file containing ros_time, duty, pos, count
    data for the servo through the encoder filter.
    The encoder count is published to the topic /encoder.
    The data is processes as fast as possible, so timestamps are not correct. 
    '''
    node = Node('lx_16a_encoder_filter_test')
    node.get_logger().info('Lewansoul LX-16A encoder filter (test)')

    # Publisher
    encoder_pub = node.create_publisher(Int64, '/encoder', 10) 

    # Encoder filter
    filter = LX16AEncoderFilter(node=node,
        classifier_filename=CLASSIFIER_FILENAME,
        regressor_filename=REGRESSOR_FILENAME,
        window=WINDOW)

    # Load data from CSV and assign names to column headings
    df = pd.read_csv(RAW_DATA_FILENAME, header=None, names=['ros_time', 'duty', 'pos', 'count'])
    node.get_logger().info('num. samples = {}'.format(len(df)))
    node.get_logger().info('shape = {}'.format(df.shape))

    start_time = node.get_clock().now().to_msg()

    for i in range(0, len(df)):
    # for i in range(0, 1000):
        # Current dataset row
        row = df.loc[i]

        # Time since start [s]
        now_time = node.get_clock().now().to_msg()
        dt = (now_time - start_time).to_sec()

        # Update the filter
        filter.update(now_time, row['duty'], row['pos'])
        servo_pos, is_valid = filter.get_servo_pos()
        count               = filter.get_count()
        revolutions         = filter.get_revolutions()

        node.get_logger().debug('time: {:.2f}, duty: {}, pos: {}, is_valid: {}'
            .format(dt, row['duty'], servo_pos, is_valid))

        node.get_logger().info('time: {:.2f}, duty: {}, count: {}, rev: {}'
            .format(dt, row['duty'], count, revolutions))

        encoder_pub.publish(count)

if __name__ == '__main__':
    main()