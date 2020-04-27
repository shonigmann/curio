#!/usr/bin/env python
# 
# coding: latin-1
# 
#   Software License Agreement (BSD-3-Clause)
#    
#   Copyright (c) 2020 Rhys Mainwaring
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

''' Lewansoul LX-16A encoder filter server.
'''

from curio_base.lx16a_encoder_filter import LX16AEncoderFilter
from curio_base.srv import EncoderFilterAdd, EncoderFilterAddResponse
from curio_base.srv import EncoderFilterGetAngularPosition, EncoderFilterGetAngularPositionResponse
from curio_base.srv import EncoderFilterGetCount, EncoderFilterGetCountResponse
from curio_base.srv import EncoderFilterGetDuty, EncoderFilterGetDutyResponse
from curio_base.srv import EncoderFilterGetInvert, EncoderFilterGetInvertResponse
from curio_base.srv import EncoderFilterGetRevolutions, EncoderFilterGetRevolutionsResponse
from curio_base.srv import EncoderFilterGetServoPos, EncoderFilterGetServoPosResponse
from curio_base.srv import EncoderFilterReset, EncoderFilterResetResponse
from curio_base.srv import EncoderFilterSetInvert, EncoderFilterSetInvertResponse
from curio_base.srv import EncoderFilterUpdate, EncoderFilterUpdateResponse
import rospy
import sys

# Maintain a dictionary of encoder filters, keyed on servo_id.
encoder_filters = {}

def handle_add(req):
    rospy.logdebug('Got: servo_id: {}, classifier_filename: {}, regressor_filename: {}, window: {}'.format(
        req.servo_id,
        req.classifier_filename,
        req.regressor_filename,
        req.window
    ))

    filter = LX16AEncoderFilter(req.classifier_filename, req.regressor_filename, req.window)
    encoder_filters[req.servo_id] = filter

    response = EncoderFilterAddResponse()
    response.status = 0
    return response

def handle_get_angular_position(req):
    rospy.logdebug('Got: servo_id: {}'.format(
        req.servo_id
    ))

    filter = encoder_filters[req.servo_id]
    angle = filter.get_angular_position()

    response = EncoderFilterGetAngularPositionResponse()
    response.angle = angle
    return response

def handle_get_count(req):
    rospy.logdebug('Got: servo_id: {}'.format(
        req.servo_id
    ))

    filter = encoder_filters[req.servo_id]
    count = filter.get_count()

    response = EncoderFilterGetCountResponse()
    response.count = count
    return response

def handle_get_duty(req):
    rospy.logdebug('Got: servo_id: {}'.format(
        req.servo_id
    ))

    filter = encoder_filters[req.servo_id]
    duty = filter.get_duty()

    response = EncoderFilterGetDutyResponse()
    response.duty = duty
    return response

def handle_get_invert(req):
    rospy.logdebug('Got: servo_id: {}'.format(
        req.servo_id
    ))

    filter = encoder_filters[req.servo_id]
    invert = filter.get_invert()

    response = EncoderFilterGetInvertResponse()
    response.invert = invert
    return response

def handle_get_revolutions(req):
    rospy.logdebug('Got: servo_id: {}'.format(
        req.servo_id
    ))

    filter = encoder_filters[req.servo_id]
    revolutions = filter.get_revolutions()

    response = EncoderFilterGetRevolutionsResponse()
    response.revolutions = revolutions
    return response

def handle_get_servo_pos(req):
    rospy.logdebug('Got: servo_id: {}'.format(
        req.servo_id
    ))

    filter = encoder_filters[req.servo_id]
    position, is_valid = filter.get_servo_pos()

    response = EncoderFilterGetServoPosResponse()
    response.position = position
    response.is_valid = is_valid
    return response

def handle_reset(req):
    rospy.logdebug('Got: servo_id: {}, position: {}'.format(
        req.servo_id,
        req.position
    ))

    filter = encoder_filters[req.servo_id]
    filter.reset(req.position)

    response = EncoderFilterResetResponse()
    response.status = 0
    return response

def handle_set_invert(req):
    rospy.logdebug('Got: servo_id: {}, is_inverted: {}'.format(
        req.servo_id,
        req.is_inverted
    ))

    filter = encoder_filters[req.servo_id]
    filter.set_invert(req.is_inverted)

    response = EncoderFilterSetInvertResponse()
    response.status = 0
    return response

def handle_update(req):
    rospy.logdebug('Got: servo_id: {}, time: {}, duty: {}, position: {}'.format(
        req.servo_id,
        req.time,
        req.duty,
        req.position
    ))

    filter = encoder_filters[req.servo_id]
    filter.update(req.time, req.duty, req. position)

    response = EncoderFilterUpdateResponse()
    response.status = 0
    return response

def lx16a_encoder_filter_server():
    rospy.init_node('lx16a_encoder_filter_server')

    # Python system info
    # rospy.logdebug('Python version: {}'.format(sys.version_info))

    # Register services
    service = rospy.Service('lx16a/encoder_filter/add',
        EncoderFilterAdd, handle_add)
    service = rospy.Service('lx16a/encoder_filter/get_angular_position',
        EncoderFilterGetAngularPosition, handle_get_angular_position)
    service = rospy.Service('lx16a/encoder_filter/get_count',
        EncoderFilterGetCount, handle_get_count)
    service = rospy.Service('lx16a/encoder_filter/get_duty',
        EncoderFilterGetDuty, handle_get_duty)
    service = rospy.Service('lx16a/encoder_filter/get_invert',
        EncoderFilterGetInvert, handle_get_invert)
    service = rospy.Service('lx16a/encoder_filter/get_revolutions',
        EncoderFilterGetRevolutions, handle_get_revolutions)
    service = rospy.Service('lx16a/encoder_filter/get_servo_pos',
        EncoderFilterGetServoPos, handle_get_servo_pos)
    service = rospy.Service('lx16a/encoder_filter/reset',
        EncoderFilterReset, handle_reset)
    service = rospy.Service('lx16a/encoder_filter/set_invert',
        EncoderFilterSetInvert, handle_set_invert)
    service = rospy.Service('lx16a/encoder_filter/update',
        EncoderFilterUpdate, handle_update)

    # Start spinning
    rospy.loginfo('lx16a_encoder_filter_server ready...')
    rospy.spin()

if __name__ == '__main__':
    lx16a_encoder_filter_server()

