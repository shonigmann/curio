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

''' Lewansoul LX-16A driver test.

    This module contains a set of tests for the LX-16A servo driver.
    The tests operate in both position and continuous mode, so the servo 
    should be free to rotate continuously and run without load.

    The module runs as a ROS node, this is because the LX-16A driver uses
    rospy logging to report warnings and errors so we need to have roscore
    running in order to see these on the terminal.

    Usage:

    1. Start roscore node

    $ roscore

    2. Start driver test

    $ rosrun curio_base lx16a_driver_test.py 

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
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist


import curio_base.lx16a_driver
from curio_base.utils import get_time_secs

SERVO_SERIAL_PORT   = '/dev/ttyUSB0'
SERVO_BAUDRATE      = 115200
SERVO_TIMEOUT       = 1.0
SERVO_ID            = 11

# Convert LX-16A position to angle in deg
def pos_to_deg(pos):
    return pos * 240.0 / 1000.0

def test_servo_properties(servo_driver):
    node = Node('test_servo_properties')
    node.get_logger().info('Test Servo Properties')

    # Display servo properties
    angle_offset = servo_driver.angle_offset_read(SERVO_ID)
    node.get_logger().info("angle_offset: {}".format(angle_offset))

    min_angle, max_angle = servo_driver.angle_limit_read(SERVO_ID)
    node.get_logger().info("angle_limit: {}, {}".format(min_angle, max_angle))

    min_vin, max_vin = servo_driver.vin_limit_read(SERVO_ID)
    node.get_logger().info("vin_limit: {}, {}".format(min_vin, max_vin))

    temp_max_limit = servo_driver.temp_max_limit_read(SERVO_ID)
    node.get_logger().info("temp_max_limit: {}".format(temp_max_limit))

    temp = servo_driver.temp_read(SERVO_ID)
    node.get_logger().info("temp: {}".format(temp))

    vin = servo_driver.vin_read(SERVO_ID)
    node.get_logger().info("vin: {}".format(vin))

    # load_or_unload = servo_driver.load_or_unload_read(SERVO_ID)
    # node.get_logger().info("load_or_unload: {}".format(load_or_unload))

    # Run servo in motor (continuous) mode
    node.get_logger().info('Set motor speed')
    speed = 800
    run_duration = 2 # Duration(seconds=2)
    pos = servo_driver.pos_read(SERVO_ID)
    angle = pos_to_deg(pos)
    node.get_logger().info("position: {}, angle: {}".format(pos, angle))
    servo_driver.motor_mode_write(SERVO_ID, speed)

    start = get_time_secs(node)
    while get_time_secs(node) < start + run_duration:
        pos = servo_driver.pos_read(SERVO_ID)
        angle = pos_to_deg(pos)
        node.get_logger().info("position: {}, angle: {}".format(pos, angle))


    servo_driver.motor_mode_write(SERVO_ID, 0)
    pos = servo_driver.pos_read(SERVO_ID)
    angle = pos_to_deg(pos)
    node.get_logger().info("position: {}, angle: {}".format(pos, angle))

def test_move_time_write(servo_driver):
    node = Node('test_move_time_write')
    node.get_logger().info('Test Move Time Write')

    # Set to servo mode
    servo_driver.servo_mode_write(SERVO_ID)

    # Initial position
    pos = servo_driver.pos_read(SERVO_ID)
    angle = pos_to_deg(pos)
    node.get_logger().info("position: {}, angle: {}".format(pos, angle))

    # Min angle in 1000 ms
    servo_driver.move_time_write(SERVO_ID, 0, 1000)
    node.create_rate(1.5).sleep()

    pos = servo_driver.pos_read(SERVO_ID)
    angle = pos_to_deg(pos)
    node.get_logger().info("position: {}, angle: {}".format(pos, angle))

    # Max angle in 1000 ms
    servo_driver.move_time_write(SERVO_ID, 1000, 1000)
    node.create_rate(1.5).sleep()

    pos = servo_driver.pos_read(SERVO_ID)
    angle = pos_to_deg(pos)
    node.get_logger().info("position: {}, angle: {}".format(pos, angle))

    # Min angle in 100 ms
    servo_driver.move_time_write(SERVO_ID, 0, 100)
    node.create_rate(1.5).sleep()

    pos = servo_driver.pos_read(SERVO_ID)
    angle = pos_to_deg(pos)
    node.get_logger().info("position: {}, angle: {}".format(pos, angle))

    # Max angle in 100 ms
    servo_driver.move_time_write(SERVO_ID, 1000, 100)
    node.create_rate(1.5).sleep()

    pos = servo_driver.pos_read(SERVO_ID)
    angle = pos_to_deg(pos)
    node.get_logger().info("position: {}, angle: {}".format(pos, angle))

def test_move_time_read(servo_driver):
    node = Node('test_move_time_read')
    node.get_logger().info('Test Move Time Read')

    # Set to servo mode
    servo_driver.servo_mode_write(SERVO_ID)

    # Initial position
    pos = servo_driver.pos_read(SERVO_ID)
    angle = pos_to_deg(pos)
    node.get_logger().info("position: {}, angle: {}".format(pos, angle))

    # Min angle in 1000 ms
    servo_driver.move_time_write(SERVO_ID, 0, 1000)
    pos, move_time = servo_driver.move_time_read(SERVO_ID)
    node.get_logger().info("pos: {}, time: {}".format(pos, move_time))
    node.create_rate(1.5).sleep()

    pos = servo_driver.pos_read(SERVO_ID)
    angle = pos_to_deg(pos)
    node.get_logger().info("position: {}, angle: {}".format(pos, angle))

    # Max angle in 1000 ms
    servo_driver.move_time_write(SERVO_ID, 1000, 1000)
    pos, move_time = servo_driver.move_time_read(SERVO_ID)
    node.get_logger().info("pos: {}, time: {}".format(pos, move_time))
    node.create_rate(1.5).sleep()

    pos = servo_driver.pos_read(SERVO_ID)
    angle = pos_to_deg(pos)
    node.get_logger().info("position: {}, angle: {}".format(pos, angle))

    # Min angle in 100 ms
    servo_driver.move_time_write(SERVO_ID, 0, 100)
    pos, move_time = servo_driver.move_time_read(SERVO_ID)
    node.get_logger().info("pos: {}, time: {}".format(pos, move_time))
    node.create_rate(1.5).sleep()

    pos = servo_driver.pos_read(SERVO_ID)
    angle = pos_to_deg(pos)
    node.get_logger().info("position: {}, angle: {}".format(pos, angle))

    # Max angle in 100 ms
    servo_driver.move_time_write(SERVO_ID, 1000, 100)
    pos, move_time = servo_driver.move_time_read(SERVO_ID)
    node.get_logger().info("pos: {}, time: {}".format(pos, move_time))
    node.create_rate(1.5).sleep()

    pos = servo_driver.pos_read(SERVO_ID)
    angle = pos_to_deg(pos)
    node.get_logger().info("position: {}, angle: {}".format(pos, angle))

def test_move_time_wait_write(servo_driver):
    node = Node('test_move_time_wait_write')
    node.get_logger().info('Test Move Time Wait Write')

    # Set to servo mode
    servo_driver.servo_mode_write(SERVO_ID)

    # Initial position
    pos = servo_driver.pos_read(SERVO_ID)
    angle = pos_to_deg(pos)
    node.get_logger().info("position: {}, angle: {}".format(pos, angle))

    # Min angle in 1000 ms
    servo_driver.move_time_wait_write(SERVO_ID, 0, 1000)
    node.get_logger().info("waiting...")
    node.create_rate(1.5).sleep()
    servo_driver.move_start(SERVO_ID)
    node.create_rate(1.5).sleep()

    pos = servo_driver.pos_read(SERVO_ID)
    angle = pos_to_deg(pos)
    node.get_logger().info("position: {}, angle: {}".format(pos, angle))

    # Max angle in 1000 ms
    servo_driver.move_time_wait_write(SERVO_ID, 1000, 1000)
    node.get_logger().info("waiting...")
    node.create_rate(1.5).sleep()
    servo_driver.move_start(SERVO_ID)
    node.create_rate(1.5).sleep()

    pos = servo_driver.pos_read(SERVO_ID)
    angle = pos_to_deg(pos)
    node.get_logger().info("position: {}, angle: {}".format(pos, angle))

    # Min angle in 100 ms
    servo_driver.move_time_wait_write(SERVO_ID, 0, 100)
    node.get_logger().info("waiting...")
    node.create_rate(1.5).sleep()
    servo_driver.move_start(SERVO_ID)
    node.create_rate(1.5).sleep()

    pos = servo_driver.pos_read(SERVO_ID)
    angle = pos_to_deg(pos)
    node.get_logger().info("position: {}, angle: {}".format(pos, angle))

    # Max angle in 100 ms
    servo_driver.move_time_wait_write(SERVO_ID, 1000, 100)
    node.get_logger().info("waiting...")
    node.create_rate(1.5).sleep()
    servo_driver.move_start(SERVO_ID)
    node.create_rate(1.5).sleep()

    pos = servo_driver.pos_read(SERVO_ID)
    angle = pos_to_deg(pos)
    node.get_logger().info("position: {}, angle: {}".format(pos, angle))

def test_move_time_wait_read(servo_driver):
    node = Node('test_move_time_wait_read')
    node.get_logger().info('Test Move Time Wait Read')

    # Set to servo mode
    servo_driver.servo_mode_write(SERVO_ID)

    # Initial position
    pos = servo_driver.pos_read(SERVO_ID)
    angle = pos_to_deg(pos)
    node.get_logger().info("position: {}, angle: {}".format(pos, angle))

    # Min angle in 1000 ms
    servo_driver.move_time_wait_write(SERVO_ID, 0 if pos > 500 else 1000, 1000)
    node.get_logger().info("waiting...")
    node.create_rate(1.5).sleep()
    servo_driver.move_start(SERVO_ID)
    # pos, move_time = servo_driver.move_time_wait_read(SERVO_ID)
    # node.get_logger().info("pos: {}, time: {}".format(pos, move_time))
    node.create_rate(1.5).sleep()

    pos = servo_driver.pos_read(SERVO_ID)
    angle = pos_to_deg(pos)
    node.get_logger().info("position: {}, angle: {}".format(pos, angle))

def test_move_stop(servo_driver):
    node = Node('test_move_stop')
    node.get_logger().info('Test Move Stop')

    # Initial position
    pos = servo_driver.pos_read(SERVO_ID)
    angle = pos_to_deg(pos)
    node.get_logger().info("position: {}, angle: {}".format(pos, angle))

    # Run the servo with speed 500
    servo_driver.motor_mode_write(SERVO_ID, 500)
    node.create_rate(1.5).sleep()

    pos = servo_driver.pos_read(SERVO_ID)
    angle = pos_to_deg(pos)
    node.get_logger().info("position: {}, angle: {}".format(pos, angle))

    # Set to servo mode
    servo_driver.servo_mode_write(SERVO_ID)

    # Stop
    servo_driver.move_stop(SERVO_ID)
    node.create_rate(1.0).sleep()

    pos = servo_driver.pos_read(SERVO_ID)
    angle = pos_to_deg(pos)
    node.get_logger().info("position: {}, angle: {}".format(pos, angle))

def test_angle_offset(servo_driver):
    node = Node('test_angle_offset')
    node.get_logger().info('Test Angle Offset')

    # Read
    offset = servo_driver.angle_offset_read(SERVO_ID)
    node.get_logger().info("angle_offset: {}".format(offset))

    # Positive offset
    servo_driver.angle_offset_adjust(SERVO_ID, 125)

    # Read
    new_offset = servo_driver.angle_offset_read(SERVO_ID)
    node.get_logger().info("angle_offset: {}".format(new_offset))

    # Negative offset
    servo_driver.angle_offset_adjust(SERVO_ID, -125)

    # Read
    new_offset = servo_driver.angle_offset_read(SERVO_ID)
    node.get_logger().info("angle_offset: {}".format(new_offset))

    # Restore
    servo_driver.angle_offset_adjust(SERVO_ID, offset)

    # Read
    offset = servo_driver.angle_offset_read(SERVO_ID)
    node.get_logger().info("angle_offset: {}".format(offset))

def test_angle_limit(servo_driver):
    node = Node('test_angle_limit')
    node.get_logger().info('Test Angle Limit')

    # Read
    min_angle, max_angle = servo_driver.angle_limit_read(SERVO_ID)
    node.get_logger().info("angle_limit: {}, {}".format(min_angle, max_angle))

    # Write
    servo_driver.angle_limit_write(SERVO_ID, 200, 800)

    # Read
    new_min_angle, new_max_angle = servo_driver.angle_limit_read(SERVO_ID)
    node.get_logger().info("angle_limit: {}, {}".format(new_min_angle, new_max_angle))

    # Restore
    servo_driver.angle_limit_write(SERVO_ID, min_angle, max_angle)

    # Read
    min_angle, max_angle = servo_driver.angle_limit_read(SERVO_ID)
    node.get_logger().info("angle_limit: {}, {}".format(min_angle, max_angle))

def test_vin_limit(servo_driver):
    node = Node('test_vin_limit')
    node.get_logger().info('Test Vin Limit')

    # Read
    min_vin, max_vin = servo_driver.vin_limit_read(SERVO_ID)
    node.get_logger().info("vin_limit: {}, {}".format(min_vin, max_vin))

    # Write
    servo_driver.vin_limit_write(SERVO_ID, 4.6, 9.6)

    # Read
    new_min_vin, new_max_vin = servo_driver.vin_limit_read(SERVO_ID)
    node.get_logger().info("vin_limit: {}, {}".format(new_min_vin, new_max_vin))

    # Restore
    servo_driver.vin_limit_write(SERVO_ID, min_vin, max_vin)

    # Read
    min_vin, max_vin = servo_driver.vin_limit_read(SERVO_ID)
    node.get_logger().info("vin_limit: {}, {}".format(min_vin, max_vin))

    # Vin Read
    vin = servo_driver.vin_read(SERVO_ID)
    node.get_logger().info("vin: {}".format(vin))

def test_temp_max_limit(servo_driver):
    node = Node('test_temp_max_limit')
    node.get_logger().info('Test Temp Max Limit')

    # Read
    max_temp = servo_driver.temp_max_limit_read(SERVO_ID)
    node.get_logger().info("temp_limit: {}".format(max_temp))

    # Write
    servo_driver.temp_max_limit_write(SERVO_ID, 65)

    # Read
    new_max_temp = servo_driver.temp_max_limit_read(SERVO_ID)
    node.get_logger().info("temp_limit: {}".format(new_max_temp))

    # Restore
    servo_driver.temp_max_limit_write(SERVO_ID, max_temp)

    # Read
    max_temp = servo_driver.temp_max_limit_read(SERVO_ID)
    node.get_logger().info("temp_limit: {}".format(max_temp))

    # Temperature Read
    temp = servo_driver.temp_read(SERVO_ID)
    node.get_logger().info("temp: {}".format(temp))

def test_mode_read(servo_driver):
    node = Node('test_mode_read')
    node.get_logger().info('Test Mode Read')

    # Initial position
    pos = servo_driver.pos_read(SERVO_ID)
    angle = pos_to_deg(pos)
    node.get_logger().info("position: {}, angle: {}".format(pos, angle))

    # Run the servo with speed 500
    servo_driver.motor_mode_write(SERVO_ID, 500)

    # Read
    mode, speed = servo_driver.mode_read(SERVO_ID)
    node.get_logger().info("mode: {}, speed: {}".format(mode, speed))

    node.create_rate(1.5).sleep()

    pos = servo_driver.pos_read(SERVO_ID)
    angle = pos_to_deg(pos)
    node.get_logger().info("position: {}, angle: {}".format(pos, angle))

    # Read
    mode, speed = servo_driver.mode_read(SERVO_ID)
    node.get_logger().info("mode: {}, speed: {}".format(mode, speed))

    # Run the servo with speed -300
    servo_driver.motor_mode_write(SERVO_ID, -300)

    # Read
    mode, speed = servo_driver.mode_read(SERVO_ID)
    node.get_logger().info("mode: {}, speed: {}".format(mode, speed))

    node.create_rate(1.5).sleep()

    pos = servo_driver.pos_read(SERVO_ID)
    angle = pos_to_deg(pos)
    node.get_logger().info("position: {}, angle: {}".format(pos, angle))

    # Read
    mode, speed = servo_driver.mode_read(SERVO_ID)
    node.get_logger().info("mode: {}, speed: {}".format(mode, speed))

    # Stop
    servo_driver.motor_mode_write(SERVO_ID, 0)
    node.create_rate(1.0).sleep()

    pos = servo_driver.pos_read(SERVO_ID)
    angle = pos_to_deg(pos)
    node.get_logger().info("position: {}, angle: {}".format(pos, angle))

def test_load_or_unload(servo_driver):
    node = Node('test_load_or_unload')
    node.get_logger().info('Test Load or Unload')

    # Read
    is_loaded = servo_driver.load_or_unload_read(SERVO_ID)
    node.get_logger().info("is_loaded: {}".format(is_loaded))

    # Write - set loaded
    servo_driver.load_or_unload_write(SERVO_ID, 1)

    # Read
    is_loaded = servo_driver.load_or_unload_read(SERVO_ID)
    node.get_logger().info("is_loaded: {}".format(is_loaded))

    # Write - set unloaded
    servo_driver.load_or_unload_write(SERVO_ID, 0)

def test_led_ctrl(servo_driver):
    node = Node('test_led_ctrl')
    node.get_logger().info('Test LED Control')

    # Read
    led_state = servo_driver.led_ctrl_read(SERVO_ID)
    node.get_logger().info("led_state: {}".format(led_state))

    # Write off
    servo_driver.led_ctrl_write(SERVO_ID, 1)
    node.create_rate(1.0).sleep()

    # Read
    led_state = servo_driver.led_ctrl_read(SERVO_ID)
    node.get_logger().info("led_state: {}".format(led_state))

    # Write on
    servo_driver.led_ctrl_write(SERVO_ID, 0)
    node.create_rate(1.0).sleep()

    # Read
    led_state = servo_driver.led_ctrl_read(SERVO_ID)
    node.get_logger().info("led_state: {}".format(led_state))

def test_led_error(servo_driver):
    node = Node('test_led_error')
    node.get_logger().info('Test LED Error')

    # Read original state
    fault_code = servo_driver.led_error_read(SERVO_ID)
    node.get_logger().info("fault_code: {}".format(fault_code))

    # Write
    for i in range(0, 7):
        servo_driver.led_error_write(SERVO_ID, i)
        new_fault_code = servo_driver.led_error_read(SERVO_ID)
        node.get_logger().info("fault_code: {}".format(new_fault_code))

    # Restore
    servo_driver.led_error_write(SERVO_ID, fault_code)
    fault_code = servo_driver.led_error_read(SERVO_ID)
    node.get_logger().info("fault_code: {}".format(fault_code))

def main(args=None):    
    rclpy.init(args=args)  
    node = Node('lx_16a_driver_test_node')
    node.get_logger().info('Lewansoul LX-16A driver test')
 
    # Initialise servo driver
    servo_driver = curio_base.lx16a_driver.LX16ADriver(node)
    servo_driver.set_port(SERVO_SERIAL_PORT)
    servo_driver.set_baudrate(SERVO_BAUDRATE)
    servo_driver.set_timeout(SERVO_TIMEOUT)
    servo_driver.open()
    
    node.get_logger().info('Open connection to servo bus board')
    node.get_logger().info('is_open: {}'.format(servo_driver.is_open()))
    node.get_logger().info('port: {}'.format(servo_driver.get_port()))
    node.get_logger().info('baudrate: {}'.format(servo_driver.get_baudrate()))
    node.get_logger().info('timeout: {}'.format(servo_driver.get_timeout()))

    # Tests.
    test_servo_properties(servo_driver)
    test_move_time_write(servo_driver)
    test_move_time_read(servo_driver)
    test_move_time_wait_write(servo_driver)

    # Not working...
    # test_move_time_wait_read(servo_driver)

    test_move_stop(servo_driver)
    test_angle_offset(servo_driver)
    test_angle_limit(servo_driver)
    test_vin_limit(servo_driver)
    test_temp_max_limit(servo_driver)
    test_mode_read(servo_driver)

    # Not working...
    # test_load_or_unload(servo_driver)

    test_led_ctrl(servo_driver)
    test_led_error(servo_driver)
    
    # Shutdown
    servo_driver.close()
    node.get_logger().info('Close connection to servo bus board')
    node.get_logger().info('is_open: {}'.format(servo_driver.is_open()))

if __name__ == '__main__':
    main()