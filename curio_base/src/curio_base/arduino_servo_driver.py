#!/usr/bin/env python

import rclpy
from rclpy.node import Node

# Load imports for the Arduino driver
from curio_msgs.msg import CurioServoCommands
from curio_msgs.msg import CurioServoPositions
from curio_base.base_controller import BaseController

class ArduinoServoDriver(Node):
    ''' Servo driver abstraction
    '''

    def __init__(self):
        ''' Constructor
        '''

        super().__init__('ArduinoServoDriver')

        # Servo positions
        self._servo_pos_msg = CurioServoPositions()
        self._servo_pos_msg.wheel_positions = [0 for i in range(BaseController.NUM_WHEELS)]
        self._servo_pos_msg.steer_positions = [0 for i in range(BaseController.NUM_STEERS)]
        self._servo_pos_sub = self.create_subscription(CurioServoPositions, '/servo/positions', self._servo_pos_callback, 10)
        
        # Servo commands
        self._servo_cmd_msg = CurioServoCommands()
        self._servo_cmd_msg.wheel_commands = [0 for i in range(BaseController.NUM_WHEELS)]
        self._servo_cmd_msg.steer_commands = [0 for i in range(BaseController.NUM_STEERS)]
        self._servo_cmd_pub = self.create_publisher(CurioServoCommands, '/servo/commands', 10)

    def set_servos(self, wheel_servos, steer_servos):
        self._wheel_servos = wheel_servos
        self._steer_servos = steer_servos

    def set_steer_command(self, i, position):
        ''' Set the servo steering command
        '''

        self._servo_cmd_msg.steer_commands[i] = position

    def set_wheel_command(self, i, duty):
        ''' Set the servo wheel command
        '''

        self._servo_cmd_msg.wheel_commands[i] = duty

    def publish_commands(self):
        ''' Publish the servo commands
        '''

        self._servo_cmd_pub.publish(self._servo_cmd_msg)

    def get_wheel_position(self, i):
        ''' Get the servo position for the i-th wheel 
        '''

        return self._servo_pos_msg.wheel_positions[i]            

    def set_angle_offset(self, i, deviation):
        ''' Set the steering angle offset (trim)
        '''

        pass

    def _servo_pos_callback(self, msg):
        ''' Callback for the subscription to `/servos/positions`.

        Parameters
        ----------
        msg : curio_msgs.msg/CurioServoStates
            The message for the servo positions.
        '''

        self._servo_pos_msg = msg