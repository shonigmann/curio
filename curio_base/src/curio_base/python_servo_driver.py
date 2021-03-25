#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from curio_base.base_controller import BaseController

# Load imports for the Python serial driver
from curio_base.lx16a_driver import LX16ADriver
from curio_msgs.msg import CurioServoStates, LX16AState, CurioServoCommands

class PythonServoDriver(Node):
    ''' Servo driver abstraction
    '''

    def __init__(self):
        ''' Constructor
        '''

        super().__init__('PythonServoDriver')

        # LX-16A servo driver - all parameters are required
        self.get_logger().info('Opening connection to servo bus board...')
        port      = self.get_parameter('port')
        baudrate  = self.get_parameter('baudrate')
        timeout   = self.get_parameter('timeout')
        self._servo_driver = LX16ADriver(self)
        self._servo_driver.set_port(port)
        self._servo_driver.set_baudrate(baudrate)
        self._servo_driver.set_timeout(timeout)
        self._servo_driver.open()        
        self.get_logger().info('is_open: {}'.format(self._servo_driver.is_open()))
        self.get_logger().info('port: {}'.format(self._servo_driver.get_port()))
        self.get_logger().info('baudrate: {}'.format(self._servo_driver.get_baudrate()))
        self.get_logger().info('timeout: {:.2f}'.format(self._servo_driver.get_timeout()))

        # Publishers
        self._states_msg = CurioServoStates()
        self._states_pub = self.create_publisher(CurioServoStates, 'servo/states', 10)

        self._servo_cmd_pub = self.create_publisher(CurioServoCommands, '/servo/commands', 10)
        self._wheel_states = [LX16AState() for x in range(BaseController.NUM_WHEELS)]
        self._steer_states = [LX16AState() for x in range(BaseController.NUM_STEERS)]

    def set_servos(self, wheel_servos, steer_servos):
        ''' 
            Removed from constructor as we now extend a rclpy Node
        '''
        
        self._wheel_servos = wheel_servos
        self._steer_servos = steer_servos

    def set_steer_command(self, i, position):
        ''' Set the servo steering command
        '''

        servo = self._steer_servos[i]
        self._servo_driver.servo_mode_write(servo.id)
        self._servo_driver.move_time_write(servo.id, position, 50)

    def set_wheel_command(self, i, duty):
        ''' Set the servo wheel command
        '''

        servo = self._wheel_servos[i]
        state = self._wheel_states[i]
        state.command = duty
        self._servo_driver.motor_mode_write(servo.id, duty)
        
    def publish_commands(self):
        ''' Publish the servo commands
        '''

        pass

    def get_wheel_position(self, i):
        ''' Get the servo position for the i-th wheel 
        '''
        
        servo = self._wheel_servos[i]
        state = self._wheel_states[i]
        pos = self._servo_driver.pos_read(servo.id) 
        state.position = pos
        return pos

    def set_angle_offset(self, i, deviation):
        ''' Set the steering angle offset (trim)
        '''

        servo = self._steer_servos[i]
        self._servo_driver.angle_offset_adjust(servo.id, servo.offset)
        # self._servo_driver.angle_offset_write(servo.id)

    # @TODO: check and test
    def update_states(self, time):
        ''' Update the servo states

        Parameters
        ----------
        time : The current time as float
            The current time.
        '''

        for i in range (BaseController.NUM_WHEELS):
            servo = self._wheel_servos[i]
            state = self._wheel_states[i]
            state.id = servo.id
            # state.temperature = self._servo_driver.temp_read(servo.id)
            # state.voltage = self._servo_driver.vin_read(servo.id)
            # state.angle_offset = self._servo_driver.angle_offset_read(servo.id)
            state.mode = LX16AState.LX16A_MODE_MOTOR

        for i in range (BaseController.NUM_STEERS):
            servo = self._steer_servos[i]
            state = self._steer_states[i]
            state.id = servo.id
            # state.temperature = self._servo_driver.temp_read(servo.id)
            # state.voltage = self._servo_driver.vin_read(servo.id)
            # state.angle_offset = self._servo_driver.angle_offset_read(servo.id)
            state.mode = LX16AState.LX16A_MODE_SERVO

    # @TODO: check and test
    def publish_states(self, _stamp):
        ''' Publish the servo states

        Parameters
        ----------
        _stamp : Current time as ros msg Node.get_clock().now().to_msg().
        '''

        # Header
        self._states_msg.header.stamp = _stamp
        self. _states_msg.header.frame_id = 'base_link'

        # LX16A state
        self._states_msg.wheel_states = self._wheel_states
        self._states_msg.steer_states = self._steer_states

        # Publish rover state
        self._states_pub.publish(self._states_msg)
