#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


from curio_base.utils import get_param_or_die, get_param_default, get_time_and_secs, get_time_secs
from curio_base.base_controller import BaseController

# Load imports for the Python serial driver
from curio_base.lx16a_driver import LX16ADriver
from curio_msgs.msg import LX16AState, CurioServoStates, CurioServoCommands, CurioServoPositions, CurioServoEncoders

class PythonServoDriver(Node):
    ''' Servo driver abstraction
    '''

    def __init__(self):
        ''' Constructor
        '''
        super().__init__('PythonServoDriver')

        self.load_ros_params()
        
    def start(self):
        # LX-16A servo driver - all parameters are required
        self.get_logger().info('Opening connection to servo bus board...')

        self._servo_driver = LX16ADriver(self)
        self._servo_driver.set_port(self.port)
        self._servo_driver.set_baudrate(self.baudrate)
        self._servo_driver.set_timeout(self.timeout)
        self._servo_driver.open()        
        self.get_logger().info('is_open: {}'.format(self._servo_driver.is_open()))

        self.get_logger().info('Opening ROS comms')
        self.open_ros_comms()
        self.control_timer = self.create_timer( 1.0 / self.control_frequency, self.update)

    def open_ros_comms(self):
        # Publishers
        self._states_msg = CurioServoStates()
        self. _states_msg.header.frame_id = 'base_link'
        self._states_pub = self.create_publisher(CurioServoStates, 'servo/states', 10)

        self._positions_msg = CurioServoPositions()        
        self._positions_msg.header.frame_id = 'base_link'
        self._positions_pub = self.create_publisher(CurioServoPositions, 'servo/positions', 10)

        # Subscribers 
        self._servo_cmd_msg = CurioServoCommands()
        self._servo_cmd_pub = self.create_subscription(Twist, 'servo/commands', self._servo_cmd_callback, 10)

    def load_ros_params(self):
        self.port              = get_param_or_die(self,'port')
        self.baudrate          = get_param_or_die(self,'baudrate')
        self.timeout           = get_param_or_die(self,'timeout')
        self.control_frequency = get_param_default(self,'control_frequency',20.0)
        self.cmd_vel_timeout   = get_param_default(self,'cmd_vel_timeout',0.5)
        
        self.get_logger().info('port: {}'.format(self._servo_driver.get_port()))
        self.get_logger().info('baudrate: {}'.format(self._servo_driver.get_baudrate()))
        self.get_logger().info('timeout: {:.2f}'.format(self._servo_driver.get_timeout()))

    def set_servos(self, wheel_servos, steer_servos):
        ''' 
            Loads servo information from base_controller.
            Should be called right after constructor.
        '''
        
        self._wheel_servos = wheel_servos
        self._steer_servos = steer_servos

        for i in range (BaseController.NUM_WHEELS):
            self._wheel_servos[i].state.id = self._wheel_servos[i].id
            self._wheel_servos[i].state.mode = LX16AState.LX16A_MODE_MOTOR

        for i in range (BaseController.NUM_STEERS):
            self._steer_servos[i].state.id = self._steer_servos[i].id
            self._steer_servos[i].state.mode = LX16AState.LX16A_MODE_SERVO
            self.set_angle_offset(i)
            
    def set_steer_command(self, i, position):
        ''' Set the servo steering command
        '''
        servo = self._steer_servos[i]
        self._steer_servos[i].command = position
        self._servo_driver.servo_mode_write(servo.id)
        self._servo_driver.move_time_write(servo.id, position, 50)

    def set_wheel_command(self, i, duty):
        ''' Set the servo wheel command
        '''
        servo = self._wheel_servos[i]
        self._wheel_servos[i].command = duty
        self._servo_driver.motor_mode_write(servo.id, duty)

    def get_wheel_position(self, i):
        ''' Get the servo position for the i-th wheel 
        '''
        servo = self._wheel_servos[i]
        pos = self._servo_driver.pos_read(servo.id) 
        self._wheel_servos[i].position = pos
        return pos

    def set_angle_offset(self, i):
        ''' Set the steering angle offset (trim)
        '''
        servo = self._steer_servos[i]
        self._servo_driver.angle_offset_adjust(servo.id, servo.offset)
        # self._servo_driver.angle_offset_write(servo.id)

    # @TODO: check and test
    def update_states(self):
        ''' Update the servo states
        '''
        for i in range (BaseController.NUM_WHEELS):
            id_i = self._wheel_servos[i].id
            state = self._wheel_servos[i].state
            state.temperature = self._servo_driver.temp_read(id_i)
            state.voltage = self._servo_driver.vin_read(id_i)
            state.angle_offset = self._servo_driver.angle_offset_read(id_i)
            state.position = self._servo_driver.pos_read(id_i)

        for i in range (BaseController.NUM_STEERS):
            id_i = self._steer_servos[i].id
            state = self._steer_servos[i].state            
            state.temperature = self._servo_driver.temp_read(id_i)
            state.voltage = self._servo_driver.vin_read(id_i)
            state.angle_offset = self._servo_driver.angle_offset_read(id_i)
            state.position = self._servo_driver.pos_read(id_i)

    # @TODO: check and test
    def publish_states(self):
        ''' Publish the servo states
        '''
        (t_msg, t_sec) = get_time_and_secs(self)
        # Update Header
        self._states_msg.header.stamp = t_msg

        # LX16A state
        self._states_msg.wheel_states = [self._wheel_servos[i].state for i in range (BaseController.NUM_WHEELS)]
        self._states_msg.steer_states = [self._steer_servos[i].state for i in range (BaseController.NUM_STEERS)]

        # Publish rover state
        self._states_pub.publish(self._states_msg)

    # @TODO: check and test
    def publish_positions(self):
        ''' Publish the servo positions
        '''
        (t_msg, t_sec) = get_time_and_secs(self)
        # Update Header
        self._positions_msg.header.stamp = t_msg

        # LX16A position 
        self._positions_msg.wheel_positions = [self._wheel_servos[i].state.position for i in range (BaseController.NUM_WHEELS)]
        self._positions_msg.steer_positions = [self._steer_servos[i].state.position  for i in range (BaseController.NUM_STEERS)]

        # Publish rover state
        self._positions_pub.publish(self._positions_msg)

    def update(self):
        self.update_states()
        self.publish_states()
        self.publish_positions()


    def _servo_cmd_callback(self, msg):
        ''' Callback for the subscription to `servo/commands`.
        Parameters
        ----------
        msg : curio_msgs.msg/CurioServoCommands
            The message for the servo commands.
        '''
        # TODO!
        self._servo_cmd_last_rec_time = get_time_secs(self)
        self.get_logger().debug('CurioServoCommand received at: {}'.format(self._servo_cmd_last_rec_time))
        self._servo_cmd_msg = msg        

    def update_servos(self):
        ''' Move the robot given servo values stored in msgs.
        '''

        # Check for timeout
        has_timed_out = get_time_secs(self) > self._servo_cmd_last_rec_time + self.cmd_vel_timeout

        # Update steer servos
        self.get_logger().debug('Updating steer servos')
        for i in range(BaseController.NUM_STEERS):
            servo = self._steer_servos[i]
            servo_pos = 0 if has_timed_out else self._servo_cmd_msg.steer_commands[i]
            self.get_logger().debug('\tid: {}, servo_pos: {}'.format(servo.id, servo_pos))
            self.set_steer_command(i, servo_pos)

        # Update wheel servos
        self.get_logger().debug('Updating wheel servos')
        for i in range(BaseController.NUM_WHEELS):
            servo = self._wheel_servos[i]
            duty =  0 if has_timed_out else self._servo_cmd_msg.wheel_commands[i]
            self.get_logger().debug('\tid: {},  servo_vel: {}'.format(servo.id, duty))
            self.set_wheel_command(i, duty)

            # Update duty array (needed for servo position classifier)
            self._wheel_servo_duty[i] = duty        