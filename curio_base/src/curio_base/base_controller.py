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

from curio_base.utils import get_time_secs, get_time_and_secs, turning_radius_and_rate, degree, quaternion_from_euler
from curio_base.lx16a_encoder_filter import LX16AEncoderFilter
from curio_base.servo import Servo
from curio_base.python_servo_driver import PythonServoDriver
from curio_base.ackermann_odometry import  AckermannOdometry

from curio_base.utils import get_param_or_die, get_param_default

from curio_msgs.msg import CurioServoEncoders, CurioServoStates,CurioServoPositions, CurioServoCommands, LX16AEncoder

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Pose, PoseWithCovariance, Vector3, Twist, TwistWithCovariance, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros.transform_broadcaster import TransformBroadcaster


class BaseController(Node):
    ''' Mobile base controller for 6-wheel powered Ackerman steering.

    A 6-wheel Ackerman steering mobile base controller where each wheel
    is driven by a servo and there are 4-steering servos for each
    of the corner wheels.

    The LX-16A servos may be position controlled through an
    angle of 240 deg. This range is not enough to allow in-place
    steering, so we specify an angle (REVERSE_SERVO_ANGLE) at which
    the servos are reversed, so for example an angle of +130 deg is
    reversed to an angle of 130 - 180 = -50 deg.

    REVERSE_SERVO_ANGLE should be set to 90 deg.

    Attributes
    ----------
    LINEAR_VEL_MAX : float
        The maximum linear velocity limit of the `base_link`,
        has (constant 0.37) [m/s]
    ANGULAR_VEL_MAX : float
        The maximum angular velocity limit of the `base_link`,
        has (constant 1.45) [rad/s]
    SERVO_ANG_VEL_MAX : float
        The maximum angular velocity limit of the servo,
        has (constant 2 * pi) [rad/s]
    SERVO_DUTY_MAX : int
        The maximum duty for the servo, has (constant 1000). 
    REVERSE_SERVO_ANGLE : float
        The angle at which we reverse the servo by 180 deg,
        has (constant 90 deg) 
    SERVO_ANGLE_MAX : float
        Maximum (abs) angle at which the servo can be set,
        has (constant 120 deg)
    SERVO_POS_MIN : int
        Minimum servo position (servo units), has (constant 0)
    SERVO_POS_MAX : int
        Maximum servo position (servo units), has (constant 1000)
    NUM_WHEELS : int
        The number of wheel servos), has (constant 6)
    NUM_STEERS : int
        The number of steering servos), has (constant 4)

    ROS Parameters
    --------------
    ~wheel_radius : float
        The wheel radius [m]
    ~mid_wheel_lat_separation : float
        The lateral distance [m] between the mid wheels
    ~front_wheel_lat_separation : float
        The lateral distance [m] between the front wheels
    ~front_wheel_lon_separation : float
        The longitudinal distance [m] from the front wheels to
        the mid wheels.
    ~back_wheel_lat_separation : float
        The lateral distance [m] between the back wheels
    ~back_wheel_lon_separation : float
        The longitudinal distance [m] from the back wheels to
        the mid wheels.
    ~wheel_radius_multiplier : float
        Wheel radius calibration multiplier to tune odometry,
        has (default = 1.0).
    ~mid_wheel_lat_separation_multiplier : float
        Wheel separation calibration multiplier to tune odometry,
        has (default = 1.0).
    ~wheel_servo_ids : list
        An array of integer wheel servo serial ids : 0 - 253
    ~wheel_servo_lon_labels : list
        An array of wheel servo longitudinal position labels:
        'front', 'mid', 'right'
    ~wheel_servo_lat_labels : list
        An array of wheel servo lateral position labels:
        'left', 'right'
    ~steer_servo_ids : list
        An array of integer steering servo serial ids : 0 - 253
    ~steer_servo_lon_labels : list
        An array of steering servo longitudinal position labels:
        'front', 'mid', 'right'
    ~steer_servo_lat_labels : list
        An array of steering servo lateral position labels:
        'left', 'right'
    ~steer_servo_angle_offsets : list
        An array of integer steering servo angle adjustments,
        used to trim of the steering angle.
    ~port : str
        The device name for the serial port (e.g. /dev/ttyUSB0)
    ~baudrate : int
        The baudrate, has default (115200).
    ~timeout : float
        The time in seconds out for the serial connection,
        has (default 1.0)
    ~classifier_window : int
        The size of the classifier window, this sets the number of
        entries in the servo history used to train the classifier.
        The classifier and regressor models must correspond to this
        setting. (default 10)
    ~classifier_filename : str
        The full filepath for the `scikit-learn` classifier model.
    ~regressor_filename : str
        The full filepath for the `scikit-learn` regressor model.

    Publications
    ------------
    odom : nav_msgs/Odometry
        Publish the odometry.
    tf : geometry_msgs/TransformStamped
        Broadcast the transfrom from `odom` to `base_link`
    servo/encoders : curio_msgs/CurioServoEncoders
        Publish servo encoder states
    servo/states : curio_msgs/CurioServoStates
        Publish the servo states
    servo/positions : curio_msgs/CurioServoPositions
        Publish the servo positions from the servo controller     
        (Arduino serial only)

    Subscriptions
    -------------
    cmd_vel : geometry_msgs/Twist
        `cmd_vel` to move the robot 
    servo/commands : curio_msgs/CurioServoCommands
        Raw Servo commands            
    
    '''

    # Velocity limits for the rover
    LINEAR_VEL_MAX =  0.37
    ANGULAR_VEL_MAX = 1.45

    # Servo limits - LX-16A has max angular velocity of approx 1 revolution per second
    SERVO_ANG_VEL_MAX = 2 * math.pi
    SERVO_DUTY_MAX = 1000

    # Steering angle limits
    REVERSE_SERVO_ANGLE = 90.0      # The angle at which we reverse the servo by 180 deg.
    SERVO_ANGLE_MAX = 120.0         # Maximum (abs) angle at which the servo can be set.
    SERVO_POS_MIN = 0.0             # Minimum servo position (servo units).
    SERVO_POS_MAX = 1000.0          # Maximum servo position (servo units).

    # 6 wheels, 4 steering.
    NUM_WHEELS = 6
    NUM_STEERS = 4

    def __init__(self):
        ''' Constructor
        '''

        super().__init__('BaseController')

        self.get_logger().info('Initialising BaseController...')
        self.load_ros_params()

        # Validate Wheel servo parameters
        self.validate_servo_param(self.wheel_servo_ids, 'wheel_servo_ids', BaseController.NUM_WHEELS)
        self.validate_servo_param(self.wheel_servo_lon_labels, 'wheel_servo_lon_labels', BaseController.NUM_WHEELS)
        self.validate_servo_param(self.wheel_servo_lat_labels, 'wheel_servo_lat_labels', BaseController.NUM_WHEELS)

        # Build Wheel Servo Handlers
        self._wheel_servos = self.build_servos(BaseController.NUM_WHEELS,self.wheel_servo_ids, self.wheel_servo_lon_labels,self.wheel_servo_lat_labels)
    
        # Validate Steer servo parameters 
        self.validate_servo_param(self.steer_servo_ids, 'steer_servo_ids', BaseController.NUM_STEERS)
        self.validate_servo_param(self.steer_servo_lon_labels, 'steer_servo_lon_labels', BaseController.NUM_STEERS)
        self.validate_servo_param(self.steer_servo_lat_labels, 'steer_servo_lat_labels', BaseController.NUM_STEERS)
        self.validate_servo_param(self.steer_servo_angle_offsets, 'steer_servo_angle_offsets', BaseController.NUM_STEERS)

        # Build Steer Servo Handlers
        self._steer_servos = self.build_servos(BaseController.NUM_STEERS, self.steer_servo_ids, self.steer_servo_lon_labels, self.steer_servo_lat_labels, self.steer_servo_angle_offsets, True)    

        # TODO: removed support for arduino version ...
        self._servo_driver = PythonServoDriver()
        self._servo_driver.set_servos(self._wheel_servos, self._steer_servos)

        # Tuning / calibration
        self.get_logger().info('Setting steer servo offsets...')
        self.set_steer_servo_offsets()

        # Odometry
        self.get_logger().info('Initialise odometry...')
        self._odometry = AckermannOdometry()
        self._odometry.reset(get_time_secs(self))
        self._odometry.set_wheel_params(
            self._wheel_radius,
            self._mid_wheel_lat_separation,
            self._wheel_radius_multiplier,
            self._mid_wheel_lat_separation_multiplier)

        self._init_odometry()

        # Encoder filters
        self._wheel_servo_duty = []
        self._encoder_filters = []

        for i in range(BaseController.NUM_WHEELS):
            wheel_servo_duty = 0
            encoder_filter = LX16AEncoderFilter(node=self, classifier_filename = self._classifier_filename, regressor_filename = self._regressor_filename, window=self._classifier_window)
            
            # Invert the encoder filters on the right side
            servo = self._wheel_servos[i]
            if servo.lat_label == Servo.RIGHT:
                encoder_filter.set_invert(True)  
            
            # Reset encoder filters      
            pos = self._servo_driver.get_wheel_position(i)    
            encoder_filter.reset(pos)

            self._wheel_servo_duty.append(wheel_servo_duty)
            self._encoder_filters.append(encoder_filter)
 
        # Encoder messages (primarily for debugging)
        self._wheel_encoders = [LX16AEncoder() for i in range(BaseController.NUM_WHEELS)] 

        # Register shutdown behaviour
        rclpy.get_default_context().on_shutdown(self.shutdown)

        # Manages access to odometry data
        self.odom_lock = threading.Lock()

        # Create ROS communications
        self.create_ros_comms()

    def create_ros_comms(self):

        ## PUBLISHERS, SRV. CLIENTS & BROADCASTERS ...............................................
        # Odometry
        self._odom_msg = Odometry()
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Encoder messages
        self._encoders_msg = CurioServoEncoders()
        self._encoders_pub = self.create_publisher(CurioServoEncoders, '/servo/encoders', 10)

        # Transform broadcaster
        self._odom_broadcaster = TransformBroadcaster(self)

        # Servo positions
        self._positions_msg = CurioServoPositions()
        self._positions_msg.header.frame_id = 'base_link'
        self._positions_pub = self.create_publisher(CurioServoPositions, '/servo/positions', 10)

        # Servo states
        self._states_msg = CurioServoStates()
        self._states_pub = self.create_publisher(CurioServoStates, '/servo/states', 10)

        ## SUBSCRIBERS, SERVERS & ACTION SERVERS ...............................................

        # Commanded velocity
        self._cmd_vel_timeout = 0.5 #Duration(seconds=0.5)
        self._cmd_vel_last_rec_time = get_time_secs(self)
        self._cmd_vel_msg = Twist()
        self._cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_callback, 10)

        # Servo commands
        self._servo_cmd_msg = CurioServoCommands()
        self._servo_cmd_sub = self.create_subscription(Twist, 'servo/commands', self._servo_cmd_callback, 10)

    def load_ros_params(self):
        ''' Load default parameters and configure them from the param server.
        '''
        
        # Wheel geometry on a flat surface 
        self._wheel_radius = get_param_default(self,'wheel_radius',0.060)
        self._mid_wheel_lat_separation    = get_param_default(self,'mid_wheel_lat_separation', 0.052)
        self._front_wheel_lat_separation  = get_param_default(self,'front_wheel_lat_separation', 0.047)
        self._front_wheel_lon_separation  = get_param_default(self,'front_wheel_lon_separation', 0.028)
        self._back_wheel_lat_separation   = get_param_default(self,'back_wheel_lat_separation', 0.047)
        self._back_wheel_lon_separation   = get_param_default(self,'back_wheel_lon_separation', 0.025)
    
        self.get_logger().info('wheel_radius: {:.2f}'.format(self._wheel_radius))
        self.get_logger().info('mid_wheel_lat_separation: {:.2f}'.format(self._mid_wheel_lat_separation))
        self.get_logger().info('front_wheel_lat_separation: {:.2f}'.format(self._front_wheel_lat_separation))
        self.get_logger().info('front_wheel_lon_separation: {:.2f}'.format(self._front_wheel_lon_separation))
        self.get_logger().info('back_wheel_lat_separation: {:.2f}'.format(self._back_wheel_lat_separation))
        self.get_logger().info('back_wheel_lon_separation: {:.2f}'.format(self._back_wheel_lon_separation))

        # Odometry calibration parameters
        self._wheel_radius_multiplier               = get_param_default(self,'wheel_radius_multiplier',1.0)
        self._mid_wheel_lat_separation_multiplier   = get_param_default(self,'mid_wheel_lat_separation_multiplier',1.0)

        self.get_logger().info('wheel_radius_multiplier: {:.2f}'.format(self._wheel_radius_multiplier))
        self.get_logger().info('mid_wheel_lat_separation_multiplier: {:.2f}'.format(self._mid_wheel_lat_separation_multiplier))

        # Wheel servo parameters - required
        self.wheel_servo_ids           = get_param_or_die(self,'wheel_servo_ids')
        self.wheel_servo_lon_labels    = get_param_or_die(self,'wheel_servo_lon_labels')
        self.wheel_servo_lat_labels    = get_param_or_die(self,'wheel_servo_lat_labels')

        # Steer servo parameters - required
        self.steer_servo_ids           = get_param_or_die(self,'steer_servo_ids')
        self.steer_servo_lon_labels    = get_param_or_die(self,'steer_servo_lon_labels')
        self.steer_servo_lat_labels    = get_param_or_die(self,'steer_servo_lat_labels')
        self.steer_servo_angle_offsets = get_param_or_die(self,'steer_servo_angle_offsets')

        # Encoder filters
        self._classifier_window        = get_param_default(self,'classifier_window', 10)
        self._classifier_filename      = get_param_or_die(self,'classifier_filename')
        self._regressor_filename       = get_param_or_die(self,'regressor_filename')

        # Set up control loop rate
        self.control_frequency         = get_param_default(self,'control_frequency',10.0)
        self.odometry_frequency         = get_param_default(self,'odometry_frequency',20.0)

    def calc_position(self,lon_label, lat_label):
        ''' Calculate servo positions using the wheel geometry parameters
        '''
        if lon_label == Servo.FRONT:
            if lat_label == Servo.LEFT:
                return [self._front_wheel_lon_separation, self._front_wheel_lat_separation/2.0]
            if lat_label == Servo.RIGHT:
                return [self._front_wheel_lon_separation, -self._front_wheel_lat_separation/2.0]
        if lon_label == Servo.MID:
            if lat_label == Servo.LEFT:
                return [0.0, self._mid_wheel_lat_separation/2.0]
            if lat_label == Servo.RIGHT:
                return [0.0, -self._mid_wheel_lat_separation/2.0]
        if lon_label == Servo.BACK:
            if lat_label == Servo.LEFT:
                return [-self._back_wheel_lon_separation, self._back_wheel_lat_separation/2.0]
            if lat_label == Servo.RIGHT:
                return [-self._back_wheel_lon_separation, -self._back_wheel_lat_separation/2.0]

        return [0.0, 0.0]

    # Utility for validating servo parameters
    def validate_servo_param(self,param, name, expected_length):
        if len(param) != expected_length:
            self.get_logger().fatal("Parameter '{}' must be an array length {}, got: {}".format(name, expected_length, len(param)))
    
    def build_servos(self,num_servos, id_list, lon_labels, lat_labels,angle_offsets=None, is_steer=False):
        servos = []
        for i in range(num_servos):
            id = id_list[i]
            lon_label = Servo.to_lon_label(lon_labels[i])
            lat_label = Servo.to_lat_label(lat_labels[i])
            orientation = 1 
            if is_steer or lat_label != Servo.LEFT:
                orientation = -1 
            servo = Servo(id, lon_label, lat_label, orientation)
            if is_steer:
                servo.offset = angle_offsets[i] 
            servo.position = self.calc_position(lon_label, lat_label)
            servos.append(servo)
            self.get_logger().info('servo: id: {}, lon_label: {}, lat_label: {}, orientation: {}, offset: {}, position: {}'
                .format(servo.id, servo.lon_label, servo.lat_label, servo.orientation, servo.offset, servo.position))
        return servos


    def start_loop(self):
        self.get_logger().info('Starting control loops at {} Hz'.format(self.control_frequency))
        self.publisher_timer = self.create_timer( 1.0 / self.control_frequency, self.update)
        self.odometry_timer = self.create_timer( 1.0 / self.odometry_frequency, self.update_odometry)


    def move(self, lin_vel, ang_vel):
        ''' Move the robot given linear and angular velocities
        for the base.

        The linear and angular velocity arguments refer to the robot's
        base_link reference frame. We assume that the base_link origin
        is located at the mid-point between the two middle
        (non-steering) wheels. 
        
        The velocities and separation of the middle wheels are used to
        determine a turning radius and rate of turn. Given this the
        velocity and steering angle are then calculated for each wheel.

        Parameters
        ----------
        lin_vel : float
            The linear velocity of base_link frame [m/s].
        ang_vel : float
            The angular velocity of base_link frame [rad/s].
        '''

        # Check for timeout
        has_timed_out = get_time_secs(self) > self._cmd_vel_last_rec_time + self._cmd_vel_timeout

        # Calculate the turning radius and rate 
        r_p, omega_p = turning_radius_and_rate(lin_vel, ang_vel, self._mid_wheel_lat_separation)
        self.get_logger().debug('r_p: {:.2f}, omega_p: {:.2f}'.format(r_p, omega_p))

        # Calculate velocity and steering angle for each wheel
        wheel_vel_max = 0.0
        wheel_lin_vel = []
        steer_angle = []
        if omega_p == 0:
            # Body frame has no angular velocity - set wheel velocity directly
            vel = 0.0 if has_timed_out else lin_vel
            wheel_vel_max = math.fabs(vel)
            for servo in self._wheel_servos:
                wheel_lin_vel.append(vel)

            for servo in self._steer_servos:
                steer_angle.append(0.0)

        else:
            for servo in self._wheel_servos:
                # Wheel position
                id = servo.id
                x = servo.position[0]
                y = servo.position[1]

                # Wheel turn radius
                r = math.sqrt(x*x + (r_p - y)*(r_p - y))

                # Wheel velocity
                sgn = -1 if (r_p - y) < 0 else 1
                vel = sgn * r * omega_p 
                vel = 0.0 if has_timed_out else vel
                wheel_vel_max = max(wheel_vel_max, math.fabs(vel))
                wheel_lin_vel.append(vel)
                # self.get_logger().debug("id: {}, r: {:.2f}, wheel_lin_vel: {:.2f}".format(id, r, vel))

            for servo in self._steer_servos:
                # Wheel position
                id = servo.id
                x = servo.position[0]
                y = servo.position[1]

                # Wheel angle
                angle = math.atan2(x, (r_p - y))
                steer_angle.append(angle)
                # self.get_logger().debug("id: {}, angle: {:.2f}".format(id, degree(angle)))

        # Apply speed limiter - preserving turning radius
        if wheel_vel_max > BaseController.LINEAR_VEL_MAX:
            speed_limiter_sf = BaseController.LINEAR_VEL_MAX / wheel_vel_max
            for i in range(len(wheel_lin_vel)):
                wheel_lin_vel[i] = wheel_lin_vel[i] * speed_limiter_sf

        # Update steer servos
        # @TODO link the time of the move to the angle which the servos turn through
        self.get_logger().debug('Updating steer servos')
        for i in range(BaseController.NUM_STEERS):
            servo = self._steer_servos[i]

            # Input angles are in radians
            angle_deg = degree(steer_angle[i])

            # Transition from turning radius outside the base footprint to inside
            # (i.e in-place turning) 
            if angle_deg > BaseController.REVERSE_SERVO_ANGLE:
                angle_deg = angle_deg - 180

            if angle_deg < -BaseController.REVERSE_SERVO_ANGLE:
                angle_deg = 180 + angle_deg

            # Map steering angle degrees [-120, 120] to servo position [0, 1000]
            servo_pos = int(map(angle_deg * servo.orientation,
                -BaseController.SERVO_ANGLE_MAX, BaseController.SERVO_ANGLE_MAX,
                BaseController.SERVO_POS_MIN, BaseController.SERVO_POS_MAX))

            self.get_logger().debug('id: {}, angle: {:.2f}, servo_pos: {}'.format(servo.id, angle_deg, servo_pos))
            self._servo_driver.set_steer_command(i, servo_pos)

        # Update wheel servos
        self.get_logger().debug('Updating wheel servos')
        for i in range(BaseController.NUM_WHEELS):
            servo = self._wheel_servos[i]

            # Wheel angular velocity
            wheel_ang_vel = wheel_lin_vel[i] / self._wheel_radius

            # Map speed to servo duty [-1000, 1000]
            duty = int(map(wheel_ang_vel * servo.orientation,
                -BaseController.SERVO_ANG_VEL_MAX, BaseController.SERVO_ANG_VEL_MAX, 
                -BaseController.SERVO_DUTY_MAX, BaseController.SERVO_DUTY_MAX))

            # Set servo speed
            self.get_logger().debug('id: {}, wheel_ang_vel: {:.2f}, servo_vel: {}'
                .format(servo.id, wheel_ang_vel, duty))
            self._servo_driver.set_wheel_command(i, duty)

            # Update duty array (needed for servo position classifier)
            self._wheel_servo_duty[i] = duty

    def servo_move(self,wheel_commands, steer_commands):
        ''' Move the robot given servo values.
        '''

        # Check for timeout
        has_timed_out = get_time_secs(self) > self._servo_cmd_last_rec_time + self._cmd_vel_timeout

        # Update steer servos
        self.get_logger().debug('Updating steer servos')
        for i in range(BaseController.NUM_STEERS):
            servo = self._steer_servos[i]
            servo_pos = 0 if has_timed_out else steer_commands[i]
            self.get_logger().debug('\tid: {}, servo_pos: {}'.format(servo.id, servo_pos))
            self._servo_driver.set_steer_command(i, servo_pos)

        # Update wheel servos
        self.get_logger().debug('Updating wheel servos')
        for i in range(BaseController.NUM_WHEELS):
            servo = self._wheel_servos[i]
            duty =  0 if has_timed_out else wheel_commands[i]
            self.get_logger().debug('\tid: {},  servo_vel: {}'.format(servo.id, duty))
            self._servo_driver.set_wheel_command(i, duty)

            # Update duty array (needed for servo position classifier)
            self._wheel_servo_duty[i] = duty

    def set_steer_servo_offsets(self):
        ''' Set angle offsets for the steering servos.
        
        The offsets are specified in the node parameters are are
        adjusted to ensure each corner wheel is centred when the robot
        is commanded to move with no turn.
        '''

        # Set the steering servo offsets to centre the corner wheels
        for i in range(BaseController.NUM_STEERS):
            servo = self._steer_servos[i]
            self.get_logger().info('id: {}, offset: {}'.format(servo.id, servo.offset))
            self._servo_driver.set_angle_offset(i, servo.offset)

    def stop(self):
        ''' Stop all servos
        '''

        self.get_logger().info('Stopping all servos')
        for i in range(BaseController.NUM_WHEELS):
            self._servo_driver.set_wheel_command(i, 0)

        self._servo_driver.publish_commands()

    def _cmd_vel_callback(self, msg):
        ''' Callback for the subscription to `/cmd_vel`.

        The callback updates the current command, and also a watchdog
        timer so that if cmd_vel messages stop, the motors stop.

        Parameters
        ----------
        msg : geometry_msgs.msg/Twist
            The message for the commanded velocity.
        '''

        self.get_logger().debug('cmd_vel: linear: {}, angular: {}'.format(msg.linear.x, msg.angular.z))
        self._cmd_vel_last_rec_time = get_time_secs(self)
        self._cmd_vel_msg = msg

        # vel commands disable servo commands
        self._servo_cmd_last_rec_time = 0
        self._servo_cmd_msg = CurioServoCommands()

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

        # servo commands disable vel commands
        self._cmd_vel_last_rec_time = 0
        self._cmd_vel_msg = Twist()


    def update(self):
        ''' Callback for the control loop.        
        This to be called at the control loop frequency by start_loop function, 
        managed by a rclpy.create_timer

        '''

        # Get the current real time (just before this function was called)
        (time, time_stamp) = get_time_and_secs(self)
        

        # Publish
        self._publish_odometry(time_stamp)        
        self._publish_tf(time_stamp)
        self._publish_encoders(time_stamp)

        self._publish_positions(time)
        self._publish_states(time_stamp)

        # PID control would go here...

        # Exec commands
        if self._cmd_vel_last_rec_time>0:
            self.move(self._cmd_vel_msg.linear.x, self._cmd_vel_msg.angular.z)
        if self._servo_cmd_last_rec_time>0:
            self.servo_move(self._servo_cmd_msg.wheel_commands, self._servo_cmd_msg.steer_commands)
        

    def _publish_positions(self, time):
        # Servo positions
        self._positions_msg.header.stamp = time
        
        self.odom_lock.acquire()
        self._positions_msg.wheel_positions = self.wheel_servo_pos
        self._positions_msg.steer_positions = self.steer_servo_pos
        self.odom_lock.release()

    
        self._positions_pub.publish(self._positions_msg)

    def get_steer_positions(self):
        steer_angles = []
        for servo in self._steer_servos:
            # Wheel position
            x = servo.position[0]
            y = servo.position[1]

            # Wheel angle
            angle = math.atan2(x, (r_p - y))
            steer_angles.append(angle)


    def _publish_states(self, time_stamp):
        # Servo states
        self._states_msg = CurioServoStates()
        self._states_pub.publish(self._states_msg )


    def shutdown(self):
        ''' Called by the node shutdown hook on exit.
        '''
        self.get_logger().info('Shutdown Curio base controller...')
        # Stop all servos - @TODO add e-stop with latch.
        self.stop()

    #################################################################### 
    # Odometry related

    def _reset_encoders(self):
        ''' Reset the encoders
        '''

        # Reset encoder filters
        for i in range(BaseController.NUM_WHEELS):
            servo = self._wheel_servos[i]
            filter = self._encoder_filters[i]
            pos = self._servo_driver.get_wheel_position(i)
            filter.reset(pos)

    # @TODO: Gives errors when running at low control loop update rate
    #        (e.g. < 15Hz).
    # 
    # The error is because the encoder filter is not updated fast enough
    # to make two measurements close enough in time either side of a
    # discontinuity in the servo position to see a jump of 1000-1400
    # counts. Instead the encoder suffers from aliasing. As result the
    # odometry will work at low velocities and then suddenly fail as
    # it is increased.
    # 
    def _update_all_wheel_servo_positions(self, time):
        ''' Get the servo positions in radians for all wheels.

        Parameters
        ----------
        time : The current time as float
            The current time.

        Returns
        -------
        list
            A list of 6 floats containing the angular position
            of each of the wheel servos [rad].
        '''

        servo_positions = [0 for i in range(BaseController.NUM_WHEELS)]
        msg = 'time: {}, '.format(time)
        for i in range (BaseController.NUM_WHEELS):
            servo = self._wheel_servos[i]
            filter = self._encoder_filters[i]

            # Calculate the encoder count
            duty = self._wheel_servo_duty[i]
            pos = self._servo_driver.get_wheel_position(i)
            filter.update(time, duty, pos)
            count = filter.get_count()
            theta = filter.get_angular_position()
            servo_positions[i] = theta

            # Append to debug message
            msg = msg + "{}: {}, ".format(servo.id, count)

        self.get_logger().info(msg)
        return servo_positions

def _update_all_steer_servo_positions(self, time):
        ''' Get the servo positions in radians for all steering servos.

        Parameters
        ----------
        time : The current time as float
            The current time.

        Returns
        -------
        list
            A list of 6 floats containing the angular position
            of each of the steer servos [rad].
        '''

        servo_positions = [0 for i in range(BaseController.NUM_STEERS)]
        msg = 'time: {}, '.format(time)
        for i in range (BaseController.NUM_STEERS):
            servo = self._steer_servos[i]

            # Calculate the encoder count
            duty = self._steer_servo_duty[i]

            servo_positions[i] = theta

        return servo_positions

    def _update_mid_wheel_servo_positions(self, time):
        ''' Update the servo positions in radians for the
        left and right mid wheels.

        Parameters
        ----------
        time : The current time as float

        Returns
        -------
        list
            A list of 2 floats containing the angular position
            of the left and right mid wheel servos [rad].
        '''

        # @TODO: resolve hardcoded index
        left_pos  = self._update_wheel_servo_position(time, 1)
        right_pos = self._update_wheel_servo_position(time, 4)

        servo_positions = [0 for i in range(2)]
        servo_positions[Servo.LEFT]  = left_pos
        servo_positions[Servo.RIGHT] = right_pos

        self.get_logger().debug("time: {}, left: {}, right: {}".format(time, left_pos, right_pos))

        return servo_positions

    def _update_wheel_servo_position(self, time, i):
        ''' Update the servo positions in radians for the i-th wheel.

        Parameters
        ----------
        time : The current time as float.
        i : int
            The index of the i-th wheel.

        Returns
        -------
        float
            The angular position of the wheel servo [rad].
        '''

        servo = self._wheel_servos[i]
        filter = self._encoder_filters[i]

        # Calculate the encoder count
        duty = self._wheel_servo_duty[i]
        pos = self._servo_driver.get_wheel_position(i)
        filter.update(time, duty, pos)
        count = filter.get_count()
        theta = filter.get_angular_position()
        return theta

    def _init_odometry(self):
        ''' Initialise the odometry

        Initialise the time independent parameters of the
        odometry message.
        '''

        odom_frame_id = 'odom'
        base_frame_id = 'base_link'
        pose_cov_diag = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01] 
        twist_cov_diag = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01] 

        self._odom_msg.header.frame_id = odom_frame_id
        self._odom_msg.child_frame_id  = base_frame_id

        self._odom_msg.pose.pose.position.y = 0.0
        self._odom_msg.pose.covariance = [
            pose_cov_diag[0], 0., 0., 0., 0., 0.,
            0., pose_cov_diag[1], 0., 0., 0., 0.,
            0., 0., pose_cov_diag[2], 0., 0., 0.,
            0., 0., 0., pose_cov_diag[3], 0., 0.,
            0., 0., 0., 0., pose_cov_diag[4], 0.,
            0., 0., 0., 0., 0., pose_cov_diag[5]
        ]

        self._odom_msg.twist.twist.linear.y  = 0.0
        self._odom_msg.twist.twist.linear.z  = 0.0
        self._odom_msg.twist.twist.angular.x = 0.0
        self._odom_msg.twist.twist.angular.y = 0.0
        self._odom_msg.twist.covariance = [
            twist_cov_diag[0], 0., 0., 0., 0., 0.,
            0., twist_cov_diag[1], 0., 0., 0., 0.,
            0., 0., twist_cov_diag[2], 0., 0., 0.,
            0., 0., 0., twist_cov_diag[3], 0., 0.,
            0., 0., 0., 0., twist_cov_diag[4], 0.,
            0., 0., 0., 0., 0., twist_cov_diag[5]
        ]

    def _publish_odometry(self, _stamp):
        ''' Populate the nav_msgs.Odometry message and publish.

        Parameters
        ----------
        _stamp : Current time as Node.get_clock().now().to_msg().
        '''

        # self.get_logger().info('x: {:.2f}, y: {:.2f}, heading: {:.2f}, lin_vel: {:.2f}, ang_vel: {:.2f}'
        #     .format(
        #         self._odometry.get_x(),
        #         self._odometry.get_y(),
        #         self._odometry.get_heading(),
        #         self._odometry.get_lin_vel(),
        #         self._odometry.get_ang_vel()))

        self.odom_lock.acquire()
        quat = quaternion_from_euler(0.0, 0.0, self._odometry.get_heading())
        self._odom_msg.pose.pose.position.x   = self._odometry.get_x()
        self._odom_msg.pose.pose.position.y   = self._odometry.get_y()
        self._odom_msg.twist.twist.linear.x    = self._odometry.get_lin_vel()
        self._odom_msg.twist.twist.angular.z   = self._odometry.get_ang_vel()
        self.odom_lock.release()

        self._odom_msg.header.stamp = _stamp
        self._odom_msg.pose.pose.orientation.x = quat[0]
        self._odom_msg.pose.pose.orientation.y = quat[1]
        self._odom_msg.pose.pose.orientation.z = quat[2]
        self._odom_msg.pose.pose.orientation.w = quat[3]

        self._odom_pub.publish(self._odom_msg)

    def update_odometry(self, time):
        ''' Update odometry

        This is the same calculation as used in the odometry
        for the ackermann_drive_controller.

        Parameters
        ----------
        time : The current time as float.
        '''

        # Get the angular position of the all wheel servos [rad] and
        # update odometry
        wheel_servo_pos = self._update_all_wheel_servo_positions(time)
        steer_servo_pos = self._update_all_steer_servo_positions(time)

        self.odom_lock.acquire()
        self.wheel_servo_pos = wheel_servo_pos
        self.steer_servo_pos = steer_servo_pos
        self._odometry.update_6(self.wheel_servo_pos, time)
        self.odom_lock.release()

        # Get the angular position of the mid wheel servos [rad]
        # and update odometry
        # wheel_servo_pos = self._update_mid_wheel_servo_positions(time)
        # self._odometry.update_2(wheel_servo_pos, time)

    def _publish_tf(self, _stamp):
        ''' Publish the transform from 'odom' to 'base_link'

        Parameters
        ----------
        _stamp : Current time as Node.get_clock().now().to_msg().
        '''

        # Broadcast the transform from 'odom' to 'base_link'
        newTF = TransformStamped()
        newTF.header.stamp = _stamp  # time_stamp
        newTF.header.frame_id = 'odom' #  parent  
        newTF.child_frame_id = 'base_link'  # child
        newTF.transform.translation.z = 0.0  # translation
        self.odom_lock.acquire()
        newTF.transform.translation.x = self._odometry.get_x()
        newTF.transform.translation.y = self._odometry.get_y()
        newTF.transform.rotation = quaternion_from_euler(0.0, 0.0, self._odometry.get_heading())  #  rotation
        self.odom_lock.release()
            
        self._odom_broadcaster.sendTransform(newTF)



    # @IMPLEMENT
    def update_state(self, time):
        ''' Update the rover's status

        Parameters
        ----------
        time : The current time as float
        '''

        pass

    # @IMPLEMENT
    def publish_states(self, time):
        ''' Publish the rover's status

        Parameters
        ----------
        time : The current time as float.
        '''

        pass

    def _publish_encoders(self, _stamp):
        ''' Publish the encoder state

        Parameters
        ----------
        _stamp : Current time as Node.get_clock().now().to_msg().
        '''
        # Update the encoder messages
        for i in range(BaseController.NUM_WHEELS):
            servo = self._wheel_servos[i]
            filter = self._encoder_filters[i]
            pos, is_valid = filter.get_servo_pos(False)
            msg = self._wheel_encoders[i]            
            msg.id = servo.id
            msg.duty = filter.get_duty()
            msg.position = pos
            msg.is_valid = is_valid
            msg.count = filter.get_count()
            msg.revolutions = filter.get_revolutions()

        # Publish
        self._encoders_msg.header.stamp = _stamp
        self._encoders_msg.header.frame_id = 'base_link'
        self._encoders_msg.wheel_encoders = self._wheel_encoders
        self._encoders_pub.publish(self._encoders_msg)
