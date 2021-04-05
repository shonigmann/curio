#!/usr/bin/env python

from curio_base.utils import get_time_secs, get_time_and_secs, turning_radius_and_rate, degree, quaternion_from_euler
from curio_base.lx16a_encoder_filter import LX16AEncoderFilter
from curio_base.servo import Servo
from curio_base.python_servo_driver import PythonServoDriver
from curio_base.ackermann_odometry import  AckermannOdometry

from curio_base.utils import LatLabel, LonLabel, get_param_or_die, get_param_default

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

        # Build Steer Servo data containers
        self._steer_servos = self.build_servos(BaseController.NUM_STEERS, self.steer_servo_ids, self.steer_servo_lon_labels, self.steer_servo_lat_labels, self.steer_servo_angle_offsets, True)    

        # TODO: removed support for arduino version ...
        self._servo_driver = PythonServoDriver()
        self._servo_driver.set_servos(self._wheel_servos, self._steer_servos)
        # Once servo driver is fully configured, it can start
        self._servo_driver.start()

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
            if servo.lat_label == LatLabel.RIGHT:
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
        self._encoders_msg.header.frame_id = 'base_link'
        self._encoders_pub = self.create_publisher(CurioServoEncoders, '/servo/encoders', 10)

        # Transform broadcaster
        # Broadcast the transform from 'odom' to 'base_link'
        self.newTF = TransformStamped()
        self.newTF.header.frame_id = 'odom' #  parent  
        self.newTF.child_frame_id = 'base_link'  # child
        self.newTF.transform.translation.z = 0.0  # translation        
        self._odom_broadcaster = TransformBroadcaster(self)

        ## SUBSCRIBERS, SERVERS & ACTION SERVERS ...............................................

        # Commanded velocity
        self._cmd_vel_timeout = 0.5
        self._cmd_vel_last_rec_time = get_time_secs(self)
        self._cmd_vel_msg = Twist()
        self._cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_callback, 10)

    def load_ros_params(self):
        ''' Load default parameters and configure them from the param server.
        '''
        
        # Wheel geometry on a flat surface 
        self._wheel_radius = get_param_default(self,'wheel_radius',0.060)
        Servo._mid_wheel_lat_separation    = self._mid_wheel_lat_separation    = get_param_default(self,'mid_wheel_lat_separation', 0.052)
        Servo._front_wheel_lat_separation  = self._front_wheel_lat_separation  = get_param_default(self,'front_wheel_lat_separation', 0.047)
        Servo._front_wheel_lon_separation  = self._front_wheel_lon_separation  = get_param_default(self,'front_wheel_lon_separation', 0.028)
        Servo._back_wheel_lat_separation   = self._back_wheel_lat_separation   = get_param_default(self,'back_wheel_lat_separation', 0.047)
        Servo._back_wheel_lon_separation   = self._back_wheel_lon_separation   = get_param_default(self,'back_wheel_lon_separation', 0.025)
    
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

    def validate_servo_param(self,param, name, expected_length):
        '''Make sure arrays have correct lenght'''
        if len(param) != expected_length:
            self.get_logger().fatal("Parameter '{}' must be an array length {}, got: {}".format(name, expected_length, len(param)))
 
    def build_servos(self,num_servos, id_list, lon_labels, lat_labels,angle_offsets=None, is_steer=False):
        """Create servo objects

        Args:
        num_servos ([int]): 
        id_list ([int]): servo serial id
        lon_labels ([string]): [front,mid,back]
        lat_labels ([string]): [left,right]
        angle_offsets ([type], optional): [steering servo offset]. Defaults to None.
        is_steer (bool, optional): [are the steering servos?]. Defaults to False.

        Returns:
        [type]: [description]
        """
        servos = []
        for i in range(num_servos):
            if not is_steer:
                angle_offset = None
            else: 
                angle_offset = angle_offsets[i]
            servo = Servo(id_list[i], lon_labels[i],lat_labels[i], angle_offset, is_steer)
            servos.append(servo)
            self.get_logger().info('servo: id: {}, lon_label: {}, lat_label: {}, orientation: {}, offset: {}, position: {}'.format(servo.id, servo.lon_label, servo.lat_label, servo.orientation, servo.offset, servo.position))
        return servos
    
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
            self._servo_driver.set_angle_offset(i)

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

    def shutdown(self):
        ''' Called by the node shutdown hook on exit.
        '''
        self.get_logger().info('Shutdown Curio base controller...')
        # Stop all servos - @TODO add e-stop with latch.
        self.stop()

    def stop(self):
        ''' Stop all servos
        '''
        self.get_logger().info('Stopping all servos')
        for i in range(BaseController.NUM_WHEELS):
            self._servo_driver.set_wheel_command(i, 0)

    def start_loop(self):
        self.get_logger().info('Starting control loops at {} Hz'.format(self.control_frequency))
        self.publisher_timer = self.create_timer( 1.0 / self.control_frequency, self.update)
        self.odometry_timer = self.create_timer( 1.0 / self.odometry_frequency, self.update_odometry)

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

        # PID control would go here...

        # Exec commands
        self.move()
              
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
        self._odom_msg.pose.pose.orientation = quat

        self._odom_pub.publish(self._odom_msg) 

    def _publish_tf(self, _stamp):
        ''' Publish the transform from 'odom' to 'base_link'

        Parameters
        ----------
        _stamp : Current time as Node.get_clock().now().to_msg().
        '''

        self.newTF.header.stamp = _stamp  # time_stamp
        self.odom_lock.acquire()
        self.newTF.transform.translation.x = self._odometry.get_x()
        self.newTF.transform.translation.y = self._odometry.get_y()
        self.newTF.transform.rotation = quaternion_from_euler(0.0, 0.0, self._odometry.get_heading())  #  rotation
        self.odom_lock.release()
            
        self._odom_broadcaster.sendTransform(self.newTF)            

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
        self._encoders_msg.wheel_encoders = self._wheel_encoders
        self._encoders_pub.publish(self._encoders_msg)

    def move(self):
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
        self._servo_cmd_msg = CurioServoCommands()
        self._servo_cmd_msg.header = self._cmd_vel_msg.header
        lin_vel = self._cmd_vel_msg.linear.x
        ang_vel = self._cmd_vel_msg.angular.z

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
