#!/usr/bin/env python

import math 

from rclpy.node import Node

from curio_base.mean_window_filter import MeanWindowFilter
from curio_base.servo import Servo
from curio_base.utils import get_time_secs


class AckermannOdometry(Node):
    ''' Odometry for the base controller (6 wheel Ackermann)
    
    This class based on its C++ equivalent in the
    `ackermann_drive_controller` module which in turn was derived
    from the `diff_drive_controller` in `ros_controllers`.

    Original odometry code:
    https://github.com/ros-controls/ros_controllers/diff_drive_controller

    License: BSD-3-Clause

    Copyright (c) 2013, PAL Robotics, S.L.
    All rights reserved.

    Authors
        Luca Marchionni
        Bence Magyar
        Enrique Fernandez
        Paul Mathieu
    '''

    def __init__(self):
        '''
        Constructor
        '''
        super().__init__('AckermannOdometry')
        self._timestamp = get_time_secs(self)
        self._heading = 0.0     # [rad]
        self._x       = 0.0     # [m]
        self._y       = 0.0     # [m]
        self._lin_vel_filter = MeanWindowFilter(window=5)   # [m/s]
        self._ang_vel_filter = MeanWindowFilter(window=5)   # [rad/s]
        self._wheel_radius = 0.06                           # [m]
        self._mid_wheel_lat_separation = 0.52               # [m]
        self._wheel_radius_multiplier = 1.0                 # [1]
        self._mid_wheel_lat_separation_multiplier = 1.0     # [1]
        self._num_wheels = 6
        self._wheel_cur_pos = [0.0 for x in range(self._num_wheels)] # [m]
        self._wheel_old_pos = [0.0 for x in range(self._num_wheels)] # [m]
        self._wheel_est_vel = [0.0 for x in range(self._num_wheels)] # [m/s]

    def reset(self, time):
        ''' Reset the odometry

        Parameters
        ----------
        time : The current time as float
        '''        

        self._timestamp = time

    def update_6(self, wheel_servo_pos, time):
        ''' Update the odometry with the latest wheel servo positions.

        Parameters
        ----------
        wheel_servo_pos : list
            A list of 6 floats denoting the angular position of the
            6 wheel servos [rad].
        time : The current time as float
            The current time.
        '''

        # Adjust the wheel radius and separation by the calibrated multipliers        
        wheel_rad = self._wheel_radius * self._wheel_radius_multiplier
        wheel_sep = self._mid_wheel_lat_separation * self._mid_wheel_lat_separation_multiplier

        for i in range(self._num_wheels):
            # Get the current wheel joint (linear) positions [m]
            self._wheel_cur_pos[i] = wheel_servo_pos[i] * wheel_rad

            # Estimate the velocity of the wheels using old and current positions
            self._wheel_est_vel[i] = self._wheel_cur_pos[i] - self._wheel_old_pos[i]

            # Update old position with current
            self._wheel_old_pos[i] = self._wheel_cur_pos[i]

        # @TODO - remove hardcoding and use a lookup instead
        MID_RIGHT = 1
        MID_LEFT  = 4

        # Compute linear and angular velocities of the mobile base (base_link frame)
        lin_vel = (self._wheel_est_vel[MID_RIGHT] + self._wheel_est_vel[MID_LEFT]) * 0.5
        ang_vel = (self._wheel_est_vel[MID_RIGHT] - self._wheel_est_vel[MID_LEFT]) / wheel_sep

        # Integrate the velocities to get the linear and angular positions
        self._integrate_velocities(lin_vel, ang_vel)

        # Cannot estimate the speed for small time intervals
        dt = (time - self._timestamp)
        if dt < 0.0001:
            return False

        # Estimate speeds using a rolling mean / mode to filter them
        self._timestamp = time

        # Add to velocity filters
        self._lin_vel_filter.update(lin_vel/dt)
        self._ang_vel_filter.update(ang_vel/dt)

        return True

    def update_2(self, wheel_servo_pos, time):
        ''' Update the odometry with the mid wheel servo positions.

        Parameters
        ----------
        wheel_servo_pos : list
            A list of 2 floats denoting the angular position of the
            2 mid wheel servos [rad]
        time : The current time as float
            The current time.
        '''

        # Adjust the wheel radius and separation by the calibrated multipliers        
        wheel_rad = self._wheel_radius * self._wheel_radius_multiplier
        wheel_sep = self._mid_wheel_lat_separation * self._mid_wheel_lat_separation_multiplier

        for i in range(2):
            # Get the current wheel joint (linear) positions [m]
            self._wheel_cur_pos[i] = wheel_servo_pos[i] * wheel_rad

            # Estimate the velocity of the wheels using old and current positions
            self._wheel_est_vel[i] = self._wheel_cur_pos[i] - self._wheel_old_pos[i]

            # Update old position with current
            self._wheel_old_pos[i] = self._wheel_cur_pos[i]

        LEFT  = Servo.LEFT
        RIGHT = Servo.RIGHT

        # Compute linear and angular velocities of the mobile base (base_link frame)
        lin_vel = (self._wheel_est_vel[RIGHT] + self._wheel_est_vel[LEFT]) * 0.5
        ang_vel = (self._wheel_est_vel[RIGHT] - self._wheel_est_vel[LEFT]) / wheel_sep

        # Integrate the velocities to get the linear and angular positions
        self._integrate_velocities(lin_vel, ang_vel)

        # Cannot estimate the speed for small time intervals
        dt = (time - self._timestamp)
        if dt < 0.0001:
            return False

        # Estimate speeds using a rolling mean / mode to filter them
        self._timestamp = time

        # Add to velocity filters
        self._lin_vel_filter.update(lin_vel/dt)
        self._ang_vel_filter.update(ang_vel/dt)

        return True

    def get_heading(self):
        ''' Get the heading [rad]

        The heading in radians, with zero being along the longtidinal
        axis (x), and positive rotation is towards the positive lateral
        axis (y) to the left.

        Returns
        -------
        float
            The heading in radians.
        '''

        return self._heading

    def get_x(self):
        ''' Get the x position [m]
        
        Returns
        -------
        float
            The x position [m].
        '''

        return self._x

    def get_y(self):
        ''' Get the y position [m]

        Returns
        -------
        float
            The y position [m].
        '''

        return self._y

    def get_lin_vel(self):
        ''' Get the linear velocity of the body [m/s]

        Returns
        -------
        float
            The linear velocity of the `base_link` [m/s].
        '''

        return self._lin_vel_filter.get_mean()

    def get_ang_vel(self):
        ''' Get the angular velocity of the body [rad/s]

        Returns
        -------
        float
            The angular velocity of the `base_link` [rad/s].
        '''

        return self._ang_vel_filter.get_mean()

    def set_wheel_params(self,
        wheel_radius,
        mid_wheel_lat_separation,
        wheel_radius_multiplier=1.0,
        mid_wheel_lat_separation_multiplier=1.0):
        ''' Set the wheel and steering geometry.

        Note: all wheels are assumed to have the same radius, and the
        mid wheels do not steer.

        Parameters
        ----------
        wheel_radius : float
            The radius of the wheels [m].
        mid_wheel_lat_separation : float
            The lateral separation [m] of the mid wheels.
        wheel_radius_multiplier : float
            Wheel radius calibration multiplier to tune odometry,
            has (default = 1.0).
        mid_wheel_lat_separation_multiplier : float
            Wheel separation calibration multiplier to tune odometry,
            has (default = 1.0).
        '''

        self._wheel_radius = wheel_radius
        self._mid_wheel_lat_separation = mid_wheel_lat_separation
        self._wheel_radius_multiplier = wheel_radius_multiplier
        self._mid_wheel_lat_separation_multiplier = mid_wheel_lat_separation_multiplier

    def _integrate_velocities(self, lin_vel, ang_vel):
        ''' Integrate the current velocities to obtain the current
        position and heading.

        Parameters
        ----------
        lin_vel : float
            The linear velocity of the `base_link`.
        ang_vel : float
            The angular velocity of the `base_link`.
        '''

        if math.fabs(ang_vel) < 1e-6:
            self._integrate_runge_kutta2(lin_vel, ang_vel)
        else:
            #  Exact integration (should solve problems when angular is zero):
            heading_old = self._heading
            r = lin_vel/ang_vel
            self._heading = self._heading + ang_vel
            self._x = self._x + r * (math.sin(self._heading) - math.sin(heading_old))
            self._y = self._y - r * (math.cos(self._heading) - math.cos(heading_old))

    def _integrate_runge_kutta2(self, lin_vel, ang_vel):
        ''' Integrate the current velocities to obtain the current
        position and heading.

        Parameters
        ----------
        lin_vel : float
            The linear velocity of the `base_link`.
        ang_vel : float
            The angular velocity of the `base_link`.
        '''
        direction = self._heading + ang_vel * 0.5

        # Runge-Kutta 2nd order integration:
        self._x = self._x + lin_vel * math.cos(direction)
        self._y = self._y + lin_vel * math.sin(direction)
        self._heading = self._heading + ang_vel
