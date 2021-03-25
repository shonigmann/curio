#!/usr/bin/env python

import math
from geometry_msgs.msg import Quaternion


def get_time_secs(a_node):
    t_msg = a_node.get_clock().now().to_msg()
    t_sec = t_msg.sec + (t_msg.nanosec / 1e9)
    return t_sec

def get_time_and_secs(a_node):
    t_msg = a_node.get_clock().now().to_msg()
    t_sec = t_msg.sec + (t_msg.nanosec / 1e9)
    return (t_msg, t_sec)

def quaternion_from_euler(roll, pitch, yaw):
    """
    From https://gist.github.com/salmagro/2e698ad4fbf9dae40244769c5ab74434
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.x = cy * cp * cr + sy * sp * sr
    q.y = cy * cp * sr - sy * sp * cr
    q.z = sy * cp * sr + cy * sp * cr
    q.w = sy * cp * cr - cy * sp * sr

    return q

def degree(rad):
    ''' Convert an angle in degrees to radians

    Parameters
    ----------
    rad : float
        An angle in radians

    Returns
    -------
    float
        The angle in degrees.
    '''

    return rad * 180.0 / math.pi

def radian(deg):
    ''' Convert an angle in radians to degrees

    Parameters
    ----------
    deg : float
        An angle in degrees

    Returns
    -------
    float
        The angle in radians.
    '''

    return deg * math.pi / 180.0

def map(x, in_min, in_max, out_min, out_max):
    ''' Map a value in one range to its equivalent in another.

    Parameters
    ----------
    x : float
        The value to be mapped
    in_min : float
        The minimum value the input variable can take. 
    in_max : float
        The maximum value the input variable can take. 
    out_min : float
        The minimum value the output variable can take. 
    out_max : float
        The maximum value the output variable can take. 

    Returns
    -------
    float
        The mapped value.
    '''

    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def clamp(x, lower, upper):
    ''' Clamp a value between a lower and upper bound.

    Parameters
    ----------
    x : float
        The value to be clamped
    in_min : float
        The lower limit of the clamp. 
    in_max : float
        The upper limit of the clamp. 

    Returns
    -------
    float
        The clamped value.
    '''

    return min(max(x, lower), upper)

def caseless_equal(left, right):
    ''' A case insensitive comparison.

    Parameters
    ----------
    left : str
        A string to compare. 
    right : str
        A string to compare. 

    Returns
    -------
    bool
        Return True if the strings are equal ignoring case. 

    '''
    return left.upper() == right.upper()

def turning_radius_and_rate(v_b, omega_b, d):
    ''' Calculate the turning radius and rate of turn about
    the instantaneous centre of curvature (ICC).

    Conventions are specifiied according to ROS REP 103:
    Standard Units of Measure and Coordinate Conventions
    https://www.ros.org/reps/rep-0103.html.

    x : forward
    y : left
    z : up

    Example
    -------
    v_b >= 0, omega_b > 0 => r_p > 0    positive turn (anti-clockwise),
    v_b >= 0, omega_b < 0 => r_p < 0    negative turn (clockwise),
    v_b >= 0, omega_b = 0 => r_p = inf  no turn.

    Parameters
    ----------
    v_b : float
        The linear velocity of the base [m/s].
    omega_b : float
        The angular velocity of the base [rad/s].
    d : float
        distance between the fixed wheels [m].

    Returns
    -------
    list
        A two element list containing r_p the turning radius [m]
        and and omega_p the rate of turn [rad/s]. If the motion
        has no angular component then r_p is float('Inf') and
        omega_p is zero.
    '''

    vl = v_b - d * omega_b / 2.0
    vr = v_b + d * omega_b / 2.0
    if vl == vr:
        return float('Inf'), 0.0
    else:
        r_p = d * (vr + vl) / (vr - vl) / 2.0
        omega_p = (vr - vl) / d
        return r_p, omega_p


