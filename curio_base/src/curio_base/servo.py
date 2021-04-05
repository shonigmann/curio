#!/usr/bin/env python


from curio_base.utils import caseless_equal, LatLabel, LonLabel
from curio_msgs.msg import LX16AState
from geometry_msgs.msg import Point

class Servo(object):
    '''Servo properties: Stores information about one servo.

    Attributes
    ----------
    lon_label : enum
        Enumeration label for the longitudinal direction:
        FRONT, MID, BACK.
    lat_label : enum
        Enumeration label for the lateral direction: LEFT, RIGHT.
    orientation : int
        Flag to indicate whether the servo is installed for positive(LEFT)
        or negative rotation: 1, -1.
    offset : int
        The servo position offset (for servo rather than motor mode).
        Used to centre servos.
    position : Point
        Servo position in the robot base. 
    '''

    #Static elements
    _mid_wheel_lat_separation = 0.052
    _front_wheel_lat_separation = 0.047
    _front_wheel_lon_separation = 0.028
    _back_wheel_lat_separation = 0.047
    _back_wheel_lon_separation = 0.025


    def __init__(self, _id, lon_label, lat_label, angle_offset, is_steer):
        ''' Constructor
        
        Parameters
        ----------
        _id : int
            servo serial id: 0 - 253.
        lon_label : str
            Label for the longitudinal direction:
            'front', 'mid', 'back'.
        lat_label : str
            Label for the lateral direction: 'left', 'right'.
        angle_offset: float
            The servo position offset (for servo mode).
        is_steer: boolean
            Servo works as steer or motor
        '''

        self.id = _id
        self.lon_label = Servo.to_lon_label(lon_label)
        self.lat_label = Servo.to_lat_label(lat_label)

        self.position = [0.0, 0.0]
        self.count = 0

        if is_steer:
            self.mode = LX16AState.LX16A_MODE_SERVO
            self.offset = angle_offset
        else:
            self.mode = LX16AState.LX16A_MODE_MOTOR
            self.offset = 0.0

        if is_steer or lat_label != LatLabel.LEFT:
                self.orientation = -1 
        
        self.state = LX16AState()
        self.position = self.calc_position()
    
    @staticmethod
    def to_lat_label(label_str):
        ''' Convert a lateral label string to a enumerated value. 
        
        Parameters
        ----------
        label_str : str
            Label for the lateral direction: 'left', 'right'.

        Returns
        -------
        int
            Enumeration label for the lateral direction:
            LEFT, RIGHT.
        '''

        if caseless_equal(label_str, 'LEFT'):
            return LatLabel.LEFT
        if caseless_equal(label_str, 'RIGHT'):
            return LatLabel.RIGHT
        else:
            return -1
 
    @staticmethod
    def to_lon_label(label_str):
        ''' Convert a longitudinal label string to a enumerated value. 
        
        Parameters
        ----------
        label_str : str
            Label for the longitudinal direction:
            'front', 'mid', 'back'.

        Returns
        -------
        int :
            Enumeration label for the longitudinal direction:
            FRONT, MID, BACK.
        '''

        if caseless_equal(label_str, 'FRONT'):
            return LonLabel.FRONT
        if caseless_equal(label_str, 'MID'):
            return LonLabel.MID
        if caseless_equal(label_str, 'BACK'):
            return LonLabel.BACK
        else:
            return -1


    def calc_position(self):
        ''' Calculate servo positions using the wheel geometry parameters
        '''
        if self.lon_label == LonLabel.FRONT:
            if self.lat_label == LatLabel.LEFT:
                return Point(x=Servo._front_wheel_lon_separation, y=Servo._front_wheel_lat_separation/2.0, z=0.0)
            if self.lat_label == LatLabel.RIGHT:
                return Point(x=Servo._front_wheel_lon_separation, y=-Servo._front_wheel_lat_separation/2.0, z=0.0)
        if self.lon_label == LonLabel.MID:
            if self.lat_label == LatLabel.LEFT:
                return Point(x=0.0, y=Servo._mid_wheel_lat_separation/2.0, z=0.0)
            if self.lat_label == LatLabel.RIGHT:
                return Point(x=0.0, y=-Servo._mid_wheel_lat_separation/2.0, z=0.0)
        if self.lon_label == LonLabel.BACK:
            if self.lat_label == LatLabel.LEFT:
                return Point(x=-Servo._back_wheel_lon_separation, y=Servo._back_wheel_lat_separation/2.0, z=0.0)
            if self.lat_label == LatLabel.RIGHT:
                return Point(x=-Servo._back_wheel_lon_separation, y=-Servo._back_wheel_lat_separation/2.0, z=0.0)

        return Point(x=0.0, y=0.0, z=0.0)
        