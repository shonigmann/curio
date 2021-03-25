#!/usr/bin/env python


from curio_base.utils import caseless_equal

class Servo(object):
    '''Servo properties.
    
    Store information about the servo.

    Attributes
    ----------
    id : int
        servo serial id: 0 - 253.
    lon_label : int
        Enumeration label for the longitudinal direction:
        FRONT, MID, BACK.
    lat_label : int
        Enumeration label for the lateral direction: LEFT, RIGHT.
    orientation : int
        Flag to indicate whether the servo is installed for positive
        or negtive rotation: 1, -1.
    offset : int
        The servo position offset (for servo rather than motor mode).
        Use to centre servos.
    position : list
        List servo position in the robot base. Two dimensional
        coordinate vector of floats.    
    '''

    # Lateral labels
    LEFT  = 0
    RIGHT = 1

    # Longitudinal labels
    FRONT = 2
    MID   = 3
    BACK  = 4

    def __init__(self, id, lon_label, lat_label, orientation):
        ''' Constructor
        
        Parameters
        ----------
        id : int
            servo serial id: 0 - 253.
        lon_label : str
            Label for the longitudinal direction:
            'front', 'mid', 'back'.
        lat_label : str
            Label for the lateral direction: 'left', 'right'.
        orientation : int
            Flag to indicate whether the servo is installed for positive
            or negtive rotation: 1, -1.
        '''

        self.id = id
        self.lon_label = lon_label
        self.lat_label = lat_label
        self.orientation = orientation
        self.offset = 0.0
        self.position = [0.0, 0.0]

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
            return Servo.LEFT
        if caseless_equal(label_str, 'RIGHT'):
            return Servo.RIGHT
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
            return Servo.FRONT
        if caseless_equal(label_str, 'MID'):
            return Servo.MID
        if caseless_equal(label_str, 'BACK'):
            return Servo.BACK
        else:
            return -1
