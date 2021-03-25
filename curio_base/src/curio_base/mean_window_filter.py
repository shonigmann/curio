class MeanWindowFilter(object):
    ''' Simple rolling window filter.
    '''

    def __init__(self, window=5):
        ''' Constructor

        Parameters
        ----------
        window : int
            The size of the rolling window, has (default 5)
        '''

        self._window = window
        self._index  = 0
        self._buffer = [0.0 for i in range(window)]        
        self._sum    = 0.0 
        self._mean   = 0.0

    def update(self, value):
        ''' Update the filter with the the next value.

        Parameters
        ----------
        value : float
            The next value to accumulate in the filter.
        '''

        # Update the ring buffer
        self._index = (self._index + 1) % self._window
        old_value = self._buffer[self._index] 
        self._buffer[self._index] = value

        # Update the stats
        self._sum  = self._sum + value - old_value
        self._mean = self._sum / self._window

    def get_mean(self):
        ''' Get the rolling mean

        Returns
        -------
        float
            The rolling mean
        '''

        return self._mean

    def get_window(self):
        ''' Get the size of the rolling window

        Returns
        -------
        int
            The size of the rolling window
        '''
        return self._window
