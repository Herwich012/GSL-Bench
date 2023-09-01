"""
| File: waypoints.py
| Author: Hajo Erwich (h.h.erwich@student.tudelft.nl)
| License: BSD-3-Clause. Copyright (c) 2023, Hajo Erwich. All rights reserved.
| Description: Class that genereates a trajectory with minimal jerk
"""
__all__ = ["Waypoints"]

import numpy as np


class Waypoints:
    """ 
    Waypoint (data) class containing different sets of waypoints for
    takeoff, landing and a waypoint mission etc.
    """
    def __init__(self, 
                 init_pos:np.ndarray, # np.ndarray with shape (3,)
                 altitude:float=2.0) -> None:
        
        self._init_pos = init_pos
        self._altitude = altitude
        self._waypoints = np.zeros((1,3,3))

        # Auxiliary variables for the waypoint index
        self.idx = 0
        self.last_idx = 0


    def get(self) -> np.ndarray:
        return self._waypoints


    def set_takeoff(self, pos:np.ndarray=None):
        if pos == None: 
            takeoff_pos = self._init_pos
            takeoff_alt = self._altitude
        else: # takeoff from new position other than init opos
            takeoff_pos = pos
            takeoff_alt = self._altitude + pos[2]
        
        self._waypoints = np.zeros((2,3,3))
        self._waypoints[0,0,:] = takeoff_pos
        self._waypoints[1,0,:] = takeoff_pos
        self._waypoints[1,0,2] = takeoff_alt
        
        self.set_last_idx()
        self.idx = 0


    def set_mission(self, waypoints:np.ndarray) -> None:
        self._waypoints = waypoints
        
        self.set_last_idx()
        self.idx = 0


    def set_testcase(self, pos:np.ndarray=None, to:tuple=None):
        if pos == None: 
            takeoff_pos = self._init_pos
            takeoff_alt = self._altitude
        else: # takeoff from new position other than init opos
            takeoff_pos = pos
            takeoff_alt = self._altitude + pos[2]
        
        self._waypoints = np.zeros((3,3,3))
        self._waypoints[0,0,:] = takeoff_pos
        self._waypoints[1,0,:] = takeoff_pos
        self._waypoints[1,0,2] = takeoff_alt

        self._waypoints[2,0,:] = np.array([to[0],to[1],takeoff_alt])
        
        self.set_last_idx()
        self.idx = 0


    def set_landing(self):
        pass


    def set_last_idx(self):
        self.last_idx = np.shape(self._waypoints)[0] - 1


    def reset(self):
        self._waypoints = np.zeros((1,3,3))
        self.idx = 0