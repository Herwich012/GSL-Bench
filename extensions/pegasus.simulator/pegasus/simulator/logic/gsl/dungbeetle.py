"""
| File: dungbeetle.py
| Author: Hajo Erwich (h.h.erwich@student.tudelft.nl)
| License: BSD-3-Clause. Copyright (c) 2023, Hajo Erwich. All rights reserved.
"""

__all__ = ["DungBeetle"]

import carb
import math
import numpy as np
from pegasus.simulator.logic.gsl import GSL  

class DungBeetle(GSL):
    def __init__(self,
                 env_dict:dict = {},
                 env_bound_sep:float = 0.5, # [m] min distance from environment bounds
                 ) -> None:
        
        # Initialize the Super class "object" attributes
        super().__init__(gsl_type="DungBeetle")
        
        self.env_spec = env_dict["env_spec"]
        self.env_bounds_sep = env_bound_sep
        self.env_bounds = [[self.env_spec["env_min"][0] + self.env_bounds_sep, # min X
                              self.env_spec["env_min"][1] + self.env_bounds_sep, # min Y
                              self.env_spec["env_min"][2] + self.env_bounds_sep], # min Z
                              [self.env_spec["env_max"][0] + self.env_bounds_sep, # max X
                              self.env_spec["env_max"][1] + self.env_bounds_sep, # max Y
                              self.env_spec["env_max"][2] + self.env_bounds_sep]] # max Z

        self.states = ['90CCW', 'ZIG_CCW', 'ZAG_CW']
        self.state = self.states[0] # set starting state

        self.zigzag_angle = 60.0 # deg
        self.gas_sensor_prev = 0.0
        self.loc_prev = np.zeros((3,3)) # previous location, to check if stuck


    def get_wp(self, loc:np.ndarray, gas_sensor:float, upwind_angle:float) -> np.ndarray:
        """
        Method that generates a new waypoint according the 'dung beetle' algorithm
        
        Args:
            loc (np.ndarray): 3x3 numpy array with the start loc, vel, and accel
            gas_sensor (float): current gas sensor reading [ohms]
            upwind_angle (float): current upwind angle: range [-pi,pi], positive to the right, 0 in y-dir
        Returns:
            np.ndarray: A 3x3 numpy matrix with the next waypoint
        """ 

        if self.state == '90CCW':
            angle = self.get_90CCW(upwind_angle)

        elif self.state == 'ZIG_CCW':
            pass

        else: # 'ZAG_CW'
            pass
            

        while True:

            wp = loc 

            if self.check_in_env(wp): 
                break
            else:
                carb.log_warn(f"Waypoint outside environment! Randomizing heading again...")
                wp = 2*np.pi*np.random.rand()

        self.gas_sensor_prev = gas_sensor
        return wp


    def get_angle(self, upwind_angle:float) -> float:
        pass


    def map_angle_to_pipi(self, angle:float) -> float: # angle in [rad]!
        k = math.floor((np.pi - angle) / (2 * np.pi))
        alpha = angle + (k * 2 * np.pi)
        
        return alpha


    def reset(self):
        self.sensor_prev = 0.0


    def check_in_env(self, wp:np.ndarray):
        """
        Method that returns True if waypoint is within the environment
        
        Args:
            wp (np.ndarray): 3x3 numpy array with the start loc, vel, and accel
        Returns:
            bool: True if waypoint is in environment
        """
        env = self.env_bounds
        if wp[0,0] < env[0][0] or wp[0,1] < env[0][1] or wp[0,2] < env[0][2] or \
            wp[0,0] > env[1][0] or wp[0,1] > env[1][1] or wp[0,2] > env[1][2]:
            return False
        else:
            return True