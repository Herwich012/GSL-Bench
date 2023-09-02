"""
| File: ecoli.py
| Author: Hajo Erwich (h.h.erwich@student.tudelft.nl)
| License: BSD-3-Clause. Copyright (c) 2023, Hajo Erwich. All rights reserved.
"""

__all__ = ["E_Coli"]

import carb
import numpy as np
from pegasus.simulator.logic.gsl import GSL  

class E_Coli(GSL):
    def __init__(self,
                 env_dict:dict = {},
                 surge_distance:float = 0.5,
                 env_bound_sep:float = 0.3, # [m] min distance from environment bounds
                ) -> None:
        
        # Initialize the Super class "object" attributes
        super().__init__(gsl_type="E_Coli")
        
        self.env_spec = env_dict["env_spec"]
        self.surge_dist = surge_distance
        self.env_bounds_sep = env_bound_sep
        self.env_bounds = [[self.env_spec["env_min"][0] + self.env_bounds_sep, # min X
                              self.env_spec["env_min"][1] + self.env_bounds_sep, # min Y
                              self.env_spec["env_min"][2] + self.env_bounds_sep], # min Z
                              [self.env_spec["env_max"][0] + self.env_bounds_sep, # max X
                              self.env_spec["env_max"][1] + self.env_bounds_sep, # max Y
                              self.env_spec["env_max"][2] + self.env_bounds_sep]] # max Z
        self.sensor_prev = 0.0
        self.surge_heading_prev = 0.0 # [rad]


    def get_wp(self, start:np.ndarray, sensor:float) -> np.ndarray:
        """
        Method that generates a new waypoint according to the 'ecoli' algorithm
        
        Args:
            start (np.ndarray): 3x3 numpy array with the start loc, vel, and accel
            sensor (float): current sensor reading [ohms]
        Returns:
            np.ndarray: A 3x3 numpy matrix with the next waypoint
        """ 
        surge = False

        while True:
            if sensor >= self.sensor_prev: # if reading is the same/gets worse, move randomly
                surge_heading =  2*np.pi*np.random.rand()
                carb.log_warn(f"[E. Coli] {'{:5.0f}'.format(sensor)} >= {'{:5.0f}'.format(self.sensor_prev)}, RANDOM  heading: {'{:1.2f}'.format(surge_heading)}")
            else:
                surge = True
                surge_heading = self.surge_heading_prev
                carb.log_warn(f"[E. Coli] {'{:5.0f}'.format(sensor)} < {'{:5.0f}'.format(self.sensor_prev)}, SURGE!!! heading: {'{:1.2f}'.format(surge_heading)}")

            movement = np.array([[self.surge_dist*np.cos(surge_heading), self.surge_dist*np.sin(surge_heading), 0.0],
                                [0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0]])
            
            wp = start + movement
            
            if self.check_in_env(wp): 
                break
            elif surge == True:
                carb.log_warn(f"Waypoint outside environment! Randomizing previous surge heading...")
                self.surge_heading_prev = 2*np.pi*np.random.rand() # update previous surge heading if it 
            else:
                carb.log_warn(f"Waypoint outside environment! Randomizing heading again...")
                surge_heading = 2*np.pi*np.random.rand()

        self.sensor_prev = sensor
        self.surge_heading_prev = surge_heading

        return wp


    def reset(self):
        self.sensor_prev = 0.0
        self.surge_heading_prev = 0.0 # [rad]


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