"""
| File: dungbeetle1.py
| Author: Hajo Erwich (h.h.erwich@student.tudelft.nl)
| License: BSD-3-Clause. Copyright (c) 2023, Hajo Erwich. All rights reserved.
"""

__all__ = ["DungBeetle1"]

import carb
import math
import numpy as np
from pegasus.simulator.logic.gsl import GSL  

class DungBeetle1(GSL):
    def __init__(self,
                 env_dict:dict = {},
                 env_bound_sep:float = 1.0, # [m] min distance from environment bounds
                 zigzag_angle:float = 60.0, # [deg] angle between windvector and path
                 wp_dist:float = 0.8 # [m] distance of each next waypoint
                 ) -> None:
        
        # Initialize the Super class "object" attributes
        super().__init__(gsl_type="DungBeetle1")
        
        self.env_spec = env_dict["env_spec"]
        self.env_bounds_sep = env_bound_sep
        self.env_bounds = np.array([[self.env_spec["env_min"][0] + self.env_bounds_sep,   # min X
                                     self.env_spec["env_min"][1] + self.env_bounds_sep,   # min Y
                                     self.env_spec["env_min"][2] + self.env_bounds_sep],  # min Z
                                    [self.env_spec["env_max"][0] - self.env_bounds_sep,   # max X
                                     self.env_spec["env_max"][1] - self.env_bounds_sep,   # max Y
                                     self.env_spec["env_max"][2] - self.env_bounds_sep]]) # max Z

        self.states = ['90CCW', 'ZIG_CCW', 'ZAG_CW']
        self.state = self.states[0] # starting state is movement perpendicular to the wind 

        self.zigzag_angle = zigzag_angle # deg
        self.wp_dist = wp_dist
        self.heading = 0.0 # [rad]

        self.gas_sensor_prev = 0.0


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

        carb.log_warn(f"[DungBeetle] gas sensor now-prev: {'{:5.0f}'.format(gas_sensor)} - {'{:5.0f}'.format(self.gas_sensor_prev)}")
        # determine state and angle
        if gas_sensor >= self.gas_sensor_prev and self.state != self.states[0]: # reading gets worse, change zigzag direction
            if self.state == self.states[1]:
                self.state = self.states[2]
                self.heading = self.map_angle_to_pipi(upwind_angle + np.deg2rad(self.zigzag_angle))
                carb.log_warn(f"[DungBeetle] changing zigzag direction to +{self.zigzag_angle}deg")
            else:
                self.state = self.states[1]
                self.heading = self.map_angle_to_pipi(upwind_angle - np.deg2rad(self.zigzag_angle))
                carb.log_warn(f"[DungBeetle] changing zigzag direction to -{self.zigzag_angle}deg")
        elif gas_sensor < self.gas_sensor_prev and self.state == self.states[0]:
            self.state = self.states[1]
            self.heading = self.map_angle_to_pipi(upwind_angle - np.deg2rad(self.zigzag_angle))
            carb.log_warn(f"[DungBeetle] initiating zigzag!")
        elif self.state == self.states[0]:
            self.heading = self.map_angle_to_pipi(upwind_angle - np.deg2rad(90))
            carb.log_warn(f"[DungBeetle] moving 90deg to the windangle...")

        while True:
            # determine waypoint
            movement = np.array([[self.wp_dist*np.sin(self.heading), self.wp_dist*np.cos(self.heading), 0.0],
                                [0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0]])

            wp = loc + movement

            if self.check_in_env(wp): 
                break
            else:
                carb.log_warn(f"Waypoint outside environment! changing angle...")
                # angle = (2*np.pi*np.random.rand()) - np.pi
                if self.state == self.states[0]:
                    self.heading = np.pi*(2*np.random.rand() - 1)
                elif self.state == self.states[1]: # change direction
                    self.heading = self.map_angle_to_pipi(self.heading + 2*np.deg2rad(self.zigzag_angle))
                else: # change direction
                    self.heading = self.map_angle_to_pipi(self.heading - 2*np.deg2rad(self.zigzag_angle))

        self.gas_sensor_prev = gas_sensor
        return wp


    def map_angle_to_pipi(self, angle:float) -> float: # angle in [rad]!
        k = math.floor((np.pi - angle) / (2 * np.pi))
        alpha = angle + (k * 2 * np.pi)
        
        return alpha


    def reset(self):
        self.state = self.states[0] # starting state is movement perpendicular to the wind 
        self.heading = 0.0 # [rad]
        self.gas_sensor_prev = 0.0


    def check_in_env(self, wp:np.ndarray):
        """
        Method that returns True if waypoint is within the environment
        
        Args:
            wp (np.ndarray): 3x3 numpy array with the start loc, vel, and accel
        Returns:
            bool: True if waypoint is in environment
        """
        env = self.env_bounds
        if wp[0,0] < env[0,0] or wp[0,1] < env[0,1] or wp[0,2] < env[0,2] or \
            wp[0,0] > env[1,0] or wp[0,1] > env[1,1] or wp[0,2] > env[1,2]:
            return False
        else:
            return True