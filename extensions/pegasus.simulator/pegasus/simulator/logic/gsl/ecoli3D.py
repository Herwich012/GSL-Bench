"""
| File: ecoli.py
| Author: Hajo Erwich (h.h.erwich@student.tudelft.nl)
| License: BSD-3-Clause. Copyright (c) 2023, Hajo Erwich. All rights reserved.
"""

__all__ = ["E_Coli3D"]

import carb
import glob
import math
import numpy as np
from pegasus.simulator.logic.gsl import GSL  

class E_Coli3D(GSL):
    def __init__(self,
                 env_dict:dict = {},
                 surge_distance:float = 0.5, # in horizontal direction
                 surge_distance_z:float = 0.3, # max dist in vertical direction
                 env_bound_sep:float = 0.3, # [m] min distance from environment bounds
                 random_walker:bool = False # always perform random actions (zero measurement)
                ) -> None:
        
        # Initialize the Super class "object" attributes
        super().__init__(gsl_type="E_Coli3D")
        
        self.env_dir = env_dict["env_dir"]
        self.env_spec = env_dict["env_spec"]
        self.surge_dist = surge_distance
        self.surge_dist_z = surge_distance_z
        self.env_bounds_sep = env_bound_sep
        self.env_bounds = np.array([[self.env_spec["env_min"][0] + self.env_bounds_sep,   # min X
                                     self.env_spec["env_min"][1] + self.env_bounds_sep,   # min Y
                                     self.env_spec["env_min"][2] + self.env_bounds_sep],  # min Z
                                    [self.env_spec["env_max"][0] - self.env_bounds_sep,   # max X
                                     self.env_spec["env_max"][1] - self.env_bounds_sep,   # max Y
                                     self.env_spec["env_max"][2] - self.env_bounds_sep]]) # max Z
        self.random_walker = random_walker

        # Load occupancy grid
        self.occ_grid = np.load(glob.glob(f"{self.env_dir}occupancy/*grid.npy")[0]) # occ data file
        
        self.sensor_prev = 0.0
        self.surge_heading_prev = [0.0, 0.0] # [rad] (direction) [m] (up, down)


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
            if sensor >= self.sensor_prev or self.random_walker: # if reading is the same/gets worse, move randomly
                surge_heading =  self.get_random_heading()
                carb.log_warn(f"[E. Coli] {'{:5.0f}'.format(sensor)} >= {'{:5.0f}'.format(self.sensor_prev)}, RANDOM  heading+height: {'{:1.2f}'.format(surge_heading[0])},{'{:1.2f}'.format(surge_heading[1])}")
            else:
                surge = True
                surge_heading = self.surge_heading_prev
                carb.log_warn(f"[E. Coli] {'{:5.0f}'.format(sensor)} < {'{:5.0f}'.format(self.sensor_prev)}, SURGE!!! heading+height: {'{:1.2f}'.format(surge_heading[0])},{'{:1.2f}'.format(surge_heading[1])}")

            movement = np.array([[self.surge_dist*np.cos(surge_heading[0]), 
                                  self.surge_dist*np.sin(surge_heading[0]), 
                                  surge_heading[1]],
                                [0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0]])
            
            wp = start + movement
            
            if self.check_in_env(wp) and self.check_env_for_obstacle3D(start,wp): 
                break
            elif surge == True:
                carb.log_warn(f"Waypoint outside environment! Randomizing previous surge heading...")
                self.surge_heading_prev = self.get_random_heading() # update previous surge heading if it is in obstacle
            else:
                carb.log_warn(f"Waypoint outside environment! Randomizing heading again...")
                surge_heading = self.get_random_heading()

        self.sensor_prev = sensor
        self.surge_heading_prev = surge_heading

        return wp


    def reset(self):
        self.sensor_prev = 0.0
        self.surge_heading_prev = 0.0 # [rad]


    def get_random_heading(self) -> list:
        return [2*np.pi*np.random.rand(), (2*self.surge_dist_z*np.random.rand() - self.surge_dist_z)]


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

    # TODO: remove this func when there is 3D obstacle avoidance
    def check_env_for_obstacle3D(self, start_wp:np.ndarray, end_wp:np.ndarray) -> bool:
        start = start_wp[0]
        end = end_wp[0]
        
        dist = np.linalg.norm((start,end))
        samples = math.ceil(dist/self.env_spec["cell_size"]) + 1 # +1 to avoid an edgecase of skipping cells

        x_points = np.linspace(start[0], end[0], samples)
        y_points = np.linspace(start[1], end[1], samples)
        z_points = np.linspace(start[2], end[2], samples)

        for _,(x,y,z) in enumerate(zip(x_points,y_points,z_points)):
            x_idx = math.floor((x - self.env_spec["env_min"][0])/self.env_spec["cell_size"])
            y_idx = math.floor((y - self.env_spec["env_min"][1])/self.env_spec["cell_size"])
            z_idx = math.floor((z - self.env_spec["env_min"][2])/self.env_spec["cell_size"])

            if self.occ_grid[z_idx, x_idx, y_idx] != 0: return False

        # Direct line of sight!
        return True
