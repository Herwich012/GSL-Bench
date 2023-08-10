"""
| Author: Hajo Erwich (h.h.erwich@student.tudelft.nl)
| License: BSD-3-Clause. Copyright (c) 2023, Hajo Erwich. All rights reserved.
"""
import carb
import numpy as np  

class EcoliGSL():
    def __init__(self,
                 surge_distance:float = 0.5,
                 env_bound_sep:float = 0.5, # [m] min distance from environment bounds
                 env_bounds:list = [[0,0,0],[10,10,10]]) -> None:
        
        self.surge_dist = surge_distance
        self.env_bounds_sep = env_bound_sep
        self.ecoli_bounds = [[env_bounds[0][0] + self.env_bounds_sep + self.surge_dist, # min X
                            env_bounds[0][1] + self.env_bounds_sep + self.surge_dist, # min Y
                            env_bounds[0][2] + self.env_bounds_sep + self.surge_dist], # min Z
                           [env_bounds[1][0] - self.env_bounds_sep - self.surge_dist, # max X
                            env_bounds[1][1] - self.env_bounds_sep - self.surge_dist, # max Y
                            env_bounds[1][2] - self.env_bounds_sep - self.surge_dist]] # max Z
        self.sensor_prev = 0.0
        self.surge_heading_prev = 0.0 # [rad]


    def get_wp(self, start:np.ndarray, sensor:float):
        """
        Method that generates a new waypoint according to the 'ecoli' algorithm
        
        Args:
            start (np.ndarray): 3x3 numpy array with the start loc, vel, and accel
            sensor_now (float): current sensor reading [ohms]
            sensor_prev (float): previous sensor reading [ohms]
            surge_vector_prev (float): previous surge vector [rad]
            surge_distance (float): surge distance [m]
        Returns:
            np.ndarray: A 3x3 numpy matrix with the next waypoint
        """ 
        surge = False

        while True:
            if sensor >= self.sensor_prev: # if reading is the same/gets worse, move randomly
                surge_heading =  2*np.pi*np.random.rand()
                carb.log_warn(f"Ecoli: {'{:5.0f}'.format(sensor)} >= {'{:5.0f}'.format(self.sensor_prev)}, RANDOM  heading: {'{:1.2f}'.format(surge_heading)}")
            else:
                surge = True
                surge_heading = self.surge_heading_prev
                carb.log_warn(f"Ecoli: {'{:5.0f}'.format(sensor)} < {'{:5.0f}'.format(self.sensor_prev)}, SURGE!!! heading: {'{:1.2f}'.format(surge_heading)}")

            movement = np.array([[self.surge_dist*np.cos(surge_heading), self.surge_dist*np.sin(surge_heading), 0.0],
                                [0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0]])
            
            wp = start + movement
            
            if self.check_in_env(wp): 
                break
            elif surge == True:
                carb.log_warn(f"Encountered obstacle! Randomizing previous surge heading...")
                self.surge_heading_prev = 2*np.pi*np.random.rand() # update previous surge heading if it would move into an obstacle
            else:
                carb.log_warn(f"Encountered obstacle! Randomizing heading again...")
                surge_heading = 2*np.pi*np.random.rand()

        self.sensor_prev = sensor
        self.surge_heading_prev = surge_heading

        return wp


    def reset(self):
        #self.sensor_reading = 0.0
        self.sensor_prev = 0.0
        self.surge_heading_prev = 0.0 # [rad]

    # TODO move out
    def check_in_env(self, wp:np.ndarray):
        """
        Method that returns True if waypoint is within the environment
        
        Args:
            wp (np.ndarray): 3x3 numpy array with the start loc, vel, and accel
            env (list): a [2,3] size list with the environment min and max bounds
        Returns:
            bool: True if waypoint is in environment
        """
        env = self.ecoli_bounds
        if wp[0,0] < env[0][0] or wp[0,1] < env[0][1] or wp[0,2] < env[0][2] or \
            wp[0,0] > env[1][0] or wp[0,1] > env[1][1] or wp[0,2] > env[1][2]:
            return False
        else:
            return True