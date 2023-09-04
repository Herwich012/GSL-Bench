"""
| File: pso.py
| Author: Hajo Erwich (h.h.erwich@student.tudelft.nl)
| License: BSD-3-Clause. Copyright (c) 2023, Hajo Erwich. All rights reserved.
"""

__all__ = ["PSO"]

from omni.isaac.debug_draw import _debug_draw # for plotting the waypoints
import carb
import math
import numpy as np
from pegasus.simulator.logic.gsl import GSL  

class PSO(GSL):
    def __init__(self,
                 env_dict:dict = {},
                 env_bound_sep:float = 1.0,   # [m] min distance from environment bounds
                 particles:int = 2,           # number of particles (multirotors)
                 weight:float = 1.0,          # particle weight 
                 cognitive_coeff:float = 1.0, # cognitive coefficient
                 social_coeff:float = 1.0     # social coefficient
                 ) -> None:
        
        # Initialize the Super class "object" attributes
        super().__init__(gsl_type="DungBeetle")
        
        self.env_spec = env_dict["env_spec"]
        self.env_bounds_sep = env_bound_sep
        self.env_bounds = np.array([[self.env_spec["env_min"][0] + self.env_bounds_sep,   # min X
                                     self.env_spec["env_min"][1] + self.env_bounds_sep,   # min Y
                                     self.env_spec["env_min"][2] + self.env_bounds_sep],  # min Z
                                    [self.env_spec["env_max"][0] - self.env_bounds_sep,   # max X
                                     self.env_spec["env_max"][1] - self.env_bounds_sep,   # max Y
                                     self.env_spec["env_max"][2] - self.env_bounds_sep]]) # max Z

        self.draw = _debug_draw.acquire_debug_draw_interface()

        self.particles = particles
        self.weight = weight
        self.cognitive_coeff = cognitive_coeff
        self.social_coeff = social_coeff

        self.pos = np.zeros((particles,2)) # particle's positions in XY
        self.pos_best = self.pos # particle's best known positions
        self.swarm_pos_best = np.zeros((2,)) # swarms best position in XY
        self.vel = np.zeros((particles,2)) # particle's velocities in XY
        self.heading = 0.0 # [rad]

        #self.gas_sensor_prev = np.zeros((particles,))
        self.gas_sensor_best = np.zeros((particles,))
        self.gas_swarm_best = np.zeros((1,))


    def get_wp(self, id:int, loc:np.ndarray, gas_sensor:float) -> np.ndarray:
        """
        Method that generates a new waypoint according the PSO algorithm
        
        Args:
            id (int): vehicle (particle) id
            loc (np.ndarray): 3x3 numpy array with the start loc, vel, and accel
            gas_sensor (float): current gas sensor reading [ohms]
        Returns:
            np.ndarray: A 3x3 numpy matrix with the next waypoint
        """ 
        # update readings and posistions
        if gas_sensor < self.gas_sensor_best[id]:
            self.gas_sensor_best[id] = gas_sensor
            self.pos_best[id] = loc[0,:2]
            if self.gas_sensor_best[id] < self.gas_swarm_best:
                self.gas_swarm_best = self.gas_sensor_best[id]
                self.swarm_pos_best = self.pos_best[id]

        carb.log_warn(f"[PSO]{id} gas sensor now-prev: {'{:5.0f}'.format(gas_sensor)} - {'{:5.0f}'.format(self.gas_sensor_prev)}")
        # Get random coefficients
        r = np.random.rand(2)
        
        # Update velocity
        self.vel[id,:] = self.weight * self.vel[id,:] + \
            r[0] * self.cognitive_coeff * (self.pos_best[id] - loc[0,:2]) + \
            r[1] * self.social_coeff * (self.swarm_pos_best - loc[0,:2])

        while True:
            # update position
            movement = np.array([[self.vel[id,0], self.vel[id,1], 0.0],
                                [0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0]])

            wp = loc + movement

            if self.check_in_env(wp): 
                break
            else:
                carb.log_warn(f"Vehicle {id}: waypoint outside environment!")
                movement = np.zeros((3,3))


        #self.gas_sensor_prev[id] = gas_sensor
        return wp

    def reset(self):
        self.state = self.states[0] # starting state is movement perpendicular to the wind 
        #self.heading = 0.0 # [rad]
        #self.gas_sensor_prev = 0.0


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
        
    def draw(self):
        #self.draw.clear_points()
        num_samples = self.particles
        height = 7.0
        point_list = [(self.pos_best[i][0], self.pos_best[i][0], height) for i in range(num_samples)]
        point_list.append((self.swarm_pos_best[0],self.swarm_pos_best[1],height))
        colors = [(1, 0, 0, 0.8)] * num_samples
        colors.append((1,1,1))
        sizes = [(0.5) for _ in range(num_samples+1)]
        self.draw.draw_points(point_list, colors, sizes)