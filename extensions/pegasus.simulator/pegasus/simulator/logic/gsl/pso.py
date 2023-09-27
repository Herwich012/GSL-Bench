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
                 env_dict:dict,
                 particles:int,               # number of particles (multirotors)
                 env_bound_sep:float = 1.0,   # [m] min distance from environment bounds
                 weight:float = 0.5,          # particle weight 
                 cognitive_coeff:float = 1.0, # cognitive coefficient
                 social_coeff:float = 1.0,    # social coefficient
                 wp_dist:float = 0.5,         # waypoint scaling factor
                 d_swarm:float = 1.5          # [m] distance between particles before they repulse
                 ) -> None:
        
        # Initialize the Super class "object" attributes
        super().__init__(gsl_type="PSO")

        
        self.env_spec = env_dict["env_spec"]
        self.env_bounds_sep = env_bound_sep
        self.env_bounds = np.array([[self.env_spec["env_min"][0] + self.env_bounds_sep,   # min X
                                     self.env_spec["env_min"][1] + self.env_bounds_sep,   # min Y
                                     self.env_spec["env_min"][2]],#+ self.env_bounds_sep],  # min Z
                                    [self.env_spec["env_max"][0] - self.env_bounds_sep,   # max X
                                     self.env_spec["env_max"][1] - self.env_bounds_sep,   # max Y
                                     self.env_spec["env_max"][2]]])# - self.env_bounds_sep]]) # max Z

        self.particles = particles
        self.weight = weight
        self.cognitive_coeff = cognitive_coeff
        self.social_coeff = social_coeff
        self.wp_dist = wp_dist
        self.d_swarm = d_swarm

        self.pos = np.zeros((particles,2)) # particle's positions in XY
        self.pos_best = self.pos # particle's best known positions
        self.swarm_pos_best = np.array([self.env_bounds[1,0]-self.env_bounds[0,0],
                                        self.env_bounds[1,1]-self.env_bounds[0,1]]) * \
                                            np.random.rand(2) - np.array([self.env_bounds[0,0],self.env_bounds[0,1]]) # swarms best position in XY
        self.vel = (2*np.random.rand(particles,2)-1) # particle's velocities in XY
        self.heading = 0.0 # [rad]

        self.gas_sensor_prev = np.zeros((particles,))
        self.gas_sensor_best = np.ones((particles,)) * 50000.0 # TODO - change value based on gas sensor type
        self.gas_swarm_best = np.inf


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

        carb.log_warn(f"[PSO] id:{id} gas sensor now-prev: {'{:5.0f}'.format(gas_sensor)} - {'{:5.0f}'.format(self.gas_sensor_prev[id])}")
        self.pos[id] = loc[0,:2]
        self.update_reading(id, gas_sensor)

        neighbours = self.check_for_neighbours(id)
        
        if not neighbours:
            self.update_velocity(id)
        else:
            self.set_repulsion_velocity(id, neighbours)
        
        # update position
        movement = np.array([[self.vel[id,0], self.vel[id,1], 0.0],
                            [0.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0]])

        wp = loc + movement

        if not self.check_in_env(wp): 
            carb.log_warn(f"Vehicle {id}: waypoint outside environment: {wp[0]}, repulsion from wall")
            wp = loc - (1.2 * movement)  # for now go back where you came from + some safety factor

        self.gas_sensor_prev[id] = gas_sensor
        return wp


    def reset(self):
        self.pos = np.zeros((self.particles,2)) # particle's positions in XY
        self.pos_best = self.pos # particle's best known positions
        self.swarm_pos_best = np.array([self.env_bounds[1,0]-self.env_bounds[0,0],
                                        self.env_bounds[1,1]-self.env_bounds[0,1]]) * \
                                            np.random.rand(2) - np.array([self.env_bounds[0,0],self.env_bounds[0,1]]) # swarms best position in XY
        self.vel = (2*np.random.rand(self.particles,2)-1) # particle's velocities in XY
        self.heading = 0.0 # [rad]

        self.gas_sensor_prev = np.zeros((self.particles,))
        self.gas_sensor_best = np.ones((self.particles,)) * 50000.0 # TODO - change value based on gas sensor type
        self.gas_swarm_best = np.inf


    def initialize(self, id, init_pos):
        self.pos_best[id] = init_pos[:2]

    
    def update_reading(self, id:int, sensor_reading:float) -> None:
        if sensor_reading and sensor_reading < self.gas_sensor_best[id]:
            self.gas_sensor_best[id] = sensor_reading
            self.pos_best[id] = self.pos[id]
            print(f"{id} updated best own position!")
            if self.gas_sensor_best[id] < self.gas_swarm_best:
                self.gas_swarm_best = self.gas_sensor_best[id]
                self.swarm_pos_best = self.pos_best[id]
                print(f"{id} updated best swarm position!")


    def update_velocity(self, id:int) -> None:
        # Get random coefficients
        r = np.random.rand(2)
        
        # Update velocity
        self.vel[id] = self.weight * self.vel[id] + \
            r[0] * self.cognitive_coeff * (self.pos_best[id] - self.pos[id]) + \
            r[1] * self.social_coeff * (self.swarm_pos_best - self.pos[id])
        
        # normalize vector
        self.vel[id] = self.wp_dist*(self.vel[id]/np.linalg.norm(self.vel[id]))
        
        print(f"vel: {self.vel[id]}")
        print(f"r: {r}")
        print(f"pos_best: {self.pos_best[id]}")
        print(f"swarm_best: {self.swarm_pos_best}")
    

    def check_for_neighbours(self, id:int) -> list:
        neighbours = []
        
        for i in [x for x in range(self.particles) if x != id]:
            if np.linalg.norm((self.pos[id] - self.pos[i])) < self.d_swarm:
                neighbours.append(i)

        return neighbours


    def set_repulsion_velocity(self, id:int, neighbours:list) -> None:
        # collect all the 'anti vectors'
        for neighbour in neighbours:
            self.vel[id] += self.pos[id] - self.pos[neighbour]

        # normalize vector
        self.vel[id] = self.wp_dist*(self.vel[id]/np.linalg.norm(self.vel[id]))

        print("REPULSION!!!!!!!!!!!!!!!!!")        
        print(f"vel: {self.vel[id]}")
        print(f"pos_best: {self.pos_best[id]}")
        print(f"swarm_best: {self.swarm_pos_best}")
        

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
        
    def draw_waypoints(self):
        self.draw.clear_points()
        num_samples = self.particles
        height = 7.0
        #point_list = [(self.pos_best[i][0], self.pos_best[i][0], height) for i in range(num_samples)]
        #colors = [(1, 0, 0, 0.8)] * num_samples
        #sizes = [(0.5) for _ in range(num_samples)]
        point_list = [(self.swarm_pos_best[0],self.swarm_pos_best[1],height)]
        colors = [(1, 0, 0, 1)]
        sizes = [1.0]
        
        self.draw.draw_points(point_list, colors, sizes)
