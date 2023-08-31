"""
| File: anemometer.py
| Author: Hajo Erwich (h.h.erwich@student.tudelft.nl)
| License: BSD-3-Clause. Copyright (c) 2023, Hajo Erwich. All rights reserved.
| Description: Simulates a anemometer. Based on the implementation provided by GADEN (https://github.com/MAPIRlab/gaden)
"""
__all__ = ["Anemometer"]

import carb
import math
import numpy as np
from typing import Tuple
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.sensors import Sensor


class Anemometer(Sensor):
    """The class that implements a anemometer sensor. This class inherits the base class Sensor.
    """

    def __init__(self, config:dict={}):
        """Initialize the Anemometer class

        Args:
            config (dict): A Dictionary that contains all the parameters for configuring the anemometer
            it can be empty or only have some of the parameters used by the sensor.
            
        Examples:
            The dictionary default parameters are

            >>> {"AutoGDM2_dir": '/home/user/AutoGDM2/',
                 "env_name": 'wh_empty_0000',
                 "env_id": 0000,
                 "env_spec": {"env_min": [0.0, 0.0, 0.0],..., "cell_size": 0.1},
                 "update_rate": 4.0,  # [Hz] update rate of sensor
                 "wind_data_time_step": 1.0, # [s] time steps between wind data iterations (in seconds to match GADEN)
                 "wind_data_start_iter": 0,  # start iteration
                 "wind_data_stop_iter": 0,   # stop iteration (0 -> to the last iteration)
            >>>  "noise_std": 0.1} # sensor noise standard deviation
        """

        # Initialize the Super class "object" attributes
        super().__init__(sensor_type="Anemometer", update_rate=config.get("update_rate", 4.0))
        
        # Location of the wind data
        self._AutoGDM2_dir = config["env_dict"].get("AutoGDM2_dir", "/home/user/AutoGDM2/")
        self._env_name = config["env_dict"].get("env_name", "wh_empty_0000")
        self._wind_data_file = f"{self._AutoGDM2_dir}environments/wind_data/{self._env_name}.npy"
        self._wind_data = np.load(self._wind_data_file)

        # Environment specification
        self._env_spec = config["env_dict"].get("env_spec", {"env_min": [0.0, 0.0, 0.0],
                                                 "env_max": [10.0, 10.0, 10.0,],
                                                 "num_cells": [100.0, 100.0, 100.0],
                                                 "cell_size": 0.1})
        
        # Check for multiple windfields, if there is only one it will be used as steady state:
        if np.shape(self._wind_data)[0] == 1:
            self._steady_state = True
            self._wind_iter = 0
            carb.log_warn("Only one windfield in the data, using it as steady state...")
        else:
            self._steady_state = False
            
            # Wind data selection (iterations)
            self._iter_start = config.get("wind_data_start_iter", 0)
            iter_stop_input = config.get("wind_data_stop_iter", 0)
            self._wind_iter = self._iter_start
            
            if iter_stop_input == 0: # loop until the last iteration
                self._iter_stop = np.shape(self._wind_data)[0] - 1
            else:
                self._iter_stop = iter_stop_input

        # Updates per wind data iteration
        # Required/recommended for the update rate to be equal of a multiple of the wind data iteration rate
        self._update_iter = 0
        self._update_rate = config.get("update_rate", 4.0) # [Hz] !!!
        self._wind_data_time_step = config.get("wind_data_time_step", 1.0) # [s] !!!
        self._updates_per_wind_iter = int(self._wind_data_time_step*self._update_rate) - 1

        self._wind_vec = np.zeros((3,)) # raw wind vector in inertial frame
        self._upwind_angle = 0.0
        self._windspeed = 0.0
        self._time_tot = 0.0
        self._sensor_noise = config.get("noise_std", 0.1)

        # Save the current state measured by the anemometer :
        self._state = {"wind_vector": self._wind_vec, # [U, V, W] [m/s]
                       "upwind_angle": self._upwind_angle, # [rad]
                       "windspeed": self._windspeed} # [m/s]
        
        # Auxiliary bool:
        self._first_reading = True

    @property
    def state(self):
        """
        (dict) The 'state' of the sensor, i.e. the data produced by the sensor at any given point in time
        """
        return self._state

    @Sensor.update_at_rate
    def update(self, state: State, dt: float):
        """Method that implements the logic of the anemometer. Here the 

        Args:
            state (State): The current state of the vehicle.
            dt (float): The time elapsed between the previous and current function calls (s).

        Returns:
            (dict) A dictionary containing the current state of the sensor (the data produced by the sensor)
        """

        # Update time step and wind iteration if necessary:
        # new wind_data                       new wind_data                            new wind_data
        # ||---------|---------|---------|---------||---------|---------|---------|---------||---------|---------|---------|---------
        # start    update    update   update     update     update   update    update      stop     update   update    update    loop to start
        self._time_tot += dt

        if self._steady_state or self._first_reading:
            pass
        else:
            if self._wind_iter == self._iter_stop and self._update_iter == self._updates_per_wind_iter:
                self._update_iter = 0 # loop to first sensor update
                self._wind_iter = self._iter_start # loop to first windfield
            elif self._update_iter != self._updates_per_wind_iter:
                self._update_iter += 1 # update the sensor, not the windfield
            else:
                self._update_iter = 0 # loop to the first sensor update
                self._wind_iter += 1 # update to new windfield

        # Select wind data, iterate after every wind_iteration_time_step
        # print(f"wind iter: {self._wind_iter}")
        # print(f"update iter: {self._update_iter}")
        wind_data = self._wind_data[self._wind_iter]
        
        # Get wind vector at location
        loc = state.position
        #loc = np.array([5.0, 3.0, 2.0]) # fixed test location
        
        cell_idx = self.cell_idx_from_pos(loc)
        self._wind_vec = wind_data[self.index_from_cell_idx(cell_idx)]

        # Simulate anemometer sensor response
        self._upwind_angle, self._windspeed = self.simulate_anemometer(self._wind_vec, state)

        # Add the values to the dictionary and return it
        self._state = {"wind_vector": self._wind_vec, # [U, V, W] [m/s]
                       "upwind_angle": self._upwind_angle, # [rad]
                       "windspeed": self._windspeed} # [m/s]
        
        # Set first reading to False
        self._first_reading = False

        # print(f"[ANEMOMETER]: angle: {'{:.4f}'.format(self._upwind_angle)} speed: {'{:.4f}'.format(self._windspeed)}")
        return self._state

    
    # stop method to reset the wind iteration and sensor dynamics
    def stop(self) -> None:
        if not self._steady_state:
            self._wind_iter = self._iter_start
        
        self._time_tot = 0.0

    # TODO - make anemometer rotate with the body frame (the body does not rotate for now so its ok)
    def simulate_anemometer(self, wind_vec:np.ndarray, state:State) -> Tuple[float,float]:
        noise = np.random.normal(0.0, self._sensor_noise, (2,)) # generate sensor noise
        wind_vec_apparent = wind_vec - state.linear_velocity # apparent wind in inertial orientation, but body velocity
        
        wind_speed_XY = np.linalg.norm(wind_vec_apparent[:-1]) + noise[0]
        
        # (IMPORTANT) Follow standards on wind measurement (real anemometers):
        # return the upwind direction in the inertial ref system, ENU convention (y-dir = North)
        # range [-pi,pi], positive to the right      
        upwind_angle = np.arctan2(-wind_vec_apparent[0], -wind_vec_apparent[1]) + noise[1] # negative for upwind dir!

        return upwind_angle, wind_speed_XY


    def cell_idx_from_pos(self, loc:np.ndarray) -> np.ndarray:
        xx = math.ceil((loc[0]-self._env_spec["env_min"][0])/self._env_spec["cell_size"])
        yy = math.ceil((loc[1]-self._env_spec["env_min"][1])/self._env_spec["cell_size"])
        zz = math.ceil((loc[2]-self._env_spec["env_min"][2])/self._env_spec["cell_size"])
        return np.array([xx,yy,zz])


    def index_from_cell_idx(self, idx:np.ndarray) -> int:
        return idx[0] + \
            (idx[1]*self._env_spec["num_cells"][0]) + \
            (idx[2]*self._env_spec["num_cells"][0]*self._env_spec["num_cells"][1])