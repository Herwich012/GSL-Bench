"""
| File: mox.py
| Author: Hajo Erwich (h.h.erwich@student.tudelft.nl)
| License: BSD-3-Clause. Copyright (c) 2023, Hajo Erwich. All rights reserved.
| Description: Simulates a Metal-oxide gas sensor. Based on the implementation provided by GADEN (https://github.com/MAPIRlab/gaden)
"""
__all__ = ["Anemometer"]

import os
import re
import carb
import yaml
import math
import numpy as np
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
                 "update_rate": 4.0,  # [Hz] update rate of sensor
                 "wind_data_time_step": 0.5, # [s] time steps between wind data iterations (in seconds to match GADEN)
                 "wind_data_start_iter": 0,  # start iteration
            >>>  "wind_data_stop_iter": 0}   # stop iteration (0 -> to the last iteration)
        """

        # Initialize the Super class "object" attributes
        super().__init__(sensor_type="Anemometer", update_rate=config.get("update_rate", 4.0))
        
        # Location of the wind data
        self._AutoGDM2_dir = config.get("AutoGDM2_dir", "/home/user/AutoGDM2/")
        self._env_name = config.get("env_name", "wh_empty_0000")
        self._wind_data_file = f"{self._AutoGDM2_dir}environments/wind_data/{self._env_name}.npy"
        self._wind_data = np.load(self._wind_data_file)

        # Environment specifications
        with open(f'{self._AutoGDM2_dir}environments/occupancy/{self._env_name}_head.txt', 'r') as file:
            self._env_dict = yaml.safe_load(file)
        
        # Check for multiple windfields, if there is only one it will be used as steady state:
        if np.shape(self._wind_data)[0] == 1:
            self._steady_state = True
            carb.log_warn("Only one windfield in the data, using it as steady state...")
        else:
            self._steady_state = False
            
            # Wind data selection (iterations)
            self._iter_start = config.get("wind_data_start_iter", 0)
            iter_stop_input = config.get("wind_data_stop_iter", 0)
            self._wind_iter = self._iter_start
            
            if iter_stop_input == 0: # loop until the last iteration
                self._iter_stop = np.shape(self._wind_data)[0]
            else:
                self._iter_stop = iter_stop_input

        # Updates per wind data iteration
        # Required/recommended for the update rate to be equal of a multiple of the wind data iteration rate
        self._update_iter = 0
        self._update_rate = config.get("update_rate", 4.0) # [Hz] !!!
        self._wind_data_time_step = config.get("gas_data_time_step", 0.5) # [s] !!!
        self._updates_per_wind_iter = int(self._wind_data_time_step/(1.0/self._update_rate)) - 1

        self._sensor_output = np.zeros((3,))
        self._first_reading = True
        self._time_tot = 0.0

        # Save the current state measured by the MOX sensor:
        self._state = {"sensor_output": self._sensor_output} # [U, V, W]

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

        # Update time step and gas iteration if necessary:
        # new wind_data                       new wind_data                            new wind_data
        # ||---------|---------|---------|---------||---------|---------|---------|---------||---------|---------|---------|---------
        # start    update    update   update     update     update   update    update      stop     update   update    update    loop to start
        self._time_tot += dt

        if self._steady_state:
            wind_data = self._wind_data[0]
        else:
            if self._wind_iter == self._iter_stop and self._update_iter == self._updates_per_wind_iter:
                self._update_iter = 0 # loop to first windfield
                self._wind_iter = self._iter_start
            elif self._update_iter != self._updates_per_wind_iter:
                self._update_iter += 1 # update the sensor, not the windfield
            else:
                self._update_iter = 0 # loop to the first sensor update
                self._wind_iter += 1 # update to new windfield

            # Initialize gas data, iterate after every gas_iteration_time_step
            # print(f"wind iter: {self._wind_iter}")
            # print(f"update iter: {self._update_iter}")
            if self._update_iter == 0:
                wind_data = self._wind_data[self._wind_iter]
            
        
        # Get gas concentration [ppm] at location
        loc = state.position
        #loc = np.array([7.5, 5.0, 3.0]) # fixed test location

        self._sensor_output = wind_data[self.index_from_3D(loc)]

        # Simulate anemometer sensor response
        # self._sensor_output = self.simulate_anemometer(dt, self._gas_conc)

        # Add the values to the dictionary and return it
        self._state = {"sensor_output": self._sensor_output}

        print(f"ANEMOMETER OUTPUT: {self._sensor_output}")
        return self._state

    
    # stop method to reset the gas iteration and sensor dynamics
    def stop(self) -> None:
        if not self._steady_state:
            self._wind_iter = self._iter_start
        self._sensor_output = np.zeros(3,)
        self._gas_conc = 0.0
        self._RS_R0 = 0.0

        self._first_reading = True

        self._time_tot = 0.0

    # TODO - add heading, velocity and noise
    def simulate_anemometer(self):
        pass


    def index_from_3D(self, loc:np.ndarray) -> int:
        return math.floor(loc[0]) + \
            (math.floor(loc[1])*self._env_dict["num_cells"][0]) + \
            (math.floor(loc[2])*self._env_dict["num_cells"][0]*self._env_dict["num_cells"][1])