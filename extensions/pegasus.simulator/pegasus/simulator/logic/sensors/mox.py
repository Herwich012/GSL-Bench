"""
| File: mox.py
| Author: Hajo Erwich (h.h.erwich@student.tudelft.nl)
| License: BSD-3-Clause. Copyright (c) 2023, Hajo Erwich. All rights reserved.
| Description: Simulates a Metal-oxide gas sensor. Based on the implementation provided by GADEN (https://github.com/MAPIRlab/gaden)
"""
__all__ = ["MOX"]

import os
import re
import math
import numpy as np
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.sensors import Sensor
from pegasus.simulator.logic.sensors.mox_utils import (
    Filament,
    labels,
    R0,
    tau_value,
    sensitivity_air,
    sensitivity_lineloglog,
)


class MOX(Sensor):
    """The class that implements a barometer sensor. This class inherits the base class Sensor.
    """

    def __init__(self, config={}):
        """Initialize the Barometer class

        Args:
            config (dict): A Dictionary that contains all the parameters for configuring the Barometer - it can be empty or only have some of the parameters used by the Barometer.
            
        Examples:
            The dictionary default parameters are

            >>> {"env_name": "wh_simple_0000"
                 "sensor_model": 0,   # ["TGS2620", "TGS2600", "TGS2611", "TGS2610", "TGS2612"]
                 "update_rate": 8.0,  # [Hz] update rate of sensor
                 "gas_data_time_step": 0.5, # [s] time steps between gas data iterations (in seconds to match GADEN)
                 "gas_data_start_iter": 0,  # start iteration
            >>>  "gas_data_stop_iter": 0}   # stop iteration (0 -> to the last iteration)
        """

        # Initialize the Super class "object" attributes
        super().__init__(sensor_type="MOX", update_rate=config.get("update_rate", 4.0))
        
        # Location of the gas data
        self._env_name = config.get("env_name", "wh_empty_0000")
        self._gas_data_dir = f"/home/hajo/AutoGDM2/environments/gas_data/{self._env_name}/"
        self._gas_data_files = os.listdir(self._gas_data_dir)
        
        # Gas data selection (iterations)
        self._iter_start = config.get("gas_data_start_iter", 300)
        iter_stop_input = config.get("gas_data_stop_iter", 0)
        self._filament_iter = self._iter_start
        
        if iter_stop_input == 0: # loop until the last iteration
            self._gas_data_files.sort(key=lambda f: int(''.join(filter(str.isdigit, f))))
            last_file = self._gas_data_files[-1]
            self._iter_stop = int(re.findall(r'\d+', last_file)[0]) # filter out the digits from the filename
        else:
            self._iter_stop = iter_stop_input

        # Updates per gas data iteration
        # Required/recommended for the update rate to be equal of a multiple of the gas data iteration rate
        self._update_iter = 0
        self._update_rate = config.get("update_rate", 4.0) # [Hz] !!!
        self._gas_data_time_step = config.get("gas_data_time_step", 0.5) # [s] !!!
        self._updates_per_gas_iter = int(self._gas_data_time_step/(1.0/self._update_rate)) - 1

        # Set sensor model
        self._sensor_model = config.get("sensor_model", 1) # see mox_utils.py for sensor models

        self._sensor_output = 0.0
        self._gas_conc = 0.0
        self._RS_R0 = 0.0

        self._first_reading = True

        self._time_tot = 0.0

        # Save the current state measured by the MOX sensor: 
        # [sensor_output, gas_conc, RS_R0]
        self._state = {"sensor_output": np.zeros((3,))}

    @property
    def state(self):
        """
        (dict) The 'state' of the sensor, i.e. the data produced by the sensor at any given point in time
        """
        return self._state

    @Sensor.update_at_rate
    def update(self, state: State, dt: float):
        """Method that implements the logic of a MOX sensor. Here the 

        Args:
            state (State): The current state of the vehicle.
            dt (float): The time elapsed between the previous and current function calls (s).

        Returns:
            (dict) A dictionary containing the current state of the sensor (the data produced by the sensor)
        """

        # Update time step and gas iteration if necessary:
        # new gas_data                        new gas_data                              new gas_data
        # ||---------|---------|---------|---------||---------|---------|---------|---------||---------|---------|---------|---------
        # start    update    update   update     update     update   update    update      stop     update   update    update    loop to start
        self._time_tot += dt

        if self._filament_iter == self._iter_stop and self._update_iter == self._updates_per_gas_iter:
            self._update_iter = 0 # loop to first filament data
            self._filament_iter = self._iter_start
        elif self._update_iter != self._updates_per_gas_iter:
            self._update_iter += 1 # update the sensor, not the filament data
        else:
            self._update_iter = 0 # loop to the first filament update
            self._filament_iter += 1 # update to new filament data

        # Initialize gas data, iterate after every gas_iteration_time_step
        # print(f"filament iter: {self._filament_iter}")
        # print(f"update iter: {self._update_iter}")
        if self._update_iter == 0:
            gas_data = np.load(f"{self._gas_data_dir}iteration_{self._filament_iter}_fil.npy")
            gas_data_head = np.load(f"{self._gas_data_dir}iteration_{self._filament_iter}_head.npy")
        
        # Get gas concentration [ppm] at location
        loc = state.position
        #loc = np.array([7.5, 5.0, 3.0]) # fixed test location

        if self._update_iter == 0: # update concentration only if new gas data
        # TODO UPDATE GAS CONCENTRATION FOR EVERY PHYSICS STEP (because location can be changed)
        #print("Update!")
            self._gas_conc = 0.0
            for fil in gas_data: # filament: id, x, y, z, sigma
                filament = Filament(fil[0],fil[1],fil[2],fil[3],fil[4])
                dist_SQR = math.pow((loc[0] - filament.x),2) + math.pow((loc[1] - filament.y),2) + math.pow((loc[2] - filament.z),2)
                limit_distance = filament.sigma*5/100
                #print(f"SQR: {dist_SQR}")
                #print(f"lim: {limit_distance}")

                # If filament is within range, calculate the contribution to the gas concentration
                if dist_SQR < math.pow(limit_distance,2): # TODO add check for obstacles
                    self._gas_conc += self.concentration_from_filament(loc, filament, gas_data_head)

        # Simulate MOX sensor response
        self._sensor_output, self._RS_R0 = self.simulate_mox_as_line_loglog(dt, self._gas_conc)

        # Add the values to the dictionary and return it
        self._state = {"sensor_output": np.array([self._sensor_output, self._gas_conc, self._RS_R0])}

        #print("{:.0f}".format(self._filament_iter), "{:.2f}".format(self._time_tot), "{:.6f}".format(self._sensor_output), "{:.6f}".format(self._gas_conc), "{:.6f}".format(self._RS_R0))
        return self._state


    def concentration_from_filament(self, loc:np.ndarray, filament:Filament, gas_data_head:np.ndarray):
        distance_cm = 100 * math.sqrt(math.pow((loc[0] - filament.x),2) + math.pow((loc[1] - filament.y),2) + \
                                      math.pow((loc[2] - filament.z),2))

        num_moles_target_cm3 = (gas_data_head['filament_num_moles_of_gas'][0] / \
                                (math.sqrt(8*math.pow(math.pi,3)) * math.pow(filament.sigma,3))) * \
                                    math.exp(-math.pow(distance_cm,2)/(2*math.pow(filament.sigma,2)))
        ppm = num_moles_target_cm3 / gas_data_head['num_moles_all_gases_in_cm3'][0] * 1000000 # parts of target gas per million
        return ppm


    def simulate_mox_as_line_loglog(self, dt:float, gas_concentration:float):
        # Initialize sensor if it is the first reading
        if self._first_reading:
            sensor_output = RS_R0 = sensitivity_air[self._sensor_model] # RS_R0 value at air
            self._previous_sensor_output = sensor_output
            self._first_reading = False
        
        else:
            #1. Set Sensor Output based on gas concentrations (gas type dependent)
            #---------------------------------------------------------------------
            # RS/R0 = A*conc^B (a line in the loglog scale)
            # TODO implement multiple gases?
            # TODO implement noise?

            resistance_variation = 0.0

            # Value of RS/R0 for the given gas and concentration
            if gas_concentration == 0.0: # if statement because python math.pow() does not like infinity
                RS_R0 = sensitivity_air[self._sensor_model]
            else:
                RS_R0 = sensitivity_lineloglog[self._sensor_model][0][0] * math.pow(gas_concentration, sensitivity_lineloglog[self._sensor_model][0][1])

            # Ensure we never overpass the baseline level (max allowed)
            if (RS_R0 > sensitivity_air[self._sensor_model]):
                RS_R0 = sensitivity_air[self._sensor_model]

            # Increment with respect the Baseline
            resistance_variation = sensitivity_air[self._sensor_model] - RS_R0

            # Calculate RS_R0 given the resistance variation
            RS_R0 = sensitivity_air[self._sensor_model] - resistance_variation

            # Ensure a minimum sensor resitance
            if (RS_R0 <= 0.0):
                RS_R0 = 0.01

            #2. Simulate transient response (dynamic behaviour, tau_r and tau_d)
            #---------------------------------------------------------------------
            if (RS_R0 < self._previous_sensor_output):  # rise
                tau = tau_value[self._sensor_model][0][0]
            else: # decay
                tau = tau_value[self._sensor_model][0][1]

            # Use a low pass filter
            # alpha value = At/(tau+At)
            alpha = (dt) / (tau+(dt))
            #print(f"Previous: {round(self._previous_sensor_output,6)}\n")

            # Filtered response (uses previous estimation):
            sensor_output = (alpha*RS_R0) + (1-alpha)*self._previous_sensor_output

            # Update values
            self._previous_sensor_output = sensor_output

        # Return Sensor response for current time instant as the Sensor Resistance in Ohms
        return(sensor_output * R0[self._sensor_model]), RS_R0