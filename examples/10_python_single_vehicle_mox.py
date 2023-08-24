#!/usr/bin/env python
"""
| File: 10_python_single_vehicle_mox.py
| Author: Hajo Erwich (h.h.erwich@student.tudelft.nl)
| License: BSD-3-Clause. Copyright (c) 2023, Hajo Erwich. All rights reserved.
| Description: This files serves as an example on how to run a GSL benchmark with a single vehicle.
"""
import numpy as np

# Imports to start Isaac Sim from this script
import carb
from omni.isaac.kit import SimulationApp

# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import, otherwise the simulator will crash
# as this is the object that will load all the extensions and load the actual simulator.
simulation_app = SimulationApp({"headless": False})

# -----------------------------------
# The actual script should start here
# -----------------------------------
import omni.timeline
from omni.isaac.core.world import World

# Import the Pegasus API for simulating drones
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.vehicles.multirotor_mox import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Import the custom python control backend and end conditions
from examples.utils.nonlinear_controller_ecoli import NonlinearController
from pegasus.simulator.logic.gsl.stop_conditions import StopCondition

# Auxiliary scipy and numpy modules
from scipy.spatial.transform import Rotation

# Use os and pathlib for parsing the desired trajectory from a CSV file
import os
from pathlib import Path


class PegasusApp:
    """
    A Template class that serves as an example on how to build a simple Isaac Sim standalone App.
    """

    def __init__(self):
        """
        Method that initializes the PegasusApp and is used to setup the simulation environment.
        """
        # TODO init settings from yaml file
        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # Start the Pegasus Interface
        self.pg = PegasusInterface()

        # Acquire the World, .i.e, the singleton that controls that is a one stop shop for setting up physics, 
        # spawning asset primitives, etc.
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Launch one of the worlds provided AutoGDM2
        self.pg.load_environment('/home/hajo/AutoGDM2/environments/isaac_sim/wh_empty_0000.usd')

        # Get the current directory used to read trajectories and save results
        self.curr_dir = str(Path(os.path.dirname(os.path.realpath(__file__))).resolve())

        self.init_pos_1 = [8.0, 5.0, 0.2]
        
        # Create the vehicle 1
        # Try to spawn the selected robot in the world to the specified namespace
        config_multirotor1 = MultirotorConfig()
        self.controller = NonlinearController(
            init_pos=self.init_pos_1,
            env_size=[[0.0, 0.0, 0.0], # env min x y z
                      [10.0, 16.0, 8.0]], # env max x y z
            Ki=[0.5, 0.5, 0.5],
            Kr=[2.0, 2.0, 2.0]
        )
        config_multirotor1.backends = [self.controller]

        Multirotor(
            "/World/quadrotor1",
            ROBOTS['Iris'],
            0,
            self.init_pos_1,
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor1,
        )

        # Auxiliar variable for repeated runs
        self.runs = 3
        self.statistics = [f"ecoli_run_{i}" for i in range(self.runs)]

        # Set stop condition(s)
        self.stop_cond = StopCondition(time=120.0,
                                       source_pos=np.array([5.0, 0.6, 2.0]), 
                                       distance2src=2.0)
        
        # Reset the simulation environment so that all articulations (aka robots) are initialized
        self.world.reset()


    def run(self):
        """
        Method that implements the application main loop, where the physics steps are executed.
        """

        # Run the simulation again for every statistics file
        for i,statistics_file in enumerate(self.statistics):
            # Set the results file
            self.controller.results_files = self.curr_dir + f"/results/{statistics_file}.npz"

            # Start the simulation
            self.timeline.play()

            while not self.stop_cond.get(time_current = self.controller.total_time,
                                         pos_current = self.controller.p):
                # Update the UI of the app and perform the physics step
                self.world.step(render=True)

            if self.stop_cond.type == "dist2src": # mark the run as a success if the source is considered found
                self.controller.run_success[0] = True

            # Stop & Reset the simulation
            self.timeline.stop()
            self.world.reset() # necessary to replicate the 'UI stop button' behaviour
            carb.log_warn(f"Finished run {i+1}/{self.runs}")
        
        # Cleanup and quit
        carb.log_warn("PegasusApp Simulation App is closing.")
        simulation_app.close()


def main():

    # Instantiate the template app
    pg_app = PegasusApp()

    # Run the application loop
    pg_app.run()


if __name__ == "__main__":
    main()
