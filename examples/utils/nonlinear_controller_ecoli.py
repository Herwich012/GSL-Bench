#!/usr/bin/env python
"""
| File: nonlinear_controller_wp.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt), Hajo Erwich (hajo_erwich@live.nl)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: The nonlinear controller in example 4 uses a built-in trajectory or is based on a .csv file.
This new implementation enables the use of waypoints from which the trajectories are generated directly,
which works on top of the nonlinear controller. Now, the multirotor can be commanded in a flexible way.
"""
import numpy as np

# Imports to be able to log to the terminal with fancy colors
import carb

# Imports from the Pegasus library
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.backends import Backend
from pegasus.simulator.logic.trajectory import TrajectoryMinJerk

# Auxiliary scipy
from scipy.spatial.transform import Rotation

class NonlinearController(Backend):
    """A nonlinear controller class. It implements a nonlinear controller that allows a vehicle to track
    aggressive trajectories. This controlers is well described in the papers
    
    [1] J. Pinto, B. J. Guerreiro and R. Cunha, "Planning Parcel Relay Manoeuvres for Quadrotors," 
    2021 International Conference on Unmanned Aircraft Systems (ICUAS), Athens, Greece, 2021, 
    pp. 137-145, doi: 10.1109/ICUAS51884.2021.9476757.
    [2] D. Mellinger and V. Kumar, "Minimum snap trajectory generation and control for quadrotors," 
    2011 IEEE International Conference on Robotics and Automation, Shanghai, China, 2011, 
    pp. 2520-2525, doi: 10.1109/ICRA.2011.5980409.
    """

    def __init__(self,
        env_size=[[-np.inf,-np.inf,-np.inf], # env min x y z
                  [np.inf,np.inf,np.inf]], # env max x y z
        Kp=[10.0, 10.0, 10.0],
        Kd=[8.5, 8.5, 8.5],
        Ki=[1.50, 1.50, 1.50],
        Kr=[3.5, 3.5, 3.5],
        Kw=[0.5, 0.5, 0.5]):

        # The current rotor references [rad/s]
        self.input_ref = [0.0, 0.0, 0.0, 0.0]

        # The current state of the vehicle expressed in the inertial frame (in ENU)
        self.p = np.zeros((3,))                   # The vehicle position
        self.R: Rotation = Rotation.identity()    # The vehicle attitude
        self.w = np.zeros((3,))                   # The angular velocity of the vehicle
        self.v = np.zeros((3,))                   # The linear velocity of the vehicle in the inertial frame
        self.a = np.zeros((3,))                   # The linear acceleration of the vehicle in the inertial frame

        # Define the control gains matrix for the outer-loop
        self.Kp = np.diag(Kp)
        self.Kd = np.diag(Kd)
        self.Ki = np.diag(Ki)
        self.Kr = np.diag(Kr)
        self.Kw = np.diag(Kw)

        self.int = np.array([0.0, 0.0, 0.0])

        # Define the dynamic parameters for the vehicle
        self.m = 1.50        # Mass in Kg
        self.g = 9.81        # The gravity acceleration ms^-2

        # Waypoint logic
        self.tr = TrajectoryMinJerk()
        self.avg_vel = 0.3 # average velocity [m/s] (< surge distance !!!)
        self.task_states = ['hold', 'move2wp']
        self.task_state = self.task_states[1]
        self.hold_end_time = np.inf # [s]
        self.hold_time = 3.0 # [s]
        self.waypoint_idx = 0
        self.waypoints = np.array([[[1.5,1.5,0.2], # px, py, pz
                                    [0.0,0.0,0.0],  # vx, vy, vz
                                    [0.0,0.0,0.0]], # ax, ay, az
                                   [[8.0,8.0,2.0],
                                    [0.0,0.0,0.0],
                                    [0.0,0.0,0.0]],
                                   [[7.0,3.0,3.0],
                                    [0.0,0.0,0.0],
                                    [0.0,0.0,0.0]]])#
                                #    [[7.5,9.0,3.0],
                                #     [0.0,0.0,0.0],
                                #     [0.0,0.0,0.0]],
                                #    [[3.0,9.0,3.0],
                                #     [0.0,0.0,0.0],
                                #     [0.0,0.0,0.0]]])
        self.waypoint_last = np.shape(self.waypoints)[0] - 1
        self.waypoint_loop_idx = 1 # at last waypoint, loop back to this one
        
        # Position, velocity... etc references
        self.trajectory = np.zeros((1,14))
        self.trajectory[0, 0:3] = np.array([self.waypoints[0,0,:]])
        self.trajectory[0, 3:6] = np.array([self.waypoints[0,1,:]])
        self.trajectory[0, 6:9] = np.array([self.waypoints[0,2,:]])
        
        self.index = 0
        self.max_index = 0
        self.total_time = 0.0

        # MOX sensor data initialization
        self.mox_raw = 0.0
        self.gas_conc = 0.0
        self.RS_R0 = 0.0

        # GSL algorithm logic (ecoli)
        self.start_wp = np.zeros((3,3))
        self.end_wp = np.zeros((3,3))
        self.surge_dist = 0.5 # [m]
        self.sensor_reading = 0.0
        self.sensor_reading_prev = 0.0
        self.surge_heading_prev = 0.0 # [rad]
        self.env_bounds_sep = 0.5 # [m] min distance from environment bounds
        self.env_bounds = [[env_size[0][0] + self.env_bounds_sep + self.surge_dist, # min X
                            env_size[0][1] + self.env_bounds_sep + self.surge_dist, # min Y
                            env_size[0][2] + self.env_bounds_sep + self.surge_dist], # min Z
                           [env_size[1][0] - self.env_bounds_sep - self.surge_dist, # max X
                            env_size[1][1] - self.env_bounds_sep - self.surge_dist, # max Y
                            env_size[1][2] - self.env_bounds_sep - self.surge_dist]] # max Z

        # Auxiliar variable, so that we only start sending motor commands once we get the state of the vehicle
        self.received_first_state = False

        # TODO add success rate, gas conc and mox reading
        # Lists used for analysing performance statistics
        self.results_files = None
        self.time_vector = []
        self.desired_position_over_time = []
        self.position_over_time = []
        self.position_error_over_time = []
        self.velocity_error_over_time = []
        self.atittude_error_over_time = []
        self.attitude_rate_error_over_time = []


    def start(self):
        """
        Reset the control and trajectory index
        """
        self.reset_statistics()


    def stop(self):
        """
        Stopping the controller. Saving the statistics data for plotting later
        """
        self.reset_waypoints()
        self.reset_gsl_ecoli()

        # Check if we should save the statistics to some file or not
        if self.results_files is None:
            return
        
        statistics = {}
        statistics["time"] = np.array(self.time_vector)
        statistics["p"] = np.vstack(self.position_over_time)
        statistics["desired_p"] = np.vstack(self.desired_position_over_time)
        statistics["ep"] = np.vstack(self.position_error_over_time)
        statistics["ev"] = np.vstack(self.velocity_error_over_time)
        statistics["er"] = np.vstack(self.atittude_error_over_time)
        statistics["ew"] = np.vstack(self.attitude_rate_error_over_time)
        np.savez(self.results_files, **statistics)
        carb.log_warn("Statistics saved to: " + self.results_files)

        self.reset_statistics()


    def update_sensor(self, sensor_type: str, data):
        """
        Update the MOX sensor

        Args:
            sensor_type (str): The name of the sensor providing the data
            data (dict): A dictionary that contains the data produced by the sensor
        """
        if sensor_type == "MOX":
            self.update_mox_data(data)
        else:
            pass


    def update_mox_data(self, data):
        """Gets called by the 'update_sensor' method to update the current MOX data

        Args:
            data (dict): The data produced by an MOX sensor
        """
        self.mox_raw = data["sensor_output"][0]
        self.gas_conc = data["sensor_output"][1]
        self.RS_R0 = data["sensor_output"][2]


    def update_state(self, state: State):
        """
        Method that updates the current state of the vehicle. This is a callback that is called at every physics step

        Args:
            state (State): The current state of the vehicle.
        """
        self.p = state.position
        self.R = Rotation.from_quat(state.attitude)
        self.w = state.angular_velocity
        self.v = state.linear_velocity

        self.received_first_state = True


    def input_reference(self):
        """
        Method that is used to return the latest target angular velocities to be applied to the vehicle

        Returns:
            A list with the target angular velocities for each individual rotor of the vehicle
        """
        return self.input_ref


    def update(self, dt: float):
        """Method that implements the nonlinear control law and updates the target angular velocities for each rotor. 
        This method will be called by the simulation on every physics step

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """
        
        if self.received_first_state == False:
            return
        
        self.total_time += dt
        # print(f"MOX sensor [raw, ppm, RSR0]: [{'{:5.0f}'.format(self.mox_raw)} {'{:2.5f}'.format(self.gas_conc)} {'{:1.2f}'.format(self.RS_R0)}]")
        
        # Load new waypoint and update the references for the controller to track
        if self.hold_end_time < self.total_time : # update trajectory when hold time is over
            self.task_state = self.task_states[1] # move2wp
            self.hold_end_time = np.inf # set holdtime to infinite (until trajectory is completed)
            self.sensor_reading = self.mox_raw # set current sensor reading

            # first follow waypoint(s) in init(), then perform GSL
            if self.waypoint_idx < self.waypoint_last:
                self.start_wp = self.waypoints[self.waypoint_idx]
                self.end_wp = self.waypoints[self.waypoint_idx + 1]
            else:
                self.start_wp = self.end_wp
                self.end_wp, self.surge_heading_prev = self.gsl_ecoli_wp(
                    self.start_wp, 
                    self.sensor_reading, 
                    self.sensor_reading_prev, 
                    self.surge_heading_prev, 
                    self.surge_dist
                )

            self.trajectory = self.tr.generate(dt, self.start_wp, self.end_wp, self.avg_vel)

            # reset index and max_index
            self.index = 0
            self.max_index = np.shape(self.trajectory)[0] - 1

            self.sensor_reading_prev = self.sensor_reading # set previous reading to current for next iteration
            self.waypoint_idx += 1 # update waypoint index
            
            print(f"Moving to waypoint {self.waypoint_idx} at {np.round(np.array(self.trajectory[self.max_index, 0:3]),2)}")

        # Check if we need to update to the next trajectory index
        if self.index < self.max_index - 1: #and self.total_time >= self.trajectory[self.index + 1, 0]:
            self.index += 1
        elif self.task_state == 'move2wp': # if task_state is hold, do not update the trajectory
            self.task_state = self.task_states[0] # hold
            self.hold_end_time = self.total_time + self.hold_time
            print(f"Holding for {self.hold_time} seconds...")

        # set references for current timestep
        p_ref = np.array(self.trajectory[self.index, 0:3])
        v_ref = np.array(self.trajectory[self.index, 3:6])
        a_ref = np.array(self.trajectory[self.index, 6:9])
        j_ref = np.array(self.trajectory[self.index, 9:12])
        yaw_ref = self.trajectory[self.index,12]
        yaw_rate_ref = self.trajectory[self.index,13]

        # -------------------------------------------------
        # Start the controller implementation
        # -------------------------------------------------

        # Compute the tracking errors
        ep = self.p - p_ref
        ev = self.v - v_ref
        self.int = self.int +  (ep * dt)
        ei = self.int

        # Compute F_des term
        F_des = -(self.Kp @ ep) - (self.Kd @ ev) - (self.Ki @ ei) + np.array([0.0, 0.0, self.m * self.g]) + (self.m * a_ref)

        # Get the current axis Z_B (given by the last column of the rotation matrix)
        Z_B = self.R.as_matrix()[:,2]

        # Get the desired total thrust in Z_B direction (u_1)
        u_1 = F_des @ Z_B

        # Compute the desired body-frame axis Z_b
        Z_b_des = F_des / np.linalg.norm(F_des)

        # Compute X_C_des 
        X_c_des = np.array([np.cos(yaw_ref), np.sin(yaw_ref), 0.0])

        # Compute Y_b_des
        Z_b_cross_X_c = np.cross(Z_b_des, X_c_des)
        Y_b_des = Z_b_cross_X_c / np.linalg.norm(Z_b_cross_X_c)

        # Compute X_b_des
        X_b_des = np.cross(Y_b_des, Z_b_des)

        # Compute the desired rotation R_des = [X_b_des | Y_b_des | Z_b_des]
        R_des = np.c_[X_b_des, Y_b_des, Z_b_des]
        R = self.R.as_matrix()

        # Compute the rotation error
        e_R = 0.5 * self.vee((R_des.T @ R) - (R.T @ R_des))

        # Compute an approximation of the current vehicle acceleration in the inertial frame (since we cannot measure it directly)
        self.a = (u_1 * Z_B) / self.m - np.array([0.0, 0.0, self.g])

        # Compute the desired angular velocity by projecting the angular velocity in the Xb-Yb plane
        # projection of angular velocity on xB − yB plane
        # see eqn (7) from [2].
        hw = (self.m / u_1) * (j_ref - np.dot(Z_b_des, j_ref) * Z_b_des) 
        
        # desired angular velocity
        w_des = np.array([-np.dot(hw, Y_b_des), 
                           np.dot(hw, X_b_des), 
                           yaw_rate_ref * Z_b_des[2]])

        # Compute the angular velocity error
        e_w = self.w - w_des

        # Compute the torques to apply on the rigid body
        tau = -(self.Kr @ e_R) - (self.Kw @ e_w)

        # Use the allocation matrix provided by the Multirotor vehicle to convert the desired force and torque
        # to angular velocity [rad/s] references to give to each rotor
        if self.vehicle:
            self.input_ref = self.vehicle.force_and_torques_to_velocities(u_1, tau)

        # ----------------------------
        # Statistics to save for later
        # ----------------------------
        self.time_vector.append(self.total_time)
        self.position_over_time.append(self.p)
        self.desired_position_over_time.append(p_ref)
        self.position_error_over_time.append(ep)
        self.velocity_error_over_time.append(ev)
        self.atittude_error_over_time.append(e_R)
        self.attitude_rate_error_over_time.append(e_w)


    @staticmethod
    def vee(S):
        """Auxiliary function that computes the 'v' map which takes elements from so(3) to R^3.

        Args:
            S (np.array): A matrix in so(3)
        """
        return np.array([-S[1,2], S[0,2], -S[0,1]])


    def reset_statistics(self):
        # Reset the lists used for analysing performance statistics
        self.time_vector = []
        self.desired_position_over_time = []
        self.position_over_time = []
        self.position_error_over_time = []
        self.velocity_error_over_time = []
        self.atittude_error_over_time = []
        self.attitude_rate_error_over_time = []


    def reset_waypoints(self):
        self.task_state = self.task_states[1]
        self.hold_end_time = np.inf # [s]
        self.waypoint_idx = 0

        # Position, velocity... etc references
        self.trajectory = np.zeros((1,14))
        self.trajectory[0, 0:3] = np.array([self.waypoints[0,0,:]])
        self.trajectory[0, 3:6] = np.array([self.waypoints[0,1,:]])
        self.trajectory[0, 6:9] = np.array([self.waypoints[0,2,:]])
        
        self.index = 0
        self.max_index = 0
        self.total_time = 0.0

    # TODO move GSL method out of controller
    # ---------------------------------------------------
    # GSL method (Gas Source Localisation)
    # ---------------------------------------------------

    def gsl_ecoli_wp(self, start:np.ndarray, sensor_now:float, sensor_prev:float, surge_heading_prev:float, surge_distance:float):
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
            if sensor_now >= sensor_prev: # if reading is the same/gets worse, move randomly
                surge_heading =  2*np.pi*np.random.rand()
                carb.log_info(f"Ecoli: {'{:5.0f}'.format(sensor_now)} >= {'{:5.0f}'.format(sensor_prev)}, RANDOM  heading: {'{:1.2f}'.format(surge_heading)}")
            else:
                surge = True
                surge_heading = surge_heading_prev
                print(f"Ecoli: {'{:5.0f}'.format(sensor_now)} < {'{:5.0f}'.format(sensor_prev)}, SURGE!!! heading: {'{:1.2f}'.format(surge_heading)}")

            movement = np.array([[surge_distance*np.cos(surge_heading), surge_distance*np.sin(surge_heading), 0.0],
                                [0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0]])
            
            wp = start + movement
            
            if self.check_in_env(wp): 
                break
            elif surge == True:
                print(f"Encountered obstacle! Randomizing previous surge heading...")
                surge_heading_prev = 2*np.pi*np.random.rand() # update previous surge heading if it would move into an obstacle
            else:
                print(f"Encountered obstacle! Randomizing heading again...")
                surge_heading = 2*np.pi*np.random.rand()

        return wp, surge_heading
    

    def reset_gsl_ecoli(self):
        self.start_wp = np.zeros((3,3))
        self.end_wp = np.zeros((3,3))
        self.sensor_reading = 0.0
        self.sensor_reading_prev = 0.0
        self.surge_heading_prev = 0.0 # [rad]
        self.valid_wp = False


    def check_in_env(self, wp:np.ndarray):
        """
        Method that returns True if waypoint is within the environment
        
        Args:
            wp (np.ndarray): 3x3 numpy array with the start loc, vel, and accel
            env (list): a [2,3] size list with the environment min and max bounds
        Returns:
            bool: True if waypoint is in environment
        """
        env = self.env_bounds
        if wp[0,0] < env[0][0] or wp[0,1] < env[0][1] or wp[0,2] < env[0][2] or wp[0,0] > env[1][0] or wp[0,1] > env[1][1] or wp[1,2] > env[1][2]:
            return False
        else:
            return True