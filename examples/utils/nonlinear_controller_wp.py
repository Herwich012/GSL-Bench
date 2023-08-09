#!/usr/bin/env python
"""
| File: nonlinear_controller_wp.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt), Hajo Erwich (hajo_erwich@live.nl)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: The nonlinear controller in example 4 uses a built-in trajectory or is based on a .csv file.
This new implementation enables the use of waypoints from which the trajectories are generated directly,
which works on top of the nonlinear controller. Now, the multirotor can be commanded in a flexible way.
"""

# Imports to be able to log to the terminal with fancy colors
import carb

# Imports from the Pegasus library
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.backends import Backend

# Auxiliary scipy and numpy modules
import numpy as np
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
        trajectory_file: str = None, 
        results_file: str=None, 
        reverse=False, 
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
        self.avg_vel = 0.5 # average velocity [m/s]
        self.task_states = ['hold', 'move2wp']
        self.task_state = self.task_states[1]
        self.hold_end_time = np.inf # [s]
        self.hold_time = 2.0 # [s]
        self.waypoint_idx = 0
        self.waypoints = np.array([[[0.0,0.0,0.07], # px, py, pz
                                    [0.0,0.0,0.0],  # vx, vy, vz
                                    [0.0,0.0,0.0]], # ax, ay, az
                                   [[0.0,0.0,1.0],
                                    [0.0,0.0,0.0],
                                    [0.0,0.0,0.0]]])
                                #    [[2.0,0.0,1.0],
                                #     [0.0,0.0,0.0],
                                #     [0.0,0.0,0.0]],
                                #    [[2.0,2.0,1.0],
                                #     [0.0,0.0,0.0],
                                #     [0.0,0.0,0.0]],
                                #    [[0.0,2.0,1.0],
                                #     [0.0,0.0,0.0],
                                #     [0.0,0.0,0.0]]])
        self.waypoint_last = np.shape(self.waypoints)[0] - 1
        self.waypoint_loop_idx = 1 # at last waypoint, loop back to this one

        # Position, velocity... etc references
        self.p_ref_all = np.zeros((1,3))
        self.v_ref_all = np.zeros((1,3))
        self.a_ref_all = np.zeros((1,3))
        self.j_ref_all = np.zeros((1,3))
        self.yaw_ref_all = np.zeros((1,))
        self.yaw_rate_ref_all = np.zeros((1,))
        
        self.index = 0
        self.max_index = 0
        self.total_time = 0.0

        self.reverse = reverse

        # Auxiliar variable, so that we only start sending motor commands once we get the state of the vehicle
        self.received_first_state = False

        # Lists used for analysing performance statistics
        self.results_files = results_file
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
        Do nothing. For now ignore all the sensor data and just use the state directly for demonstration purposes. 
        This is a callback that is called at every physics step.

        Args:
            sensor_type (str): The name of the sensor providing the data
            data (dict): A dictionary that contains the data produced by the sensor
        """
        pass


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

        # Load new waypoint and update the references for the controller to track
        if self.hold_end_time < self.total_time : # update trajectory
            self.task_state = self.task_states[1] # move2wp
            self.hold_end_time = np.inf # set holdtime to infinite (until trajectory is completed)

            # set new start and end waypoint
            start = self.waypoints[self.waypoint_idx]
            if self.waypoint_idx == self.waypoint_last:
                end = self.waypoints[self.waypoint_loop_idx]
            else:
                end = self.waypoints[self.waypoint_idx + 1]
            
            # TODO clean up into one 'trajectory' array?
            self.p_ref_all, self.v_ref_all, self.a_ref_all, self.j_ref_all, self.yaw_ref_all, self.yaw_rate_ref_all = self.gen_trajectory(dt, start, end, self.avg_vel)

            # reset index and max_index
            self.index = 0
            self.max_index = np.shape(self.p_ref_all)[0] - 1

            # update waypoint index
            if self.waypoint_idx == self.waypoint_last:
                self.waypoint_idx = self.waypoint_loop_idx # loop around
            else:
                self.waypoint_idx += 1
            
            print(f"Moving to waypoint {self.waypoint_idx} at {np.array([self.p_ref_all[self.max_index, 0], self.p_ref_all[self.max_index, 1], self.p_ref_all[self.max_index, 2]])}...")

        # check if we need to update to the next trajectory index
        if self.index < self.max_index - 1: #and self.total_time >= self.trajectory[self.index + 1, 0]:
            self.index += 1
        elif self.task_state == 'move2wp': # if task_state is hold, do not update the trajectory
            self.task_state = self.task_states[0] # hold
            self.hold_end_time = self.total_time + self.hold_time
            print(f"Holding for {self.hold_time} seconds...")

        # set references for current timestep
        p_ref = np.array(self.p_ref_all[self.index, :])
        v_ref = np.array(self.v_ref_all[self.index, :])
        a_ref = np.array(self.a_ref_all[self.index, :])
        j_ref = np.array(self.j_ref_all[self.index, :])
        yaw_ref = self.yaw_ref_all[self.index]
        yaw_rate_ref = self.yaw_rate_ref_all[self.index]

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

        # self.index = 0
        # # If we received an external trajectory, reset the time to 0.0
        # if self.trajectory is not None:
        #     self.total_time = 0.0
        # # if using the internal trajectory, make the parametric value start at -5.0
        # else:
        #     self.total_time = -5.0

        # Reset the lists used for analysing performance statistics
        self.time_vector = []
        self.desired_position_over_time = []
        self.position_over_time = []
        self.position_error_over_time = []
        self.velocity_error_over_time = []
        self.atittude_error_over_time = []
        self.attitude_rate_error_over_time = []

    # ---------------------------------------------------
    # Definition of an exponential trajectory for example
    # This can be used as a reference if not trajectory file is passed
    # as an argument to the constructor of this class
    # ---------------------------------------------------

    def gen_trajectory(self, dt:float, start:np.ndarray, end:np.ndarray, vel:float=0.4):
        """
        Method that generates a linear trajectory based on a start and endpoint.
        Desired is zero velocity and acceleration at the start and endpoints.
        Therefore, a quintic (5th) order polynomial is uses as the motion law:
        
        s(t) = a_0 + a_1*t + a_2*t² + a_3*t³ + a_4*t⁴ + a_5*t⁵

        with the boundary conditions:
        s(0) = startpoint, s(T) = endpoint, sd(0)=sd(T)=sdd(0)=sdd(T)=0

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
            start (np.ndarray): 3x3 numpy array with the start loc, vel, and accel in XYZ
            end (np.ndarray): 3x3 numpy array with the end loc, vel, and accel in XYZ
            vel (float): average speed from start to end
        Returns:
            p_ref (np.ndarray): np.array with the desired position over time
            v_ref (np.ndarray): np.array with the desired velocity over time
            a_ref (np.ndarray): np.array with the desired acceleration over time
            j_ref (np.ndarray): np.array with the desired jerk over time
            yaw_ref (np.ndarray): np.array with the desired yaw over time
            yaw_rate_ref (np.ndarray): np.array with the desired yaw rate over time
        """
        if not np.array_equal(start, end): # only generate trajectory if waypoints are different
            # Endtime T due to the average speed vel:
            T = np.linalg.norm(end[0,:] - start[0,:]) / vel
            xdata = np.linspace(0, T, int(T/dt))

            # Initialize the ref arrays
            p_ref =  np.zeros((int(T/dt),3))
            v_ref =  np.zeros((int(T/dt),3))
            a_ref =  np.zeros((int(T/dt),3))
            j_ref =  np.zeros((int(T/dt),3))
            yaw_ref = np.zeros((int(T/dt),))
            yaw_rate_ref = np.zeros((int(T/dt),))

            # Generate the motion in XYZ
            for i in range(3):
                coefs = self.solvequintic(T, start[:,i], end[:,i])

                p_ref[:,i] = self.poly5func(xdata, *coefs)
                v_ref[:,i] = self.poly5func_d(xdata, *coefs[1:])
                a_ref[:,i] = self.poly5func_dd(xdata, *coefs[2:])
                j_ref[:,i] = self.poly5func_ddd(xdata, *coefs[3:])
        
        else: # use the same waypoint
            p_ref = np.array([start[0,:]])
            v_ref = np.zeros((1,3))
            a_ref = np.zeros((1,3))
            j_ref = np.zeros((1,3))
            yaw_ref = np.zeros((1,))
            yaw_rate_ref = np.zeros((1,))

        return p_ref, v_ref, a_ref, j_ref, yaw_ref, yaw_rate_ref


    def solvequintic(self, T:float, start:np.ndarray, end:np.ndarray):
        """
        Solve the quintic polinomial
        
        Args:
            T (float): The time required for traversal of the path
            start (np.ndarray): 1x3 numpy array with the start loc, vel, and accel
            end (np.ndarray): 1x3 numpy array with the end loc, vel, and accel
        Returns:
            np.ndarray: A 1x5 numpy matrix with the coefficients of the quintic polynomial
        """

        # a_0, a_1, a_2 can be calculated with the boundary conditions
        a_0 = start[0]     # equal to the starting posisiton
        a_1 = start[1]     # equal to the starting velocity
        a_2 = 0.5*start[2] # equal to half the starting acceleration

        # a_3, a_4, a_5 can be solved with solving a system of lin. eqs
        # (https://github.com/ChenBohan/Robotics-Path-Planning-04-Quintic-Polynomial-Solver)
        mat_A = np.matrix([[np.power(T,3), np.power(T,4), np.power(T,5)],
                        [3*np.power(T,2), 4*np.power(T,3), 5*np.power(T,4)],
                        [6*T, 12*np.power(T,2), 20*np.power(T,3)]])

        mat_B = np.matrix([[end[0] - (start[0] + start[1]*T + 0.5*start[2]*T*T)],
                        [end[1] - (start[1] + start[2]*T)],
                        [end[2] - start[2]]])

        mat_A_inv = np.linalg.inv(mat_A)

        mat_C = mat_A_inv * mat_B

        coefs = np.array([a_0, a_1, a_2, float(mat_C[0]), float(mat_C[1]), float(mat_C[2])])

        return coefs


    def poly5func(self, t, a_0, a_1, a_2, a_3, a_4, a_5): # position
        return a_0 + a_1*t + a_2*np.power(t,2) + a_3*np.power(t,3) + a_4*np.power(t,4) + a_5*np.power(t,5)


    def poly5func_d(self, t, a_1, a_2, a_3, a_4, a_5): # velocity
        return a_1 + 2*a_2*t + 3*a_3*np.power(t,2) + 4*a_4*np.power(t,3) + 5*a_5*np.power(t,4)


    def poly5func_dd(self, t, a_2, a_3, a_4, a_5): # acceleration
        return 2*a_2 + 6*a_3*t + 12*a_4*np.power(t,2) + 20*a_5*np.power(t,3)


    def poly5func_ddd(self, t, a_3, a_4, a_5): # jerk
        return 6*a_3 + 24*a_4*t + 60*a_5*np.power(t,2)