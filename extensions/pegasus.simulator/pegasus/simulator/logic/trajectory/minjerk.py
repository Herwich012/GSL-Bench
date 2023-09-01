"""
| File: minjerk.py
| Author: Hajo Erwich (h.h.erwich@student.tudelft.nl)
| License: BSD-3-Clause. Copyright (c) 2023, Hajo Erwich. All rights reserved.
| Description: Class that genereates a trajectory with minimal jerk
"""
__all__ = ["TrajectoryMinJerk"]

import numpy as np


class TrajectoryMinJerk:
    """ Trajectory generation with minimal jerk
    """
    def __init__(self, time2wp:float=None, avg_vel:float=None) -> None:
        if time2wp == None and avg_vel == None:
            self._mode = 'time2wp'
            self._T = 5.0 # [s]
        elif time2wp == None and avg_vel != None:
            self._mode = 'avg_vel'
            self._avg_vel = avg_vel # [m/s]
        else:
            self._mode = 'time2wp'
            self._T = time2wp # [s]
        


    def generate(self, dt:float, start:np.ndarray, end:np.ndarray) -> np.ndarray:
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
            trajectory (np.ndarray):
                p_ref : the desired position over time in XYZ
                v_ref : the desired velocity over time in XYZ
                a_ref : the desired acceleration over time in XYZ
                j_ref : the desired jerk over time in XYZ
                yaw_ref : the desired yaw over time in XYZ
                yaw_rate_ref : the desired yaw rate over time in XYZ
        """
        if not np.array_equal(start, end): # only generate trajectory if waypoints are different
            # Endtime T due to the average speed vel:
            if self._mode == 'time2wp':
                T = self._T
            else:
                T = np.linalg.norm(end[0,:] - start[0,:]) / self._avg_vel
            
            xdata = np.linspace(0, T, int(T/dt))

            # Initialize the ref arrays
            p_ref =  np.zeros((int(T/dt),3))
            v_ref =  np.zeros((int(T/dt),3))
            a_ref =  np.zeros((int(T/dt),3))
            j_ref =  np.zeros((int(T/dt),3))
            yaw_ref = np.zeros((int(T/dt),1))
            yaw_rate_ref = np.zeros((int(T/dt),1))

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
            yaw_ref = np.zeros((1,1))
            yaw_rate_ref = np.zeros((1,1))

        return np.hstack((p_ref, v_ref, a_ref, j_ref, yaw_ref, yaw_rate_ref))


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
