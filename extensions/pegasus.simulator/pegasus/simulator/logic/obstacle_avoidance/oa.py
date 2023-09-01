"""
| File: default.py
| Author: Hajo Erwich (h.h.erwich@student.tudelft.nl)
| License: BSD-3-Clause. Copyright (c) 2023, Hajo Erwich. All rights reserved.
| Description: Class that contains the default obstacle avoidance with the
| given groundtruth from AutoGDM
"""
__all__ = ["ObstacleAvoidance"]

import carb
import math
import numpy as np
from pegasus.simulator.logic.obstacle_avoidance.gridmap import OccupancyGridMap
from pegasus.simulator.logic.obstacle_avoidance.a_star import a_star
from pegasus.simulator.logic.obstacle_avoidance.utils import (
    expand_boundaries, 
    shortcut_path, 
    outside_obstacle2D, 
    check_env_for_obstacle2D)


class ObstacleAvoidance:
    """ 
    ObstacleAvoidance class containing two main methods for OA given a ground truth:
    - Generate path around obstacles using A*
    - Get point outside of obstacle in line with the goal wp
    """

    def __init__(self, 
                 env_dict:dict,
                 altitude:float=2.0,
                 expand:float=0.4) -> None:
        
        self._env_dict = env_dict
        self._env_spec = self._env_dict["env_spec"]
        self._altitude = altitude # [m]
        self._bounds_expansion = expand # [m]

        occ_data_file = f"{self._env_dict['AutoGDM2_dir']}environments/occupancy/{self._env_dict['env_name']}_grid.npy" # occ data file
        z_idx = math.ceil((self._altitude-self._env_spec["env_min"][2])/self._env_spec["cell_size"])

        occ_data_array = np.transpose(np.load(occ_data_file)[z_idx]) # transpose to correct XY orientation
        self._occ_data = expand_boundaries(occ_data_array, # expand bourndaries
                                           self._bounds_expansion,
                                           cell_size=self._env_spec["cell_size"])
        
        self._gmap = OccupancyGridMap(self._occ_data, self._env_spec)


    def check_for_obstacle(self, start_wp:np.ndarray, end_wp:np.ndarray) -> int: # 0: no obstacle, 1: path obstructed, 2: in obstacle
        # TODO - add possibility for wp to be outside of env -> for now handled by the GSL algorithm
        end_2D = start_wp[0,:2] # reverse waypoints to start evaluating from the end_wp
        start_2D = end_wp[0,:2]
        
        if check_env_for_obstacle2D(self._env_spec, self._occ_data, start_2D, end_2D):
            return 0 # Direct line of sight
        else:
            points_xy = outside_obstacle2D(self._env_spec, self._occ_data, start_2D, end_2D)
            if len(points_xy) > 1:
                return 1 # path obstructed
            else:
                return 2 # wp in obstacle


    def get_go_around_mission(self, start_wp:np.ndarray, end_wp:np.ndarray) -> np.ndarray:
        """
        Get a series of waypoints to the goal waypoint around present obstacles
        """
        carb.log_warn("Waypoint behind obstacle, going around...")
        # convert wp to 2D points
        start_2D = start_wp[0,:2] 
        end_2D = end_wp[0,:2]

        # run A*
        path, path_cells = a_star(start_2D, end_2D, self._gmap, movement='4N')
        # print(f"path: {path}")
        # print(f"path_cells: {path_cells}")
        
        if path: # dirty fix in case a_star returns nothing because the path is clear?
            # get the idx of only the necessary points in the path
            shortcut_idx = shortcut_path(self._env_spec, self._occ_data, path, path_cells)
            # print(f"shortcut_idx: {shortcut_idx}")

            # make waypoints
            waypoints = np.zeros((len(shortcut_idx),3,3)) 
            for i,idx in enumerate(shortcut_idx):
                waypoints[i,0,:] = np.array([path[idx][0], path[idx][1], start_wp[0,2]])
        else:
            waypoints = np.zeros((1,3,3))
            waypoints[0] = end_wp

        return waypoints        


    def get_outside_wp(self, start_wp:np.ndarray, end_wp:np.ndarray) -> np.ndarray:
        """
        Get a waypoint just outside of an obstacle
        """
        carb.log_warn("Waypoint in obstacle, going to closest point...")
        start_2D = start_wp[0,:2] 
        end_2D = end_wp[0,:2]
        points_xy = outside_obstacle2D(self._env_spec, self._occ_data, start_2D, end_2D)
        
        waypoint = np.zeros((1,3,3))
        waypoint[0,0,:] = np.array([points_xy[-2][0], points_xy[-2][1],start_wp[0,2]])

        return waypoint