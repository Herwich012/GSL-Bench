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
from gridmap import OccupancyGridMap
from a_star import a_star
from utils import expand_boundaries, shortcut_path


class ObstacleAvoidance:
    """ 
    ObstacleAvoidance class containing two main methods for OA given a ground truth:
    - Generate path around obstacles using A*
    - Get point outside of obstacle closest to given point inside obstacle
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
        
        self._gmap = OccupancyGridMap(self._occ_data, cell_size=self._env_spec["cell_size"])

    
    def get_go_around_mission(self, start_wp, end_wp):
        """
        Get a series of waypoints to the goal waypoint around present obstacles
        """
        # convert wp to 2D points
        start_2D, end_2D = start_wp[0,:2], end_wp[0,:2]

        # run A*
        path, path_cells = a_star(start_2D, end_2D, self._gmap, movement='4N')
        
        # get the idx of only the necessary points in the path
        shortcut_idx = shortcut_path(self._env_spec, self._occ_data, path, path_cells, self._altitude)

        # 

    
    def get_outside_wp(self):
        pass

