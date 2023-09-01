"""
Taken from (https://github.com/richardos/occupancy-grid-a-star.git)
"""
__all__ = ["OccupancyGridMap"]
import math
import numpy as np

# takes into account the env_min instead of assuming that the env_min is in the origin.
class OccupancyGridMap:
    def __init__(self, data_array, env_spec, occupancy_threshold=0.8):
        """
        Creates a grid map
        :param data_array: a 2D array with a value of occupancy per cell (values from 0 - 1)
        :param cell_size: cell size in meters
        :param occupancy_threshold: A threshold to determine whether a cell is occupied or free.
        A cell is considered occupied if its value >= occupancy_threshold, free otherwise.
        """

        self.data = data_array
        self.env_spec = env_spec
        self.dim_cells = np.shape(data_array)[::-1]
        self.dim_meters = (env_spec["env_max"][0]-env_spec["env_min"][0], 
                           env_spec["env_max"][1]-env_spec["env_min"][1])
        self.cell_size = env_spec["cell_size"]
        self.occupancy_threshold = occupancy_threshold
        # 2D array to mark visited nodes (in the beginning, no node has been visited)
        self.visited = np.zeros(self.dim_cells, dtype=np.float32)

    def mark_visited_idx(self, point_idx):
        """
        Mark a point as visited.
        :param point_idx: a point (x, y) in data array
        """
        x_index, y_index = point_idx
        if x_index < 0 or y_index < 0 or x_index >= self.dim_cells[0] or y_index >= self.dim_cells[1]:
            raise Exception('Point is outside map boundary')

        self.visited[x_index][y_index] = 1.0

    def mark_visited(self, point):
        """
        Mark a point as visited.
        :param point: a 2D point (x, y) in meters
        """
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)

        return self.mark_visited_idx((x_index, y_index))

    def is_visited_idx(self, point_idx):
        """
        Check whether the given point is visited.
        :param point_idx: a point (x, y) in data array
        :return: True if the given point is visited, false otherwise
        """
        x_index, y_index = point_idx
        if x_index < 0 or y_index < 0 or x_index >= self.dim_cells[0] or y_index >= self.dim_cells[1]:
            raise Exception('Point is outside map boundary')

        if self.visited[x_index][y_index] == 1.0:
            return True
        else:
            return False

    def is_visited(self, point):
        """
        Check whether the given point is visited.
        :param point: a 2D point (x, y) in meters
        :return: True if the given point is visited, false otherwise
        """
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)

        return self.is_visited_idx((x_index, y_index))

    def get_data_idx(self, point_idx):
        """
        Get the occupancy value of the given point.
        :param point_idx: a point (x, y) in data array
        :return: the occupancy value of the given point
        """
        x_index, y_index = point_idx
        if x_index < 0 or y_index < 0 or x_index >= self.dim_cells[0] or y_index >= self.dim_cells[1]:
            raise Exception('Point is outside map boundary')

        return self.data[y_index][x_index]

    def get_data(self, point):
        """
        Get the occupancy value of the given point.
        :param point: a 2D point (x, y) in meters
        :return: the occupancy value of the given point
        """
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)

        return self.get_data_idx((x_index, y_index))

    def set_data_idx(self, point_idx, new_value):
        """
        Set the occupancy value of the given point.
        :param point_idx: a point (x, y) in data array
        :param new_value: the new occupancy values
        """
        x_index, y_index = point_idx
        if x_index < 0 or y_index < 0 or x_index >= self.dim_cells[0] or y_index >= self.dim_cells[1]:
            raise Exception('Point is outside map boundary')

        self.data[y_index][x_index] = new_value

    def set_data(self, point, new_value):
        """
        Set the occupancy value of the given point.
        :param point: a 2D point (x, y) in meters
        :param new_value: the new occupancy value
        """
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)

        self.set_data_idx((x_index, y_index), new_value)

    def is_inside_idx(self, point_idx):
        """
        Check whether the given point is inside the map.
        :param point_idx: a point (x, y) in data array
        :return: True if the given point is inside the map, false otherwise
        """
        x_index, y_index = point_idx
        if x_index < 0 or y_index < 0 or x_index >= self.dim_cells[0] or y_index >= self.dim_cells[1]:
            return False
        else:
            return True

    def is_inside(self, point):
        """
        Check whether the given point is inside the map.
        :param point: a 2D point (x, y) in meters
        :return: True if the given point is inside the map, false otherwise
        """
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)

        return self.is_inside_idx((x_index, y_index))

    def is_occupied_idx(self, point_idx):
        """
        Check whether the given point is occupied according the the occupancy threshold.
        :param point_idx: a point (x, y) in data array
        :return: True if the given point is occupied, false otherwise
        """
        x_index, y_index = point_idx
        if self.get_data_idx((x_index, y_index)) >= self.occupancy_threshold:
            return True
        else:
            return False

    def is_occupied(self, point):
        """
        Check whether the given point is occupied according the the occupancy threshold.
        :param point: a 2D point (x, y) in meters
        :return: True if the given point is occupied, false otherwise
        """
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)

        return self.is_occupied_idx((x_index, y_index))

    def get_index_from_coordinates(self, x, y):
        """
        Get the array indices of the given point.
        :param x: the point's x-coordinate in meters
        :param y: the point's y-coordinate in meters
        :return: the corresponding array indices as a (x, y) tuple
        """

        x_index = math.floor((x - self.env_spec["env_min"][0])/self.env_spec["cell_size"])
        y_index = math.floor((y - self.env_spec["env_min"][1])/self.env_spec["cell_size"])

        return x_index, y_index

    def get_coordinates_from_index(self, x_index, y_index):
        """
        Get the coordinates of the given array point in meters.
        :param x_index: the point's x index
        :param y_index: the point's y index
        :return: the corresponding point in meters as a (x, y) tuple
        """

        x = (x_index*self.env_spec["cell_size"]) + self.env_spec["env_min"][0]
        y = (y_index*self.env_spec["cell_size"]) + self.env_spec["env_min"][1]

        return x, y
