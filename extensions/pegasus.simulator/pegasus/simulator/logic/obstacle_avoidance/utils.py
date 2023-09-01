"""
Taken from (https://github.com/richardos/occupancy-grid-a-star.git)
"""
import math
import numpy as np


def dist2d(point1, point2):
    """
    Euclidean distance between two points
    :param point1:
    :param point2:
    :return:
    """

    x1, y1 = point1[0:2]
    x2, y2 = point2[0:2]

    dist2 = (x1 - x2)**2 + (y1 - y2)**2

    return math.sqrt(dist2)


def expand_boundaries(arr:np.ndarray, exp:float, cell_size:float) -> np.ndarray:
    arr_exp = np.copy(arr)
    exp_ops = math.ceil(exp/cell_size)

    for _ in range(exp_ops):
        for idx in np.argwhere(arr == 0):
            if arr[idx[0]-1:idx[0]+2,idx[1]-1:idx[1]+2].any():
                arr_exp[idx[0],idx[1]] = 1
        arr = arr_exp
        arr_exp = np.copy(arr)

    return arr_exp


def shortcut_path(env_spec:dict, grid:np.ndarray, path_loc:list, path_idx:list) -> list:
    corner_idx = find_path_corner_idx(path_idx)

    idx = 0
    end_idx = len(corner_idx) - 1
    short_idx = [0]

    while idx < (end_idx - 1):
        shortened = False
        check_idx = end_idx

        while check_idx != (idx + 1):
            if check_env_for_obstacle2D(env_spec, grid,
                                      np.array([path_loc[corner_idx[idx]][0],path_loc[corner_idx[idx]][1]]),
                                      np.array([path_loc[corner_idx[check_idx]][0],path_loc[corner_idx[check_idx]][1]])):
                short_idx.append(corner_idx[check_idx])
                idx = check_idx - 1 # -1 because it will get iterated in the outer while loop
                shortened = True
                break

            check_idx -= 1
        
        idx += 1
        if not shortened:
            short_idx.append(corner_idx[idx])
    
    if short_idx[-1] != corner_idx[-1]: # dirty fix in case last idx is not appended in while loops
        short_idx.append(corner_idx[end_idx])

    return short_idx
                                      


def find_path_corner_idx(path:list) -> list:
    corner_idx = [0]
    idx = 0
    if path[0][0] == path[1][0]:
        direction = 'y'
    else:
        direction = 'x'
    
    direction_prev = direction
    
    while True:
        while direction_prev == direction and idx != (len(path) - 2):
            idx += 1
            direction_prev = direction
            point1 = path[idx]
            point2 = path[idx+1]
            
            if point1[0] == point2[0]:
                direction = 'y'
            else:
                direction = 'x'
        
        if idx == (len(path) - 2):
            corner_idx.append(idx+1)
            break

        corner_idx.append(idx)
        direction_prev = direction

    return corner_idx


def check_env_for_obstacle2D(env_spec:dict, grid:np.ndarray, start:np.ndarray, end:np.ndarray) -> bool:
    dist = np.linalg.norm((start,end))
    samples = math.ceil(dist/env_spec["cell_size"]) + 1 # +1 to avoid an edgecase of skipping cells

    x_points = np.linspace(start[0], end[0], samples)
    y_points = np.linspace(start[1], end[1], samples)
    
    for _,(x,y) in enumerate(zip(x_points,y_points)):
        x_idx = math.floor((x - env_spec["env_min"][0])/env_spec["cell_size"])
        y_idx = math.floor((y - env_spec["env_min"][1])/env_spec["cell_size"])

        if grid[y_idx, x_idx] != 0: return False

    # Direct line of sight!
    return True


def outside_obstacle2D(env_spec:dict, grid:np.ndarray, start:np.ndarray, end:np.ndarray) -> list:
    dist = np.linalg.norm((start,end))
    samples = math.ceil(dist/env_spec["cell_size"]) + 1 # +1 to avoid an edgecase of skipping cells

    # x_points = np.linspace(start_pos[0], end_pos[0], samples)
    # y_points = np.linspace(start_pos[1], end_pos[1], samples)
    x_points = np.linspace(start[0], end[0], samples)
    y_points = np.linspace(start[1], end[1], samples)
    
    check_path = []
    for _,(x,y) in enumerate(zip(x_points,y_points)):
        x_idx = math.floor((x - env_spec["env_min"][0])/env_spec["cell_size"])
        y_idx = math.floor((y - env_spec["env_min"][1])/env_spec["cell_size"])

        check_path.append((x,y))
        if grid[y_idx, x_idx] != 0: return check_path

    return check_path


# def check_env_for_obstacle3D(env:dict, grid:np.ndarray, start:np.ndarray, end:np.ndarray) -> bool:
#     start_pos = get_pos(start) # [m]
#     end_pos = get_pos(end) # [m]

#     # First check if start or end position are outside environment
#     if not (check_in_env(env, start_pos) or check_in_env(env, end_pos)): return False
    
#     dist = np.linalg.norm((start_pos,end_pos))
#     samples = math.ceil(dist/env["cell_size"]) + 1 # +1 to avoid an edgecase of skipping cells

#     x_points = np.linspace(start_pos[0], end_pos[0], samples)
#     y_points = np.linspace(start_pos[1], end_pos[1], samples)
#     z_points = np.linspace(start_pos[2], end_pos[2], samples)
    
#     check_path = []
#     for _,(x,y,z) in enumerate(zip(x_points,y_points,z_points)):
#         x_idx = math.floor((x - env["env_min"][0])/env["cell_size"])
#         y_idx = math.floor((y - env["env_min"][1])/env["cell_size"])
#         z_idx = math.floor((z - env["env_min"][2])/env["cell_size"])

#         check_path.append((x_idx,y_idx))
#         if grid[z_idx, x_idx, y_idx] != 0: return False, check_path 

#     # Direct line of sight!
#     return True


def check_in_env(env:dict, loc:np.ndarray) -> bool:
    """
    Method that returns True if point is within the environment
    
    Args:
        wp (np.ndarray): (3,3)/(3,) numpy array with the start loc, vel, and accel or loc only
        env (list): a [2,3] size list with the environment min and max bounds
    Returns:
        bool: True if waypoint is in environment
    """
    pos = get_pos(loc)
    
    if pos[0] < env["env_min"][0] or pos[1] < env["env_min"][1] or pos[2] < env["env_min"][2] or \
        pos[0] > env["env_max"][0] or pos[1] > env["env_max"][1] or pos[2] > env["env_max"][2]:
        return False
    else:
        return True


def get_pos(loc:np.ndarray) -> np.ndarray:
    if np.ndim(loc) > 1:
        return loc[0,:]
    else:
        return loc
