import math
import numpy as np
import matplotlib.pyplot as plt

class Filament():
    def __init__(self, id=0, x=0.0, y=0.0, z=0.0, sigma=0.0):
        self.id: int = id
        self.x: float = x
        self.y: float = y
        self.z: float = z
        self.sigma: float = sigma


def concentration_from_filament(loc:np.ndarray, filament:Filament, gas_data_head:np.ndarray) -> float:
    distance_cm = 100 * math.sqrt(math.pow((loc[0] - filament.x),2) + math.pow((loc[1] - filament.y),2) + \
                                    math.pow((loc[2] - filament.z),2))

    num_moles_target_cm3 = (gas_data_head['filament_num_moles_of_gas'][0] / \
                            (math.sqrt(8*math.pow(math.pi,3)) * math.pow(filament.sigma,3))) * \
                                math.exp(-math.pow(distance_cm,2)/(2*math.pow(filament.sigma,2)))
    ppm = num_moles_target_cm3 / gas_data_head['num_moles_all_gases_in_cm3'][0] * 1000000 # parts of target gas per million
    return ppm


def concentration_at_location(gas_data: np.ndarray, gas_data_head: np.ndarray, loc: np.ndarray) -> float:
    gas_conc = 0.0
    for fil in gas_data: # filament: id, x, y, z, sigma
        filament = Filament(fil[0],fil[1],fil[2],fil[3],fil[4])
        dist_SQR = math.pow((loc[0] - filament.x),2) + math.pow((loc[1] - filament.y),2) + math.pow((loc[2] - filament.z),2)
        limit_distance = filament.sigma*5/100

        # If filament is within range, calculate the contribution to the gas concentration
        if dist_SQR < math.pow(limit_distance,2): # TODO add check for obstacles
            gas_conc += concentration_from_filament(loc, filament, gas_data_head)
    
    return gas_conc


def ppm_cells(env_size: np.ndarray, cell_size: float) -> np.ndarray:
    return np.array(env_size/cell_size, dtype=int)


def cell_center_coordinates(env_size: np.ndarray, cell_size: float, ppm_cells: np.ndarray, env_orig=np.zeros((3,))) -> np.ndarray:
    x = np.linspace(env_orig[0] + (0.5 * cell_size), env_size[0] + env_orig[0] - (0.5 * cell_size), ppm_cells[0])
    y = np.linspace(env_orig[1] + (0.5 * cell_size), env_size[1] + env_orig[1] - (0.5 * cell_size), ppm_cells[1])
    z = np.linspace(env_orig[2] + (0.5 * cell_size), env_size[2] + env_orig[2] - (0.5 * cell_size), ppm_cells[2])

    xv, yv, zv = np.meshgrid(x, y, z, sparse=True)

    return x, y, z


env_size = np.array([15.0, 15.0, 8.0])
ppm_cell_size = 0.25 # resolution of the concentration grid
ppm_shape = ppm_cells(env_size, ppm_cell_size)
gas_data_dir = "/home/hajo/0THESIS/environments/005/gas_data/wh_complex_0010/" # gas data folder
gas_iteration = 400
gas_data = np.load(f"{gas_data_dir}iteration_{gas_iteration}_fil.npy")
gas_data_head = np.load(f"{gas_data_dir}iteration_{gas_iteration}_head.npy")


# calculate the ppm for each cell
ppm_array = np.zeros((ppm_shape))
xv, yv, zv = cell_center_coordinates(env_size, ppm_cell_size, ppm_shape)
for k, z_loc in enumerate(zv):
    for j, y_loc in enumerate(yv):
        for i, x_loc in enumerate(xv):
            ppm_array[i,j,k] = concentration_at_location(gas_data, gas_data_head, np.array([x_loc, y_loc, z_loc]))
            print(f'{str(i).zfill(3)}_{str(j).zfill(3)}_{str(k).zfill(3)}')

np.savez('ppm_env_005.npz', ppm_array)


# loc = np.array([7.0, 5.0, 3.0])
# print(concentration_at_location(gas_data, gas_data_head, loc))


# npz = np.load("ppm_array.npz")
# ppm = npz['arr_0']

# fig, ax = plt.subplots()
# im = ax.imshow(ppm[:,:,10])

# ax.set_title("Concentration of gass in ppm")
# fig.tight_layout()
# plt.show()