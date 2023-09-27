"""
| plot_sr_loc_multi.py
| Author: Hajo Erwich (hajo_erwich@live.nl)
| License: BSD-3-Clause. Copyright (c) 2023, Hajo Erwich. All rights reserved.
| Description: Create map plot of starting positions and the success rates of algorithms
"""
import math
import glob
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
plt.rcParams['figure.dpi'] = 300
plt.rcParams['pdf.fonttype'] = 42
plt.rcParams['ps.fonttype'] = 42
plt.rcParams["font.family"] = "Times New Roman"
HOME_DIR = Path.home()
PEGASUS_DIR = f"{HOME_DIR}/Omniverse_extensions/PegasusSimulator"
RESULTS_DIR = f"{PEGASUS_DIR}/examples/results"
PLOT_DIR = f"{PEGASUS_DIR}/examples/utils/plot/figures"

### Save Params & env id ###
save_plot = False
filetype = 'png'
env_id = 3
save_fname = f"{PLOT_DIR}/success_{str(env_id).zfill(3)}.{filetype}"

### Data selection ###
exp_id_starts = [82, 100, 91]
height = 4.0
annotations = ['E:', 'B:', 'R:'] # E. Coli, Dung beetle, Random
runs_per_exp = 9 # amount of runs in one experiment
posittion_grid = np.array([[3.0, 3.0, 0.2], # 0
                           [7.5, 3.0, 0.2], # 1 
                           [12.0,3.0, 0.2], # 2
                           [3.0, 7.5, 0.2], # 3
                           [7.5, 7.5, 0.2], # 4
                           [12.0,7.5, 0.2], # 5
                           [3.0, 12.0,0.2], # 6
                           [7.5, 12.0,0.2], # 7
                           [12.0,12.0,0.2]]) # 8

fig, ax = plt.subplots()
fig.set_figheight(4)
fig.set_figwidth(5)

#-------------------------
# Plot Occupancy
#-------------------------
# TODO: automate occupancy file selection
# occ_data_file = f"{HOME_DIR}/0THESIS/environments/003-004/occupancy/wh_simple_0000_grid.npy" # occ data file
occ_data_file = f"{HOME_DIR}/0THESIS/environments/006/occupancy/wh_complex_0010_grid.npy" # occ data file
z_idx = math.ceil((height)/0.2)
occmap = np.transpose(np.load(occ_data_file)[z_idx])[2:-1,2:-1]
plt.imshow(occmap, vmin=0, vmax=1, origin='lower', interpolation='none', cmap='binary', extent=(0.,15.,0.,15.), alpha=1)
plt.draw()

#-------------------------
# Plot Gas
#-------------------------
with np.load(f'./ppm_data/ppm_env_{str(env_id).zfill(3)}.npz') as data:
    ppm = data['arr_0']

im = ax.imshow(np.flip(np.transpose(ppm[:,:,(int(4*height) - 1)]),axis=0), cmap='gray_r', extent=(0.,15.,0.,15.), alpha=0.8)
# Create colorbar
cbar = ax.figure.colorbar(im, ax=ax)
cbar.ax.set_ylabel("ppm", rotation=-90, va="bottom")

#-------------------------
# Start postitions and source loc
#-------------------------
plt.scatter(posittion_grid[:,0], posittion_grid[:,1], c='r', marker='^', label='start', zorder=3.5)
if (env_id%2) == 0:
    plt.scatter([1.], [10.], c='g', label='source', zorder=3.5)
else:
    plt.scatter([5.], [1.], c='g', label='source', zorder=3.5)


#-------------------------
# Annotations
#-------------------------
for h,(start,annotation) in enumerate(zip(exp_id_starts,annotations)):
    exp_id_start = start

    experiments = [str(i + exp_id_start).zfill(3) for i in range(runs_per_exp)]
    success = []
    succtest = np.zeros((runs_per_exp,10))

    for j,exp_id in enumerate(experiments):
        # stat_dir = f"{HOME_DIR}/0THESIS/experiments/{exp_id}"
        # files = glob.glob(f"{stat_dir}/*")
        files = glob.glob(f"{RESULTS_DIR}/{exp_id}/*")
        files_sorted = [files[i] for i in np.argsort(files)]
        
        for k,file in enumerate(files_sorted):
            with np.load(file) as data:
                success.append(data['run_success'][0][0])
                succtest[j][k] = data['run_success'][0][0]

    success_rate = round(success.count(True) / len(success), 3)
    succ_rate_per_loc = np.count_nonzero(succtest, axis=1)

    print(success_rate)
    print(succtest)
    print(succ_rate_per_loc)

    for i,succ_rate in enumerate(succ_rate_per_loc):
        ax.annotate(f'{annotation} {succ_rate/10}', 
                   #xy=(posittion_grid[i,0] - 0.7, posittion_grid[i,1]+ 1.8 - (h*0.6))) # top
                   xy=(posittion_grid[i,0] + 0.5, posittion_grid[i,1]+ 1.8 - (h*0.6))) # top right
                   #xy=(posittion_grid[i,0] + 0.5, posittion_grid[i,1] - 0.5 - (h*0.6))) # bottom right
                   #xy=(posittion_grid[i,0] - 0.7, posittion_grid[i,1] - 1.0 - (h*0.6))) # bottom


plt.title(f'Success Rates Per Location - Environment {env_id}')
plt.axis('scaled')
plt.grid()
plt.xlabel('x [m]')
plt.ylabel('y [m]')
#plt.legend(loc='lower right')
plt.legend(loc='lower left')
# plt.legend(loc='upper right')
fig.tight_layout()

if save_plot:
    plt.savefig(str(save_fname), format=filetype, bbox_inches='tight')
else:
    plt.show()
