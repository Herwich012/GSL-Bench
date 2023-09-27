import os
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


exp_id_start = 37
amount = 9
save_plot = False
filetype = 'pdf'
save_fname = f"{PLOT_DIR}/success_{exp_id_start}.{filetype}"

experiments = [str(i + exp_id_start).zfill(3) for i in range(amount)]
success = []
succtest = np.zeros((amount,10))

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


with np.load('ppm_env_002.npz') as data: # TODO: add env selection
    ppm = data['arr_0']

im = ax.imshow(np.flip(np.transpose(ppm[:,:,15]),axis=0), cmap='gray_r', extent=(0.,15.,0.,15.))
# Create colorbar
cbar = ax.figure.colorbar(im, ax=ax)
cbar.ax.set_ylabel("ppm", rotation=-90, va="bottom")

plt.scatter(posittion_grid[:,0], posittion_grid[:,1], c='r', label='start', zorder=3.5)
plt.scatter([1.], [10.], c='g', label='source', zorder=3.5)
#plt.scatter([5.], [1.], c='g', label='source', zorder=3.5)

for i,succ_rate in enumerate(succ_rate_per_loc):
    ax.annotate(f'{succ_rate/10}', xy=(posittion_grid[i,0] + 0.5, posittion_grid[i,1]))

plt.title(f'Success Rate, Random Walker, Experiments {exp_id_start}-{exp_id_start + amount - 1}')
plt.axis('scaled')
plt.grid()
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.legend(loc='lower left')
fig.tight_layout()

if save_plot:
    plt.savefig(str(save_fname), format=filetype, bbox_inches='tight')
plt.show()
