"""
| plot_ground_tracks.py
| Author: Hajo Erwich (hajo_erwich@live.nl)
| License: BSD-3-Clause. Copyright (c) 2023, Hajo Erwich. All rights reserved.
| Description: Create map plot of ground tracks from one experiment (save multiple at once)
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
ENVS_DIR =    f"{PEGASUS_DIR}/examples/environments"
PLOT_DIR =    f"{PEGASUS_DIR}/examples/utils/plot"

### Save Params ###
exp_id = 199
save_plot = False
filetype = 'pdf'
env_id = 6
multiple = True  # create multiple plots from multiple experiments
exp_amount = 9 # amount of experiments

def plot_pos(exp_id:str,
             env:int,
             save:bool = False,
             height:float   = 4.0,
             plot_occ:bool  = True,
             filetype:str   = 'pdf'
             ) -> None :
   
    save_fname = f"{PLOT_DIR}/figures/pos_{exp_id}.{filetype}"
    
    files = glob.glob(f"{RESULTS_DIR}/{exp_id}/*")
    files_sorted = [files[i] for i in np.argsort(files)]
    print(files_sorted)
    
    #-------------------------
    # Ground track plot
    #-------------------------

    fig, ax = plt.subplots()
    fig.set_figheight(4)
    fig.set_figwidth(5)

    for _,file in enumerate(files_sorted):

        with np.load(file) as data:
            locs = data['p']

        xdata = locs[:,0]
        ydata = locs[:,1]
        plt.plot(xdata,ydata)

    #-------------------------
    # Plot Occupancy
    #-------------------------
    alpha_gas = 1
    if plot_occ:
        alpha_gas = 0.8
        occ_data_file = glob.glob(f"{ENVS_DIR}/{str(env_id).zfill(3)}/occupancy/*grid.npy")[0] # occ data file
        z_idx = math.ceil((height)/0.2)
        occmap = np.transpose(np.load(occ_data_file)[z_idx])[2:-1,2:-1]
        plt.imshow(occmap, vmin=0, vmax=1, origin='lower', interpolation='none', cmap='binary', extent=(0.,15.,0.,15.), alpha=1)
        plt.draw()

    #-------------------------
    # Plot Gas
    #-------------------------
    with np.load(f'{PLOT_DIR}/ppm_data/ppm_env_{str(env).zfill(3)}.npz') as data:
        ppm = data['arr_0']

    im = ax.imshow(np.flip(np.transpose(ppm[:,:,(int(4*height) - 1)]),axis=0), cmap='gray_r', extent=(0.,15.,0.,15.), alpha=alpha_gas)
    # Create colorbar
    cbar = ax.figure.colorbar(im, ax=ax)
    cbar.ax.set_ylabel("ppm", rotation=-90, va="bottom")

    # plot start and source
    plt.scatter([xdata[0]], [ydata[0]], c='r', label='start', marker="^", zorder=3.5)
    if (env%2) == 0:
        plt.scatter([1.], [10.], c='g', label='source', zorder=3.5)
    else:
        plt.scatter([5.], [1.], c='g', label='source', zorder=3.5)

    # plot inlet outlet annotaions
    ax.annotate('', xy=(2.2, 1.3), xytext=(0.2, 1.3), color='black',
                arrowprops=dict(facecolor='black', shrink=0.05))
    ax.annotate('', xy=(14.8, 12.7), xytext=(12.8, 12.7), color='black',
                arrowprops=dict(facecolor='black', shrink=0.05))
    ax.annotate('inlet', xy=(0.5, 2))
    ax.annotate('outlet', xy=(12.8, 13.5))

    #plt.title(f'Dung Beetle Algorithm - Experiment {exp_id}')
    plt.title(f'E. Coli Algorithm - Ground Track')
    plt.axis('scaled')
    plt.grid()
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    
    if (env%2) == 0:
        plt.legend(loc='lower right')
    else:
        plt.legend(loc='upper left')
    
    fig.tight_layout()

    if save:
        plt.savefig(str(save_fname), format=filetype, bbox_inches='tight')
    else:
        plt.show()


def plot_multiple(start:int, env:int, amount:int=9, save:bool=False, filetype:str='pdf') -> None:
    experiments = [str(i + start).zfill(3) for i in range(amount)]
    
    occ = True
    if env <= 2: occ = False
    
    for exp in experiments:
        plot_pos(exp, env=env, save=save, plot_occ=occ, filetype=filetype)


if multiple:
    plot_multiple(exp_id,
                env=env_id,
                amount=exp_amount,
                save=save_plot,
                filetype=filetype) # if not showing, then saving
else:
    plot_pos(exp_id, env=env_id, save=save_plot, filetype=filetype)