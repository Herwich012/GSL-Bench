"""
| plot_gas_sensor.py
| Author: Hajo Erwich (hajo_erwich@live.nl)
| License: BSD-3-Clause. Copyright (c) 2023, Hajo Erwich. All rights reserved.
| Description: Create gas sensor plots over time
"""
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

### Experiment id & Save params ###
exp_id = 190
save_plot = False
filetype = 'png'
save_fname = f"{PLOT_DIR}/gas_{str(exp_id).zfill(3)}.{filetype}"

### Files ###
# stat_dir = f"{HOME_DIR}/0THESIS/experiments/{exp_id}"
# files = glob.glob(f"{stat_dir}/*")
files = glob.glob(f"{RESULTS_DIR}/{exp_id}/*")
files_sorted = [files[i] for i in np.argsort(files)]

#-------------------------
# Concentration / sensor plots
#-------------------------
fig, ax = plt.subplots(2,1)
fig.set_figheight(4)
fig.set_figwidth(5)
# fig.suptitle(f'Gas Concentration and Sensor Reading - Experiment {str(exp_id).zfill(3)}')
fig.suptitle(f'Dung Beetle Algorithm - Gas Concentration and Sensor Reading')

for _,file in enumerate(files_sorted):

    with np.load(file) as data:
        time = data['time']
        conc = data['c'][:,0]
        mox = data['mox'][:,0]

    ax[0].plot(time, conc)
    ax[1].plot(time, mox)

ax[0].set_ylabel('Concentraiton [ppm]')
ax[1].set_ylabel('Sensor reading [ohms]')
ax[1].ticklabel_format(axis='y', scilimits=(0,0))
plt.xlabel('time [s]')

fig.subplots_adjust(top=0.9, left=0.1, right=0.9, bottom=0.12)  # create some space below the plots by increasing the bottom-value
#ax.flatten()[0].legend(loc='upper left', bbox_to_anchor=(1.01, 1.0)) # upper right of the plots
#ax.flatten()[-1].legend(loc='upper center', bbox_to_anchor=(0.5, -0.3), ncol=3) # at the bottom of the plots
fig.tight_layout()

if save_plot:
    plt.savefig(str(save_fname), format=filetype, bbox_inches='tight')
else:
    plt.show()
