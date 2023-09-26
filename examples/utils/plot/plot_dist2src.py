"""
| plot_dist2src.py
| Author: Hajo Erwich (hajo_erwich@live.nl)
| License: BSD-3-Clause. Copyright (c) 2023, Hajo Erwich. All rights reserved.
| Description: Create distance to source plots over time
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

### Experiment id & Save params ###
exp_id = 108
save_plot = False
filetype = 'png'
save_fname = f"{HOME_DIR}/0THESIS/figures/dist2src_{str(exp_id).zfill(3)}.{filetype}"

### Source location ###
source_xy = np.array([5,1])

stat_dir = f"{HOME_DIR}/0THESIS/experiments/{exp_id}"
files = glob.glob(f"{stat_dir}/*")
files_sorted = [files[i] for i in np.argsort(files)]

#-------------------------
# Distance to source plots
#-------------------------
fig, ax = plt.subplots()
fig.set_figheight(4)
fig.set_figwidth(5)

for _,file in enumerate(files_sorted):
    dists = []
    times = []
    with np.load(file) as data:
        time = data['time']
        locs = data['p']

        xdata = locs[:,0]
        ydata = locs[:,1]
    
    for x,y in zip(xdata,ydata):
        dist = np.linalg.norm((np.array([x,y])-source_xy))
        dists.append(dist)
    
    times.append(time[-1])
    ax.plot(time, dists)

avg_time = round(np.mean(times),1)

ax.set_xlim(0,None)
ax.set_ylabel('Distance To Source (m)')
ax.set_ylim(1, None)
ax.annotate(f'avg: {avg_time}s', xy=(avg_time + 5, 12.8)) # bottom
plt.axvline(avg_time, linestyle='--')
plt.xlabel('Time (s)')
plt.grid()
plt.title('Dung Beetle Algotirhm - Distance to Source')
# fig.subplots_adjust(top=0.9, left=0.1, right=0.9, bottom=0.12)  # create some space below the plots by increasing the bottom-value
#ax.flatten()[0].legend(loc='upper left', bbox_to_anchor=(1.01, 1.0)) # upper right of the plots
#ax.flatten()[-1].legend(loc='upper center', bbox_to_anchor=(0.5, -0.3), ncol=3) # at the bottom of the plots
fig.tight_layout()

if save_plot:
    plt.savefig(str(save_fname), format=filetype, bbox_inches='tight')
else:
    plt.show()
