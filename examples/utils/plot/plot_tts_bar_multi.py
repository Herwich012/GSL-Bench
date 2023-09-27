"""
| plot_tts_bar_multi.py
| Author: Hajo Erwich (hajo_erwich@live.nl)
| License: BSD-3-Clause. Copyright (c) 2023, Hajo Erwich. All rights reserved.
| Description: Create bar plot with time to source 
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

### Save params ###
save_plot = False
filetype = 'png'
save_fname = f"{HOME_DIR}/0THESIS/figures/avg_tts.{filetype}"

### Data selection ###
algorithms = ("E. Coli", "Dung Beetle", "Random Walker")
environments = ("1", "2", "3", "4", "5", "6")
exp_id_starts = [[ 10,  19,  28], # env 001
                 [ 46,  55,  37], # env 002
                 [ 82, 100,  91], # ...
                 [109, 127, 118],
                 [136, 154, 145],
                 [163, 181, 172]]
amount = 9


def get_times2source(start:int) -> list:
    experiments = [str(i + start).zfill(3) for i in range(amount)]
    times2source = []

    for _,exp_id in enumerate(experiments):
        # stat_dir = f"{HOME_DIR}/0THESIS/experiments/{exp_id}"
        # files = glob.glob(f"{stat_dir}/*")
        files = glob.glob(f"{RESULTS_DIR}/{exp_id}/*")
        files_sorted = [files[i] for i in np.argsort(files)]
        
        for _,file in enumerate(files_sorted):
            with np.load(file) as data:
                # if data['run_success'][0][0] == True:
                #     times2source.append(round(data['time'][-1],2)) # append latest time
                times2source.append(round(data['time'][-1],2)) # append latest time
    
    return times2source

ttss_dict = {}
time_threshold = 298.0
for i,algorithm in enumerate(algorithms): # per algorithm
    ttss = []
    for j,_ in enumerate(environments): # per environment
        tts = get_times2source(exp_id_starts[j][i])
        #print(f'{tts}\n')
        tts_filtered = [i for i in tts if i < time_threshold]
        
        if tts_filtered:
            tts_avg = round(np.mean(tts_filtered),1)
        else:
            tts_avg = 0
        
        ttss.append(tts_avg)

    ttss_dict[algorithm] = ttss

print(ttss_dict)

#-------------------------
# Plot Bargraph
#-------------------------
fig, ax = plt.subplots()
fig.set_figheight(4)
fig.set_figwidth(5)

x = np.arange(len(environments))  # the label locations
width = 0.25  # the width of the bars
multiplier = 0
hatching = ('..','///','O')

for (attribute,measurement),hatchtype in zip(ttss_dict.items(),hatching):
    offset = width * multiplier
    rects = ax.bar(x + offset, measurement, width,
                   edgecolor='black', alpha=0.6,
                   label=attribute, hatch=hatchtype)
    ax.bar_label(rects, padding=3, rotation=90)
    multiplier += 1

# Add some text for labels, title and custom x-axis tick labels, etc.
ax.set_ylabel('Average time to source (s)')
ax.set_xlabel('Environment')
ax.set_title('Average Times To Source Per Environment')
ax.set_xticks(x + width, environments)
ax.legend(loc='upper center', ncols=3)
#ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15), ncol=3)
#ax.legend(loc='upper right')
#ax.set_ylim(0, 185)
ax.set_ylim(0,205)
fig.tight_layout()

if save_plot:
    plt.savefig(str(save_fname), format=filetype, bbox_inches='tight')
else: 
    plt.show()
