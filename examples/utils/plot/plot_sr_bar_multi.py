"""
| plot_sr_bar_multi.py
| Author: Hajo Erwich (hajo_erwich@live.nl)
| License: BSD-3-Clause. Copyright (c) 2023, Hajo Erwich. All rights reserved.
| Description: Create bar plot with overall success rates
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

### Save params ###
save_plot = False
filetype = 'pdf'
save_fname = f"{PLOT_DIR}/success_overall.{filetype}"

### Data selection ###
algorithms = ("E. Coli", "Dung Beetle", "Random Walker")
environments = ("1", "2", "3", "4", "5", "6")
exp_id_starts = [[ 10,  19,  28], # env 001
                 [ 46,  55,  37], # env 002
                 [ 82, 100,  91], # ...
                 [109, 127, 118],
                 [136, 154, 145],
                 [199, 181, 172]]
runs_per_exp = 9 # amount of runs in one experiment

def get_sr(start:int) -> list:
    experiments = [str(i + start).zfill(3) for i in range(runs_per_exp)]
    success = []

    for _,exp_id in enumerate(experiments):
        # stat_dir = f"{RESULTS_DIR}/{exp_id}"
        # files = glob.glob(f"{stat_dir}/*")
        files = glob.glob(f"{RESULTS_DIR}/{exp_id}/*")
        files_sorted = [files[i] for i in np.argsort(files)]
        
        for k,file in enumerate(files_sorted):
            with np.load(file) as data:
                success.append(data['run_success'][0][0])

    success_rate = round(success.count(True) / len(success), 3)
    
    return success_rate


success_dict = {}
for i,algorithm in enumerate(algorithms): # per algorithm
    successes = []
    for j,_ in enumerate(environments): # per environment
        sr = get_sr(exp_id_starts[j][i])
        successes.append(sr*100) # in percent
    
    success_dict[algorithm] = successes

#print(success_dict)

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

for (attribute,measurement),hatchtype in zip(success_dict.items(),hatching):
    offset = width * multiplier
    rects = ax.bar(x + offset, measurement, width,
                   edgecolor='black', alpha=0.6,
                   label=attribute, hatch=hatchtype)
    ax.bar_label(rects, padding=3, rotation=90)
    multiplier += 1

# Add some text for labels, title and custom x-axis tick labels, etc.
ax.set_title('Success Rate Per Environment')
ax.set_xlabel('Environment')
ax.set_ylabel('Success Rate (%)')
ax.set_xticks(x + width, environments)
#ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.1), ncol=3)
#ax.legend(loc='upper center', ncols=3)
ax.legend(loc='upper right')
#plt.axhline(100, linestyle='--')
ax.set_ylim(0, 110)
fig.tight_layout()

if save_plot:
    plt.savefig(str(save_fname), format=filetype, bbox_inches='tight')
else:
    plt.show()
