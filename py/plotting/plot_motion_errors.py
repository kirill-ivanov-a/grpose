import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
import pandas as pd
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('table')
parser.add_argument('--stat', nargs='?', default='median', 
                    const='median', choices=['median', 'average'])
args = parser.parse_args()

data = pd.read_csv(args.table)

method_names = np.unique(data['method_name'])
metric_names = ['ATE', 'RTE', 'ARE']
motion_lengths = np.sort(np.unique(data['motion_length']))
angles = np.sort(np.unique(data['turning_angle']))
num_experiments = max(data['experiment_num']) + 1
print(f'{method_names=}\n{motion_lengths=}\n{angles=}\n{num_experiments=}')

grouped = data.groupby(['method_name','motion_length','turning_angle'])
grouped = grouped.median() if args.stat == 'median' else grouped.mean()
del grouped['experiment_num']

metric_units = {'ATE':'m', 'RTE':'deg', 'ARE':'deg'}

fig, ax = plt.subplots(len(method_names), len(metric_names))
ax = ax.reshape(len(method_names), len(metric_names))

for method_i, method in enumerate(method_names):
    all_errors = grouped.loc[method]
    for metric_i, metric in enumerate(metric_names):
        errors = all_errors[metric].to_numpy()\
            .reshape((len(motion_lengths),len(angles)))
        
        angles_deg = angles * 180.0 / np.pi
        if metric in ['RTE', 'ARE']:
            errors = errors * 180.0 / np.pi

        cur_ax = ax[method_i,metric_i]
        errors[errors < 1e-16] = 1e-16
        errors = np.log10(errors)
        pcm = cur_ax.pcolormesh(angles_deg,motion_lengths,errors)
        fig.colorbar(pcm, ax=cur_ax)
        cur_ax.set_title(f'{method}, $\log_{{10}}({metric}, {metric_units[metric]})$')
        cur_ax.set_xlabel('Angle of rotation, deg')
        cur_ax.set_ylabel('Distance moved, m')

mng = plt.get_current_fig_manager()
mng.resize(*mng.window.maxsize())

fig.tight_layout(w_pad=-2, h_pad=-2)


plt.show()
