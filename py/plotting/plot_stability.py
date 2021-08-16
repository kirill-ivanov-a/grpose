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
scene_widths = np.sort(np.unique(data['scene_width']))
angle_stds = np.sort(np.unique(data['angle_std']))
angle_stds = angle_stds * 180.0 / np.pi
num_experiments = max(data['experiment_num']) + 1
print(f'{method_names=}\n{scene_widths=}\n{angle_stds=}\n{num_experiments=}')

grouped = data.groupby(['method_name','scene_width','angle_std'])
grouped = grouped.median() if args.stat == 'median' else grouped.mean()
del grouped['experiment_num']

metric_units = {'ATE':'m', 'RTE':'deg', 'ARE':'deg'}

fig, ax = plt.subplots(len(method_names), len(metric_names))

for method_i, method in enumerate(method_names):
    all_errors = grouped.loc[method]
    for metric_i, metric in enumerate(metric_names):
        errors = all_errors[metric].to_numpy()\
            .reshape((len(scene_widths),len(angle_stds)))
        
        if metric in ['RTE', 'ARE']:
            errors = errors * 180.0 / np.pi

        cur_ax = ax[method_i,metric_i]
        errors[errors < 1e-16] = 1e-16
        errors = np.log10(errors)
        pcm = cur_ax.pcolormesh(angle_stds, scene_widths, errors)
        fig.colorbar(pcm, ax=cur_ax)
        cur_ax.set_title(f'{method}, $\log_{{10}}({metric}, {metric_units[metric]})$')
        cur_ax.set_xlabel('Standard deviation of direction angle, deg')
        cur_ax.set_ylabel('Scene width, m')

mng = plt.get_current_fig_manager()
mng.resize(*mng.window.maxsize())

fig.tight_layout(w_pad=-8, h_pad=-2)


plt.show()
