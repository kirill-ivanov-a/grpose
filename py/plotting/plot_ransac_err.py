import sys
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
import pandas as pd
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('table')
parser.add_argument('--max_iter', nargs='?', type=int, default=20001)
parser.add_argument('--metric_names', nargs='+', default=['ate', 'rte', 'are'])
args = parser.parse_args()

data = pd.read_csv(args.table)

data['are'] = data['are'].apply(lambda x: x * 180.0 / np.pi)
data['rte'] = data['rte'].apply(lambda x: x * 180.0 / np.pi)

print(f'Original {data.shape[0]=}')
# Filter out maximal numbers of iterations
data = data.loc[data["num_iter"] < args.max_iter]
print(f'After filtering {data.shape[0]=}')

fig, ax = plt.subplots(len(args.metric_names), 1, sharex=True)
if len(args.metric_names) == 1:
    ax = [ax]

titles = {'ate' : 'Absolute translational error (m)',
          'rte' : 'Angular translational error (deg)',
          'are' : 'Absolute rotational error (deg)'}

for i, metric in enumerate(args.metric_names):
    ax[i].set_title(titles[metric])
    sns.lineplot(x="outlier_frac",y=metric,hue="min_solver",data=data, ax=ax[i])

plt.show()
