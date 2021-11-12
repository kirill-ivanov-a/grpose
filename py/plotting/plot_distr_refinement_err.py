import sys
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
import pandas as pd
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('table')
parser.add_argument('--crop_val', type=float, nargs='?', default=100)
parser.add_argument('--types', type=str, nargs='+', 
                    default=['min_solver','reproj_2d','sampson_3d','sampson_2d',
                             'dot_prod','sampson_pinhole','algebraic',
                             'symmetric_epipolar_pinhole',
                             'symmetric_epipolar_cosine'])
parser.add_argument('--plot_iter', action='store_true')
parser.add_argument('--ecdf', action='store_true')
args = parser.parse_args()

data = pd.read_csv(args.table)
total = len(data.index)

data = data[data['are'] < args.crop_val]
data = data[data['rte'] < args.crop_val]
cropped = len(data.index)
print(f'Retained {cropped} out of {total}')

data = data[data['type'].isin(args.types)]

medians = data.groupby('type')['rte','are'].median()
print(f'medians:\n{medians}')
averages = data.groupby('type')['rte','are'].mean()
print(f'averages:\n{averages}')
stds = data.groupby('type')['rte','are'].std()
print(f'stds:\n{stds}')

if args.plot_iter:
    if args.ecdf:
        sns.ecdfplot(data, x='num_iter', hue='type')
    else:
        sns.histplot(data, x='num_iter', hue='type')
else:
    fig, ax = plt.subplots(1, 2)
    for i, error in enumerate(['are', 'rte']):
        if args.ecdf:
            sns.ecdfplot(data, x=error, hue='type', ax=ax[i])
        else:
            sns.histplot(data, x=error, hue="type", ax=ax[i])
plt.show()
