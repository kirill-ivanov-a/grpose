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
                    default=['min_solver','ba','reproj_2d','sampson_3d','dot_prod'])
args = parser.parse_args()

data = pd.read_csv(args.table)
total = len(data.index)

data = data[data['are'] < args.crop_val]
data = data[data['rte'] < args.crop_val]
cropped = len(data.index)
print(f'Retained {cropped} out of {total}')

data = data[data['type'].isin(args.types)]

fig, ax = plt.subplots(1, 2)
for i, error in enumerate(['are', 'rte']):
    sns.histplot(data, x=error, hue="type", ax=ax[i])
plt.show()
