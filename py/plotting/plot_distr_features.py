import sys
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
import pandas as pd
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('table')
parser.add_argument('--values', nargs='?', default='num_corresps',
                    choices=['num_corresps','inlier_ratio','rte','are'])
args = parser.parse_args()

data = pd.read_csv(args.table)
data['inlier_ratio'] = data['num_inliers'] / data['num_corresps']
data['are'] *= 180.0 / np.pi
data['rte'] *= 180.0 / np.pi

ax = sns.displot(data, x=args.values, hue="feature_name")
if args.values == 'rte':
    ax.set(xlabel='Angular translation error, deg')
elif args.values == 'are':
    ax.set(xlabel='Angular rotation error, deg')
plt.show()
