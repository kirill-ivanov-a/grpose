import sys
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
import pandas as pd
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('table')
parser.add_argument('--success_prob', nargs='?', type=float, default=0.99)
parser.add_argument('--max_iter', nargs='?', type=int, default=20001)
args = parser.parse_args()

def predict_num_iter(success_prob, outlier_frac, sample_size):
    return np.log(1 - success_prob) \
                / np.log(1 - (1 - outlier_frac) ** sample_size)

data = pd.read_csv(args.table)

data['are'] = data['are'].apply(lambda x: x * 180.0 / np.pi)
data['rte'] = data['rte'].apply(lambda x: x * 180.0 / np.pi)

print(f'Original {data.shape[0]=}')
# Filter out maximal numbers of iterations
data = data.loc[data["num_iter"] < args.max_iter]
print(f'After filtering {data.shape[0]=}')

predicted_iter = predict_num_iter(args.success_prob, 
                                  data['outlier_frac'], data['sample_size'])
data['Real / predicted number of iterations'] = data['num_iter'] / predicted_iter

sns.lineplot(x="outlier_frac", y='Real / predicted number of iterations',
             hue="min_solver", data=data)

plt.show()
