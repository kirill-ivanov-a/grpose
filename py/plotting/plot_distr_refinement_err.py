import sys
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
import pandas as pd
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('table')
args = parser.parse_args()

data = pd.read_csv(args.table)

fig, ax = plt.subplots(1, 2)
for i, error in enumerate(['are', 'rte']):
    sns.displot(data, x=error, hue="type", ax=ax[i])
plt.show()
