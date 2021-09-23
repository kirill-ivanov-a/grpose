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
method_names = np.unique(data['method_name'])

sns.displot(data, x="time", hue="method_name")
plt.show()
