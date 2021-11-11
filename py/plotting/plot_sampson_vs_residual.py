import sys
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
import pandas as pd
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('table')
parser.add_argument('--absolute', action='store_true')
args = parser.parse_args()

data = pd.read_csv(args.table)
sampson_res = data[data['type'] == 'Sampson'].drop('type', axis=1).to_numpy()
ba_res = data[data['type'] == 'BA'].drop('type', axis=1).to_numpy()

assert ba_res.shape == sampson_res.shape

ba_norm = np.linalg.norm(ba_res, axis=1)
diff = np.linalg.norm(ba_res - sampson_res, axis=1)

x = np.log10(ba_norm)
y = np.log10(diff) if args.absolute else np.log10(diff / ba_norm)
inds = [i for i in range(len(y)) if np.isfinite(y[i]) and np.isfinite(x[i])]
sns.histplot(x=x[inds], y=y[inds])
plt.xlabel('$\log_{10}($Norm of the Bundle Adjustment residual (pix) $)$')
if args.absolute:
    plt.ylabel('$\log_{10}($Absolute error with Sampson estimate (pix)$)$')
else:
    plt.ylabel('$\log_{10}($Relative error with Sampson estimate$)$')

plt.show()