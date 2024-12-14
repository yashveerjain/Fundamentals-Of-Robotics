
import os
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def get_data(filepath):
    if not os.path.isfile(filepath):
        print(f"{filepath} doesn't exist")
        sys.exit(1)

    try:
        data = pd.read_csv(filepath,header=None)
    except Exception as e:
        print(f"Error while reading file : {e}")
        sys.exit(1)
    print()
    return data


args = sys.argv    


if (len(args)<3):
    print("Input files are not provided to plot")
    sys.exit(1)

gt_filepath = args[1]
gt_data = get_data(gt_filepath)

est_filepath = args[2]
est_data = get_data(est_filepath)

est_X = est_data.iloc[:,0].values
est_Y = est_data.iloc[:,1].values

gt_X = gt_data.iloc[:,0].values
gt_Y = gt_data.iloc[:,1].values

plt.plot(est_Y,est_X,"r-",label="EST")
plt.plot(gt_Y,gt_X,"b--",label="GT")
plt.legend()
# plt.savefig("plot.png")
plt.show()