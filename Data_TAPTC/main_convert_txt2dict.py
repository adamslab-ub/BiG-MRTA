# Author: Payam Ghassemi, payamgha@buffalo.edu
# Dec 02, 2020
# Copyright 2020 Payam Ghassemi

import numpy as np
import pandas as pd

from matplotlib import pyplot as plt

from scipy.spatial import distance as dist

import scipy.io

import pickle

## TAPTC Dataset
group_list = [1,2]
instance_list = [0, 1, 2]
ratio_deadline_list = [1, 2, 3, 4]
robotSize_list = [2, 3, 5, 7]

for G in group_list:
    for D in ratio_deadline_list:
        for I in instance_list:
            for R in robotSize_list:
                agent_name = "a"+str(R)+"i0"+str(I)
                data_name = "r"+str(G)+str(D)+agent_name
                dir_name = "group"+str(G)
                file_name = dir_name+"/"+data_name+".txt"
                tasks = pd.read_csv(file_name, sep=" ", header=None, skiprows=1)
                tasks.columns = ["id", "x", "y", "w", "T"]
                file_name = "agent/"+agent_name+".txt"
                robots = pd.read_csv(file_name, sep=" ", header=None, skiprows=1)
                robots.columns = ["id", "x", "y", "c"]
                print(robots)
                exit()                