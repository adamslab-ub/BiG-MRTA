# Author: Payam Ghassemi, payamgha@buffalo.edu
# Sep 8, 2018
# Copyright 2018 Payam Ghassemi

import numpy as np

from matplotlib import pyplot as plt

from scipy.spatial import distance as dist

import scipy.io

import pickle

import networkx as nx

from time import time

tics = []

def tic():
    tics.append(time())

def toc():
    if len(tics)==0:
        return None
    else:
        return time()-tics.pop()

def getNextTask(currentTime, iRobot, robotState, robotNodes, taskNodes, distanceMatrix, timeMatrix, timeDeadline, TotalTime=300):#, alg='MFMC'):
    # robotState
    #    0: index of current active task (mission),
    #    1: time when achieve its current active task (mission), 
    #    2: Ditance travelled to finish the current task
    #    3: Current Remained Payload,
    #    4: Remained Range
    #    5: Overall distance travelled
    #    6: Overall flight time
    availableTasks = []
    i = iRobot
    if (robotState[i,3] > 0) and (robotState[i,4] > 0): # Check whether i-th robot has enough range or payload to compete for tasks 
        for taskNode in taskNodes: # it must be integer
            taskNode = int(taskNode)
            iRobtoState = int(robotState[i,0])
            finishTime = robotState[i,1] + timeMatrix[iRobtoState,taskNode]
            finishRange = robotState[i,4] - distanceMatrix[iRobtoState,taskNode] - distanceMatrix[taskNode, 0]
            if (finishTime <= timeDeadline[taskNode]) and (finishRange >= 0):
                availableTasks = np.append(availableTasks, taskNode)
    nTask = len(availableTasks)
    if  nTask > 0:
        idxLoc = np.random.randint(nTask)
        nxLoc = availableTasks[idxLoc]

    else:
        nxLoc = 0
        
    return nxLoc, 0
    
def getParameters(letancyTime=0, Q=5, Range=100, Vavg=40/60, timeMax=5*60, timeStep=1, decTime=0):
    ## Alg parameters
    #Q = 5 # Robot capacity (max number of payloads)
    #Range = 100 # flight range in km (Max. 148, we used prev. 140)
    #Vavg = 40/60 # 40 km/h = 2/3 km/min
    #timeMax = 5*60 # 5hr End of simulation in min
    #timeStep = 1 # min
    #decTime
    #letancyTime
    return [Q, Range, Vavg, timeMax, timeStep, decTime, letancyTime]
