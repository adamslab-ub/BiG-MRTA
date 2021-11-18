#!/usr/bin/env python
import os
import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
from scipy.spatial import distance as dist
import scipy.io
import pickle
import csv
import networkx as nx
from time import time

from bigmrta_taptc import tic, toc, getNextTask, getParameters

# Directory that you want to save results and outputs
output_dir = "Results_taptc"
# If folder doesn't exist, then create it.
if not os.path.isdir(output_dir):
    os.makedirs(output_dir)

isDemo = False
isDebug = False
enableTimeDisplay = False
enableFullTour = False # No need to return to starting point

iRun = 0
if isDemo:
    maxRun = 1
    isDebug = True
else:
    maxRun = 96

timeScaleUp = 1

results_score = {}
results_computingtime_all = {}

if isDemo:
    group_list = [0]
    instance_list = [0]
    ratio_deadline_list = [1]
    robotSize_list = [2]
else:
    ## TAPTC Dataset
    group_list = [1,2]
    instance_list = [0, 1, 2]
    ratio_deadline_list = [1, 2, 3, 4]
    robotSize_list = [2, 3, 5, 7]
root_dir = "Data_TAPTC/"

for G in group_list:
    for D in ratio_deadline_list:
        for R in robotSize_list:
            for I in instance_list:
                agent_name = "a"+str(R)+"i0"+str(I)
                data_name = "r"+str(G)+str(D)+agent_name
                if isDemo:
                    instance_name = "demo_r"+str(G)+str(D)+"a"+str(R)+"0"+str(I)
                    file_name = "demo_case.txt"
                else:
                    instance_name = "r"+str(G)+str(D)+"a"+str(R)+"0"+str(I)
                    dir_name = "group"+str(G)
                    file_name = dir_name+"/"+data_name+".txt"
                tasks = pd.read_csv(root_dir+file_name, sep=" ", header=None, skiprows=1)
                tasks.columns = ["id", "x", "y", "w", "T"]
                file_name = "agent/"+agent_name+".txt"
                robots = pd.read_csv(root_dir+file_name, sep=" ", header=None, skiprows=1)
                robots.columns = ["id", "x", "y", "c"]
                velVal = 1
                nRobot = R

                if enableTimeDisplay:
                    print("--BEGIN: "+data_name+"--------------------------------------------------")
                # Read the CaseStudy data

                ## Alg parameters
                Vavg = velVal/timeScaleUp
                Range = 1e6
                Q = 100
                nTask = len(tasks)
                decTime = 0
                letancyTime = 0
                MaxCost = 1000
                timeStep = 1
                
                depotLocation = [0,0]
                taskLocation = tasks.iloc[:,1:3]
                taskWorkload = tasks.iloc[:,3]*timeScaleUp
                eTaskWorkload = np.concatenate(([0], taskWorkload)) # Insert depot/starting point to taskWorkload
                taskTime = tasks.iloc[:,-1]*timeScaleUp # Tasks' time deadline
                
                timeMax = int(1.1*max(taskTime)) * timeScaleUp
                TotalTime = timeMax

                timeMatrix = {}
                for iRobot in range(nRobot):
                    if isDemo:
                        startLocation = depotLocation
                    else:
                        startLocation = robots.iloc[iRobot,1:3]
                    taskRate = robots.iloc[iRobot,-1] # It's called capacity, too. It means how much fast it can perform the task.
                    loc = np.vstack((startLocation, taskLocation))
                    distanceMatrix = dist.cdist(loc, loc, metric='euclidean')
                    taskServiceTime = np.tile(np.ceil(eTaskWorkload/taskRate), (nTask+1, 1))
                    np.fill_diagonal(taskServiceTime, 0)
                    timeMatrix["robot-"+str(iRobot+1)] = distanceMatrix/Vavg +  taskServiceTime
                timeDeadline = np.hstack((np.array([0]), taskTime))

                robotNodes = []
                for i in range(nRobot):
                    robotNodes = np.append(robotNodes, 'r'+str(i+1))

                if isDemo:
                    print(timeDeadline)
                    print(timeMatrix)
                    print(taskTime)
                    print(taskLocation)
                    print(taskWorkload)
                taskNodes = list(np.arange(1,nTask+1))
            
                robotState = np.zeros((nRobot,9))
                robotState[:,3] = Q
                robotState[:,4] = Range
                coefRTask = 4
                # robotState - 0:2 -> Next, 3-6: Current
                    #    0: index of current active task (mission),
                    #    1: time when achieve its current active task (mission), 
                    #    2: Ditance travelled to finish the current task
                    #    3: Current Remained Payload,
                    #    4: Remained Range
                    #    5: Overall distance travelled
                    #    6: Overall computation time
                    #    7: Numer of tasks
                    #    8: Trip counter 
                tempRobotStatus = np.zeros(nRobot)

                robotHistory = {'r1': []}
                decisionHistory = [[-1,-1,-1,-1,-1],]
                for robotNode in robotNodes:
                    robotHistory[robotNode]= [[0, 0, len(taskNodes), 0, 0, Q], ] # time, computing Time, Num of Task, Graph Size, Next (Active) Task, Remained Payload
                
                tripLengthHistory = {}
                for robotNode in robotNodes:
                    tripLengthHistory[robotNode] = []

                ## Simulation
                stepSize = int((timeMax+1)/timeStep)
                for t in np.linspace(0,timeMax,stepSize):
                    if enableTimeDisplay:
                        print(t)
                    if len(taskNodes) > 0:
                        for iRobot in range(nRobot): # Communicate to update their status
                            # Check is near to goal (<60 sec)
                            if (robotState[iRobot,1] - t <= decTime):
                                if robotState[iRobot,0] == 0: # Returned to depot: refill payloads and reset range
                                    robotState[iRobot,3] = Q
                                    robotState[iRobot,4] = Range
                                    if t > 0:
                                        robotState[iRobot,8] += 1
                                    tripLengthHistory['r'+str(iRobot+1)].append(0)
                                    Warning("Returned to depot!")
                                else:  # Update state as finished task
                                    robotState[iRobot,3] = robotState[iRobot,3] - 1
                                    robotState[iRobot,4] = robotState[iRobot,4] - robotState[iRobot,2]
                                    robotState[iRobot,7] += 1
                                    tripLengthHistory['r'+str(iRobot+1)][-1] += robotState[iRobot,2]
                                robotState[iRobot,5] = robotState[iRobot,5] + robotState[iRobot,2]
                                
                        for iRobot in range(nRobot): # Robot take decisions
                            # Check is near to goal (<60 sec)
                            if (robotState[iRobot,1] - t <= decTime):
                                nCurrentTask = len(taskNodes)
                                tic()
                                prvLoc = int(robotState[iRobot,0])
                                nxtLoc, graphSize = getNextTask(t, iRobot, robotState, robotNodes, taskNodes, distanceMatrix, timeMatrix,\
                                                            timeDeadline, TotalTime, MaxCost)
                                tm = toc()
                                tempRobotStatus[iRobot] = nxtLoc
                                robotState[iRobot,6] = robotState[iRobot,6] + tm
                                if isDebug:
                                    print('{} -> {}; @t={} with computing time of {} secs'.format(robotNodes[iRobot],nxtLoc,t,tm))
                                robotHistory[robotNodes[iRobot]] = np.vstack((robotHistory[robotNodes[iRobot]],\
                                                                                [t, tm, nCurrentTask, graphSize, nxtLoc, robotState[iRobot,3]]))
                                                                                
                                decisionHistory = np.vstack((decisionHistory,
                                                            [t, tm, graphSize, nxtLoc, iRobot]))
                                
                        for iRobot in range(nRobot): # Robot Communicate to inform about their decisions
                            if (robotState[iRobot,1] - t <= decTime):
                                nxtLoc = int(tempRobotStatus[iRobot])
                                if nxtLoc != 0:
                                    if isDebug:
                                        print(prvLoc, nxtLoc, iRobot, taskNodes)
                                    taskNodes.remove(nxtLoc)
                                prvLoc = int(robotState[iRobot,0])
                                robotState[iRobot,0] = nxtLoc
                                robotState[iRobot,1] = robotState[iRobot,1] + timeMatrix["robot-"+str(iRobot+1)][prvLoc,nxtLoc]
                                robotState[iRobot,2] = distanceMatrix[prvLoc,nxtLoc]
                    else:
                        break
                if enableFullTour:
                    for iRobot in range(nRobot): # Ensure all go back to depot
                        if (robotState[iRobot,0] != 0):
                            nxtLoc = 0
                            prvLoc = int(robotState[iRobot,0])
                            robotState[iRobot,0] = nxtLoc
                            robotState[iRobot,1] = robotState[iRobot,1] + timeMatrix["robot-"+str(iRobot+1)][prvLoc,nxtLoc]
                            robotState[iRobot,2] = distanceMatrix[prvLoc,nxtLoc]            
                            robotState[iRobot,5] = robotState[iRobot,5] + robotState[iRobot,2]
                            robotState[iRobot,8] += 1
                
                numTaskDone = nTask - len(taskNodes)
                totalCost = sum(robotState[:,5])
                computationTimeWhole = np.mean(robotState[:,6])

                print('Results:')
                print('Task Done = {}, Total Cost = {}, Total Computing Time (average across robots): {}'.format(numTaskDone, totalCost, computationTimeWhole))
                results_score[instance_name] = numTaskDone
                results_computingtime_all[instance_name] = np.sum(robotState[:,6])
                results = {'nRobot': nRobot, 'nTask': nTask, 'range': Range, 'payload': Q, 'instance': I, 'numTaskDone': numTaskDone, 'objVal': numTaskDone, 'decisionHistory': decisionHistory, 'totalCost': totalCost,\
                            'computationTime': computationTimeWhole, 'robotState': robotState, 'robotHistory': robotHistory, 'tripLengthHistory': tripLengthHistory}
                fileName = output_dir + '/DecMataResults_TAPTC_hungarian_'+data_name+"_Range"+str(Range)+"_Q"+str(Q)
                with open(fileName+'.pickle', 'wb') as handle:
                    pickle.dump(results, handle, protocol=pickle.HIGHEST_PROTOCOL)
                if enableTimeDisplay:
                    print("--END: "+data_name+" --------------------------------------------------\n")
                
                iRun += 1
                if iRun >= maxRun:
                    with open(output_dir+'/DecMataResults_TAPTC_hungarian_score.txt', 'w') as handle:
                        for key, val in results_score.items():
                            handle.write("%s %s\n" % (key, val))
                    with open(output_dir + '/DecMataResults_TAPTC_hungarian_time.txt', 'w') as handle:
                        for key, val in results_computingtime_all.items():
                            handle.write("%s %s\n" % (key, val))
                    quit()
with open(output_dir + '/DecMataResults_TAPTC_hungarian_score.txt', 'w') as handle:
    for key, val in results_score.items():
        handle.write("%s %s\n" % (key, val))
with open(output_dir + '/DecMataResults_TAPTC_hungarian_time.txt', 'w') as handle:
    for key, val in results_computingtime_all.items():
        handle.write("%s %s\n" % (key, val))
