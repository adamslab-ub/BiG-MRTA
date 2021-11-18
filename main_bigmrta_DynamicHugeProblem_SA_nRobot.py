#!/usr/bin/env python
import os
import numpy as np
from matplotlib import pyplot as plt
from scipy.spatial import distance as dist
import scipy.io
import pickle
import networkx as nx
from time import time

from bigmrta import tic, toc, getNextTask, getParameters

# Directory that you want to save results and outputs
output_dir = "Results_bigmrta"
# If folder doesn't exist, then create it.
if not os.path.isdir(output_dir):
    os.makedirs(output_dir)
isDebug = False

maxRun = 1
nAllTasks = 1000
nInitTasks = 500

for nRobot in [5, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100]:
    for iRun in range(1,maxRun+1):
        print("--BEGIN: "+str(iRun)+"--------------------------------------------------\n")
        # Read the CaseStudy data
        data = scipy.io.loadmat('Data/FloodSim_DataSet_n'+str(nAllTasks)+'_run_'+str(iRun)+'.mat')
        print("Run using FloodSim_DataSet_n"+str(nAllTasks)+"_run_"+str(iRun)+"\n")

        taskData = data['taskData']
        taskLocation = taskData[:,:2]
        taskTime = taskData[:,-1]
        depotLocation = data['depotLocation']
        loc = np.vstack((depotLocation, taskLocation))

        ## Alg parameters
        [Q, Range, Vavg, timeMax, timeStep, decTime, letancyTime] = getParameters()
        beta = timeMax
        
        #nRobot = 10
        nTask = np.shape(taskLocation)[0]

        distanceMatrix = dist.cdist(loc, loc, metric='euclidean')
        timeMatrix = distanceMatrix/Vavg
        timeDeadline = np.hstack((np.array([0]), taskTime))

        robotNodes = []
        for i in range(nRobot):
            robotNodes = np.append(robotNodes, 'r'+str(i+1))

        taskNodes = list(np.arange(1,nInitTasks+1))
        futureTaskNodes = list(np.arange(nInitTasks+2,nTask+1))

        robotState = np.zeros((nRobot,7))
        robotState[:,3] = Q
        robotState[:,4] = Range
        # robotState - 0:2 -> Next, 3-6: Current
            #    0: index of current active task (mission),
            #    1: time when achieve its current active task (mission),
            #    2: Ditance travelled to finish the current task
            #    3: Current Remained Payload,
            #    4: Remained Range
            #    5: Overall distance travelled
            #    6: Overall computation time
        tempRobotStatus = np.zeros(nRobot)
        
        ## Simulation
        addTaskTime = 0
        addTaskTimeLimit = 12 # Each 12 mins
        nSamplePerUpdate = 10

        robotHistory = {'r1': [], 'r2': []}
        decisionHistory = [[-1,-1,-1,-1,-1],]
        for robotNode in robotNodes:
            robotHistory[robotNode]= [[0, 0, len(taskNodes), 0, 0, Q], ] # time, computing Time, Num of Task, Graph Size, Next Task, Remained Payload
        number_steps = int((timeMax+1)/timeStep)
        for t in np.linspace(0, timeMax, number_steps):
            if t % 10 == 0 or isDebug:
                print(t)
            nTaskChange = min(nSamplePerUpdate, len(futureTaskNodes))
            addTaskTime = addTaskTime + 1
            if (nTaskChange > 0) and (addTaskTime > addTaskTimeLimit):
                # Remove some current tasks, and add new tasks
                addTo = futureTaskNodes[0:addTaskTime]
                [futureTaskNodes.remove(tk) for tk in addTo]
                [taskNodes.append(tk) for tk in addTo]
                addTaskTime = 0
            nCurrentTask = len(taskNodes)
            if nCurrentTask == 0 and len(futureTaskNodes) == 0:
                break
            else:
                robotList = np.arange(nRobot)
                np.random.shuffle(robotList)
                for iRobot in robotList: # Communicate to update their status
                    # Check is near to goal (<60 sec)
                    if (robotState[iRobot,1] - t <= decTime):
                        if robotState[iRobot,0] == 0: # Returned to depot: refill payloads and reset range
                            robotState[iRobot,3] = Q
                            robotState[iRobot,4] = Range
                        else:
                            robotState[iRobot,3] = robotState[iRobot,3] - 1
                            robotState[iRobot,4] = robotState[iRobot,4] - robotState[iRobot,2]
                        robotState[iRobot,5] = robotState[iRobot,5] + robotState[iRobot,2]
                for iRobot in robotList: # Robot take decisions
                    # Check is near to goal (<60 sec)
                    if (robotState[iRobot,1] - t <= decTime):
                        tic()
                        prvLoc = int(robotState[iRobot,0])
                        if robotState[iRobot,3] == 0:
                            nxtLoc = 0
                        elif (robotState[iRobot,4]-distanceMatrix[prvLoc,0]>0):
                            nxtLoc, graphSize = getNextTask(t, iRobot, robotState, robotNodes, taskNodes, distanceMatrix, timeMatrix, timeDeadline, beta)
                        else:
                            nxtLoc = 0
                        tm = toc()
                        tempRobotStatus[iRobot] = nxtLoc
                        robotState[iRobot,6] = robotState[iRobot,6] + tm
                        if isDebug:
                            print('{} -> {}; t={}'.format(robotNodes[iRobot],nxtLoc,tm))
                        robotHistory[robotNodes[iRobot]] = np.vstack((robotHistory[robotNodes[iRobot]],
                                                                        [t, tm, nCurrentTask, graphSize, nxtLoc, robotState[iRobot,3]]))
                                                                        
                        decisionHistory = np.vstack((decisionHistory,
                                                    [t, tm, graphSize, nxtLoc, iRobot]))

                for iRobot in robotList: # Robot Communicate to inform about their decisions
                    if (robotState[iRobot,1] - t <= decTime):
                        nxtLoc = int(tempRobotStatus[iRobot])
                        if nxtLoc != 0:
                            if isDebug:
                                print(prvLoc, nxtLoc, iRobot, taskNodes)
                            taskNodes.remove(nxtLoc)
                        prvLoc = int(robotState[iRobot,0])
                        robotState[iRobot,0] = nxtLoc
                        robotState[iRobot,1] = robotState[iRobot,1] + timeMatrix[prvLoc,nxtLoc]
                        robotState[iRobot,2] = distanceMatrix[prvLoc,nxtLoc]
        for iRobot in range(nRobot): # Ensure all go back to depot
            if (robotState[iRobot,0] != 0):
                nxtLoc = 0
                prvLoc = int(robotState[iRobot,0])
                robotState[iRobot,0] = nxtLoc
                robotState[iRobot,1] = robotState[iRobot,1] + timeMatrix[prvLoc,nxtLoc]
                robotState[iRobot,2] = distanceMatrix[prvLoc,nxtLoc]
                robotState[iRobot,5] = robotState[iRobot,5] + robotState[iRobot,2]

        numTaskDone = nTask - len(taskNodes) - len(futureTaskNodes)
        totalCost = sum(robotState[:,5])
        computationTimeWhole = np.mean(robotState[:,6])
        if isDebug:
            print('Incompleted Current Tasks')
            for taskNode in taskNodes:
                print("{}: {}".format(taskNode,timeDeadline[taskNode]))
            print('Incompleted Future Tasks')
            for taskNode in futureTaskNodes:
                print("{}: {}".format(taskNode,timeDeadline[taskNode]))
        print('Results:')
        print('Task Done = {}, Total Cost = {}, Total Computing Time (average across robots): {}'.format(numTaskDone, totalCost, computationTimeWhole))

        results = {'nRobot': nRobot, 'nTask': nTask, 'iRun': iRun, 'numTaskDone': numTaskDone, 'objVal': numTaskDone,
                   'totalCost': totalCost, 'computationTime': computationTimeWhole, 'robotState': robotState, 'decisionHistory': decisionHistory, 'robotHistory': robotHistory}
        fileName = output_dir + '/DecMataResults_hungarian_DynamicCase_updated_m'+str(nRobot)+"_n"+str(nInitTasks)+'_'+str(nTask)+"_"+str(iRun)+"_b_"+str(beta)
        with open(fileName+'.pickle', 'wb') as handle:
            pickle.dump(results, handle, protocol=pickle.HIGHEST_PROTOCOL)
        print("--END: "+str(iRun)+"--------------------------------------------------\n")
