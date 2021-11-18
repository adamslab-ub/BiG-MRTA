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

enable_visualization = True
if enable_visualization:
    colorPalette = [ "#F2F3F4", "#229954", "#F1C40F", "#E74C3C",
                 "#BA4A00", "#8E44AD", "#e74c3c", "#a65d42", "#6e5200","#dcc4d2"]
    activeTaskColor = colorPalette[0]
    doneTaskColor = '#00FF00' #colorPalette[1]
    chosenTaskColor = colorPalette[2]
    #expiredTaskColor = colorPalette[3]
    addedTaskColor = 'b' #colorPalette[3]
    depotColor = '#FFFFF0' #colorPalette[4]
    robotColor = 'k'#colorPalette[5]
    taskMarker = 's'
    depotMarker = 'h'
    robotMarker = '>'
isDebug = False
maxRun = 1
modelName = "FloodSim_DataSet_n100_run_"
max_number_task = 20
payload_capacity = 4

# Directory that you want to save results and outputs
output_dir = "Results_bigmrta_demo"
# If folder doesn't exist, then create it.
if not os.path.isdir(output_dir):
    os.makedirs(output_dir)

if not os.path.isdir("Results_Illustration"):
    os.makedirs("Results_Illustration")

for nRobot in [4]:
    for iRun in range(1,maxRun+1):
        print("--BEGIN: "+str(iRun)+"--------------------------------------------------\n")
        # Read the CaseStudy data
        data = scipy.io.loadmat("Data/"+modelName+str(iRun)+".mat")
        print("Run using "+modelName+str(iRun)+"_m"+str(nRobot)+"\n")

        taskDataNs = data['taskData'][:max_number_task, :]
        taskData = taskDataNs[taskDataNs[:,3].argsort()]
        taskLocation = taskData[:,:2]
        taskTime = taskData[:,-1]
        depotLocation = data['depotLocation'] #[:,:]
        loc = np.vstack((depotLocation, taskLocation))

        ## Alg parameters
        [Q, Range, Vavg, timeMax, timeStep, decTime, letancyTime] = getParameters()
        Q = payload_capacity
        
        nTask = np.shape(taskLocation)[0]

        distanceMatrix = dist.cdist(loc, loc, metric='euclidean')
        timeMatrix = distanceMatrix/Vavg
        timeDeadline = np.hstack((np.array([0]), taskTime))

        robotNodes = []
        for i in range(nRobot):
            robotNodes = np.append(robotNodes, 'r'+str(i+1))

        taskNodes = list(np.arange(1,nTask+1))
    
        robotState = np.zeros((nRobot,7))
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
        tempRobotStatus = np.zeros(nRobot)

        robotHistory = {'r1': []}
        decisionHistory = [[-1,-1,-1,-1,-1],]
        for robotNode in robotNodes:
            robotHistory[robotNode]= [[0, 0, len(taskNodes), 0, 0, Q], ] # time, computing Time, Num of Task, Graph Size, Next Task, Remained Payload
        if enable_visualization:
            flood_img_dir = "Figures_Flood"
            img_filename = flood_img_dir + "/FloodSimulationResults_21-Sep-2018_32_SimulationStart0.png"
            img = plt.imread(img_filename)
            fig, ax = plt.subplots()
            #ax.imshow(img)
            plt.plot(loc[taskNodes,0],loc[taskNodes,1],taskMarker, color=activeTaskColor, alpha = 0.7, markersize=2)
            plt.plot(depotLocation[:,0],depotLocation[:,1],depotMarker, markersize=4, color=depotColor)
            ax.imshow(img, aspect='auto', extent=(0,30,0,20), alpha=1, origin='upper', zorder=-1)
            ax.set_xticks(np.arange(0,31,5))
            ax.set_yticks(np.arange(0,21,5))
            plt.savefig(output_dir + "/FloodSimulation_iter_"+str(iterT)+".png", format='png', dpi=300, bbox_inches='tight')
            #plt.show()
            plt.close()
        tickCounter = 0
        ## Simulation
        number_steps = int((timeMax+1)/timeStep)
        for t in np.linspace(0, timeMax, number_steps):
            if enable_visualization:
                tickCounter += 1
                fig, ax = plt.subplots()
                plt.plot(loc[taskNodes,0],loc[taskNodes,1],taskMarker, color=activeTaskColor, alpha = 0.7, markersize=2)
                plt.plot(depotLocation[:,0],depotLocation[:,1],depotMarker, color=depotColor, markersize=4)
            if t % 10 == 0 or isDebug:
                print(t)
            if len(taskNodes) > 0:
                for iRobot in range(nRobot): # Communicate to update their status
                    if isDebug:
                        print(iRobot)
                    # Check is near to goal (<60 sec)
                    if (robotState[iRobot,1] - t <= decTime):
                        if robotState[iRobot,0] == 0: # Returned to depot: refill payloads and reset range
                            robotState[iRobot,3] = Q
                            robotState[iRobot,4] = Range
                        else:
                            robotState[iRobot,3] = robotState[iRobot,3] - 1
                            robotState[iRobot,4] = robotState[iRobot,4] - robotState[iRobot,2]
                        robotState[iRobot,5] = robotState[iRobot,5] + robotState[iRobot,2]
                        
                for iRobot in range(nRobot): # Robot take decisions
                    # Check is near to goal (<60 sec)
                    if (robotState[iRobot,1] - t <= decTime):
                        nCurrentTask = len(taskNodes)
                        tic()
                        prvLoc = int(robotState[iRobot,0])
                        if robotState[iRobot,3] > 0 and (robotState[iRobot,4]-distanceMatrix[prvLoc,0]>0):
                            nxtLoc, graphSize = getNextTask(t, iRobot, robotState, robotNodes, taskNodes,
                                                            distanceMatrix, timeMatrix, timeDeadline,
                                                            isVisualize=True)
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
                        
                for iRobot in range(nRobot): # Robot Communicate to inform about their decisions
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
            else:
                break
        for iRobot in range(nRobot): # Ensure all go back to depot
            if (robotState[iRobot,0] != 0):
                nxtLoc = 0
                prvLoc = int(robotState[iRobot,0])
                robotState[iRobot,0] = nxtLoc
                robotState[iRobot,1] = robotState[iRobot,1] + timeMatrix[prvLoc,nxtLoc]
                robotState[iRobot,2] = distanceMatrix[prvLoc,nxtLoc]            
                robotState[iRobot,5] = robotState[iRobot,5] + robotState[iRobot,2]
        
        numTaskDone = nTask - len(taskNodes)
        totalCost = sum(robotState[:,5])
        computationTimeWhole = np.mean(robotState[:,6])

        print('Results:')
        print('Task Done = {}, Total Cost = {}, Total Computing Time (average across robots): {}'.format(numTaskDone, totalCost, computationTimeWhole))

        results = {'nRobot': nRobot, 'nTask': nTask, 'iRun': iRun, 'numTaskDone': numTaskDone, 'objVal': numTaskDone, 'decisionHistory': decisionHistory, 'totalCost': totalCost, 'computationTime': computationTimeWhole, 'robotState': robotState, 'robotHistory': robotHistory}
        fileName = output_dir + '/DecMataResults_hungarian_m'+str(nRobot)+"_n"+str(nTask)+"_"+str(iRun)
        with open(fileName+'.pickle', 'wb') as handle:
            pickle.dump(results, handle, protocol=pickle.HIGHEST_PROTOCOL)
        print("--END: "+str(iRun)+"--------------------------------------------------\n")

if enable_visualization:
    # plt.scatter(depotLocation[0, 0], depotLocation[0, 1], marker='o')
    # plt.scatter(taskDataNs[:, 0], taskDataNs[:, 1], marker='s', c="k")  # , c=taskDataNs[:, 2])
    colors = ["r", "b", "g", "c"]
    for iRobot in range(nRobot):
        iRobotHistory = robotHistory[robotNodes[iRobot]]
        prvLoc = depotLocation[0]
        for iRobotSnap in iRobotHistory:
            taskId = int(iRobotSnap[4]) - 1
            if taskId == -1:
                nxtLoc = depotLocation[0]
            else:
                nxtLoc = taskLocation[taskId, :]
            waypoints = np.stack((prvLoc, nxtLoc), axis=0)
            plt.plot(waypoints[:, 0], waypoints[:, 1], colors[iRobot], marker="s")
            if taskId != -1:
                plt.annotate(str(taskId+1), nxtLoc+0.1)
            prvLoc = nxtLoc
    plt.savefig(output_dir + "/bigmrta-demo-path.png", format="png", dpi=300, bbox_inches="tight")
    plt.show()