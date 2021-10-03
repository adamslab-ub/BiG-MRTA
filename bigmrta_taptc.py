# Author: Payam Ghassemi, payamgha@buffalo.edu
# Copyright 2020 Payam Ghassemi

import numpy as np

from matplotlib import pyplot as plt

from scipy.spatial import distance as dist
from scipy.optimize import linear_sum_assignment

import scipy.io

import pickle


import collections
import itertools

from networkx.algorithms.bipartite.matrix import biadjacency_matrix
from networkx.algorithms.bipartite import sets as bipartite_sets
import networkx as nx

#import cynetworkx as nx
from time import time

tics = []

def tic():
    tics.append(time())

def toc():
    if len(tics)==0:
        return None
    else:
        return time()-tics.pop()

def getNextTask(currentTime, iRobot, robotState, robotNodes, taskNodes,\
                distanceMatrix, timeMatrix, timeDeadline, TotalTime=300, MaxCost=140, theta=[1],\
                incentiveModelType=6, isFullTaskState=True, isVisualize=False, matching_algorithm="hungarian", pos=None):#, alg='MFMC'):
    isSameStartPoint = False 
    # robotState
    #    0: index of current active task (mission),
    #    1: time when achieve its current active task (mission), 
    #    2: Ditance travelled to finish the current task
    #    3: Current Remained Payload,
    #    4: Remained Range
    #    5: Overall distance travelled
    #    6: Overall flight time
    iTask = robotState[iRobot, 0]
    iTime = robotState[iRobot, 1]
    iKey = robotNodes[iRobot]
    nRobot = np.shape(robotState)[0]
    # Using max flow min cost @note There are some serious issues with MaxFlowMinCost: 
    # 1) Issue with non-integer weights
    # 2) Non-unique answers
    #D = nx.DiGraph()
    #D.add_nodes_from(robotNodes)
    #D.add_nodes_from(taskNodes)
    #D.add_nodes_from(['s','t'])
    if isSameStartPoint and currentTime == 0:
        nRobot = 1
        weight_list = []
        task_list = []
    else:
        B = nx.Graph()
        B.add_nodes_from(robotNodes, bipartite=0)
        B.add_nodes_from(taskNodes, bipartite=1)
    #print('Make Graph')
    for i in range(nRobot):
        if (robotState[i,3] > 0) and (robotState[i,4] > 0): # Check whether i-th robot has enough range or payload to compete for tasks 
            #D.add_edge('s', robotNodes[i], weight=0, capacity=1)
            # Just keep top 4*m tasks (ordered by dt = tdi - tri)
            k = int(robotState[i,0])
            if not isFullTaskState:
                print("Limited Case")
                delayTime = timeDeadline[taskNodes] - (currentTime + timeMatrix["robot-"+str(i+1)][k,taskNodes])
                idxTasks = np.argsort(delayTime)
                #print(isinstance(delayTime,np.ndarray))
                #print(np.shape(delayTime))
                #print(isinstance(idxTasks,np.ndarray))
                #print(isinstance(taskNodes,list))
                #allowedTasks_i = taskNodes[:4*nRobot]
                nAllowedTasks = min(len(idxTasks),4*nRobot)
                allowedTasks_i = np.array(taskNodes)[idxTasks[:nAllowedTasks]]
            else:
                allowedTasks_i = taskNodes.copy()
            for taskNode in allowedTasks_i: #taskNodes: # it must be integer
                finishTime = robotState[i,1] + timeMatrix["robot-"+str(i+1)][k,taskNode]
                finishRange = robotState[i,4] - distanceMatrix[k,taskNode]# - distanceMatrix[taskNode, 0]
                #travelledDistance = robotState[i,5] + distanceMatrix[k,taskNode]
                if (finishTime <= timeDeadline[taskNode]):# and (finishRange >= 0):#  and ( (TotalTime*0.5 + currentTime) >= timeDeadline[taskNode] ):
                    #D.add_edge(robotNodes[i], taskNode, weight=finishTime, capacity=1) #WeightType1
                    #B.add_edge(robotNodes[i], taskNode, weight=-finishTime) #WeightType1
                    #B.add_edge(robotNodes[i], taskNode, weight=100*np.exp(-finishTime/TotalTime)) #WeightType2
                    #B.add_edge(robotNodes[i], taskNode, weight=beta*finishRange*np.exp(-finishTime/TotalTime)) #WeightType2
                    #weight = finishRange**beta+finishRange*np.exp(-finishTime/TotalTime)
                    #normalTime = (TotalTime-finishTime)/TotalTime
                    normalTime = (-finishTime)/TotalTime
                    normalRange = finishRange / MaxCost
                    if incentiveModelType == 1:
                        weight = theta[0] + theta[1] * normalRange + theta[2] *normalRange**2 +\
                                theta[3] * normalTime + theta[4] * normalTime ** 2
                    elif incentiveModelType == 2:
                        weight = theta[0] + theta[1] * normalRange + theta[2] *normalRange**2 +\
                                theta[3] * np.exp(normalTime) + theta[4] * np.exp(-normalTime ** 2)
                    elif incentiveModelType == 3:
                        weight = theta[0] + theta[1] * normalRange +\
                                 theta[2] * np.exp(-normalTime)
                    elif incentiveModelType == 4:
                        weight = theta[0] * normalRange * np.exp(-theta[1] * normalTime)
                    elif incentiveModelType == 5:
                        weight = theta * normalRange
                    elif incentiveModelType == 6: # Original that used in MRS2019 and RAS2020
                        weight = finishRange*np.exp(-finishTime/TotalTime)
                    elif incentiveModelType == 7:
                        marginTime = timeDeadline[taskNode] - finishTime
                        weight = finishRange*marginTime
                    else:
                        weight = finishRange*np.exp(-finishTime/TotalTime)
                    if isSameStartPoint and currentTime == 0:
                        weight_list.append(weight)
                        task_list.append(taskNode)
                    else:
                        B.add_edge(robotNodes[i], taskNode, weight=weight) #WeightType2
                    #B.add_edge(robotNodes[i], taskNode, weight=100*np.exp(-(timeDeadline[taskNode]-finishTime))) #WeightType3


    #for taskNode in taskNodes:
    #    D.add_edge(taskNode, 't', weight=0, capacity=1)

    #tic()
    #print(B.edges.data())
    #print('Run MaxFlowMinCost')
    #if D.out_degree(robotNodes[iRobot]) > 0:
    if isSameStartPoint and currentTime == 0:
        sorted_weight_task = sorted(zip(weight_list, task_list))
        _, sorted_task_list = zip(*sorted_weight_task)
        nxLoc = sorted_task_list[iRobot]
    else:
        if B.degree(robotNodes[iRobot]) > 0:
            if matching_algorithm == "hungarian":
                #cost = -np.triu(nx.to_numpy_matrix(B))
                #row_ind, col_ind = linear_sum_assignment(cost)
                #nxLoc = taskNodes[col_ind[iRobot] - nRobot]
                #print(B.nodes(), np.shape(cost), cost, nxLoc)
                sol = maximum_weight_full_matching(B, robotNodes)
                nxLoc = 0
                for x in sol:
                    if x == iKey:
                        nxLoc = sol[x]
                        break
            else:
                #sol = nx.max_flow_min_cost(D, 's', 't', capacity='capacity', weight='weight')
                sol = nx.max_weight_matching(B, maxcardinality=True)
                #print(sol)
                nxLoc = 0
                for x, y in sol:
                    if x == iKey:
                        nxLoc = y
                        break
                    elif y == iKey:
                        nxLoc = x
                        break
                    #print(sol)
                #for key in sol[iKey].keys():
                #    if sol[iKey][key] == 1:
                #        nxLoc = key
                #        break
                #print(len(sol),sol)
        else:
            nxLoc = 0
            #t = toc()
    #print(nxLoc)
    if isVisualize:
        top = nx.bipartite.sets(B)[0]
        if pos is None:
            pos = nx.bipartite_layout(B, top)
        edges, weights = zip(*nx.get_edge_attributes(B,'weight').items())
        nx.draw(B, pos, node_color='#DC143C', edgelist=edges, edge_color=weights, width=4.0, edge_cmap=plt.cm.Blues)
        nx.draw_networkx_nodes(B, pos, nodelist=list(robotNodes), node_size=800, node_color="#E5E4E2",\
                                edgecolors="#254117", linewidths=3, node_shape="o")
        print(robotNodes[iRobot])
        nx.draw_networkx_nodes(B, pos, nodelist=[robotNodes[iRobot]], node_size=800, node_color="#FDD017",\
                                edgecolors="#254117", linewidths=3, node_shape="o")
        nx.draw_networkx_nodes(B, pos, nodelist=list(taskNodes), node_size=800, node_color="#CCFFFF",\
                                edgecolors="#254117", linewidths=3, node_shape="s")
                                
        nx.draw_networkx_labels(B, pos, font_size=18)
        fileName = 'Results_Illustration/Illustration_DecMataResults_t'+str(currentTime)+"_r"+str(iRobot)+"_nx"+str(nxLoc)
        plt.savefig(fileName+".pdf", dpi=300, format="pdf")
        plt.show()

    if currentTime == 0:
        graphSize = 0
    else:
        graphSize = B.size(weight=None)
    if isVisualize:
        return nxLoc, graphSize, pos
    else:
        return nxLoc, graphSize

def getParameters(letancyTime=0, Q=5, Range=140, Vavg=40/60, timeMax=5*60, timeStep=1, decTime=0):
    ## Alg parameters
    #Q = 5 # Robot capacity (max number of payloads)
    #Range = 100 # flight range in km (Max. 148, we used prev. 140)
    #Vavg = 40/60 # 40 km/h = 2/3 km/min
    #timeMax = 5*60 # 5hr End of simulation in min
    #timeStep = 1 # min
    #decTime
    #letancyTime
    return [Q, Range, Vavg, timeMax, timeStep, decTime, letancyTime]

def maximum_weight_full_matching(G, top_nodes=None, weight='weight'):
    r"""Returns the maximum weight full matching of the bipartite graph `G`.

    Let :math:`G = ((U, V), E)` be a complete weighted bipartite graph with
    real weights :math:`w : E \to \mathbb{R}`. This function then produces
    a maximum matching :math:`M \subseteq E` which, since the graph is
    assumed to be complete, has cardinality
   
    .. math::
       \lvert M \rvert = \min(\lvert U \rvert, \lvert V \rvert),

    and which minimizes the sum of the weights of the edges included in the
    matching, :math:`\sum_{e \in M} w(e)`.
    
    When :math:`\lvert U \rvert = \lvert V \rvert`, this is commonly
    referred to as a perfect matching; here, since we allow
    :math:`\lvert U \rvert` and :math:`\lvert V \rvert` to differ, we
    follow Karp [1]_ and refer to the matching as *full*.

    Parameters
    ----------
    G : NetworkX graph

      Undirected bipartite graph

    top_nodes : container

      Container with all nodes in one bipartite node set. If not supplied
      it will be computed.

    weight : string, optional (default='weight')

       The edge data key used to provide each value in the matrix.

    Returns
    -------
    matches : dictionary

      The matching is returned as a dictionary, `matches`, such that
      ``matches[v] == w`` if node `v` is matched to node `w`. Unmatched
      nodes do not occur as a key in matches.

    Raises
    ------
    ValueError : Exception

      Raised if the input bipartite graph is not complete.

    ImportError : Exception

      Raised if SciPy is not available.

    Notes
    -----
    The problem of determining a minimum weight full matching is also known as
    the rectangular linear assignment problem. This implementation defers the
    calculation of the assignment to SciPy.

    References
    ----------
    .. [1] Richard Manning Karp:
       An algorithm to Solve the m x n Assignment Problem in Expected Time
       O(mn log n).
       Networks, 10(2):143–152, 1980.

    """
    try:
        import scipy.optimize
    except ImportError:
        raise ImportError('minimum_weight_full_matching requires SciPy: ' +
                          'https://scipy.org/')
    left = set(top_nodes)
    right = set(G) - left
    # Ensure that the graph is complete. This is currently a requirement in
    # the underlying  optimization algorithm from SciPy, but the constraint
    # will be removed in SciPy 1.4.0, at which point it can also be removed
    # here.
    #for (u, v) in itertools.product(left, right):
        # As the graph is undirected, make sure to check for edges in
        # both directions
    #    if (u, v) not in G.edges() and (v, u) not in G.edges():
    #        raise ValueError('The bipartite graph must be complete.')
    U = list(left)
    V = list(right)
    weights = biadjacency_matrix(G, row_order=U,
                                 column_order=V, weight=weight).toarray()
    left_matches = scipy.optimize.linear_sum_assignment(-weights)
    d = {U[u]: V[v] for u, v in zip(*left_matches)}
    # d will contain the matching from edges in left to right; we need to
    # add the ones from right to left as well.
    d.update({v: u for u, v in d.items()})
    return d