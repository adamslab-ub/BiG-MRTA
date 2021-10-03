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
    nxLoc = 0
    if isSameStartPoint and currentTime == 0:
        nRobot = 1
        weight_list = []
        task_list = []
    else:
        B = nx.Graph()
        B.add_nodes_from(robotNodes, bipartite=0)
        B.add_nodes_from(taskNodes, bipartite=1)
    #print('Make Graph')
    if (robotState[iRobot,3] > 0) and (robotState[iRobot,4] > 0): # Check whether i-th robot has enough range or payload to compete for tasks 
        k = int(robotState[iRobot,0])
        i_task = 0
        for taskNode in taskNodes: #taskNodes: # it must be integer
            if i_task == 0:
                nxLoc = taskNode
                i_task += 1
            if timeDeadline[taskNode] < timeDeadline[nxLoc]:
                nxLoc = taskNode
            elif timeDeadline[taskNode] == timeDeadline[nxLoc] and \
                 distanceMatrix[k, taskNode] < distanceMatrix[k, nxLoc]:
                 nxLoc = taskNode    
    graphSize = 0
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