import os
import networkx as nx
import math 
import collections
import preprocessing as pre
import shapefile
#import matplotlib.pyplot as plt

cwd = os.getcwd()
print('cwd: ',cwd)

#######################################################
#################### Input data #######################
#######################################################


# Select Origin-Destination pair:
ODpair = 3

edgePath = 'Aassee_Edges_area_with_DS_new_' + str(ODpair) + '.shp'
pathToEdges = os.path.join(cwd,'data_Maicol',edgePath)

ODnodeNumbers = {
        1: [395,7775],
        2: [7775,2196],
        3: [2196,6767]
        }
startNode = ODnodeNumbers[ODpair][0] # as in the input data
endNode = ODnodeNumbers[ODpair][1] # as in the input data

# Assign weight of scores:
# 1: TS = 0.5 Local Score + 0.5 Distant Score
# 2: TS = (DS = w1 and LS = (1-w1). Linear change of DS and LS towards the destination till DS = (1-w1) and LS = w1)
weightScenario = 2
w1 = 0.6

# Assign scaling factor for the Total Score importance in regards to heurestic. 
# 1: the same importance
# 2: twice as important...
TotalScoreToHeuresticScaleFactor = 0.5


#######################################################
#################### Functions ########################
#######################################################

def readData(path):
    return nx.read_shp(path)

def createNetwork(posNodes,edg):
    network = nx.Graph()
    network.add_nodes_from(posNodes.keys())
    network.add_edges_from(edg)
    return network
 
def drawNetwork(network,posNodes):
    nx.draw_networkx_nodes(network,posNodes,node_size=10, node_color='r')
    nx.draw_networkx_edges(network,posNodes)
    return 
 
def drawShortestPath(network,shortestPath,posNodes):
    node_colors = ["blue" if n in shortestPath else "white" for n in network.nodes()]
    edge_colors = ["blue" if n in shortestPath else "white" for n in network.edges()]
    nx.draw_networkx_nodes(network,posNodes,node_size=1, node_color=node_colors)
    nx.draw_networkx_edges(network,posNodes,edge_color=edge_colors)
    return

def getPositionOfNodesFromEdges(edges):
   posDict = {}
   for value in enumerate(edges.edges(data=True)):
        coord1 = value[1][0]
        coord2 = value[1][1]
        u = value[1][2]['u']
        v = value[1][2]['v']
        posDict.update({u:coord1})  
        posDict.update({v:coord2})
   return posDict

def getPositionOfEdges(edges):
    lines = [set(x) for x in edges.edges()]
    return lines

def createDictFromEdgesCoords(edges,edg):
    startEast = []
    startNorth = []
    endEast = []
    endNorth = []
    coordsDict = {}
    
    for x in edges.edges():
        startEast.append(x[0][0])
        startNorth.append(x[0][1])
        endEast.append(x[1][0])
        endNorth.append(x[1][1])
            
    for idx, x in enumerate(startEast):
        coordsDict.update({list(edg)[idx]:{'startEast': startEast[idx], 'startNorth': startNorth[idx],
                         'endEast': endEast[idx], 'endNorth': endNorth[idx]}})
    return coordsDict

def calcDistance(network,edg):
    distances = {}
    for idx, x in enumerate(network.edges(data=True)):
        coords = list(network.edges(data=True))[idx][2]['coord']
        distance = round(math.sqrt(((coords['endNorth']-coords['startNorth'])**2) +
                                   ((coords['endEast']-coords['startEast'])**2)),2)
        distances.update({list(edg)[idx]:float(distance)})
    return distances 

def createDictFromLocalScore(edges,edg):
    LS = []
    LSDict = {}
           
    for x in list(edges.edges(data=True)):
        LS.append(x[2]['LS'])
            
    for idx, x in enumerate(LS):
        LSDict.update({list(edg)[idx]:LS[idx]})
    return LSDict

def createDictFromDistantScore(edges,edg):
    DS = []
    DSDict = {}
           
    for x in list(edges.edges(data=True)):
        DS.append(x[2]['DS'])
            
    for idx, x in enumerate(DS):
        DSDict.update({list(edg)[idx]:DS[idx]})
    return DSDict

def calcAdjacency(network):
    adjacency = [(n, nbrdict) for n, nbrdict in network.adjacency()] 
    return adjacency

def getAdjacentAndNeigborNodes(adjacentNodes):
    nodeAdjacencyNeighborsDict = collections.defaultdict(list) 
    for idx, x in enumerate(adjacentNodes):
        node = adjacentNodes[idx][0]
        keys = adjacentNodes[idx][1].keys()
        nodeAdjacencyNeighborsDict.update({node:{'adj':list(keys)}})
        
    keys = nodeAdjacencyNeighborsDict.keys()
    for idy, y in enumerate(nodeAdjacencyNeighborsDict):
        listOfValues = list(nodeAdjacencyNeighborsDict.values())[idy]['adj']
        listOfKeys = getKeysByValues(nodeAdjacencyNeighborsDict,listOfValues)
        listOfKeysWithoutDups = removeDuplicatesFromList(listOfKeys)
        nodeAdjacencyNeighborsDict[list(keys)[idy]].update({'neighbours': listOfKeysWithoutDups})
    return nodeAdjacencyNeighborsDict

def removeDuplicatesFromList(x):
  return list(dict.fromkeys(x))

def getKeysByValues(dictOfElements, listOfValues):
    listOfKeys = list()
    listOfItems = dictOfElements.items()
    for item  in listOfItems:
        idx = 0
        while idx < len(item[1]['adj']):
            if item[1]['adj'][idx] in listOfValues:
                listOfKeys.append(item[0]) 
                idx+=1
            else:
                idx+=1       
    return  listOfKeys 

#######################################################
####### Insert and adjust A Star source code ##########
#######################################################
    
    # -*- coding: utf-8 -*-
#    Copyright (C) 2004-2019 by
#    Aric Hagberg <hagberg@lanl.gov>
#    Dan Schult <dschult@colgate.edu>
#    Pieter Swart <swart@lanl.gov>
#    All rights reserved.
#    BSD license.
#
# Authors: Salim Fadhley <salimfadhley@gmail.com>
#          Matteo Dell'Amico <matteodellamico@gmail.com>
"""Shortest paths and path lengths using the A* ("A star") algorithm.
"""
from heapq import heappush, heappop
from itertools import count

def astar_path_Kasia(G, source, target, scenario, coordDict): # weight=GlobalScore and add later LocalScore
    """Returns a list of nodes in a shortest path between source and target
    using the A* ("A-star") algorithm.
    
    Heurestic function changed to include the dictionairy with the coordinates
    
    Weights used in the function include distant and local scores of the edges
    
    Heurestic values are ~100 times higher than edge scores. Theis needs to be included.

    """
    def heuristic_Kasia(theNode, theTarget, coordDict):
        nodeX = coordDict[theNode][0]
        nodeY = coordDict[theNode][1]
        targetX = coordDict[theTarget][0]
        targetY = coordDict[theTarget][1]
        distanceToTarget = math.sqrt(math.pow((targetX-nodeX),2)+math.pow((targetY-nodeY),2))
        return distanceToTarget

    if source not in G or target not in G:
        msg = 'Either source {} or target {} is not in G'
        raise nx.NodeNotFound(msg.format(source, target))

    '''if heuristic_Kasia is None:
        # The default heuristic is h=0 - same as Dijkstra's algorithm
        def heuristic_Kasia(u, v):
            return 0'''

    push = heappush
    pop = heappop
    
    # Weight of the Global Score and Local Score will depend on the distance of each node to the target
    
    # Prepare data for calculating the weights
    distanceToTarget = heuristic_Kasia(source, target, coordDict)
    print('Max heuristic value (eucidlian distance to destination from the starting node): ', distanceToTarget)
    print('Max Total Score of the edge: 1')
    # Assign scenario
    # 1: TS = 0.5 Local Score + 0.5 Distant Score'
    # 2: TS = (DS = 0.8 and LS = 0.8. Linear change of DS and LS towards the destination till DS = 0.2 and LS = 0.8))
    if scenario == 1:
        weightDS = 0.5
        weightLS = 0.5
    elif scenario == 2:
        weightDS = w1
        weightLS = (1-w1)
        
    # Apply the factor to scale the weights (0-1) to match heurestic (0-~1000 m)
    heuristicCostRatio = 1000
    
    # The queue stores priority, node, cost to reach, and parent.
    # Uses Python heapq to keep in priority order.
    # Add a counter to the queue to prevent the underlying heap from
    # attempting to compare the nodes themselves. The hash breaks ties in the
    # priority and is guaranteed unique for all nodes in the graph.
    c = count()
    queue = [(0, next(c), source, 0, None)]

    # Maps enqueued nodes to distance of discovered paths and the
    # computed heuristics to target. We avoid computing the heuristics
    # more than once and inserting the node into the queue too many times.
    enqueued = {}
    # Maps explored nodes to parent closest to the source.
    explored = {}

    while queue:
        # Pop the smallest item from queue.
        _, __, curnode, dist, parent = pop(queue)
                
        if curnode == target:
            path = [curnode]
            node = parent
            while node is not None:
                path.append(node)
                node = explored[node]
            path.reverse()
            return path

        if curnode in explored:
            continue

        explored[curnode] = parent

        for neighbor, w in G[curnode].items():
            
            if neighbor in explored:
                continue
                        
            # If the weight are linear, calculate the weights, depending on their eucidlian distance to the destination
            if scenario==2:
                weightDS = (2*w1-1)/distanceToTarget * heuristic_Kasia(neighbor, target, coordDict) + (1-w1)
                weightLS = (-1)*(2*w1-1)/distanceToTarget * heuristic_Kasia(neighbor, target, coordDict) + w1
                        
            ncost = dist + TotalScoreToHeuresticScaleFactor * heuristicCostRatio * (weightDS * w.get('DS', 1) + weightLS * w.get('LS', 1)) ## dist = sum of weights
            if neighbor in enqueued:
                qcost, h = enqueued[neighbor]
                # if qcost <= ncost, a less costly pathl from the
                # neighbor to the source was already determined.
                # Therefore, we won't attempt to push this neighbor
                # to the queue
                if qcost <= ncost:
                    continue
            else:
                h = heuristic_Kasia(neighbor, target, coordDict)
               
            enqueued[neighbor] = ncost, h
            push(queue, (ncost + h, next(c), neighbor, ncost, curnode))
            
    raise nx.NetworkXNoPath("Node %s not reachable from %s" % (target, source))

def exportResultsToPoints(astar_result, keyNodes):  # PROBLEMS WITH THE ORDER OF NODES ==> WRONG COOORDINATES

        result_coords = []
        #result_nodes = []
        
        #keyNodes = astar
        
        print(keyNodes)
        
        # Create dict nodeID: [LS,DS,dist,coords]
        
        result_LS = []
        result_LS.append(0)
        result_DS = []
        result_DS.append(0)
        result_dist = []
        result_dist.append(0)
        
        listWithResultEdges = list(astar_result.edges(data=True))
        print(listWithResultEdges)
        
        # Save results in lists
        for i in listWithResultEdges:
            result_coords.append([float(i[2]['coord']['startEast']),float(i[2]['coord']['startNorth'])])
            #result_nodes.append(i[0])
            
            result_LS.append(i[2]['LS'])
            result_DS.append(i[2]['DS'])
            result_dist.append(i[2]['dist'])
            
        lastNode = listWithResultEdges[len(listWithResultEdges)-1]
        result_coords.append([float(lastNode[2]['coord']['endEast']),float(lastNode[2]['coord']['endNorth'])])
        #result_nodes.append(lastNode[0])
        
        resultName = 'OriginDestination'+str(ODpair)
        result = shapefile.Writer(resultName, shapeType=1)
        #result.field('nodeID', 'N', decimal=0)
        result.field('LocalScore', 'N', decimal=30)
        result.field('DistantScore', 'N', decimal=30)
        result.field('distance', 'N', decimal=30)
        
        for i in range(0,len(result_coords)):
            result.point(result_coords[i][0],(result_coords[i][1]))
        for i in range(0,len(result_coords)):
            #result.record(result_nodes[i],result_LS[i],result_DS[i], result_dist[i])
            result.record(result_LS[i],result_DS[i], result_dist[i])
        
        result.close()


#######################################################
####################### Main ##########################
#######################################################

def main():
    ##### Read in edges #####
    print('input data: ',pathToEdges)
    
    ##### Pre-process #####
    # Build network
    edges = nx.read_shp(pathToEdges)
    posNodes = pre.getPositionOfNodesFromEdges(edges)
    posEdges = pre.getPositionOfEdges(edges)
    # Create a list of tuples containing the start and end node of an edge
    edg = pre.getEdges(posEdges,posNodes)
    
    # Create the network based on the position of nodes and the edges defined by start and end nodes IDs
    network = pre.createNetwork(posNodes,edg)
    drawNetwork(network,posNodes)
    
    ##### Add Weight to edges #####
    # 1. Get coordinates of edges and apply to the network
    coordinates = pre.createDictFromEdgesCoords(edges,edg)
    nx.set_edge_attributes(network, name='coord', values=coordinates)
    
    # 2. Get Local Score of edges and apply to the network
    LocalScore = createDictFromLocalScore(edges,edg)
    nx.set_edge_attributes(network, name='LS', values=LocalScore)
    
    # 3. Get Distant Score of edges and apply to the network
    DistantScore = createDictFromDistantScore(edges,edg)
    nx.set_edge_attributes(network, name='DS', values=DistantScore)
    
    # 4. Calculate the distance and apply to the network
    distance = pre.calcDistance(network,edg)  
    nx.set_edge_attributes(network, name='dist', values=distance) 
    
    ##### Run A Star alghoritm #####
    print('Start node: ', startNode)
    print('End node: ', endNode)
    #print('Network cost: ', networkWeight)
    print('Weight scenario: ', weightScenario)
    print('TS - Total Score, DS - Distant Score, LS - Local Score')
    print('   1: TS = 0.5 LS + 0.5 DS')
    print('   2: TS = (DS = 0.8 and LS = 0.2. Linear change of DS and LS towards the destination till DS = 0.2 and LS = 0.8))')
    print('Scale factor of the Total Score in reagrds to heurestic: ',TotalScoreToHeuresticScaleFactor)
    astar = astar_path_Kasia(network,startNode,endNode,weightScenario, posNodes)
    print('Results of A Star algorithm:')
    print(astar)
    
    shortestPathAStar = drawShortestPath(network,astar,posNodes)
    shortestPathAStar
    # Save results
    # Subset a graph
    astar_result = network.subgraph(astar)
    # Save as points
    exportResultsToPoints(astar_result, astar)
    
          
main()             
                
        
    
   
    
    







