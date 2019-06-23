import os
import networkx as nx
import math 
import collections

cwd = os.getcwd()
print(cwd)

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

def getPositionOfNodesFromEdges(nodes):
    pos = {index:value for index,value in enumerate(nodes.nodes())}
    return pos

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
        distances.update({list(edg)[idx]:{float(distance)}})
    #Try with algorithm
    print(distances)
    print(coords['endNorth'])
    print(coords['endEast'])
    print(coords['startNorth'])
    print(coords['startEast'])
    return distances 

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

#from networkx.utils import not_implemented_for



def astar_path_Kasia(G, source, target, weight, coordDict):
    """Returns a list of nodes in a shortest path between source and target
    using the A* ("A-star") algorithm.
    
    Heurestic function changed to include the dictionairy with the coordinates

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
            ncost = dist + w.get(weight, 1)
            if neighbor in enqueued:
                qcost, h = enqueued[neighbor]
                # if qcost <= ncost, a less costly path from the
                # neighbor to the source was already determined.
                # Therefore, we won't attempt to push this neighbor
                # to the queue
                if qcost <= ncost:
                    continue
            else:
                h = heuristic_Kasia(neighbor, target, coordDict) # niech się♣ oblicza dla kązdego neighbour
            enqueued[neighbor] = ncost, h
            push(queue, (ncost + h, next(c), neighbor, ncost, curnode))

    raise nx.NetworkXNoPath("Node %s not reachable from %s" % (target, source))

#######################################################
########### End of the A Star source code #############
#######################################################

def main():
    #Read in edges
    pathToEdges = os.path.join(cwd,'data','Muenster_edges.shp')
    print(pathToEdges)
    edges = nx.read_shp(pathToEdges)
    posNodes = getPositionOfNodesFromEdges(edges)
    posEdges = getPositionOfEdges(edges)
    edg = [tuple(key for key,value in posNodes.items() if value in line) for line in posEdges]
    network = createNetwork(posNodes,edg)
    drawNetwork(network,posNodes)
    #Add Weight to edges
    #Get coordinates of edges
    coordinates = createDictFromEdgesCoords(edges,edg)
    nx.set_edge_attributes(network, name='coord', values=coordinates)
    #Calc Distance
    distance = calcDistance(network,edg)   
    #Apply dist on edges
    nx.set_edge_attributes(network, name='dist', values=distance) 
    #Dijkstra
    #dijkstra = nx.dijkstra_path(network,1,70,weight='dist'[0])
    #print('Prints results of Dijkstra algorithm')
    #print(dijkstra)
    #A Star
    astar = astar_path_Kasia(network,1,700,'dist'[0], posNodes) ## weight has to be chnged to the landmark score
    print('Prints results of A Star algorithm')
    print(astar)
    shortestPathAStar = drawShortestPath(network,astar,posNodes)
    shortestPathAStar
          
main()             
                
        
    
   
    
    







