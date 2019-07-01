import networkx as nx
import math 
import collections


#Preproccesing
def getEdges(posEd,posNo):
    nodeTupleArray = []
    for key,value in posNo.items():
        for line in posEd:
            if value == line[0][0]:
                first = getKeysByValue(posNo,value)
                second = getKeysByValue(posNo,line[0][1])
                result = [first,second]
                tupleRes = tuple(result) 
                if(tupleRes not in nodeTupleArray):
                    nodeTupleArray.append(tupleRes)  
            elif value == line[0][1]:
                second = getKeysByValue(posNo,value)
                first = getKeysByValue(posNo,line[0][0])
                result = [first,second]
                tupleRes = tuple(result)
                if(tupleRes not in nodeTupleArray):
                    nodeTupleArray.append(tupleRes) 
    return nodeTupleArray         

def getKeysByValue(dictOfElements, valueToFind):
    listOfKeys = list()
    listOfItems = dictOfElements.items()
    for item  in listOfItems:
        if item[1] == valueToFind:
            listOfKeys.append(item[0])        
    return  listOfKeys[0]

def readData(path):
    return nx.read_shp(path)

def createNetwork(posNodes,edg):
    network = nx.Graph()
    network.add_nodes_from(posNodes.keys())
    network.add_edges_from(edg)
    return network

def setPosNodes(network,posNodes):
     #Set Pos of edges and nodes
    nx.set_node_attributes(network, name='loc',values=posNodes)
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
   posDict = collections.OrderedDict()
   for value in enumerate(edges.edges(data=True)):
        coord1 = value[1][0]
        coord2 = value[1][1]
        u = value[1][2]['u']
        v = value[1][2]['v']
        posDict.update({u:coord1})
        posDict.update({v:coord2})
   return posDict

def getPositionOfEdges(edges):
    lines = []
    for x in edges.edges():
        lines.append([x])
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
        print(list(edg)[idx])
        distances.update({list(edg)[idx]:float(distance)})
    #Try with algorithm
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

def getAngle(point1, point2):
    x1,y1 = point1
    x2,y2 = point2
    dx = x2 - x1
    dy = y2 - y1
    radians = math.atan2(dy,dx)
    rad2deg = math.degrees(radians)
    return rad2deg

#Input is tuple (start,end)
def getCoords(network,start,end):
    lineData = network.get_edge_data(start,end)
    startEast,startNorth = (lineData['coord']['startEast'],lineData['coord']['startNorth']) 
    endEast, endNorth = (lineData['coord']['endEast'],lineData['coord']['endNorth']) 
    coords = [(startEast,startNorth),(endEast,endNorth)]
    return coords

# Change To 180 - angle
def anglesOfLines(network,edge1Start,edge1End,edge2End):
    edge1StartPoint, edge1EndPoint = getCoords(network,edge1Start,edge1End)
    edge2StartPoint, edge2EndPoint = getCoords(network,edge1End,edge2End)
    angle1 = getAngle(edge1StartPoint, edge1EndPoint)
    angle2 = getAngle(edge2StartPoint, edge2EndPoint)
    angle = abs(angle1-angle2)
    turn = abs(180-angle)
    return turn

def getMinAngle(angleArray):
    minValue = 0
    maxValue = 180
    minAngle = min(angleArray)
    #scale angle from 0 to 1
    scaled = (minAngle - minValue) / (maxValue-minValue)
    return scaled

def scaleCosts(costs,minValue,maxValue):
    return (costs - minValue) / (maxValue-minValue)
    
        
