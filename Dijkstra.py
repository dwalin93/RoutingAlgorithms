import os
import networkx as nx
import math 

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

def main():
    #Read in edges
    pathToEdges = os.path.join(cwd,'data','Muenster_edges.shp')
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
    print(list(network.edges(data=True))[0][2]['coord'])
    #Calc Distance
    distance = calcDistance(network,edg)
    print(distance)   
    #Apply dist on edges
    nx.set_edge_attributes(network, name='dist', values=distance)
    #Dijkstra
    dijkstra = nx.dijkstra_path(network,4,8312,weight='dist'[0])
    print(dijkstra)
    shortestPath = drawShortestPath(network,dijkstra,posNodes)
    #Get adjacent edges
    
    
    
    
    
    







