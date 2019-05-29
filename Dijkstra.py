import os
import networkx as nx

cwd = os.getcwd()
print(cwd)

def readData(path):
    return nx.read_shp(path)

def createNetwork(edges):
    posNodes = getPositionOfNodesFromEdges(edges)
    network = nx.Graph()
    network.add_nodes_from(posNodes.keys())
    posEdges = getPositionOfEdges(edges)
    # Search for incomplete tuple
    edg = [tuple(key for key,value in posNodes.items() if value in line) for line in posEdges]
    nx.draw_networkx_nodes(network,posNodes,node_size=10, node_color='r')
    network.add_edges_from(edg)
    nx.draw_networkx_edges(network,posNodes)
    return 
 

    
def getPositionOfNodesFromEdges(nodes):
    pos = {index:value for index,value in enumerate(nodes.nodes())}
    return pos

def getPositionOfEdges(edges):
    lines = [set(x) for x in edges.edges()]
    return lines

def createDictFromEdgesCoords(edges):
    start = []
    end = []
    coordsDict = {}
    for x in edges:
        print(x[0])
        start.append(x[0])
        end.append(x[1])  
        
    
    coordsDict = dict()
    print(start[0])
    
    return coordsDict


    
        
    

def main():
    #Read in edges
    pathToEdges = os.path.join(cwd,'data','Muenster_edges.shp')
    edges = nx.read_shp(pathToEdges)
    pos = getPositionOfNodesFromEdges(edges)
    network = createNetwork(edges)
    #Add Weight to edges
    network = nx.Graph()
    network.add_nodes_from(pos.keys())
    posEdges = getPositionOfEdges(edges)
    print(posEdges)
    edg = [tuple(key for key,value in pos.items() if value in line) for line in posEdges]
    print(list(edg)[0])
    network.add_edges_from(edg)
    #Get coordinates of edges
    
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
    print(coordsDict)
    
    
    nx.set_edge_attributes(network, name='coord', values=list(coordsDict))
    test = (list(network.edges(data=True))[1][2])
    #Calculate dist
    for x in edges.edges():
        print(x[1])
        
        
    t = createDictFromEdgesCoords(edges.edges)    
    print(network.edges(data=True))
    #Apply dist on edges
    #Try with algorithm
    







