import preprocessing as pre
import networkx as nx
import os
import glob


cwd = os.getcwd()

def getAdj(neighbours,u):
    return neighbours.get(u)['adj']

def getNeighbours(neighbours,u):
    return neighbours.get(u)['neighbours']

def preprocessRoutes(routeFile):
    edges = nx.read_shp(routeFile)
    posNodes = pre.getPositionOfNodesFromEdges(edges)
    posEdges = pre.getPositionOfEdges(edges)
    edg = pre.getEdges(posEdges,posNodes)   
    network = pre.createNetwork(posNodes,edg)
    return network

def getAngles(network,adjacentAndNeighborNodesDict):
    angles = []
    for edge in network.edges(data = True):
        v = edge[0]
        u = edge[1]
        neighbours = getNeighbours(adjacentAndNeighborNodesDict,v)
        adj = getAdj(adjacentAndNeighborNodesDict,u)
        for neighbour in neighbours:
            if(v != neighbour and neighbour in adj):
                angle = pre.anglesOfLines(network,v,u,neighbour)
                angles.append(angle)        
    return angles

def getWaitingTime(network):
    times = []
    for edge in network.edges(data = True):
        times.append(edge[2]['traffic'])
    return times    

def getEdgesWithoutCoords(network):
    removeEdges = []
    for edge in network.edges(data =True):
        if edge[2] == {}:
            removeEdges.append((edge[0],edge[1]))
    return removeEdges        
        

def analyseODFromDijkstra():
    #arrays for calculations
    angularChange = []
    waitingTime = []
    #Read in edges
    path = os.path.join(cwd,'Results','Dijkstra','*.shp')
    files = glob.glob(path)
    files
    for file in files:
        #edges = nx.read_shp(file)
        edges = nx.read_shp(file)
        #print(list(edges.edges(data=True))[12])
        posNodes = pre.getPositionOfNodesFromEdgesForAnalysis(edges)
        posNodes.keys()
        #New Edge position 
        posEdges = pre.getPositionOfEdgesForAnalysis(edges)
        #New edge calculation
        edg = pre.getEdgesForAnalysis(posEdges,posNodes)   

        network = pre.createNetwork(posNodes,edg)
        
        coordinates = pre.createDictFromEdgesCoords(edges,edg)
        nx.set_edge_attributes(network, name='coord', values=coordinates)
        
        removeEdges = getEdgesWithoutCoords(network)
        network.remove_edges_from(removeEdges)
        
        distance = pre.calcDistance(network,edg)
        nx.set_edge_attributes(network, name='dist', values=distance)
        
        trafficLights = pre.readcsv(os.path.join(cwd,'traffic_lights.csv'))
        trafficDict = pre.createDictFromTrafficLights(trafficLights)
        TrafficDuration = pre.createCompleteTrafficDict(network,trafficDict)
        nx.set_edge_attributes(network, name='traffic', values=TrafficDuration)
        
        adjacency = pre.calcAdjacency(network)
        adjacentAndNeighborNodesDict = pre.getAdjacentAndNeigborNodes(adjacency)
        
       #Calc TotalAngularchange and traffic light waiting time 
        anglesForEachOD = getAngles(network,adjacentAndNeighborNodesDict)
        angularChange.append(anglesForEachOD)      
        trafficWaitingForEachOD = getWaitingTime(network)
        waitingTime.append(trafficWaitingForEachOD)
        
    return angularChange, waitingTime   


def analyseODFromPedestrians():
    #arrays for calculations
    angularChange = []
    waitingTime = []
    #Read in edges
    path = os.path.join(cwd,'data','Digitized Route Shape File','*.shp')
    files = glob.glob(path)
    for file in files:
        #edges = nx.read_shp(file)
        edges = nx.read_shp(file)
        #print(list(edges.edges(data=True))[12])
        posNodes = pre.getPositionOfNodesFromEdgesForAnalysis(edges)
        posNodes.keys()
        #New Edge position 
        posEdges = pre.getPositionOfEdgesForAnalysis(edges)
        #New edge calculation
        edg = pre.getEdgesForAnalysis(posEdges,posNodes)   

        network = pre.createNetwork(posNodes,edg)
        
        coordinates = pre.createDictFromEdgesCoords(edges,edg)
        nx.set_edge_attributes(network, name='coord', values=coordinates)
        
        removeEdges = getEdgesWithoutCoords(network)
        network.remove_edges_from(removeEdges)
        
        distance = pre.calcDistance(network,edg)
        nx.set_edge_attributes(network, name='dist', values=distance)
        
        trafficLights = pre.readcsv(os.path.join(cwd,'traffic_lights.csv'))
        trafficDict = pre.createDictFromTrafficLights(trafficLights)
        TrafficDuration = pre.createCompleteTrafficDict(network,trafficDict)
        nx.set_edge_attributes(network, name='traffic', values=TrafficDuration)
        
        adjacency = pre.calcAdjacency(network)
        adjacentAndNeighborNodesDict = pre.getAdjacentAndNeigborNodes(adjacency)
        
       #Calc TotalAngularchange and traffic light waiting time 
        anglesForEachOD = getAngles(network,adjacentAndNeighborNodesDict)
        angularChange.append(anglesForEachOD)      
        trafficWaitingForEachOD = getWaitingTime(network)
        waitingTime.append(trafficWaitingForEachOD)
        
    return angularChange, waitingTime        
        
        
def main():
    totalAngularChange = []
    totalWaiting = []
    totalAngDijkstra = []
    totalWaitingDijkstra = []
    PedestrianRoutesAngles, PedestrianRoutesWaiting  = analyseODFromPedestrians()
    for angle in PedestrianRoutesAngles:
        sumAngle = sum(angle)
        totalAngularChange.append(sumAngle)
    
    for waiting in PedestrianRoutesWaiting:
        meanWaiting = sum(waiting)
        totalWaiting.append(meanWaiting)       
    
    DijkstraRoutesAngles, DijkstraRoutesWaiting  = analyseODFromDijkstra()
    for angle in DijkstraRoutesAngles:
        sumAngle = sum(angle)
        totalAngDijkstra.append(sumAngle)
    
    for waiting in DijkstraRoutesWaiting:
        meanWaiting = sum(waiting)
        totalWaitingDijkstra.append(meanWaiting)      
    
    print('totalAngularChange')
    print(totalAngularChange)
    print('totalWaiting')
    print(totalWaiting)
    print('totalAngDijkstra')
    print(totalAngDijkstra)
    print('totalWaitingDijkstra')
    print(totalWaitingDijkstra)
    
main()    