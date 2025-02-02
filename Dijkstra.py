import os
import networkx as nx
from heapq import heappush, heappop
from itertools import count
import preprocessing as pre
import collections

cwd = os.getcwd()
print(cwd)


####Implementation of Dijkstra shortest path. Code taken from https://github.com/networkx/networkx####
def getAdj(neighbours,u):
    return neighbours.get(u)['adj']

def getNeighbours(neighbours,u):
    return neighbours.get(u)['neighbours']


def _weight_function(G, weight):
    if callable(weight):
        return weight
    # If the weight keyword argument is not callable, we assume it is a
    # string representing the edge attribute containing the weight of
    # the edge.
    if G.is_multigraph():
        minValue =  lambda u, v, d: min(attr.get(weight, 1) for attr in d.values())
        return minValue
    value = lambda u, v, data: data.get(weight, 1)  
    return value

def dijkstra_path(G, source, target,neighborsDict, weight='weight'):
    (length, path) = single_source_dijkstra(G, source,neighborsDict, target=target,
                                            weight=weight)
    return path


def single_source_dijkstra(G, source,neighborsDict, target=None, cutoff=None,
                           weight='weight'):
    return multi_source_dijkstra(G, {source},neighborsDict, cutoff=cutoff, target=target,
                                 weight=weight)
    
def multi_source_dijkstra(G, sources,neighborsDict, target=None, cutoff=None,
                          weight='weight'):
    if not sources:
        raise ValueError('sources must not be empty')
    if target in sources:
        return (0, [target])
    weight = _weight_function(G, weight)
    paths = {source: [source] for source in sources}  # dictionary of paths
    dist = _dijkstra_multisource(G, sources, neighborsDict, weight, paths=paths,
                                 cutoff=cutoff, target=target)
    if target is None:
        return (dist, paths)
    try:
        return (dist[target], paths[target])
    except KeyError:
        raise nx.NetworkXNoPath("No path to {}.".format(target))
        
# Angle: >90 to < 180 and  <181 to >270 REMAINDER       

def _dijkstra_multisource(G, sources,neighborsDict, weight, pred=None, paths=None,
                          cutoff=None, target=None):
    G_succ = G._succ if G.is_directed() else G._adj

    push = heappush
    pop = heappop
    dist = {}  # dictionary of final distances
    seen = {}
    # fringe is heapq with 3-tuples (distance,c,node)
    # use the count c to avoid comparing nodes (may not be able to)
    c = count()
    fringe = []
    #Angle array
    angles = []
    for source in sources:
        if source not in G:
            raise nx.NodeNotFound("Source {} not in G".format(source))
        seen[source] = 0
        push(fringe, (0, next(c), source))
    while fringe:
        (d, _, v) = pop(fringe)
        if v in dist:
            continue  # already searched this node.
        dist[v] = d
        if v == target:
            break
        for u, e in G_succ[v].items():
            cost = weight(v, u, e)
            #print('COST' + str(cost))
            #scaledCost = pre.scaleCosts(cost,0.21,1193.32)
            neighbours = getNeighbours(neighborsDict,v)
            adj = getAdj(neighborsDict,u)
            for neighbour in neighbours:
                if(v != neighbour and neighbour in adj):
                    angle = pre.anglesOfLines(G,v,u,neighbour)
                    angles.append(angle)
            if cost is None:
                continue
            if not angles:
                vu_dist = dist[v] + cost 
            else:
                minAngle = pre.getMinAngle(angles)
                vu_dist = dist[v] + cost + minAngle 
                angles = []
            if cutoff is not None:
                if vu_dist > cutoff:
                    continue
            if u in dist:
                if vu_dist < dist[u]:
                    raise ValueError('Contradictory paths found:',
                                     'negative weights?')
            elif u not in seen or vu_dist < seen[u]:
                seen[u] = vu_dist
                push(fringe, (vu_dist, next(c), u))
                if paths is not None:
                    paths[u] = paths[v] + [u]
                if pred is not None:
                    pred[u] = [v]
            elif vu_dist == seen[u]:
                if pred is not None:
                    pred[u].append(v)

    # The optional predecessor and path dictionaries can be accessed
    # by the caller via the pred and paths objects passed as arguments.
    return dist


####Main Function

def main():
    #Read in edges
    pathToEdges = os.path.join(cwd,'data','Muenster_edges.shp')
    edges = nx.read_shp(pathToEdges)
 
    posNodes = pre.getPositionOfNodesFromEdges(edges)

    #New Edge position 
    posEdges = pre.getPositionOfEdges(edges)
    #New edge calculation
    edg = pre.getEdges(posEdges,posNodes)
   
    #pre.getEdges(posEd,posNo)    
    
    network = pre.createNetwork(posNodes,edg)

    #Set position of Nodes for exporting as shp
    pre.setPosNodes(network, posNodes)
    #pre.drawNetwork(network,posNodes)
    #Add Weight to edges
    #Get coordinates of edges
    coordinates = pre.createDictFromEdgesCoords(edges,edg)

    nx.set_edge_attributes(network, name='coord', values=coordinates)
        
    #Calc Distance
    distance = pre.calcDistance(network,edg)

    key_max = max(distance.keys(), key=(lambda k: distance[k]))
    key_min = min(distance.keys(), key=(lambda k: distance[k]))
    print(key_max)
    print(key_min)
    distance.get((51, 2795))
    distance.get((5513, 6486))
    
    #Apply dist on edges
    nx.set_edge_attributes(network, name='dist', values=distance)

    H1 = network.subgraph([24,1946])
    H = network.edge_subgraph([(24,1946)])
    nx.draw_networkx_nodes(H1,posNodes,node_size=10, node_color='r')
    nx.draw_networkx_edges(H,posNodes,edge_color = 'b')

    
    adjacency = pre.calcAdjacency(network)
    adjacentAndNeighborNodesDict = pre.getAdjacentAndNeigborNodes(adjacency)

    trafficLights = pre.readcsv(os.path.join(cwd,'traffic_lights.csv'))
    
    trafficDict = pre.createDictFromTrafficLights(trafficLights)
    
    TrafficDuration = pre.createCompleteTrafficDict(network,trafficDict)
    
    nx.set_edge_attributes(network, name='traffic', values=TrafficDuration)
    
    totalCosts = pre.calcCosts(network)
    
    nx.set_edge_attributes(network, name='costs', values=totalCosts)
    
    
    dijkstraOD1 = dijkstra_path(network,395,7775,adjacentAndNeighborNodesDict,weight='costs')
    dijkstraOD2 = dijkstra_path(network,7775,2196,adjacentAndNeighborNodesDict,weight='costs')
    dijkstraOD3 = dijkstra_path(network,2196,6768,adjacentAndNeighborNodesDict,weight='costs')
    print(dijkstraOD1)
    print(dijkstraOD2)
    print(dijkstraOD3)
    #print(list(network.edges(data = True)))
    #shortestPath = pre.drawShortestPath(network,dijkstra,posNodes)
    
    #print(network.nodes[0])
    #mapping = {old_label:network.nodes[idx]['loc'] for idx, old_label in enumerate(network.nodes())}
    #print(mapping)
    #type(mapping[0][0])
#    H = nx.relabel_nodes(network, mapping)
#    HD = H.to_directed()
#    shp.write_shp(H,cwd) # doctest +SKIP
   
          
#call main()
main()              
                
        
    
   
    
    







