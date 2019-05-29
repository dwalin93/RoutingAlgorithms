import os
#import osmnx as ox
import networkx as nx
import matplotlib.pyplot as plt


cwd = os.getcwd()
print(cwd)

def dist(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

pathToFile = os.path.join(cwd, 'data', 'Muenster_edges.shp')
pathToNodes = os.path.join(cwd,'data', 'Muenster_nodes.shp')
nodes = nx.read_shp(pathToNodes)
streetNetwork = nx.read_shp(pathToFile)
pos = {k: v for k,v in enumerate(nodes.nodes())}

X = nx.Graph()
X.add_nodes_from(pos.keys())
l = [set(x) for x in streetNetwork.edges()] #11075
print(l[0][0])
edg = [tuple(k for k,v in pos.items() if v in sl) for sl in l]
print(edg[0])
nx.draw_networkx_nodes(X,pos,node_size=10, node_color='r')
X.add_edges_from(edg)
nx.draw_networkx_edges(X,pos)
plt.title('From shapefiles to NetworkX')
#plt.xlim(406000, 406500) 
#plt.ylim(5758000, 5758200) 
#plt.xlabel('X [m]')
#plt.ylabel('Y [m]')

#################
## Test A Star ##
#################

'''print("Test A Star algorithm")

X_Astar = nx.grid_graph(dim=[]) '''

def distance(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

X_Astar = X
print(edg[0])
print(nx.astar_path(X_Astar, 0, 3, heuristic=distance))

##### Assign weight
first=list(X.edges(data=True))[0]
print(first)


'''nx.set_edge_attributes(X_Astar, {e: e[1][0]*2 for e in X_Astar.edges()}, 'cost')

print(nx.astar_path(X_Astar, (0,0), (2,2), heuristic=dist, weight='cost'))

nx.draw(X_Astar,with_labels=True,node_size=300)

G = nx.grid_graph(dim=[3, 3]) # nodes are two-tuples (x,y)
for e in G.edges():
    print (e[1][0])
    print(e)
nx.set_edge_attributes(G, {e: e[1][0]*2 for e in G.edges()}, 'cost')

print(G[0,0])
print(G[1,1])
print(G[2,2])


def dist(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
print(nx.astar_path(G, (0, 0), (2, 2), heuristic=dist, weight='cost'))'''

G = nx.grid_2d_graph(85,85)
pos1 = dict(zip(G,G)) # dictionary of node names->positions
nx.draw(G,pos1,with_labels=True)
plt.show

print(nx.astar_path(G, (0, 0), (2, 2), heuristic=dist, weight='cost'))

