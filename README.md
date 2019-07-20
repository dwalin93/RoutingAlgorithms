# Evaluating Routing-algorithm Approaches for Pedestrian Behaviour Simulation

###### Created by Katarzyna Goch, Maicol Camargo, Mutaz Qafisheh, Somnath Chaudhuri and Philipp Glahe

This repository includes the algorithms and results of the study project "Evaluating Routing-algorithm Approaches for Pedestrian Behaviour Simulation". 

The aim of this project was to understand how pedestrians formulate routes in a familiar urban environment. Furthermore, the main objective was to evaluate routing algorithms of pedestrian behavior by comparing computed
routes with observed routes. 

## Installation

To run the scripts, you need the to install following library (using [pip](https://pypi.org/project/pip/) or [conda](https://docs.conda.io/en/latest/)):

- [networkX](https://github.com/networkx/networkx)

## Scripts
The repository includes two main scripts:
- [Dijkstra.py](https://github.com/dwalin93/RoutingAlgorithms/blob/master/Dijkstra.py)
- [A_Star_1.py](https://github.com/dwalin93/RoutingAlgorithms/blob/master/A_Star_1.py)

[Dijkstra.py](https://github.com/dwalin93/RoutingAlgorithms/blob/master/Dijkstra.py) script utilises [dijkstra_path](https://networkx.github.io/documentation/stable/reference/algorithms/generated/networkx.algorithms.shortest_paths.weighted.dijkstra_path.html#networkx.algorithms.shortest_paths.weighted.dijkstra_path) function from [networkX](https://github.com/networkx/networkx) library, created by  Salim Fadhley and Matteo Dell'Amico. 
The function was extended using not only the shortest distance as cost factor, but also include angular change and possible traffic lights as costs. 

[A_Star_1.py](https://github.com/dwalin93/RoutingAlgorithms/blob/master/A_Star_1.py) script utilises [astar_path](https://networkx.github.io/documentation/stable/reference/algorithms/generated/networkx.algorithms.shortest_paths.astar.astar_path.html#networkx.algorithms.shortest_paths.astar.astar_path) function from [networkX](https://github.com/networkx/networkx) library.
The function calculating shortest paths and path lengths using the A* ("A star") algorithm was amended aiming to include the landmark score of the network links as the link cost and the cartesian distance between each node and the final destination as the algorithm heurestic.


## License
[MIT](https://choosealicense.com/licenses/mit/)