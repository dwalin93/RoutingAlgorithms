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
- [Dijkstra.py](https://github.com/dwalin93/RoutingAlgorithms/blob/dev/Dijkstra.py)
- [A_Star_1.py](https://github.com/dwalin93/RoutingAlgorithms/blob/dev/A_Star_1.py)

Add further information here
[A_Star_1.py] script utilises astar_path function from networkX library, created by  Salim Fadhley and Matteo Dell'Amico. 
The function calculating shortest paths and path lengths using the A* ("A star") algorithm was amended aiming to include the landmark score of the network links as the link cost and the cartesian distance between each node and the final destination as the algorithm heurestic.

```python
import foobar

foobar.pluralize('word') # returns 'words'
foobar.pluralize('goose') # returns 'geese'
foobar.singularize('phenomena') # returns 'phenomenon'
```
## License
[MIT](https://choosealicense.com/licenses/mit/)