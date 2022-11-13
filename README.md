# COSC 69 Final project

## Architecture Summary
This project is divided into the following components:

### exploration
Exploration is divided into the following components:
1. Mapping and localization. This is accomplished with the `gmapping` package.
2. Goal generation. This is done within the node `/simple_maze_exploring/nodes/explorer_elliot`.
    It uses BFS to find the shortest path (avoiding walls) to an unexplored node on the graph.

2. Generating path to goal. This is done using the external package (TODO)

### map_merging
Map merging is divided into the following components:
1. Map forwarding. This is done within the node `/map_merging_utils/nodes/map_forwarder`.
    It enables merging of map data from gmapping and from map merging. Robots listen to the
    map published by the forwarder.
2. 

### robot detection
### TODO -- graph generation ?? 