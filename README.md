# Game AI Programming — Flocking with Spatial Partitioning

## Project Overview

This project implements a flocking simulation in Unreal Engine as part of a Game AI Programming assignment. It consists of two main components: a flocking behavior system built on combined steering, and a spatial partitioning optimization using a flat grid structure.

## Flocking Behaviors

Flocking is implemented as a set of steering behaviors combined through a **Blended Steering** system, which computes a weighted average of all active behavior outputs. The three core flocking behaviors are:

- **Cohesion** — Steers each agent toward the average position of its neighbors, implemented by delegating to a Seek behavior targeting the neighborhood centroid.
- **Separation** — Steers each agent away from nearby neighbors using an inverse-distance weighting (1/x), so closer neighbors exert a stronger repulsive force. The resulting vector is normalized and scaled to the agent's maximum speed to ensure consistent magnitude relative to other behaviors.
- **Alignment (Velocity Match)** — Steers each agent to match the average velocity of its neighbors, encouraging the flock to move as a cohesive unit.

These three behaviors are blended alongside **Seek** (toward a mouse-placed target) and **Wander** (random drift). The blended result is then wrapped in a **Priority Steering** system, which first evaluates an **Evade** behavior against a designated agent to evade — if that produces output, it takes priority over the flock blend.

Behavior weights are adjustable at runtime via an ImGui panel.

## Spatial Partitioning

To optimize neighbor lookups, a flat grid-based **CellSpace** partitioning structure is implemented. Without partitioning, every agent must be compared against every other agent each frame (O(n²)). The spatial partition reduces this by only checking agents in cells that overlap the query neighborhood radius.

The grid is centred on the world origin and divided into an equal number of rows and columns. Each cell stores a list of agent pointers. Each frame, agents are moved between cells when they cross a cell boundary via `UpdateAgentCell`, and neighbors are found by building an axis-aligned bounding box around the query radius and testing only overlapping cells via `RegisterNeighbors`.

Spatial partitioning can be toggled at compile time by commenting or uncommenting the following line at the top of `Flock.h`:

```cpp
#define GAMEAI_USE_SPACE_PARTITIONING
```

## Debug Rendering

The following debug visualizations are available via the ImGui panel at runtime:

- **Debug Steering** — draws steering vectors per agent
- **Debug Neighborhood** — draws the neighborhood radius circle and highlights neighbors of the first agent in yellow; when partitioning is enabled, also draws the query bounding box in green
- **Debug Partitions** *(partitioning only)* — draws the full cell grid in blue with agent counts displayed per occupied cell

## Extra assignement

For the extra assignment, I implemented a wolfpack-style hunting behavior. Five agents function as coordinated hunters, each steering toward the prey while maintaining separation to avoid clustering. The prey agent uses an Evade behavior to dynamically flee from the closest wolf, resulting in a small but convincing predator-prey interaction.