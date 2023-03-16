**Table of Contents**

<!-- TOC -->

- [Introduction](#introduction)
- [RRT Algorithm](#rrt-algorithm)
- [RRT* Algorithm](#rrt-algorithm)
- [Design Details](#design-details)
- [License](#license)
- [Resources](#resources)

<!-- /TOC -->

# Introduction
A rapidly exploring random tree (RRT) is an algorithm designed to efficiently search nonconvex, high-dimensional spaces by randomly building a space-filling tree. The tree is constructed incrementally from samples drawn randomly from the search space and is inherently biased to grow towards large unsearched areas of the problem. RRTs were developed by Steven M. LaValle and James J. Kuffner Jr. They easily handle problems with obstacles and differential constraints (nonholonomic and kinodynamic) and have been widely used in autonomous robotic motion planning.

RRTs can be viewed as a technique to generate open-loop trajectories for nonlinear systems with state constraints. An RRT can also be considered as a Monte-Carlo method to bias search into the largest Voronoi regions of a graph in a configuration space. Some variations can even be considered stochastic fractals.

RRTs can be used to compute approximate control policies to control high dimensional nonlinear systems with state and action constraints.

# RRT Algorithm

```py
RRT Pseudocode

Input: Initial configuration qinit, number of vertices in RRT K, incremental distance Δq
Output: RRT graph G

G.init(qinit)
for k = 1 to K do
    qrand ← RAND_CONF()
    qnear ← NEAREST_VERTEX(qrand, G)
    qnew ← NEW_CONF(qnear, qrand, Δq)
    G.add_vertex(qnew)
    G.add_edge(qnear, qnew)
return G
```
- "←" denotes assignment. For instance, "largest ← item" means that the value of largest changes to the value of item.
- "return" terminates the algorithm and outputs the following value.

In the algorithm above, **RAND_CONF** grabs a random configuration $q_{rand}$ in C. This may be replaced with a function **RAND_FREE_CONF** that uses samples in $C_{free}$, while rejecting those in Cobs using some collision detection algorithm.

**NEAREST_VERTEX** is a function that runs through all vertices *v* in graph *G*, calculates the distance between $q_{rand}$ and *v* using some measurement function thereby returning the nearest vertex.

**NEW_CONF** selects a new configuration $q_{new}$ by moving an incremental distance $\delta{q}$ from $q_{near}$ in the direction of $q_{rand}$.

[![RRT Algorithm](https://img.youtube.com/vi/NQdXxTMQ4So/0.jpg)](https://www.youtube.com/watch?v=NQdXxTMQ4So)

*Video: Implementation of RRT Algorithm in Unity 3D.*

# RRT* Algorithm

RRT* is an optimized version of RRT. When the number of nodes approaches infinity, the RRT* algorithm will deliver the shortest possible path to the goal. While realistically unfeasible, this statement suggests that the algorithm does work to develop a shortest path. The basic principle of RRT* is the same as RRT, but two key additions to the algorithm result in significantly different results.

1. RRT* records the distance each vertex has traveled relative to its parent vertex. This is referred to as the cost()of the vertex. After the closest node is found in the graph, a neighborhood of vertices in a fixed radius from the new node are examined. If a node with a cheaper cost() than the proximal node is found, the cheaper node replaces the proximal node. The effect of this feature can be seen with the addition of fan shaped twigs in the tree structure. The cubic structure of RRT is eliminated.
2. RRT* adds is the rewiring of the tree. After a vertex has been connected to the cheapest neighbor, the neighbors are again examined. Neighbors are checked if being rewired to the newly added vertex will make their cost decrease. If the cost does indeed decrease, the neighbor is rewired to the newly added vertex. This feature makes the path more smooth.

```py
RRT* Pseudo Code

Rad = r
G(V,E) //Graph containing edges and vertices
For itr in range(0…n)
    Xnew = RandomPosition()
    If Obstacle(Xnew) == True, try again
    Xnearest = Nearest(G(V,E),Xnew)
    Cost(Xnew) = Distance(Xnew,Xnearest)
    Xbest,Xneighbors = findNeighbors(G(V,E),Xnew,Rad)
    Link = Chain(Xnew,Xbest)
    For x’ in Xneighbors
        If Cost(Xnew) + Distance(Xnew,x’) < Cost(x’)
            Cost(x’) = Cost(Xnew)+Distance(Xnew,x’)
            Parent(x’) = Xnew
            G += {Xnew,x’}
    G += Link 
Return G
```

[![RRT Star Algorithm](https://img.youtube.com/vi/BAnt3JvI_Ww/0.jpg)](https://www.youtube.com/watch?v=BAnt3JvI_Ww)

# Design Details

Designed by: [Parth Patel](mailto:parth.pmech@gmail.com)

# License

This project is licensed under [GNU General Public License v3.0](https://www.gnu.org/licenses/gpl-3.0.en.html) (see [LICENSE.md](LICENSE.md)).

Copyright 2023 Parth Patel

Licensed under the GNU General Public License, Version 3.0 (the "License"); you may not use this file except in compliance with the License.

You may obtain a copy of the License at

_https://www.gnu.org/licenses/gpl-3.0.en.html_

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.

# Resources
- [Medium: Robotic Path Planning: RRT and RRT*](https://theclassytim.medium.com/robotic-path-planning-rrt-and-rrt-212319121378)
- [Wikipedia: Rapidly-exploring random tree](https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree)