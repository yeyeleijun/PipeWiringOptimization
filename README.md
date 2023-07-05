# PipeWiringOptimization

## 👋Documentation
Automatic pipeline routing is studied in this project. One of the most crucial steps in this problem is the description of the set of feasible routes for
the pipes. We will consider a graph-based discretization of the space that allows us to use
Path searching tools for efficiently solving the problem. The graph is generated
taking into account the shape of the region and obstacles, the positions of the source and
destination points of the pipes, the size of the pipelines, and so on.

The crucial ingredient is [A*](https://en.wikipedia.org/wiki/A*_search_algorithm#:~:text=A*%20is%20an%20informed%20search,shortest%20time%2C%20etc.) search algorithm and
the main reference is this [paper](https://www.sciencedirect.com/science/article/abs/pii/S0305048322000664).


<div style="display: flex; align-items: center;">
  <div style="width: 50%; text-align: center;">
    <img src="https://github.com/yeyeleijun/PipeWiringOptimization/raw/master/test_case/init_pipes.png" alt="Image 1" style="width: 80%;">
    <p style="font-size: 16px;">Pipe wiring before optimization</p>
  </div>
  <div style="width: 50%; text-align: center;">
    <img src="https://github.com/yeyeleijun/PipeWiringOptimization/raw/master/test_case/optimized_pipes.png" alt="Image 2" style="width: 80%;">
    <p style="font-size: 16px;">Pipe wiring after optimization</p>
  </div>
</div>


## 🚀Authors

- [@leopold](https://github.com/lepodl)
- [@yeleijun](https://github.com/yeyeleijun)

## 🚗Installation
This project relies on the visualization software [mayavi](https://docs.enthought.com/mayavi/mayavi/).

```bash
  pip install PyQt5
  pip install mayavi
  pip install vtk
```
## 🧩TOC
| Directory  | Description                                             |
|:-----------|:--------------------------------------------------------|
| `model`    | `basic implementation of A* and multi pipe routing.`    |
| `plotting` | `plotting module for 2D or 3D instance`                 |
| `refdemo`  | `some old code` |

## 🎲Demo
Some effective test codes can be found in `test_case/test.py`. The gif blow shows the visualization of A* searching.

<p align="center">
  <img src="https://github.com/yeyeleijun/PipeWiringOptimization/raw/master/test_case/Astar.gif" alt="Image 1" title="Visualization of A* searching" width="60%">
</p>

## 🔑Limitation and future direction
1. Some hard constraints can be replaced by soft constrains by increasing the cost of conflict path.
    - conflict of overlapping with obstacles.
    - minimum required distance between consecutive elbows in the paths
2. Customized written code using C++ to speed the running.
3. Consider more economics and physical constrains.



## 🛸Other references
1. 建筑物离散化，对每条管道分别找到最短路

[Ship Pipe Route Design Using Improved A∗ Algorithm and Genetic Algorithm](https://ieeexplore.ieee.org/abstract/document/9172005)

[A Distance-Field-Based Pipe-Routing Method](https://www.mdpi.com/1996-1944/15/15/5376)

2. 蚁群算法（多条管道全局最优）

[A co-evolutionary improved multi-ant colony optimization for ship multiple and branch pipe route design](https://www.sciencedirect.com/science/article/pii/S0029801815001031)

3. 分别计算每条管道满足约束下的最优路径，再建立二叉树解决冲突

[From Multi-Agent Pathfinding to 3D Pipe Routing](https://aaai.org/papers/00011-from-multi-agent-pathfinding-to-3d-pipe-routing/)

[Conflict-based search for optimal multi-agent pathfinding](https://www.sciencedirect.com/science/article/pii/S0004370214001386#se0260)

4. MIP

[Optimal Pipe Routing Techniques in an Obstacle-Free 3D Space](https://hal.science/hal-02865302v1)

Optimization of Cable Harness Routing Mathematical Modelling, Lagrangian Relaxation, and Subgra-dient Optimization
