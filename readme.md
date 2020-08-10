Multi-Agent Path Finding
===

A simulator and visualizer of Multi-Agent Path Finding (MAPF), used in a paper "Iterative Refinement for Realtime MAPF".
It is written in C++(17) and tested on OSX 10.15.
The visualizer uses [openFrameworks](https://openframeworks.cc).

The implementations include: HCA\* and WHCA\* [1], PIBT [2], CBS [3], ICBS [4], ECBS [5], PIBT-Complete, and IR.

## Demo
![100 agents in arena](/material/arena_100agents.gif)

100 agents in arena, planned by PIBT in 67ms

![1000 agents in brc202d](/material/brc202d_1000agents.gif)

1000 agents in brc202d, planned by PIBT-Complete in 84sec


## Building

```
mkdir build
cd build
cmake ..
make
```

## Usage
PIBT
```sh
./app -i ../instances/sample.txt -s PIBT -o result.txt -v
```

IR (the result is saved in `result.txt`)
```sh
./app -i ../instances/random-32-32-20_70agents_1.txt -s IR -n 50 -t 3000 -v
```

You can find details and explanations for all parameters with:
```sh
./app --help
```

Please see `instances/sample.txt` for parameters of instances, e.g., filed, number of agents, time limit, etc.

## Visualizer

### Building
You need to install openFrameworks beforehand and export `OF_ROOT` of your environment.
```sh
export OF_ROOT={your openFrameworks directory}
```

Build as follows.
```sh
cd visualizer/
make build
cd ..
chmod +x ./visualize.sh
```

### Usage
```sh
cd build
../visualize.sh result.txt
```

## Licence
This software is released under the MIT License, see [LICENSE.txt](LICENCE.txt).

## Notes
- Maps in `maps/` are from [MAPF benchmarks](https://movingai.com/benchmarks/mapf.html).
  When you add a new map, please place it in the `maps/` directory.
- The font in `visualizer/bin/data` is from [Google Fonts](https://fonts.google.com/).
- Scripts for the experiments are in `test_scripts/`.

## Author
anonymous

## Reference
1. Silver, D. (2005).
   Cooperative pathfinding.
   In AIIDE’05 Proceedings of the First AAAI Conference on Artificial Intelligence and Interactive Digital Entertainment (pp. 117–122).
1. Okumura, K., Machida, M., Défago, X., & Tamura, Y. (2019).
   Priority Inheritance with Backtracking for Iterative Multi-agent Path Finding.
   In Proceedings of the Twenty-Eighth International Joint Conference on Artificial Intelligence (pp. 535–542).
1. Sharon, G., Stern, R., Felner, A., & Sturtevant, N. R. (2015).
   Conflict-based search for optimal multi-agent pathfinding.
   Artificial Intelligence, 219, 40–66.
1. Boyarski, E., Felner, A., Stern, R., Sharon, G., Tolpin, D., Betzalel, O., & Shimony, E. (2015, June).
   ICBS: improved conflict-based search algorithm for multi-agent pathfinding.
   In Twenty-Fourth International Joint Conference on Artificial Intelligence.
1. Barer, M., Sharon, G., Stern, R., & Felner, A. (2014).
   Suboptimal Variants of the Conflict-Based Search Algorithm for the Multi-Agent Pathfinding Problem.
   In Seventh Annual Symposium on Combinatorial Search.
