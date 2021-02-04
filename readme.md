Multi-Agent Path Finding
===
[![MIT License](http://img.shields.io/badge/license-MIT-blue.svg?style=flat)](LICENSE)

A simulator and visualizer of Multi-Agent Path Finding (MAPF), used in a paper "Iterative Refinement for Realtime MAPF".
It is written in C++(17) with [CMake](https://cmake.org/) build and tested on OSX 10.15.
The visualizer uses [openFrameworks](https://openframeworks.cc).

The implementations include: HCA\* and WHCA\* [1], PIBT [2], CBS [3], ICBS [4], ECBS [5], Revisit Prioritized Planning [6], Push and Swap [7], winPIBT [8], PIBT-Complete, and IR.

| platform | status |
| ---: | :--- |
| macos-latest | ![test_macos](https://github.com/Kei18/mapf-IR/workflows/test_macos/badge.svg) ![build_visualizer_macos](https://github.com/Kei18/mapf-IR/workflows/build_visualizer_macos/badge.svg) |
| ubuntu-latest | ![test_ubuntu](https://github.com/Kei18/mapf-IR/workflows/test_ubuntu/badge.svg) ![build_visualizer_ubuntu](https://github.com/Kei18/mapf-IR/workflows/build_visualizer_ubuntu/badge.svg) |

You can see the performance of each solver from [auto\_record repo](https://github.com/Kei18/mapf-IR/tree/auto_record). The records were created by Github Actions.

## Demo
![100 agents in arena](/material/arena_100agents.gif)

100 agents in arena, planned by PIBT in ~~67ms~~ 7ms.

![1000 agents in brc202d](/material/brc202d_1000agents.gif)

1000 agents in brc202d, planned by PIBT-Complete in ~~84sec~~ 1683ms.
The gif shows a part of an MAPF plan.

## Building

```
git clone --recursive https://github.com/Kei18/mapf-IR
cd mapf-IR
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

### Output File

This is an example output of `../instances/sample.txt`.
Note that `(x, y)` denotes location.
`(0, 0)` is the left-top point.
`(x, 0)` is the location at `x`-th column and 1st row.
```
instance= ../instances/sample.txt
agents=100
map_file=arena.map
solver=PIBT
solved=1
soc=3403
makespan=68
comp_time=58
starts=(32,21),(40,4),(20,22),(26,18), [...]
goals=(10,16),(30,21),(11,42),(44,6), [...]
solution=
0:(32,21),(40,4),(20,22),(26,18), [...]
1:(31,21),(40,5),(20,23),(27,18), [...]
[...]
```

## Visualizer

### Building
It takes around 10 minutes.
```sh
sh ./third_party/openFrameworks/scripts/osx/download_libs.sh
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

You can manipulate it via your keyboard. See printed info.

## Licence
This software is released under the MIT License, see [LICENSE.txt](LICENCE.txt).

## Notes
- Maps in `maps/` are from [MAPF benchmarks](https://movingai.com/benchmarks/mapf.html).
  When you add a new map, please place it in the `maps/` directory.
- The font in `visualizer/bin/data` is from [Google Fonts](https://fonts.google.com/).
- Scripts for the experiments are in `exp_scripts/`.

## Author
[Keisuke Okumura](https://kei18.github.io) is a Ph.D. candidate at Tokyo Institute of Technology, working on multiple moving agents.

## Reference
1. Silver, D. (2005).
    Cooperative pathfinding.
    Proc. AAAI Conf. on Artificial Intelligence and Interactive Digital Entertainment (AIIDE-05)
1. Okumura, K., Machida, M., Défago, X., & Tamura, Y. (2019).
   Priority Inheritance with Backtracking for Iterative Multi-agent Path Finding.
   Proc. Intel. Joint Conf. on Artificial Intelligence (IJCAI)
1. Sharon, G., Stern, R., Felner, A., & Sturtevant, N. R. (2015).
   Conflict-based search for optimal multi-agent pathfinding.
   Artificial Intelligence
1. Boyarski, E., Felner, A., Stern, R., Sharon, G., Tolpin, D., Betzalel, O., & Shimony, E. (2015).
   ICBS: improved conflict-based search algorithm for multi-agent pathfinding.
   Proc. Intel. Joint Conf. on Artificial Intelligence (IJCAI)
1. Barer, M., Sharon, G., Stern, R., & Felner, A. (2014).
   Suboptimal Variants of the Conflict-Based Search Algorithm for the Multi-Agent Pathfinding Problem.
   Annual Symposium on Combinatorial Search (SoCS)
1. Čáp, M., Novák, P., Kleiner, A., & Selecký, M. (2015).
   Prioritized planning algorithms for trajectory coordination of multiple mobile robots.
   IEEE Trans. on automation science and engineering
1. Luna, R., & Bekris, K. E. (2011).
   Push and swap: Fast cooperative path-finding with completeness guarantees.
   Proc. Intel. Joint Conf. on Artificial Intelligence (IJCAI)
1. Okumura, K., Tamura, Y. & Défago, X. (2020).
   winPIBT: Extended Prioritized Algorithm for Iterative Multi-agent Path Finding.
   IJCAI Workshop on Multi-Agent Path Finidng (WoMAPF)
