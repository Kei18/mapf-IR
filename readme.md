Multi-agent Path Finding Simulator
===

Tested on OSX 10.15

## Demo
![MAPF](/material/mapf-example.gif)

![MAPF](/material/ost003d.gif)


## Building

### Visualizer
The visualization relies on [openFrameworks](https://openframeworks.cc).
You need to install openFrameworks beforehand and export `OF_ROOT` of your environment.
```
export OF_ROOT={your openFrameworks directory}
```

Then, at the project directory, run as;
```sh
cd visualize/
make
cd ..
chmod +x ./visualize.sh
```

## Usage


## Licence
This software is released under the MIT License, see [LICENSE.txt](LICENCE.txt).

## Notes
There is a [previous version](https://github.com/Kei18/pibt) of this simulator, however, the new one is simple and fast, e.g., PIBT is speedup almost 3 times.

## Author
[Keisuke Okumura](https://kei18.github.io) is a Ph.D. candidate at Tokyo Institute of Technology, working on multiple moving agents.
