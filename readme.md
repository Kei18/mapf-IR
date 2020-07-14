Multi-agent Path Finding Simulator
===

Tested on OSX 10.15

## Demo
![MAPF](/material/mapf-example.gif)

![MAPF](/material/ost003d.gif)


## Building

```
mkdir build
cd build
cmake ..
make
```

## Usage
```
./app -i ../instance/sample.txt -s PIBT -o result.txt -v
```

You can find details and explanations for all parameters with:
```
./app --help
```

## Visualizer

### Building
The visualization relies on [openFrameworks](https://openframeworks.cc).
You need to install openFrameworks beforehand and export `OF_ROOT` of your environment.
```
export OF_ROOT={your openFrameworks directory}
```

Then, at the project directory, run as;
```sh
cd visualizer/
make
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
There is a [previous version](https://github.com/Kei18/pibt) of this simulator, however, the new one is simple and fast, e.g., PIBT is speedup almost 3 times.

## Author
[Keisuke Okumura](https://kei18.github.io) is a Ph.D. candidate at Tokyo Institute of Technology, working on multiple moving agents.
