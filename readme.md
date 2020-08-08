Multi-Agent Path Finding
===

A simulator and visualizer of Multi-Agent Path Finding (MAPF), used in a paper "Iterative Refinement for Realtime MAPF".
It is written in C++(17) and tested on OSX 10.15.
The visualizer relies on [openFrameworks](https://openframeworks.cc).

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
The map is from [MAPF benchmarks](https://movingai.com/benchmarks/mapf.html).

## Author
