# Route Planning Project

This is an A* based route planning project. The planner uses OpenStreetMap data and finds the shortest path between a given start and a destination. I completed this project while I attended Udacity's C++ software engineer nanodegree. I am a mentor now in the same program. 

<img src="map.png" width="600" height="450" />

## Installation 

- #### Dependencies for Running Locally
  * cmake >= 3.11.3
    * All OSes: [click here for installation instructions](https://cmake.org/install/)
  * make >= 4.1 (Linux, Mac), 3.81 (Windows)
    * Linux: make is installed by default on most Linux distros
    * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
    * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
  * gcc/g++ >= 7.4.0
    * Linux: gcc / g++ is installed by default on most Linux distros
    * Mac: same instructions as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
    * Windows: recommend using [MinGW](http://www.mingw.org/)
  * IO2D
    * Installation instructions for all operating systems can be found [here](https://github.com/cpp-io2d/P0267_RefImpl/blob/master/BUILDING.md)
    * This library must be built in a place where CMake `find_package` will be able to find it
  
## Clone

When cloning this project, be sure to use the `--recurse-submodules` flag.
```
git clone https://github.com/Robotawi/CppND-Astar-Route-Planner.git --recurse-submodules
```

## Setup
  ```
  cd CppND-Astar-Route-Planner
  mkdir build 
  cd build
  cmake ..
  make
  ```

## Running
The executable will be placed in the `build` directory. From within `build`, you can run the project as follows:
```
./OSM_A_star_search
```
Or to specify a map file:
```
./OSM_A_star_search -f ../<your_osm_file.osm>
```

## Testing

The testing executable is also placed in the `build` directory. From within `build`, you can run the unit tests as follows:
```
./test
```

## Contact
If you are interested in the presented work/ideas or if you have any questions, you are welcome to connect with me on [LinkedIn](https://www.linkedin.com/in/mohraess). We can discuss about this project and other interesting projects.