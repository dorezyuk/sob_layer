# Small-Objects-Big (Sob) Layer

The sob_layer provides a costmap_2d::Layer plugin for move_base (and all 
compatible frameworks). Its intention is to offer an alternative to the
commonly used [costmap_2d::InflationLayer](http://wiki.ros.org/costmap_2d/hydro/inflation).

## Build

The build process is the same as for every catkin-based package.
Assuming you are in our catkin_ws and have catkin-tools installed:

```
git clone https://github.com/dorezyuk/sob_layer.git && cd sob_layer
catkin build --this --no-deps --cmake-args -DCMAKE_BUILD_TYPE=Release
```

By default it will also try to build the benchmark target.
In order to run the benchmarks you will need to install the [benchmark](https://github.com/google/benchmark) library.
If you don't want to build the benchmark target, add
```
-Dsob_layer_BENCHMARK=OFF
```
to your build command. The benchmarking should help to figure you out, if the
sob_layer offers a reasonable improvement for your specific usecase/platform.

## Requirements

The library requires at least C++11. It has been tested on Ubuntu 18.04 running ros-melodic. Following compilers are known to work
* GCC 7.5.0
* Clang 6.0.0


## Benchmarks
## Config