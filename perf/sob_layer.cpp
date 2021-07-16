/*
 * MIT License
 *
 * Copyright (c) 2020 Dima Dorezyuk
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <sob_layer/sob_layer.hpp>

#include <costmap_2d/inflation_layer.h>
#include <costmap_2d/layered_costmap.h>

#include <benchmark/benchmark.h>

#include <cmath>
#include <vector>

using namespace sob_layer;

constexpr double default_radius = 1;

struct BMSobLayer : public sob_layer::SobLayer {
  BMSobLayer(costmap_2d::LayeredCostmap& _lm) {
    layered_costmap_ = &_lm;
    matchSize();

    // set the inflation radius (here doesn't matter)
    inflation_radius_ = default_radius;
    enabled_ = true;
    onFootprintChanged();
  }
};

struct BMInflationLayer : public costmap_2d::InflationLayer {
  BMInflationLayer(costmap_2d::LayeredCostmap& _lm) {
    layered_costmap_ = &_lm;
    matchSize();

    // set the inflation_radius, so we can update the caches
    inscribed_radius_ = inflation_radius_ = default_radius;
    enabled_ = true;
    onFootprintChanged();
  }
};

using geometry_msgs::Point;
Point
make_point(double x, double y) {
  Point out;
  out.x = x;
  out.y = y;
  return out;
}

costmap_2d::LayeredCostmap
make_master(benchmark::State& _state) noexcept {
  const auto size = static_cast<size_t>(_state.range(0));
  const auto res = static_cast<double>(_state.range(1)) / 1000.;
  // setup the map
  costmap_2d::LayeredCostmap lm("map", false, false);
  lm.resizeMap(size, size, res, 0, 0);

  // setup a footprint with 1 meter radius - needed for the inflation-layer
  const std::vector<Point> fp{make_point(-0.5, -1), make_point(0.5, -1),
                              make_point(0.5, 1), make_point(-0.5, 1)};

  lm.setFootprint(fp);
  return lm;
}

template <typename _Layer>
void
squared_obstacle(benchmark::State& _state) {
  auto master = make_master(_state);
  _Layer layer(master);
  double dummy;
  layer.updateBounds(0, 0, 0, &dummy, &dummy, &dummy, &dummy);

  // range contains (size, resolution in mm, percentage of blocked area)
  const auto size = static_cast<size_t>(_state.range(0));
  auto blocked_area = static_cast<double>(_state.range(2)) / 100.;
  // clamp it to the interval [0, 1]
  blocked_area = std::max(blocked_area, 0.);
  blocked_area = std::min(blocked_area, 1.);

  // paint the blocked area
  const auto blocked_start =
      static_cast<size_t>((0.5 - blocked_area / 2) * size);
  const auto blocked_end = static_cast<size_t>((0.5 + blocked_area / 2) * size);

  auto map = master.getCostmap();
  for (size_t cc = blocked_start; cc != blocked_end; ++cc)
    for (size_t rr = blocked_start; rr != blocked_end; ++rr)
      map->setCost(rr, cc, costmap_2d::LETHAL_OBSTACLE);

  // backup the map
  const auto map_copy = *map;
  for (auto _ : _state) {
    _state.PauseTiming();
    *map = map_copy;
    _state.ResumeTiming();
    layer.updateCosts(*master.getCostmap(), 0, 0, size, size);
  }
}
BENCHMARK_TEMPLATE(squared_obstacle, BMSobLayer)
    ->Args({100, 50, 0})
    ->Args({100, 50, 10})
    ->Args({100, 50, 20})
    ->Args({100, 50, 30})
    ->Args({100, 50, 40})
    ->Args({100, 50, 50})
    ->Args({100, 50, 60})
    ->Args({100, 50, 70})
    ->Args({100, 50, 80})
    ->Args({100, 50, 90})
    ->Args({1000, 5, 0})
    ->Args({1000, 5, 10})
    ->Args({1000, 5, 20})
    ->Args({1000, 5, 30})
    ->Args({1000, 5, 40})
    ->Args({1000, 5, 50})
    ->Args({1000, 5, 60})
    ->Args({1000, 5, 70})
    ->Args({1000, 5, 80})
    ->Args({1000, 5, 90});
BENCHMARK_TEMPLATE(squared_obstacle, BMInflationLayer)
    ->Args({100, 50, 0})
    ->Args({100, 50, 10})
    ->Args({100, 50, 20})
    ->Args({100, 50, 30})
    ->Args({100, 50, 40})
    ->Args({100, 50, 50})
    ->Args({100, 50, 60})
    ->Args({100, 50, 70})
    ->Args({100, 50, 80})
    ->Args({100, 50, 90})
    ->Args({1000, 5, 0})
    ->Args({1000, 5, 10})
    ->Args({1000, 5, 20})
    ->Args({1000, 5, 30})
    ->Args({1000, 5, 40})
    ->Args({1000, 5, 50})
    ->Args({1000, 5, 60})
    ->Args({1000, 5, 70})
    ->Args({1000, 5, 80})
    ->Args({1000, 5, 90});

template <typename _Layer>
void
pixels(benchmark::State& _state) {
  auto master = make_master(_state);
  _Layer layer(master);
  double dummy;
  layer.updateBounds(0, 0, 0, &dummy, &dummy, &dummy, &dummy);

  // range contains (size, resolution in mm, every nth pixel is blocked)
  const auto size = static_cast<size_t>(_state.range(0));
  const auto pixel = _state.range(2);

  // paint the map
  auto map = master.getCostmap();
  int ii = 0;
  for (size_t cc = 0; cc != size; ++cc)
    for (size_t rr = 0; rr != size; ++rr, ++ii) {
      const auto mask = (ii % pixel) == 0;
      map->setCost(rr, cc, costmap_2d::LETHAL_OBSTACLE * mask);
    }

  // backup the map
  const auto map_copy = *map;
  for (auto _ : _state) {
    _state.PauseTiming();
    *map = map_copy;
    _state.ResumeTiming();
    layer.updateCosts(*master.getCostmap(), 0, 0, size, size);
  }
}
BENCHMARK_TEMPLATE(pixels, BMSobLayer)
    ->Args({100, 50, 101})
    ->Args({100, 50, 51})
    ->Args({100, 50, 41})
    ->Args({100, 50, 31})
    ->Args({100, 50, 21})
    ->Args({100, 50, 11})
    ->Args({100, 50, 6})
    ->Args({100, 50, 3})
    ->Args({100, 50, 2})
    ->Args({1000, 5, 101})
    ->Args({1000, 5, 51})
    ->Args({1000, 5, 41})
    ->Args({1000, 5, 31})
    ->Args({1000, 5, 21})
    ->Args({1000, 5, 11})
    ->Args({1000, 5, 6})
    ->Args({1000, 5, 3})
    ->Args({1000, 5, 2});
BENCHMARK_TEMPLATE(pixels, BMInflationLayer)
    ->Args({100, 50, 101})
    ->Args({100, 50, 51})
    ->Args({100, 50, 41})
    ->Args({100, 50, 31})
    ->Args({100, 50, 21})
    ->Args({100, 50, 11})
    ->Args({100, 50, 6})
    ->Args({100, 50, 3})
    ->Args({100, 50, 2})
    ->Args({1000, 5, 101})
    ->Args({1000, 5, 51})
    ->Args({1000, 5, 41})
    ->Args({1000, 5, 31})
    ->Args({1000, 5, 21})
    ->Args({1000, 5, 11})
    ->Args({1000, 5, 6})
    ->Args({1000, 5, 3})
    ->Args({1000, 5, 2});

int
main(int argc, char** argv) {
  ::benchmark::Initialize(&argc, argv);
  if (::benchmark::ReportUnrecognizedArguments(argc, argv))
    return 1;

  // switch off ros-logging
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Error)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  ::benchmark::RunSpecifiedBenchmarks();
}