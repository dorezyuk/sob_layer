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

#include <pluginlib/class_list_macros.hpp>
#include <ros/node_handle.h>

#include <boost/bind.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>

namespace sob_layer {

constexpr char sob_layer__[] = "[sob_layer] ";

#define SL_DEBUG(args) ROS_DEBUG_STREAM(sob_layer__ << args)
#define SL_INFO(args) ROS_INFO_STREAM(sob_layer__ << args)
#define SL_WARN(args) ROS_WARN_STREAM(sob_layer__ << args)
#define SL_ERROR(args) ROS_ERROR_STREAM(sob_layer__ << args)
#define SL_FATAL(args) ROS_FATAL_STREAM(sob_layer__ << args)

inline bool
is_free(const unsigned char& _c) noexcept {
  return _c < costmap_2d::LETHAL_OBSTACLE;
}

template <typename T>
inline void
throw_if_not_positive(const T& _v, const std::string& _name) {
  if (_v <= 0)
    throw std::runtime_error(_name + " must be positive");
}

template <typename T>
inline void
throw_if_positive(const T& _v, const std::string& _name) {
  if (_v > 0)
    throw std::runtime_error(_name + " must be negative or zero");
}

void
SobLayer::onInitialize() {
  SL_INFO("initializing...");
  // we cannot be outdated
  enabled_ = true;
  current_ = true;

  // reinflate always after the init
  need_reinflation_ = true;

  // match the parrent size
  matchSize();

  ros::NodeHandle nh("~/" + name_);

  // load the config
  inscribed_radius_ = nh.param("inscribed_radius", -1);
  use_auto_inscribed_radius_ = inscribed_radius_ <= 0;

  // setup the config server
  config_server_.reset(new config_server(nh));
  config_server_->setCallback(
      boost::bind(&SobLayer::reconfigureCallback, this, _1, _2));
  SL_INFO("initialized");
}

void
SobLayer::matchSize() {
  std::lock_guard<std::mutex> lock(m_);

  // adjust the size of our map
  const auto master = layered_costmap_->getCostmap();

  // get the target size
  const auto size_x = master->getSizeInCellsX();
  const auto size_y = master->getSizeInCellsY();

  // resize our map
  need_reinflation_ |= map_x_.size() != size_x * size_y;
  map_x_.resize(size_x * size_y);

  // fetch the new resolution and update the cache
  const auto new_resolution = master->getResolution();
  need_reinflation_ |= resolution_ != new_resolution;
  resolution_ = new_resolution;

  // allocate enough space for one row
  // z and v follow the paper
  // (http://cs.brown.edu/people/pfelzens/papers/dt-final.pdf)
  v.resize(size_x);
  z.resize(size_x + 1);

  SL_INFO("resized to " << map_x_.size());
}

void
SobLayer::onFootprintChanged() {
  std::lock_guard<std::mutex> lock(m_);

  // allow the user to set his own radius
  if (!use_auto_inscribed_radius_)
    return;

  const auto new_inscribed_radius = layered_costmap_->getInscribedRadius();

  need_reinflation_ |= inscribed_radius_ != new_inscribed_radius;
  inscribed_radius_ = new_inscribed_radius;
}

void
SobLayer::reconfigureCallback(config_type& _config, uint32_t _level) {
  std::lock_guard<std::mutex> lock(m_);

  // if the inflation radius has been changed, we need to redo everything
  need_reinflation_ |= inflation_radius_ != _config.inflation_radius;
  need_reinflation_ |= decay_ != -_config.cost_scaling_factor;

  inflation_radius_ = _config.inflation_radius;
  decay_ = -_config.cost_scaling_factor;

  enabled_ = _config.enabled;
  // let the user know
  SL_INFO("enabled: " << std::boolalpha << _config.enabled);

  // warn the user for unsupported parameters
  ROS_WARN_STREAM_COND(_config.inflate_unknown,
                       sob_layer__ << "inflate_unknown unsupported");
}

void
SobLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                       double* min_x, double* min_y, double* max_x,
                       double* max_y) {
  std::lock_guard<std::mutex> lock(m_);

  // skip any action
  if (!enabled_)
    return;

  // as in the original implementation
  if (need_reinflation_) {
    SL_INFO("reinflating");

    // update the cache
    computeCache();

    // we need to repaint everything!
    const auto master = layered_costmap_->getCostmap();

    // get the size, otherwise it gets unreadable
    const auto size_x = master->getSizeInCellsX();
    const auto size_y = master->getSizeInCellsY();

    double x, y;

    // get the lower edge
    master->mapToWorld(0, 0, x, y);

    *min_x = std::min(x, *min_x);
    *min_y = std::min(y, *min_y);

    // get the upper edge
    master->mapToWorld(size_x, size_y, x, y);

    *max_x = std::max(x, *max_x);
    *max_y = std::max(y, *max_y);
    need_reinflation_ = false;
  }
  else {
    // just repaint what was passed to us
    *min_x -= inflation_radius_;
    *min_y -= inflation_radius_;
    *max_x += inflation_radius_;
    *max_y += inflation_radius_;
  }
}

void
SobLayer::verticalSwipe(const Costmap2D& _master, int dist, int min_i,
                        int min_j, int max_i, int max_j) noexcept {
  const auto grid_m = _master.getCharMap();

  // init the first row: its either 0 or dist
  {
    // don't polute the namespace
    // dd is the destination iterator, ss the source iterator
    const auto dd_end = map_x_.begin() + _master.getIndex(max_i, min_j);
    auto dd = map_x_.begin() + _master.getIndex(min_i, min_j);
    auto ss = grid_m + _master.getIndex(min_i, min_j);
    // https://gcc.gnu.org/onlinedocs/gcc/Loop-Specific-Pragmas.html
    // we know that the source and destination domains never overlap
#pragma GCC ivdep
    for (; dd != dd_end; ++dd, ++ss)
      *dd = is_free(*ss) * dist;
  }

  // swipe up, writing from lower (ll) to upper (uu)
  for (auto jj_ll = min_j, jj_uu = jj_ll + 1; jj_uu != max_j;
       ++jj_ll, ++jj_uu) {
    // init the iterators. dd is destination and ss source.
    auto dd = map_x_.begin() + _master.getIndex(min_i, jj_uu);
    auto ss = map_x_.begin() + _master.getIndex(min_i, jj_ll);
    auto gg = grid_m + _master.getIndex(min_i, jj_uu);
    const auto dd_end = map_x_.begin() + _master.getIndex(max_i, jj_uu);

    // branchless formulation saying its either 0 or the previos incremented
    // we know also that the domains never overlap, so lets convice the compiler
#pragma GCC ivdep
    for (; dd != dd_end; ++dd, ++ss, ++gg)
      *dd = is_free(*gg) * (*ss + 1);
  }

  // swipe down, writing from upper (uu) to lower (ll)
  for (auto jj_uu = max_j - 1, jj_ll = jj_uu - 1; jj_ll >= min_j;
       --jj_uu, --jj_ll) {
    // again get the iterators for the inner loops. dd and ss - you know it
    auto dd = map_x_.begin() + _master.getIndex(min_i, jj_ll);
    auto ss = map_x_.begin() + _master.getIndex(min_i, jj_uu);
    const auto dd_end = map_x_.begin() + _master.getIndex(max_i, jj_ll);
    const auto ss_end = map_x_.begin() + _master.getIndex(max_i, jj_uu);

    // gcc auto-vectorization seems to fail with std::min(*dd, *ss + 1)
    // so we do it step-wise and abuse the v vector as temp-storage
#pragma GCC ivdep
    for (auto vv = v.begin(); ss != ss_end; ++ss, ++vv)
      *vv = *ss + 1;

      // min of current value and the uppers increment
#pragma GCC ivdep
    for (auto vv = v.begin(); dd != dd_end; ++dd, ++vv)
      *dd = std::min(*dd, *vv);
  }
}

double inline parabolaIntersection(int x_1, int y_1, int x_2,
                                   int y_2) noexcept {
  return ((y_1 + std::pow(x_1, 2)) - (y_2 + std::pow(x_2, 2))) /
         (2 * x_1 - 2 * x_2);
}

void
SobLayer::horizontalSwipe(Costmap2D& _master, int dist, int min_i, int min_j,
                          int max_i, int max_j) {
  const auto grid_m = _master.getCharMap();
  const auto qq_max = max_i - min_i;
  const auto sq_dist = std::pow(dist, 2);
  int k = 0;

  // follows http://cs.brown.edu/people/pfelzens/papers/dt-final.pdf
  for (auto jj = min_j; jj != max_j; ++jj) {
    // const-elements
    // calculate the start and end indices for the current row (rr)
    const auto max_rr = _master.getIndex(max_i, jj);
    const auto min_rr = _master.getIndex(0, jj);

    // init vars
    v[0] = min_i;
    z[0] = std::numeric_limits<double>::lowest();
    z[1] = std::numeric_limits<double>::max();
    k = 0;

    // init the first element of a row, since we start at 1
    // if the first element is out of range, make sure that its 'parabola'
    // starts at inf, so the intersection is below 0
    map_x_[min_rr + min_i] = std::pow(map_x_[min_rr + min_i], 2);
    if (map_x_[min_rr + min_i] >= sq_dist)
      map_x_[min_rr + min_i] = std::numeric_limits<int>::max();

    for (int ii = min_i + 1; ii != max_i; ++ii) {
      // ignore everything we don't care about
      auto& ii_v = map_x_[min_rr + ii];
      if (ii_v >= dist)
        continue;

      ii_v = std::pow(ii_v, 2);

      auto s = parabolaIntersection(ii, ii_v, v[k], map_x_[min_rr + v[k]]);
      while (s <= z[k]) {
        --k;
        s = parabolaIntersection(ii, ii_v, v[k], map_x_[min_rr + v[k]]);
      }
      ++k;
      v[k] = ii;
      z[k] = s;
      z[k + 1] = std::numeric_limits<double>::max();
    }

    if (k == 0 && map_x_[min_rr + min_i] >= sq_dist)
      continue;

    auto k_end = k + 1;
    k = 0;

    // skip all intervals ending in the negative
    for (; k != k_end; ++k)
      if (z[k + 1] >= min_i)
        break;

    // skip all intervals starting to high
    for (; k_end > k; --k_end)
      if (z[k_end - 1] < max_i)
        break;

    // clamp the start and end to the range
    z[k] = std::max<double>(z[k], min_i);
    z[k_end] = std::min<double>(z[k_end], max_i);

    // apply ceil to everything relevant [k, k_end]
    const auto zz_end = z.begin() + k_end + 1;
    for (auto zz = z.begin() + k; zz != zz_end; ++zz)
      *zz = std::ceil(*zz);

    for (; k != k_end; ++k) {
      // init the helpers
      const auto row = static_cast<size_t>(std::sqrt(map_x_[min_rr + v[k]]));
      const auto& cache_row = cache_.at(row);
      const auto cache_size = (int)cache_row.size();

      const int v_l = v[k] - cache_size / 2;
      const int q_l = std::max<int>(z[k], v_l);
      const int c_l = q_l - v_l;
      const int c_u = std::min<int>(cache_size, z[k + 1] - v_l);

      // skip degenerated intervals
      if (c_l >= c_u)
        continue;

      auto dd = grid_m + min_rr + q_l;
      auto ss = cache_row.begin() + c_l;
      const auto ss_end = cache_row.begin() + c_u;

      // copy the data - again no overlap possible
#pragma GCC ivdep
      for (; ss != ss_end; ++ss, ++dd)
        *dd = std::max(*dd, *ss);

      // if we are on a horizontal line, copy the last cost
      const auto sq_row = map_x_[min_rr + v[k]];
      --ss;
      for (; k != k_end - 1; ++k, ++dd) {
        if (z[k + 2] - z[k + 1] > 1 || map_x_[min_rr + v[k + 1]] != sq_row)
          break;
        *dd = std::max(*dd, *ss);
      }
    }
  }
}

void
SobLayer::updateCosts(costmap_2d::Costmap2D& _master, int min_i, int min_j,
                      int max_i, int max_j) {
  std::lock_guard<std::mutex> lock(m_);

  if (!enabled_ || cache_.empty() || min_i >= max_i || min_j >= max_j)
    return;

  // get the cell-distance
  const int dist = _master.cellDistance(inflation_radius_) + 1;

  // do a vertical swipe
  verticalSwipe(_master, dist, min_i, min_j, max_i, max_j);

  // do a horizontal swipe
  horizontalSwipe(_master, dist, min_i, min_j, max_i, max_j);
}

constexpr double epsilon = 1e-9;

SobLayer::cost_type
SobLayer::computeCost(const double _distance) noexcept {
  // adjusted from original costmap_2d::InflationLayer
  if (_distance == 0)
    return costmap_2d::LETHAL_OBSTACLE;
  else if (_distance <= inscribed_radius_ && _distance <= inflation_radius_)
    return costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  else if (_distance <= inflation_radius_ + epsilon) {
    const auto factor = std::exp(decay_ * (_distance - inscribed_radius_));
    return static_cast<cost_type>(
        (costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
  }
  return costmap_2d::FREE_SPACE;
}

void
SobLayer::computeCacheImpl() {
  // clear the current cache
  cache_.clear();

  // check the parameters - functions do what they state
  throw_if_not_positive(resolution_, "resolution");
  throw_if_not_positive(inscribed_radius_, "inscribed radius");
  throw_if_not_positive(inflation_radius_, "inflated radius");
  throw_if_positive(decay_, "decay");

  SL_INFO("updating the cache");

  // compute the cached cost grid
  const auto size = std::ceil(inflation_radius_ / resolution_) + 1;
  cache_.resize(size);
  for (size_t cc = 0; cc != size; ++cc) {
    // get the current row
    auto& row = cache_.at(cc);
    for (size_t rr = 0; rr != size; ++rr) {
      // get the metric distance. note: std::hypot returns double
      const auto distance = std::hypot(rr, cc) * resolution_;

      // get the corresponding cost
      const auto cost = computeCost(distance);

      // we are done with the row once we see FREE_SPACE
      if (cost == costmap_2d::FREE_SPACE)
        break;
      row.push_back(cost);
    }
    // duplicate the cost so [a, b, c] becomes to [c, b, a, b, c]
    // this simplifies the copy-operation later on
    // better safe then sorry (we need this for std::prev)
    if (!row.empty())
      row.insert(row.begin(), row.rbegin(), std::prev(row.rend()));
  }
}

void
SobLayer::computeCache() noexcept {
  if (!need_reinflation_)
    return;

  try {
    computeCacheImpl();
  }
  catch (const std::runtime_error& _ex) {
    SL_WARN("failed to compute the cache: " << _ex.what());
  }
}

}  // namespace sob_layer

PLUGINLIB_EXPORT_CLASS(sob_layer::SobLayer, costmap_2d::Layer);
