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

#pragma once

#include <costmap_2d/Costmap2DConfig.h>
#include <costmap_2d/InflationPluginConfig.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/layer.h>
#include <dynamic_reconfigure/server.h>

#include <memory>
#include <mutex>
#include <vector>

namespace sob_layer {

using costmap_2d::Costmap2D;
using costmap_2d::Layer;

/**
 * @brief Small-Objects-Big (Sob) Layer
 *
 * Plugin-replacement for the default costmap_2d::InflationLayer.
 *
 * @section Basic
 *
 * The offers a (likely) faster implementation of the default
 * costmap_2d::InflationLayer.
 *
 * The computation is divided into vertical and horizontal swipes.
 *
 * The horizontal swipe (see SobLayer::horizontalSwipe) is performed first and
 * determines the minimum distances to obstacles within each column. This
 * computation is done by the well-known two-pass technique.
 *
 * The vertical swipe (see SobLayer::verticalSwipe) is based on the work of
 * Felzenszwalb and Huttenlocher (2004). The code modified to the particular
 * problem but follows the nomenclature from the original paper.
 *
 * @section Configuration
 *
 * The SobLayer tries to be compatible with the original
 * costmap_2d::InflationLayer.
 *
 * @subsection Parameters
 *
 * Following parameters are supported:
 *
 * @subsubsection enabled (Bool)
 *
 * Defaults to true. If false, the SobLayer::updateCosts and
 * SobLayer::updateBounds will be no-op.
 *
 * @subsubsection inflation_radius (Float)
 *
 * Defaults to 0. Indicates the metric radius of the inflation.
 *
 * @subsubsection cost_scaling_factor (Float)
 *
 * Defaults to 0. Indicates the decay factor for costs in the range
 * [inscribed_radius, inflation_radius]
 *
 * @subsubsection inscribed_radius (Float)
 *
 * Defaults to -1. A negative value will auto-compute the
 * inscribed_radius from the footprint-parameters. This is what the original
 * implementation does. Use a positive value to set your own radius.
 *
 * @subsection Deprectated Parameters
 *
 * @subsubsection inflate_unknown
 *
 * @section References
 *
 * Felzenszwalb, Pedro & Huttenlocher, Daniel. (2004).
 * Distance Transforms of Sampled Functions. Theory of
 * Computing. 8. 10.4086/toc.2012.v008a019.
 *
 */
struct SobLayer : public Layer {
  void
  onInitialize() override;

  void
  updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
               double* min_y, double* max_x, double* max_y) override;

  void
  updateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i,
              int max_j) override;

  void
  matchSize() override;

  void
  onFootprintChanged() override;

protected:
  using config_type = costmap_2d::InflationPluginConfig;
  using config_server = dynamic_reconfigure::Server<config_type>;
  using config_server_ptr = std::unique_ptr<config_server>;

  using cost_type = unsigned char;

  void
  reconfigureCallback(config_type& _config, uint32_t level);

  void
  computeCache() noexcept;

  void
  computeCacheImpl();

  cost_type
  computeCost(double _distance) noexcept;

  void
  verticalSwipe(const Costmap2D& _master, int dist, int min_i, int min_j,
                int max_i, int max_j) noexcept;

  void
  horizontalSwipe(Costmap2D& _master, int dist, int min_i, int min_j, int max_i,
                 int max_j);

  std::mutex m_;

  std::vector<int> map_x_;
  std::vector<int> map_x_sq_;
  std::vector<int> v;
  std::vector<double> z;
  std::vector<std::vector<cost_type>> cache_;

  double inscribed_radius_ = 0;
  double inflation_radius_ = 0;
  double resolution_ = 0;
  double decay_ = 0;
  bool need_reinflation_ = true;
  bool use_auto_inscribed_radius_ = true;
  bool inflate_unknown_ = true;

  config_server_ptr config_server_;
};

}  // namespace sob_layer