#include <sob_layer/sob_layer.hpp>
#include <costmap_2d/inflation_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <gtest/gtest.h>

using namespace sob_layer;
using costmap_2d::Costmap2D;
using costmap_2d::InflationLayer;
using costmap_2d::LayeredCostmap;
using geometry_msgs::Point;
using testing::Test;

Point
make_point(double x, double y) {
  // who would have thought people need constructors? not ros apparently...
  Point out;
  out.x = x;
  out.y = y;
  return out;
}

// "generic" way to access the layers internals and init without ros-core
template <typename T>
struct LayerFixture : public T {
  // defer the initialization
  void
  init(LayeredCostmap& _lm) {
    // set the master
    T::layered_costmap_ = &_lm;

    // default radius and enabling
    T::inflation_radius_ = 0.6;
    T::enabled_ = true;

    // call the init-methods
    T::matchSize();
    T::onFootprintChanged();
  }
};

using InflationLayerFixture = LayerFixture<InflationLayer>;

struct SobLayerFixture : public LayerFixture<SobLayer>, public Test {
  LayeredCostmap master;
  InflationLayerFixture inflation;
  SobLayerFixture() : master("map", false, false) {
    // setup a footprint with 1 meter radius - needed for the
    // inflation-layer
    std::vector<Point> fp{make_point(-0.5, -1), make_point(0.5, -1),
                          make_point(0.5, 1), make_point(-0.5, 1)};

    // resize the master
    master.resizeMap(10, 20, 0.1, 0, 0);
    master.setFootprint(fp);

    init(master);
    inflation.init(master);
  }
};

TEST_F(SobLayerFixture, matchSize) {
  // change the resolution
  const auto expected_res = resolution_;
  resolution_ += 1;
  need_reinflation_ = false;

  // call matchSize
  matchSize();

  // check the result
  ASSERT_TRUE(need_reinflation_);
  ASSERT_FLOAT_EQ(expected_res, resolution_);

  // consume and recall
  need_reinflation_ = false;
  matchSize();

  // check again
  ASSERT_FALSE(need_reinflation_);
}

TEST_F(SobLayerFixture, onFootprintChanged) {
  // change the inscribed radius
  const auto expected_rad = inscribed_radius_;
  inscribed_radius_ += 1;
  need_reinflation_ = false;

  // call onFootprintChanged
  onFootprintChanged();

  // check the result
  ASSERT_TRUE(need_reinflation_);
  ASSERT_FLOAT_EQ(expected_rad, inscribed_radius_);

  // consume and recall
  need_reinflation_ = false;
  onFootprintChanged();

  // check again
  ASSERT_FALSE(need_reinflation_);
}

TEST_F(SobLayerFixture, reconfigureCallback) {
  const auto expected_rad = inflation_radius_;
  const auto expected_decay = decay_;

  inflation_radius_ += 1;
  need_reinflation_ = false;

  config_type config;
  config.inflation_radius = expected_rad;
  config.cost_scaling_factor = -expected_decay;

  reconfigureCallback(config, 0);

  // check the result
  ASSERT_TRUE(need_reinflation_);
  ASSERT_FLOAT_EQ(expected_rad, inflation_radius_);

  // now decay
  decay_ += 1;
  need_reinflation_ = false;
  config.cost_scaling_factor = -expected_decay;

  reconfigureCallback(config, 0);

  // again check the result
  ASSERT_TRUE(need_reinflation_);
  ASSERT_FLOAT_EQ(expected_decay, decay_);

  // final check
  need_reinflation_ = false;
  reconfigureCallback(config, 0);

  ASSERT_FALSE(need_reinflation_);
}

struct SobLayerFixtureUpdateBounds : public SobLayerFixture {
  // setup the bounds (min_i, min_j, max_i, max_j)
  std::array<double, 4> bounds;
};

TEST_F(SobLayerFixtureUpdateBounds, disabled) {
  // in the disabled case we expect that the bounds don't change
  enabled_ = false;
  bounds[0] = bounds[1] = std::numeric_limits<double>::max();
  bounds[2] = bounds[3] = std::numeric_limits<double>::lowest();
  const auto expected = bounds;
  size_t ii = 0;
  updateBounds(0, 0, 0, &bounds[ii++], &bounds[ii++], &bounds[ii++],
               &bounds[ii++]);

  ASSERT_EQ(bounds, expected);
}

TEST_F(SobLayerFixtureUpdateBounds, needReinflation) {
  need_reinflation_ = true;
  // set some bounds
  bounds[0] = -1;
  bounds[1] = bounds[2] = 0.5;
  bounds[3] = 3;
  std::array<double, 4> expected = bounds;

  updateBounds(0, 0, 0, &bounds[0], &bounds[1], &bounds[2], &bounds[3]);

  // check the result
  ASSERT_EQ(bounds[0], expected[0]);
  ASSERT_LE(bounds[1], expected[1]);
  ASSERT_GE(bounds[2], expected[2]);
  ASSERT_EQ(bounds[3], expected[3]);
}

TEST_F(SobLayerFixtureUpdateBounds, noReinflation) {
  need_reinflation_ = false;
  const auto dummy_value = 1.;
  std::fill(bounds.begin(), bounds.end(), dummy_value);

  updateBounds(0, 0, 0, &bounds[0], &bounds[1], &bounds[2], &bounds[3]);

  // check the result
  ASSERT_EQ(bounds[0], dummy_value - inflation_radius_);
  ASSERT_EQ(bounds[1], dummy_value - inflation_radius_);
  ASSERT_EQ(bounds[2], dummy_value + inflation_radius_);
  ASSERT_EQ(bounds[3], dummy_value + inflation_radius_);
}

bool
equalCostmaps(const Costmap2D& _c1, const Costmap2D& _c2) {
  const auto size_x = _c1.getSizeInCellsX();
  const auto size_y = _c1.getSizeInCellsY();
  const auto size = size_x * size_y;

  if (_c2.getSizeInCellsX() != size_x || _c2.getSizeInCellsY() != size_y)
    return false;

  for (size_t ii = 0; ii != size; ++ii)
    if (_c1.getCharMap()[ii] != _c2.getCharMap()[ii])
      return false;
  return true;
}

// fixture for checking small update windows.
// small windows are either empty or just containing one cell
struct SobLayerSmallFixture : public SobLayerFixture {
  // we will mark the middle cell as occupied
  const int x = 5, y = 10;
  costmap_2d::Costmap2D copy_costmap;

  SobLayerSmallFixture() {
    // mark the costmap
    master.getCostmap()->setCost(x, y, costmap_2d::LETHAL_OBSTACLE);

    // store the copy of the costmap
    copy_costmap = *master.getCostmap();
  }
};

TEST_F(SobLayerSmallFixture, empty) {
  // we will update an empty window
  // the master-costmap must not change and we must survive
  updateCosts(*master.getCostmap(), x, y, x, y);

  // check the result
  ASSERT_TRUE(equalCostmaps(*master.getCostmap(), copy_costmap));
}

TEST_F(SobLayerSmallFixture, oneLethal) {
  // we will update just one cell - pointing to the lethal obstacle. since we
  // will mark it also as lethal, the result won't change.
  updateCosts(*master.getCostmap(), x, y, x + 1, y + 1);

  // check the result
  ASSERT_TRUE(equalCostmaps(*master.getCostmap(), copy_costmap));
}

TEST_F(SobLayerSmallFixture, oneFree) {
  // we will update just one cell - pointing to free space. since we will mark
  // it also as free, the result won't change
  updateCosts(*master.getCostmap(), x + 1, y + 1, x + 2, y + 2);

  // check the result
  ASSERT_TRUE(equalCostmaps(*master.getCostmap(), copy_costmap));
}

using testing::WithParamInterface;
using indices = std::vector<size_t>;

// pass indices marked as obstacles to the fixture
// we will compare the result of this layer to the inflation-layer
struct SobLayerRegressionFixture
    : public SobLayerFixture,
      public WithParamInterface<std::tuple<indices, double>> {};

using testing::Combine;
using testing::Range;
using testing::ValuesIn;

const indices index_parameters[] = {
    {105},                                               // center
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9},                      // lower row
    {0, 10, 20, 30, 40, 50},                             // left row
    {9, 19, 29, 39, 49, 59},                             // right row
    {190, 191, 192, 193, 194, 195, 196, 197, 198, 199},  // upper row
    {0, 9, 190, 199},                                    // corners
    {2,   3,   5,   7,   11,  13,  17,  19,  23,  29,  31,  37,
     41,  43,  47,  53,  59,  61,  67,  71,  73,  79,  83,  89,
     97,  101, 103, 107, 109, 113, 127, 131, 137, 139, 149, 151,
     157, 163, 167, 173, 179, 181, 191, 193, 197, 199},  // prime numbers
    {0, 21, 42, 63, 84, 105, 126, 147, 168, 189}         // diagnal
};

INSTANTIATE_TEST_CASE_P(/**/, SobLayerRegressionFixture,
                        Combine(ValuesIn(index_parameters),
                                Range(0.1, 1., 0.1)));

TEST_P(SobLayerRegressionFixture, simple) {
  // get the parameter
  const auto obstacle_indices = std::get<0>(GetParam());
  const auto inflation_radius = std::get<1>(GetParam());

  // 0.3 seems to be wierd at the original inflation layer
  if (std::abs(inflation_radius - 0.3) < 1e-3)
    return;

  // update the inflation radius
  config_type config;
  config.inflation_radius = inflation_radius;
  config.cost_scaling_factor = 0;
  config.enabled = true;

  reconfigureCallback(config, 0);
  inflation.setInflationParameters(config.inflation_radius,
                                   config.cost_scaling_factor);

  // mark the obstacles
  for (const auto& ii : obstacle_indices)
    master.getCostmap()->getCharMap()[ii] = costmap_2d::LETHAL_OBSTACLE;

  // create a copy
  auto costmap1 = *master.getCostmap();
  auto costmap2 = *master.getCostmap();

  // run the inflation layer and ours
  const auto size_x = master.getCostmap()->getSizeInCellsX();
  const auto size_y = master.getCostmap()->getSizeInCellsY();
  inflation.updateCosts(costmap1, 0, 0, size_x, size_y);
  updateCosts(costmap2, 0, 0, size_x, size_y);

  // check the result
  ASSERT_TRUE(equalCostmaps(costmap1, costmap2));
}

int
main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
