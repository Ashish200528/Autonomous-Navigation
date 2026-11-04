#include <gtest/gtest.h>
#include <limits>
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
using namespace nav2_costmap_2d;
class ObservationLayer : public CostmapLayer {
public:
  ObservationLayer() {enabled_=true;setDefaultValue(NO_INFORMATION);resizeMap(8,8,.1,0,0);resetMaps();}
  void reset() override {resetMaps();}
  bool isClearable() override {return true;}
  void updateBounds(double,double,double,double*,double*,double*,double*) override {}
  void updateCosts(Costmap2D&,int,int,int,int) override {}
  void fuse(Costmap2D &master) {updateWithMax(master,0,0,8,8);}
};
TEST(SensorFusion, UnknownIsNotFreeAndNeitherSourceClearsAnObstacle) {
  Costmap2D master(8,8,.1,0,0,NO_INFORMATION);ObservationLayer rays;
  // Sensor-built SLAM map provides observed free space and an observed obstacle.
  master.setCost(1,1,FREE_SPACE);master.setCost(2,2,LETHAL_OBSTACLE);
  // Current rays provide new free space, a conflicting obstacle, and a ray gap.
  rays.setCost(3,3,FREE_SPACE);rays.setCost(1,1,LETHAL_OBSTACLE);
  rays.setCost(2,2,FREE_SPACE);rays.setCost(4,4,NO_INFORMATION);
  rays.fuse(master);
  EXPECT_EQ(master.getCost(1,1),LETHAL_OBSTACLE);
  EXPECT_EQ(master.getCost(2,2),LETHAL_OBSTACLE);
  EXPECT_EQ(master.getCost(3,3),FREE_SPACE);
  EXPECT_EQ(master.getCost(4,4),NO_INFORMATION);
}
TEST(Footprint, PredictionRemainsBlockedByUnknownAndOccupiedCells) {
  Costmap2D map(8,8,.1,0,0,FREE_SPACE);
  std::vector<geometry_msgs::msg::Point> footprint;
  for(auto xy: {std::pair<double,double>{-.1,-.1},{.1,-.1},{.1,.1},{-.1,.1}}) {
    geometry_msgs::msg::Point p;p.x=xy.first;p.y=xy.second;footprint.push_back(p);
  }
  FootprintCollisionChecker<Costmap2D*> checker(&map);
  EXPECT_EQ(checker.footprintCostAtPose(.4,.4,0,footprint),FREE_SPACE);
  map.setCost(5,5,NO_INFORMATION);
  EXPECT_EQ(checker.footprintCostAtPose(.4,.4,0,footprint),NO_INFORMATION);
  map.setCost(5,5,LETHAL_OBSTACLE);
  EXPECT_EQ(checker.footprintCostAtPose(.4,.4,0,footprint),LETHAL_OBSTACLE);
}


#include "laser_geometry/laser_geometry.hpp"
TEST(NoReturnProjection, ExactMaximumIsDroppedButSupportedInfPreprocessingClears) {
  sensor_msgs::msg::LaserScan scan;scan.angle_min=0;scan.angle_max=0;
  scan.angle_increment=1;scan.range_min=.12;scan.range_max=6.;scan.ranges={6.};
  laser_geometry::LaserProjection projection;sensor_msgs::msg::PointCloud2 cloud;
  projection.projectLaser(scan,cloud);EXPECT_EQ(cloud.width,0u);
  // Nav2 laserScanValidInfCallback uses this epsilon for positive infinity.
  scan.ranges={scan.range_max-.0001f};
  projection.projectLaser(scan,cloud);EXPECT_EQ(cloud.width,1u);
  scan.ranges={std::numeric_limits<float>::quiet_NaN()};
  projection.projectLaser(scan,cloud);EXPECT_EQ(cloud.width,0u);
}
