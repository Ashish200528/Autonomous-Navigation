#include <gtest/gtest.h>
#include <limits>
#include "anc_nav2_sequential_controller/primitive_controller.hpp"
using anc_nav2_sequential_controller::PrimitiveController;

TEST(Primitives, StartupAndPrimitiveSwitchRequireBothStopConditions) {
  PrimitiveController p;
  EXPECT_DOUBLE_EQ(p.update(0,0,2,0,0,0).v,0);
  EXPECT_DOUBLE_EQ(p.update(1.24,0,2,0,0,0).v,0);
  EXPECT_GT(p.update(1.25,0,2,0,0,0).v,0);
  EXPECT_DOUBLE_EQ(p.update(2,.5,2,0,.08,0).w,0);
  EXPECT_DOUBLE_EQ(p.update(3.3,.5,2,0,.01,0).w,0);
  EXPECT_DOUBLE_EQ(p.update(3.4,.5,2,0,0,0).w,0);
  EXPECT_DOUBLE_EQ(p.update(3.89,.5,2,0,0,0).w,0);
  EXPECT_GT(p.update(3.91,.5,2,0,0,0).w,0);
}
TEST(Primitives, TurnStopsBeforeHeadingCrossingAndNeverReversesDirectly) {
  PrimitiveController p;p.update(0,.5,2,0,0,0);
  EXPECT_GT(p.update(1.25,.5,2,0,0,0).w,0);
  EXPECT_DOUBLE_EQ(p.update(2,.039,2,0,0,.05).w,0);
  EXPECT_DOUBLE_EQ(p.update(2.5,-.5,2,0,0,-.02).w,0);
  EXPECT_DOUBLE_EQ(p.update(3.3,-.5,2,0,0,0).w,0);
  EXPECT_LT(p.update(3.81,-.5,2,0,0,0).w,0);
}
TEST(Primitives, InvalidDataAndClockRewindResetToZero) {
  PrimitiveController p;p.update(0,0,2,0,0,0);p.update(2,0,2,0,0,0);
  EXPECT_DOUBLE_EQ(p.update(3,0,2,0,0,0,false).v,0);
  EXPECT_DOUBLE_EQ(p.update(3.1,0,2,0,0,0).v,0);
  EXPECT_GT(p.update(4.4,0,2,0,0,0).v,0);
  EXPECT_DOUBLE_EQ(p.update(4.5,std::numeric_limits<double>::quiet_NaN(),2,0,0,0).v,0);
  p.update(5,0,2,0,0,0);p.update(7,0,2,0,0,0);
  EXPECT_DOUBLE_EQ(p.update(6,0,2,0,0,0).v,0);
}
TEST(Primitives, DenseChangingTargetsNeverRequestMixedOrReverseMotion) {
  PrimitiveController p;
  for (int i=0;i<2000;++i) {
    const double t=i*.125,e=std::sin(t/4.0);
    auto c=p.update(t,e,2,0,0,0);
    EXPECT_GE(c.v,0);EXPECT_LE(c.v,.08);EXPECT_LE(std::abs(c.w),.05);
    EXPECT_TRUE(c.v==0 || c.w==0);
  }
}
TEST(Primitives, FinalHeadingAndSpeedLimitPreservePureMotion) {
  PrimitiveController p;p.update(0,0,.1,1,0,0);
  auto c=p.update(1.25,0,.1,1,0,0);EXPECT_EQ(c.v,0);EXPECT_GT(c.w,0);
  EXPECT_EQ(p.update(2,0,.1,.02,0,.05).w,0);
  PrimitiveController d;d.update(0,0,2,0,0,0);
  EXPECT_DOUBLE_EQ(d.update(2,0,2,0,0,0,true,.03).v,.03);
  EXPECT_DOUBLE_EQ(d.update(3,0,2,0,0,0,true,0).v,0);
}
