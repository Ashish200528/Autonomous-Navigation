#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include "anc_nav2_sequential_controller/primitive_controller.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace anc_nav2_sequential_controller {
class SequentialController : public nav2_core::Controller {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    node_=parent;if (node_.expired()) throw std::runtime_error("controller lifecycle node missing");
    tf_=tf;costmap_=costmap;name_=name;
    if (!node()->has_parameter(name+".lookahead_dist")) node()->declare_parameter(name+".lookahead_dist",.60);
    lookahead_=node()->get_parameter(name+".lookahead_dist").as_double();
    if (!std::isfinite(lookahead_) || lookahead_<.35 || lookahead_>1.) throw std::invalid_argument("invalid sequential lookahead");
    fixedParameter("linear_speed_mps",PrimitiveController::linear_cap);
    fixedParameter("angular_speed_radps",PrimitiveController::angular_cap);
    fixedParameter("path_heading_limit_rad",PrimitiveController::drive_heading_limit);
    fixedParameter("turn_stop_error_rad",PrimitiveController::turn_stop_error);
    fixedParameter("settle_wall_s",PrimitiveController::settle_seconds);
    state_pub_=node()->create_publisher<std_msgs::msg::String>("sequential_controller/state",10);
    RCLCPP_INFO(node()->get_logger(),"Sequential Nav2 proposals only; external safety guard retains actuator authority");
  }
  void cleanup() override {std::lock_guard<std::mutex> lock(mutex_);plan_.poses.clear();state_pub_.reset();node_.reset();}
  void activate() override {state_pub_->on_activate();reset();}
  void deactivate() override {reset();state_pub_->on_deactivate();}
  void reset() override {std::lock_guard<std::mutex> lock(mutex_);engine_.reset();}
  bool cancel() override {reset();return true;}
  void setPlan(const nav_msgs::msg::Path &path) override {
    std::lock_guard<std::mutex> lock(mutex_);
    bool changed=plan_.poses.empty() || path.poses.empty() || plan_.header.frame_id!=path.header.frame_id;
    if (!changed) {
      const auto &a=plan_.poses.back().pose;const auto &b=path.poses.back().pose;
      changed=std::hypot(a.position.x-b.position.x,a.position.y-b.position.y)>1e-5 ||
              std::abs(wrap(tf2::getYaw(a.orientation)-tf2::getYaw(b.orientation)))>1e-5;
    }
    plan_=path;if (changed) engine_.reset();
  }
  void setSpeedLimit(const double &limit,const bool &percentage) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!std::isfinite(limit) || limit<0) speed_limit_=0;
    else if (limit==0) speed_limit_=PrimitiveController::linear_cap; // Nav2 NO_SPEED_LIMIT convention.
    else speed_limit_=std::clamp(percentage ? PrimitiveController::linear_cap*limit/100. : limit,0.,PrimitiveController::linear_cap);
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &pose,const geometry_msgs::msg::Twist &speed,
      nav2_core::GoalChecker *) override {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    auto fail=[&](const std::string &reason) {engine_.stop(now);throw nav2_core::NoValidControl(reason);};
    if (plan_.poses.empty() || plan_.header.frame_id.empty()) fail("sequential plan missing");
    const double age=(node()->now()-rclcpp::Time(pose.header.stamp)).seconds();
    if (!std::isfinite(age) || age<-.05 || age>.25) fail("sequential robot pose stale");
    if (!finitePose(pose.pose) || !std::isfinite(speed.linear.x) || !std::isfinite(speed.angular.z)) fail("sequential pose or speed invalid");
    geometry_msgs::msg::PoseStamped global;
    if (!nav2_util::transformPoseInTargetFrame(pose,global,*tf_,plan_.header.frame_id,.1)) fail("sequential map transform missing");
    const auto &goal=plan_.poses.back().pose;
    if (!finitePose(goal) || !finitePose(global.pose)) fail("sequential goal invalid");
    const auto &p=global.pose.position;size_t nearest=0;double nearest_distance=std::numeric_limits<double>::infinity();
    for (size_t i=0;i<plan_.poses.size();++i) {
      const auto &point=plan_.poses[i].pose.position;
      if (!std::isfinite(point.x) || !std::isfinite(point.y)) fail("sequential path point invalid");
      const double d=std::hypot(point.x-p.x,point.y-p.y);
      if (d<nearest_distance) {nearest_distance=d;nearest=i;}
    }
    size_t target=nearest;double arc=nearest_distance;
    while (target+1<plan_.poses.size() && arc<lookahead_) {
      const auto &a=plan_.poses[target].pose.position;const auto &b=plan_.poses[target+1].pose.position;
      arc+=std::hypot(b.x-a.x,b.y-a.y);++target;
    }
    const auto &carrot=plan_.poses[target].pose.position;const double yaw=tf2::getYaw(global.pose.orientation);
    const double heading=wrap(std::atan2(carrot.y-p.y,carrot.x-p.x)-yaw);
    const double distance=std::hypot(goal.position.x-p.x,goal.position.y-p.y);
    const double goal_heading=wrap(tf2::getYaw(goal.orientation)-yaw);
    auto command=engine_.update(now,heading,distance,goal_heading,speed.linear.x,speed.angular.z,true,speed_limit_);
    if (command.v!=0 || command.w!=0) {
      geometry_msgs::msg::PoseStamped local;
      if (!costmap_->isCurrent() || !nav2_util::transformPoseInTargetFrame(pose,local,*tf_,costmap_->getGlobalFrameID(),.1)) fail("sequential local costmap unavailable");
      auto *map=costmap_->getCostmap();std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*map->getMutex());
      nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(map);
      const double a=tf2::getYaw(local.pose.orientation);
      for (int i=0;i<=8;++i) {
        const double t=i*.1;
        const double cost=checker.footprintCostAtPose(local.pose.position.x+command.v*t*std::cos(a),
          local.pose.position.y+command.v*t*std::sin(a),a+command.w*t,costmap_->getRobotFootprint());
        if (cost<0 || cost>=nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
          std_msgs::msg::String blocked;std::ostringstream info;
          info<<"{\"reason\":\"footprint_rejected\",\"cost\":"<<cost
              <<",\"x\":"<<local.pose.position.x<<",\"y\":"<<local.pose.position.y
              <<",\"yaw\":"<<a<<",\"prediction_s\":"<<t
              <<",\"map_origin\":["<<map->getOriginX()<<","<<map->getOriginY()<<"]"
              <<",\"inscribed_radius\":"<<costmap_->getLayeredCostmap()->getInscribedRadius()
              <<",\"footprint\":[";
          bool first=true;for (const auto &point:costmap_->getRobotFootprint()) {
            if (!first) {info<<",";}
            first=false;info<<"["<<point.x<<","<<point.y<<"]";
          }
          info<<"]}";blocked.data=info.str();state_pub_->publish(blocked);
          fail("sequential footprint occupied or unknown");
        }
      }
    }
    std_msgs::msg::String state;std::ostringstream out;
    out<<"{\"phase\":\""<<(engine_.phase()==Phase::TURN ? "turn" : engine_.phase()==Phase::DRIVE ? "drive" : "settle")
       <<"\",\"heading_error_rad\":"<<heading<<",\"goal_distance_m\":"<<distance
       <<",\"goal_heading_error_rad\":"<<goal_heading<<",\"proposed_v\":"<<command.v<<",\"proposed_w\":"<<command.w
       <<",\"runtime_truth_used\":false,\"actuator_authority\":false}";
    state.data=out.str();state_pub_->publish(state);
    geometry_msgs::msg::TwistStamped result;result.header=pose.header;
    result.twist.linear.x=command.v;result.twist.angular.z=command.w;return result;
  }
private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node() const {
    auto value=node_.lock();if (!value) throw std::runtime_error("controller lifecycle node expired");return value;
  }
  static double wrap(double a) {return std::atan2(std::sin(a),std::cos(a));}
  static bool finitePose(const geometry_msgs::msg::Pose &p) {
    const auto &q=p.orientation;const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    return std::isfinite(p.position.x) && std::isfinite(p.position.y) && std::isfinite(norm) && norm>.99 && norm<1.01;
  }
  void fixedParameter(const std::string &suffix,double expected) {
    const auto key=name_+"."+suffix;rcl_interfaces::msg::ParameterDescriptor d;d.read_only=true;
    if (!node()->has_parameter(key)) node()->declare_parameter(key,expected,d);
    if (std::abs(node()->get_parameter(key).as_double()-expected)>1e-9) throw std::invalid_argument("bounded controller constant mismatch: "+key);
  }
  std::mutex mutex_;PrimitiveController engine_;nav_msgs::msg::Path plan_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr state_pub_;
  std::string name_;double lookahead_{.6},speed_limit_{PrimitiveController::linear_cap};
};
}  // namespace anc_nav2_sequential_controller
PLUGINLIB_EXPORT_CLASS(anc_nav2_sequential_controller::SequentialController,nav2_core::Controller)
