#pragma once
#include <algorithm>
#include <cmath>
#include <optional>

namespace anc_nav2_sequential_controller {
struct Command {double v{0.0}; double w{0.0};};
enum class Phase {SETTLE, DRIVE, TURN};

// Proposal sequencing only. It cannot grant motion authority, certify a scan,
// or publish actuator commands. The independent guard retains all its checks.
class PrimitiveController {
public:
  void reset() {phase_=Phase::SETTLE; stopped_.reset(); slow_.reset(); last_.reset();}
  void stop(double now) {phase_=Phase::SETTLE; stopped_=now; slow_.reset();}
  Phase phase() const {return phase_;}
  static constexpr double linear_cap = .08;
  static constexpr double angular_cap = .05;
  static constexpr double settle_seconds = 1.25;
  static constexpr double slow_hold_seconds = .50;
  // Planning band: at .05 rad/s the existing .12 rad/s^2 smoother adds
  // .01042 rad before zero; the independent fault-stop bound is .05 rad.
  // Include one control period + uncertainty => .09 rad planning allowance.
  // Stop near its midpoint; qualify the actual normal response in Gazebo.
  static constexpr double turn_stop_error = .04;
  static constexpr double drive_heading_limit = .12;

  Command update(double now,double path_error,double goal_distance,double goal_error,
                 double observed_v,double observed_w,bool valid=true,double speed_limit=linear_cap) {
    if (!valid || !std::isfinite(now) || !std::isfinite(path_error) || !std::isfinite(goal_distance) ||
        !std::isfinite(goal_error) || !std::isfinite(observed_v) || !std::isfinite(observed_w) ||
        !std::isfinite(speed_limit) || goal_distance<0 || (last_ && now<*last_)) {
      reset();return {};
    }
    last_=now;
    const bool final=goal_distance<=.12;
    const double error=final ? goal_error : path_error;
    if (phase_==Phase::DRIVE && (final || std::abs(error)>drive_heading_limit)) stop(now);
    if (phase_==Phase::TURN && (turn_sign_*error<=turn_stop_error || final!=turn_for_goal_)) stop(now);
    if (phase_==Phase::SETTLE) {
      if (!stopped_) stopped_=now;
      if (std::abs(observed_v)<=.005 && std::abs(observed_w)<=.01) {
        if (!slow_) slow_=now;
      } else slow_.reset();
      if (now-*stopped_<settle_seconds || !slow_ || now-*slow_<slow_hold_seconds) return {};
      if (final && std::abs(error)<=turn_stop_error) return {};
      if (final || std::abs(error)>drive_heading_limit) {
        phase_=Phase::TURN;turn_sign_=error>0 ? 1.0 : -1.0;turn_for_goal_=final;
      } else phase_=Phase::DRIVE;
    }
    if (phase_==Phase::TURN) return {0.0,turn_sign_*angular_cap};
    const double cap=std::clamp(speed_limit,0.0,linear_cap);
    return {std::min(cap,std::max(.02,linear_cap*goal_distance/.50)),0.0};
  }
private:
  Phase phase_{Phase::SETTLE};
  std::optional<double> stopped_,slow_,last_;
  double turn_sign_{1};bool turn_for_goal_{false};
};
}  // namespace anc_nav2_sequential_controller
