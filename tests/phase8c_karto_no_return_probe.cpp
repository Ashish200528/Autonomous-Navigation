// Standalone contract probe against the installed Karto implementation.
// No ROS node, motion command or simulator-truth input.
#include <karto_sdk/Karto.h>
#include <iostream>
#include <limits>

int main() {
  auto *laser = karto::LaserRangeFinder::CreateLaserRangeFinder(
      karto::LaserRangeFinder_Custom, karto::Name("no_return_contract_probe"));
  laser->SetMinimumRange(0.12);
  laser->SetMaximumRange(6.0);
  laser->SetMinimumAngle(-karto::KT_PI);
  laser->SetMaximumAngle(karto::KT_PI);
  laser->SetAngularResolution(2.0 * karto::KT_PI / 359.0);
  karto::SensorManager::GetInstance()->RegisterSensor(laser);
  karto::RangeReadingsVector readings(360, 6.0);
  readings[0] = 2.0;  // One real return, followed by 359 normalized no-returns.
  auto count = [&](double threshold, bool filtered) {
    laser->SetRangeThreshold(threshold);
    karto::LocalizedRangeScan scan(laser->GetName(), readings);
    scan.SetOdometricPose(karto::Pose2());
    scan.SetCorrectedPose(karto::Pose2());
    size_t finite = 0;
    for (const auto &p : scan.GetPointReadings(filtered)) {
      if (std::isfinite(p.GetX()) && std::isfinite(p.GetY())) ++finite;
    }
    return finite;
  };
  auto baseline = count(6.0, false);
  auto lower_threshold = count(5.99, false);
  auto lower_filtered = count(5.99, true);
  for (size_t i = 1; i < readings.size(); ++i)
    readings[i] = std::numeric_limits<double>::quiet_NaN();
  auto encoded = count(6.0, false);
  std::cout << "{\"baseline_finite_unfiltered_endpoints\":" << baseline
            << ",\"lower_threshold_finite_unfiltered_endpoints\":" << lower_threshold
            << ",\"lower_threshold_filtered_endpoints\":" << lower_filtered
            << ",\"nan_no_return_finite_unfiltered_endpoints\":" << encoded << "}\n";
  karto::SensorManager::GetInstance()->UnregisterSensor(laser);
  delete laser;
  return baseline == 360 && lower_threshold == 360 && lower_filtered == 1 && encoded == 1 ? 0 : 1;
}
