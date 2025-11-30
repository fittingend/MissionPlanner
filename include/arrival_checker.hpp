#pragma once

#include <cmath>
#include <functional>
#include <optional>
#include <string>

namespace autoware::mission_planner_universe
{

// 2D Pose (frame + x,y,yaw)
struct Pose2D
{
  std::string frame_id;
  double x{0.0};
  double y{0.0};
  double yaw{0.0};  // rad
};

// 차량 정지 여부를 duration_sec 동안 검사하는 콜백
using StopCheckerFn = std::function<bool(double duration_sec)>;

// - angle_rad : 허용 yaw 오차 (rad)
// - distance  : 허용 위치 오차 (m)
// - duration_sec : 정지 시간 (초)
class ArrivalChecker
{
public:
  ArrivalChecker(
    double angle_rad,
    double distance,
    double duration_sec,
    StopCheckerFn stop_checker);

  void clear_goal();
  void set_goal(const Pose2D & goal);

  bool is_arrived(const Pose2D & pose) const;

private:
  double angle_;
  double distance_;
  double duration_;
  StopCheckerFn stop_checker_;
  std::optional<Pose2D> goal_;

  static double normalizeRadian(double rad);
};

}  // namespace autoware::mission_planner_universe
