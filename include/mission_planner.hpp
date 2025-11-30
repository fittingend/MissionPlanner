#pragma once

#include "mission_planner_types.hpp"
#include "route_handler.hpp"

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace autoware::mission_planner_universe
{

class DefaultPlanner;  // forward declaration

using TransformPoseFn = std::function<Pose(const Pose &, const Header &)>;

class MissionPlanner
{
public:
  MissionPlanner(
    const std::string & frame_id,
    std::shared_ptr<DefaultPlanner> planner,
    TransformPoseFn transform_pose_fn);

  LaneletRoute makeRouteFromSegments(
    const Header & header,
    const std::vector<LaneletSegment> & segments,
    const Pose & start_pose,
    const Pose & goal_pose,
    const UUID & uuid,
    bool allow_goal_modification) const;

  LaneletRoute makeRouteFromWaypoints(
    const Header & header,
    const std::vector<Pose> & waypoints,
    const Pose & start_pose,
    const Pose & goal_pose,
    const UUID & uuid,
    bool allow_goal_modification) const;

  LaneletRoute makeRouteForModifiedGoal(
    const PoseWithUuidStamped & msg,
    const LaneletRoute * const current_route,
    const Pose & current_pose) const;

private:
  std::string map_frame_;
  std::shared_ptr<DefaultPlanner> planner_;
  TransformPoseFn transform_pose_;
};

}  // namespace autoware::mission_planner_universe
