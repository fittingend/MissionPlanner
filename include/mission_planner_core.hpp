#pragma once

#include "mission_planner_types.hpp"
#include "route_handler_core.hpp"

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace autoware::mission_planner_universe
{

class DefaultPlannerCore;  // forward declaration

using TransformPoseFn = std::function<Pose(const Pose &, const Header &)>;

class MissionPlannerCore
{
public:
  MissionPlannerCore(
    const std::string & frame_id,
    std::shared_ptr<DefaultPlannerCore> planner,
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
  std::shared_ptr<DefaultPlannerCore> planner_;
  TransformPoseFn transform_pose_;
  RouteHandlerCore route_handler_;
};

}  // namespace autoware::mission_planner_universe
