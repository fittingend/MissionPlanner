#include "mission_planner.hpp"
#include "default_planner.hpp"

namespace autoware::mission_planner_universe
{

MissionPlanner::MissionPlanner(
  const std::string & frame_id,
  std::shared_ptr<DefaultPlanner> planner,
  TransformPoseFn transform_pose_fn)
: map_frame_(frame_id)
, planner_(std::move(planner))
, transform_pose_(std::move(transform_pose_fn))
{
}

/**
 * @brief 미리 생성된 LaneletSegment 들을 이용해 Route 를 구성
 *
 * 인자로 들어오는 start_pose / goal_pose 는
 * 모두 header.frame_id 좌표계라고 가정하고, 내부에서 map_frame_ 으로 변환해서 저장함.
 */
LaneletRoute MissionPlanner::makeRouteFromSegments(
  const Header & header,
  const std::vector<LaneletSegment> & segments,
  const Pose & start_pose,
  const Pose & goal_pose,
  const UUID & uuid,
  const bool allow_goal_modification) const
{
  LaneletRoute route;

  route.header.stamp = header.stamp;
  route.header.frame_id = map_frame_;

  route.start_pose = transform_pose_(start_pose, header);
  route.goal_pose = transform_pose_(goal_pose, header);
  route.segments = segments;
  route.uuid = uuid;
  route.allow_modification = allow_goal_modification;

  return route;
}
/**
 * @brief start/goal + 중간 waypoint 들을 이용해 DefaultPlanner 로부터 Route 생성
 *
 * 인자로 들어오는 start_pose / waypoints/ goal_pose 는
 * 모두 header.frame_id 좌표계라고 가정.
 * 내부에서 transform_pose_ 를 통해 map_frame_ 으로 변환한 뒤 DefaultPlanner::plan() 에 전달.
 */
LaneletRoute MissionPlanner::makeRouteFromWaypoints(
  const Header & header,
  const std::vector<Pose> & waypoints,
  const Pose & start_pose,
  const Pose & goal_pose,
  const UUID & uuid,
  const bool allow_goal_modification) const
{
  // 🔹 이전에는 PlannerPlugin::RoutePoints 를 썼지만,
  //     이제는 DefaultPlanner::RoutePoints = std::vector<Pose> 그대로 사용
  DefaultPlanner::RoutePoints points;
  points.clear();

  // 시작점
  points.push_back(start_pose);

  // header.frame_id 기준으로 들어온 waypoint → map_frame 으로 변환
  for (const auto & wp_header_frame : waypoints) {
    points.push_back(transform_pose_(wp_header_frame, header));
  }

  // goal 도 동일하게 변환
  points.push_back(transform_pose_(goal_pose, header));

  // 🔹 바로 DefaultPlanner::plan 호출
  LaneletRoute route = planner_->plan(points);
  route.header.stamp = header.stamp;
  route.header.frame_id = map_frame_;
  route.uuid = uuid;
  route.allow_modification = allow_goal_modification;

  return route;
}
/**
 * @brief Goal 이 수정되었을 때, 기존 route 와 현재 pose를 이용해 새로운 route 생성
 *
 * - msg.header.frame_id 기준 goal pose 가 들어온다고 가정
 * - current_route 가 있으면 start_pose 는 기존 route 의 start 를 유지
 * - current_route 가 없으면 start_pose 는 현재 pose 를 사용
 * - waypoints 는 간단하게 "현재 pose 한 개" 만 추가 (필요시 확장 가능)
 */
LaneletRoute MissionPlanner::makeRouteForModifiedGoal(
  const PoseWithUuidStamped & msg,
  const LaneletRoute * const current_route,
  const Pose & current_pose) const
{
    const auto & header = msg.header;
    const auto & goal_pose = msg.pose;
    const auto & uuid = msg.uuid;

    const bool allow_goal_modification =
      (current_route != nullptr) ? current_route->allow_modification : true;

    // start_pose 는 header 좌표계라고 가정
    Pose start_pose_header_frame{};
    if (current_route != nullptr) {
      // 기존 route 의 start_pose 는 보통 map 좌표계지만,
      // header.frame_id == map_frame_ 인 경우 transform_pose_ 가 항등이도록 구현하면 문제 없음.
      start_pose_header_frame = current_route->start_pose;
    } else {
      start_pose_header_frame = current_pose;
    }

    // waypoints: header 기준으로 현재 pose 한 개만 사용
    std::vector<Pose> waypoints_header_frame;
    if (current_route != nullptr) {
      waypoints_header_frame.push_back(current_pose);
    }

    return makeRouteFromWaypoints(
      header,
      waypoints_header_frame,
      start_pose_header_frame,
      goal_pose,
      uuid,
      allow_goal_modification);
  }

}  // namespace autoware::mission_planner_universe
