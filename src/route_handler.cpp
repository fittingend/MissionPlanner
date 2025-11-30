#include "route_handler.hpp"

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_routing/Route.h>

#include <iostream>

namespace autoware::mission_planner_universe
{

using ConstLanelet  = RouteHandler::ConstLanelet;
using ConstLanelets = RouteHandler::ConstLanelets;

// ─────────────────────────────────────────────────────────────
//  쿼터니언 → yaw
// ─────────────────────────────────────────────────────────────
double RouteHandler::yawFromQuaternion(const Pose & pose)
{
  const auto & q = pose.orientation;
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

// ─────────────────────────────────────────────────────────────
//  lanelet centerline 방향 (rad) 계산
//   - 여기서는 단순히 centerline 첫/마지막 점을 이용
//   - 더 정밀하게 하려면 goal 근처 segment로 보정 가능
// ─────────────────────────────────────────────────────────────
double RouteHandler::computeLaneletYaw(const ConstLanelet & ll)
{
  const auto & cl = ll.centerline();
  if (cl.size() < 2) {
    return 0.0;
  }
  const auto p0 = cl.front().basicPoint2d();
  const auto p1 = cl.back().basicPoint2d();
  const double dx = p1.x() - p0.x();
  const double dy = p1.y() - p0.y();
  return std::atan2(dy, dx);
}

// ─────────────────────────────────────────────────────────────
//  start/goal 근처에서 "주행 가능한" lanelet 찾기
//   - 단순 nearest(1) 대신, 일정 반경 안의 여러 lanelet을 보면서
//     traffic_rules->canPass() 로 필터링
// ─────────────────────────────────────────────────────────────
std::optional<ConstLanelet> RouteHandler::getClosestDrivableLanelet(
  const lanelet::BasicPoint2d & pt) const
{
  if (!map_ || !rules_) {
    return std::nullopt;
  }
  
  // 충분히 많은 후보를 본다 (예: 10개)
  auto candidates = map_->laneletLayer.nearest(pt, 10);
  double best_dist = std::numeric_limits<double>::max();
  std::optional<ConstLanelet> best_ll;

  for (const auto & ll : candidates) {
    // 주행 가능 lanelet만 사용 (Autoware에서 RouteHandler가 하는 역할과 유사)
    if (!rules_->canPass(ll)) {
      continue;
    }

    double d = lanelet::geometry::distance2d(ll, pt);
    if (d < best_dist) {
      best_dist = d;
      best_ll   = ll;
    }
  }

  // 혹시 canPass() 통과한 게 하나도 없으면, fallback으로 그냥 가장 가까운 lanelet 사용
  if (!best_ll && !candidates.empty()) {
    best_ll = candidates.front();
  }

  return best_ll;
}

// ─────────────────────────────────────────────────────────────
//  goal pose yaw vs lanelet 진행방향 체크
// ─────────────────────────────────────────────────────────────
bool RouteHandler::isGoalAngleValid(
  const Pose & goal_pose,
  const ConstLanelet & goal_ll) const
{
  double lane_yaw = computeLaneletYaw(goal_ll);
  double goal_yaw = yawFromQuaternion(goal_pose);

  // [-pi, pi] 범위에서 차이 계산
  double diff = std::atan2(std::sin(lane_yaw - goal_yaw), std::cos(lane_yaw - goal_yaw));
  diff = std::fabs(diff);

  return diff <= param_.goal_angle_threshold_rad;
}

// ─────────────────────────────────────────────────────────────
//  메인: start/goal → LaneletRoute 생성
// ─────────────────────────────────────────────────────────────
bool RouteHandler::planRoute(
   const Pose & start_pose,
   const Pose & goal_pose,
  LaneletRoute & out_route) const
{
  out_route = LaneletRoute{};  // 초기화

  if (!isReady()) {
    std::cerr << "[RouteHandler] not ready (map/graph/rules missing)\n";
    return false;
  }

  // 1) start/goal 2D 포인트로 변환
  lanelet::BasicPoint2d start_pt(start_pose.position.x, start_pose.position.y);
  lanelet::BasicPoint2d goal_pt(goal_pose.position.x,  goal_pose.position.y);

  // 2) 주행 가능한 lanelet 중에서 start/goal에 가장 가까운 것 찾기
  auto start_ll_opt = getClosestDrivableLanelet(start_pt);
  auto goal_ll_opt  = getClosestDrivableLanelet(goal_pt);

  if (!start_ll_opt || !goal_ll_opt) {
    std::cerr << "[RouteHandler] failed to find drivable lanelet for start/goal\n";
    return false;
  }

  auto start_ll = *start_ll_opt;
  auto goal_ll  = *goal_ll_opt;

  std::cout << "[RouteHandler] start_ll id=" << start_ll.id()
            << ", goal_ll id=" << goal_ll.id() << std::endl;


  // 3) goal pose 유효성 체크 (yaw vs lanelet 방향)
  if (!isGoalAngleValid(goal_pose, goal_ll)) {
    std::cerr << "[RouteHandler] goal yaw is not aligned with lanelet direction\n";
    //return false;
  }

  // (추가로 Autoware는 goal footprint가 lanelet 내부에 있는지도 검사하지만,
  //  여기서는 생략하거나 나중에 확장해도 됨.)

  // 4) RoutingGraph를 이용한 route 계산
  auto route = graph_->getRoute(start_ll, goal_ll);
  if (!route) {
    std::cerr << "[RouteHandler] getRoute() failed (no path between lanelets)\n";
    return false;
  }

  auto shortest_path = route->shortestPath();
  if (shortest_path.empty()) {
    std::cerr << "[RouteHandler] shortestPath() is empty\n";
    return false;
  }

  // 5) Autoware-style LaneletRoute 메시지로 변환
  out_route.start_pose = start_pose;
  out_route.goal_pose  = goal_pose;
  out_route.segments.clear();

  for (const auto & ll : shortest_path) {
    LaneletSegment seg{};

    LaneletPrimitive prim;
    prim.id             = ll.id();        // lanelet::Id
    prim.primitive_type = "lane";        // Autoware도 기본은 "lane" 사용

    seg.preferred_primitive = prim;
    seg.primitives.push_back(prim);

    out_route.segments.push_back(seg);
  }

  // Autoware에서 uuid, allow_modification 등은 상위 레벨에서 세팅하므로
  // 여기서는 그대로 두거나 필요시 추가로 세팅
  out_route.allow_modification = false;

  return true;
}

}  // namespace autoware::mission_planner_universe::lanelet2
