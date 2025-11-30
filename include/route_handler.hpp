#pragma once

#include "mission_planner_types.hpp"   // Pose, LaneletRoute, LaneletSegment, LaneletPrimitive 등
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Point.h>

#include <memory>
#include <optional>
#include <vector>
#include <string>
#include <cmath>

namespace autoware::mission_planner_universe
{

struct DefaultPlannerParam
{
  double goal_angle_threshold_rad{M_PI / 6.0};  // goal yaw vs lanelet 방향 허용 각도 (30도 정도)
  double search_radius{30.0};                   // start/goal 근처 lanelet 탐색 반경
};
using RouteHandlerParam = DefaultPlannerParam;

class RouteHandler
{
public:
  using LaneletMapPtr      = std::shared_ptr<lanelet::LaneletMap>;
  using RoutingGraphPtr    = std::shared_ptr<lanelet::routing::RoutingGraph>;
  using TrafficRulesPtr    = std::shared_ptr<const lanelet::traffic_rules::TrafficRules>;
  using ConstLanelet       = lanelet::ConstLanelet;
  using ConstLanelets      = std::vector<ConstLanelet>;

  RouteHandler() = default;

  RouteHandler(
    const LaneletMapPtr & map,
    const RoutingGraphPtr & graph,
    const TrafficRulesPtr & rules,
    const RouteHandlerParam & param = RouteHandlerParam())
  : map_(map), graph_(graph), rules_(rules), param_(param)
  {}

  void setMapAndGraph(
    const LaneletMapPtr & map,
    const RoutingGraphPtr & graph,
    const TrafficRulesPtr & rules,
    const DefaultPlannerParam & param = DefaultPlannerParam())
  {
    map_   = map;
    graph_ = graph;
    rules_ = rules;
    param_ = param;
  }

  bool isReady() const
  {
    return static_cast<bool>(map_) && static_cast<bool>(graph_) && static_cast<bool>(rules_);
  }

  /// start/goal Pose를 받아 LaneletRoute 메시지를 생성
  /// 성공하면 true, 실패 (goal invalid, route 없음 등)면 false 반환
  bool planRoute(
    const Pose & start_pose,
    const Pose & goal_pose,
    LaneletRoute & out_route) const;

private:
  // start/goal 근처에서 "주행 가능한" lanelet 찾기 (Autoware RouteHandler 역할의 핵심)
  std::optional<ConstLanelet> getClosestDrivableLanelet(
    const lanelet::BasicPoint2d & pt) const;

  // goal pose의 yaw와 lanelet 진행 방향이 threshold 안에 들어오는지 체크
  bool isGoalAngleValid(
    const Pose & goal_pose,
    const ConstLanelet & goal_ll) const;

  // lanelet centerline 방향 (rad) 계산
  static double computeLaneletYaw(const ConstLanelet & ll);

  // 쿼터니언 → yaw 변환
  static double yawFromQuaternion(const Pose & pose);

private:
  LaneletMapPtr   map_;
  RoutingGraphPtr graph_;
  TrafficRulesPtr rules_;
  RouteHandlerParam param_;
};

}  // namespace autoware::mission_planner_universe
