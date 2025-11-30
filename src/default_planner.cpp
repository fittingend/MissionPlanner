#include "default_planner.hpp"
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_projection/UTM.h>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Point.h>

#include <iostream>
#include <limits>
#include <algorithm>

namespace autoware::mission_planner_universe
{

DefaultPlanner::DefaultPlanner(
  const DefaultPlannerParam & param,
  const SimpleVehicleInfo & vehicle_info)
: param_(param), vehicle_info_(vehicle_info)
{
}

// OSM 파일 로드 + Lanelet2 RoutingGraph 생성
void DefaultPlanner::loadOsmMap(
  const std::string & osm_file,
  const lanelet::Origin & origin)
{
  lanelet::ErrorMessages errors;

  // 1) 위경도 → local (map frame) 변환 projector
  lanelet::projection::UtmProjector projector(origin);

  // 2) OSM 파일 로드
  map_ = lanelet::load(osm_file, projector, &errors);

  if (!errors.empty()) {
    for (const auto & e : errors) {
      std::cerr << "[Lanelet2] load error: " << e << std::endl;
    }
  }

  if (!map_) {
    std::cerr << "[Lanelet2] Failed to load map\n";
    is_graph_ready_ = false;
    return;
  }

  std::cout << "[Lanelet2] map loaded. lanelet count = "
            << map_->laneletLayer.size() << std::endl;

  // 디버그: XY range
  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();

  for (const auto & ll : map_->laneletLayer) {
    for (const auto & pt : ll.centerline()) {
      const auto & p = pt.basicPoint();
      min_x = std::min(min_x, p.x());
      max_x = std::max(max_x, p.x());
      min_y = std::min(min_y, p.y());
      max_y = std::max(max_y, p.y());
    }
  }

  std::cout << "[Lanelet2] map XY range: "
            << "x in [" << min_x << ", " << max_x << "], "
            << "y in [" << min_y << ", " << max_y << "]" << std::endl;

  if (!map_->laneletLayer.empty()) {
    const auto & any_ll = *map_->laneletLayer.begin();
    if (!any_ll.centerline().empty()) {
      auto p = any_ll.centerline().front().basicPoint();
      std::cout << "[Lanelet2] sample lanelet point: ("
                << p.x() << ", " << p.y() << ")\n";
    }
  }
    std::cout << "[Debug] Lanelet IDs in map:\n";
  for (const auto & ll : map_->laneletLayer) {
    std::cout << "  lanelet id=" << ll.id() << "\n";
  }

  // 3) Traffic rules 설정 (독일 차량 예시)
  traffic_rules_ = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);

  // 4) RoutingGraph 생성
  routing_graph_ = lanelet::routing::RoutingGraph::build(*map_, *traffic_rules_);
  is_graph_ready_ = (routing_graph_ != nullptr);

  // 5) RouteHandler에 map / rules / graph / param을 넘겨서 초기화
  route_handler_.setMapAndGraph(
    map_,                  // lanelet::LaneletMapPtr
    routing_graph_,        // lanelet::routing::RoutingGraphPtr
    traffic_rules_,       // lanelet::traffic_rules::TrafficRulesPtr
    param_                 // DefaultPlannerParam (goal_angle_threshold_rad 등 포함)
  );
}

// Rosless-Lanelet2 사용 시에는 loadOsmMap()만 사용
// void DefaultPlanner::setMap(const LaneletMapBin & msg)
// {
//   (void)msg;
// }

LaneletRoute DefaultPlanner::plan(const RoutePoints & points)
{
  LaneletRoute route_msg;

  if (!is_graph_ready_ || points.size() < 2) {
    return route_msg;
  }

  // 1) start / goal 세팅
  Pose start_pose = points.front();
  Pose goal_pose = points.back();
  route_msg.start_pose = points.front();
  route_msg.goal_pose  = points.back();

  std::cout << "[RouteHandler] start_pose=("
          << route_msg.start_pose.position.x << ", "
          << route_msg.start_pose.position.y << ")\n";
  std::cout << "[RouteHandler] goal_pose=("
          << route_msg.goal_pose.position.x << ", "
          << route_msg.goal_pose.position.y << ")\n";

  // 2) RouteHandler에서 실제 lanelet route 계산
  if (!route_handler_.planRoute(start_pose, goal_pose,route_msg)) {
  std::cerr << "[DefaultPlanner] RouteHandler::planRoute() failed\n";
  return LaneletRoute{};
  }
  return route_msg;
}
}  // namespace autoware::mission_planner_universe

/*
  // 1) start / goal 좌표를 lanelet2 포맷으로
  lanelet::BasicPoint2d start_pt(
    points.front().position.x, points.front().position.y);
  lanelet::BasicPoint2d goal_pt(
    points.back().position.x, points.back().position.y);

  // 2) 맵에서 가장 가까운 lanelet 찾기
  auto start_lanelets = map_->laneletLayer.nearest(start_pt, 1);
  auto goal_lanelets  = map_->laneletLayer.nearest(goal_pt, 1);
  if (start_lanelets.empty() || goal_lanelets.empty()) {
    return route_msg;
  }

  auto start_ll = start_lanelets.front();
  auto goal_ll  = goal_lanelets.front();


  // --- 디버그: start/goal 에서 가장 가까운 lanelet 찾기 ---
  auto nearest_start = map_->laneletLayer.nearest(start_pt, 1);
  auto nearest_goal  = map_->laneletLayer.nearest(goal_pt, 1);

  if (!nearest_start.empty()) {
    const auto & ll = nearest_start.front();
    double dist = lanelet::geometry::distance2d(ll, start_pt);
    std::cout << "  nearest lanelet to START: id=" << ll.id()
              << ", distance2d=" << dist << " m\n";
  } else {
    std::cout << "  nearest lanelet to START: <none>\n";
  }

  if (!nearest_goal.empty()) {
    const auto & ll = nearest_goal.front();
    double dist = lanelet::geometry::distance2d(ll, goal_pt);
    std::cout << "  nearest lanelet to GOAL : id=" << ll.id()
              << ", distance2d=" << dist << " m\n";
  } else {
    std::cout << "  nearest lanelet to GOAL : <none>\n";
  }


  // 3) RoutingGraph로 route 계산
  auto route = routing_graph_->getRoute(start_ll, goal_ll);
  if (!route) {
    return route_msg;
  }

  auto shortest_path = route->shortestPath();
  route_msg.segments.clear();

  for (const auto & ll : shortest_path) {
    LaneletSegment seg{};

    // 대표 primitive
    LaneletPrimitive prim;
    prim.id = ll.id();                   // lanelet::Id
    prim.primitive_type = "lane";        // 혹은 ll.attribute("type").value() 등으로 확장 가능

    seg.preferred_primitive = prim;
    seg.primitives.push_back(prim);

    // centerline 좌표를 Pose로 따로 저장하고 싶으면,
    // LaneletSegment에 centerline 벡터를 새로 추가하거나,
    // 나중에 별도 Path 타입으로 뽑아 쓰는 방향으로 설계 변경 필요
    // (지금 struct 정의에는 centerline이 없으니까, 우선은 primitives만 채우는 걸로)

    route_msg.segments.push_back(seg);

    */


// void DefaultPlanner::updateRoute(const LaneletRoute & route)
// {
//   (void)route;
// }

// void DefaultPlanner::clearRoute()
// {
// }

