#pragma once

#include "mission_planner_types.hpp"
#include "route_handler.hpp"
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_projection/UTM.h>

#include <memory>
#include <string>

namespace autoware::mission_planner_universe
{

// struct DefaultPlannerParam
// {
//   double goal_angle_threshold_deg{10.0};
//   bool check_footprint_inside_lanes{false};
//   bool consider_no_drivable_lanes{false};
//   bool enable_correct_goal_pose{false};
// };

struct SimpleVehicleInfo
{
  double length{4.5};
  double width{1.8};
  double wheel_base{2.7};
  double max_longitudinal_offset_m{10.0};
};

class DefaultPlanner
{
public:
  using RoutePoints = std::vector<Pose>;

  DefaultPlanner(
    const DefaultPlannerParam & param,
    const SimpleVehicleInfo & vehicle_info);

  void loadOsmMap(
    const std::string & osm_file,
    const lanelet::Origin & origin);

  // Output route (Autoware-like LaneletRoute)
  LaneletRoute plan(const RoutePoints & points);

private:
  DefaultPlannerParam param_;
  SimpleVehicleInfo vehicle_info_;

  lanelet::LaneletMapPtr map_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_;
  lanelet::routing::RoutingGraphPtr routing_graph_;

  bool is_graph_ready_{false};

  RouteHandler route_handler_;
};

}  // namespace autoware::mission_planner_universe
