#pragma once

#include "mission_planner_types.hpp"

#include <memory>
#include <string>

// Forward declare lanelet2 types only in header
namespace lanelet
{
class LaneletMap;
class Origin;

namespace routing
{
class RoutingGraph;
}  // namespace routing
}  // namespace lanelet

namespace autoware::mission_planner_universe
{

struct DefaultPlannerParam
{
  double goal_angle_threshold_deg{10.0};
  bool check_footprint_inside_lanes{false};
  bool consider_no_drivable_lanes{false};
  bool enable_correct_goal_pose{false};
};

struct SimpleVehicleInfo
{
  double length{4.5};
  double width{1.8};
  double wheel_base{2.7};
  double max_longitudinal_offset_m{10.0};
};

class DefaultPlannerCore
{
public:
  using RoutePoints = std::vector<Pose>;

  DefaultPlannerCore(
    const DefaultPlannerParam & param,
    const SimpleVehicleInfo & vehicle_info);

  void loadOsmMap(
    const std::string & osm_file,
    const lanelet::Origin & origin);

  // 최종 출력은 LaneletRoute
  LaneletRoute plan(const RoutePoints & points);

private:
  DefaultPlannerParam param_;
  SimpleVehicleInfo vehicle_info_;

  std::shared_ptr<lanelet::LaneletMap> map_;
  std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_;
  bool is_graph_ready_{false};
};

}  // namespace autoware::mission_planner_universe
