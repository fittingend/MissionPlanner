#include "mission_planner.hpp"
#include "default_planner.hpp"
#include "arrival_checker.hpp"
#include <iostream>
#include <random>
#include <string>   // std::string
#include <cmath>    // M_PI

#include <lanelet2_projection/UTM.h>
#include <lanelet2_core/primitives/GPSPoint.h>

using namespace autoware::mission_planner_universe;

int main(int argc, char ** argv)
{
  // 1) DefaultPlanner 준비
  DefaultPlannerParam param;
  SimpleVehicleInfo vehicle_info;
  auto planner = std::make_shared<DefaultPlanner>(param, vehicle_info);

  // // 맵 설정 (오토웨어 스타일)
  // LaneletMapBin map;
  // planner->setMap(map);

  // 2) OSM 파일 path (osm 맵 파일을 바로 읽어오는 방식으로 진행함)
  std::string osm_file = (argc > 1) ? argv[1] : "../sample-map-planning/lanelet2_map.osm";
  // 3) origin hard-code (map_config.yaml 스크립트에서 가져옴)
  lanelet::GPSPoint origin_gps;
  origin_gps.lat = 35.23808753540768;
  origin_gps.lon = 139.9009591876285;
  origin_gps.ele = 0.0;

  lanelet::Origin origin(origin_gps);
  planner->loadOsmMap(osm_file, origin);

  // 4) pose 변환 함수 (여기서는 맵 좌표 쓰므로 아무처리 없이 그대로 반환)
  TransformPoseFn tf_fn = [](const Pose & p, const Header &) {
    return p;
  };

  // 5) MissionPlanner 생성
  MissionPlanner core("map", planner, tf_fn);

  // 6) start / goal, header, uuid 준비
  Pose start{};
  start.position.x = 3708.456298828125;   // 3628 ~ 3920 사이
  start.position.y = 73666.421875;  // 73453 ~ 73882 사이
  start.position.z = 19.533;  // 73453 ~ 73882 사이
  start.orientation.x = 0;
  start.orientation.y = 0;
  start.orientation.z = 0.24;
  start.orientation.w = 0.97;

  Pose goal{};
  goal.position.x = 3760.0371;    
  goal.position.y = 73693.2578;  
  goal.position.z = 19.624334708;   
  goal.orientation.x = 0;
  goal.orientation.y = 0;
  goal.orientation.z = 0.2469;
  goal.orientation.w = 0.96902;
  Header h;
  h.frame_id = "map"; //맵좌표계 사용

  UUID uuid{};
  uuid[0] = 1; // 간단히 1로 설정 (실제 사용 시에는 random UUID 추천)

  // 7) (옵션) 중간 경유지 설정 – 지금은 비워두고 start→goal 바로 연결
  std::vector<Pose> waypoints;

  // 8) Route 생성
  auto route = core.makeRouteFromWaypoints(h, waypoints, start, goal, uuid, true);

  std::cout << "Route created: start=("
            << route.start_pose.position.x << ", "
            << route.start_pose.position.y << "), goal=("
            << route.goal_pose.position.x << ", "
            << route.goal_pose.position.y << ")\n";
  
  std::cout << "Route segments size = " << route.segments.size() << std::endl;

  for (std::size_t i = 0; i < route.segments.size(); ++i) {
      const auto & seg = route.segments[i];
      std::cout << "  seg " << i;

      if (!seg.primitives.empty()) {
        std::cout << " lanelet_id=" << seg.primitives.front().id;
      } else {
        std::cout << " lanelet_id=<none>";
      }

      std::cout << " primitives=" << seg.primitives.size() << std::endl;
    }

  // ArrivalChecker 테스트
  ArrivalChecker arrival_checker(
    /*angle_rad*/ 5.0 * M_PI / 180.0,
    /*distance*/ 1.0,
    /*duration_sec*/ 2.0,
    /*stop_checker*/ [](double) {
      // 예제: 항상 정지했다고 가정
      return true;
    });

  Pose2D goal2d{"map", goal.position.x, goal.position.y, 0.0};
  arrival_checker.set_goal(goal2d);

  Pose2D current{"map", goal.position.x + 0.3, goal.position.y + 0.2, 0.01};
  bool arrived = arrival_checker.is_arrived(current);
  std::cout << "Arrived? " << (arrived ? "YES" : "NO") << std::endl;

  return 0;
}