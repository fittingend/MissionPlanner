cmake_minimum_required(VERSION 3.10)
project(standalone_mission_planner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# add_library(planner_core STATIC
#   src/mission_planner_core.cpp
#   src/default_planner_core.cpp
#   src/arrival_checker_core.cpp
# )

# add_executable(planner_demo
#   src/main.cpp
# )

# target_link_libraries(planner_demo
#   planner_core
# )

# lanelet2 include / lib 경로 추가
include_directories(/opt/rosless-lanelet2/include)
include_directories(/usr/include/eigen3)
link_directories(/opt/rosless-lanelet2/lib)

# 🔹 Eigen3 찾기
find_package(Eigen3 REQUIRED)

add_executable(mission_planner
  src/main.cpp
  src/mission_planner_core.cpp
  src/default_planner_core.cpp
  src/arrival_checker_core.cpp
)

target_link_libraries(mission_planner
  lanelet2_core
  lanelet2_io
  lanelet2_projection
  lanelet2_routing
  lanelet2_traffic_rules
)