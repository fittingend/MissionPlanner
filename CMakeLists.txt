cmake_minimum_required(VERSION 3.10)
project(standalone_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ----------------------------------------------------------------------
# 1) Lanelet2 서브프로젝트 추가
# ----------------------------------------------------------------------
add_subdirectory(external/Rosless-Lanelet2)

# ----------------------------------------------------------------------
# 2) standalone executable
# ----------------------------------------------------------------------
add_executable(mission_planner
    src/main.cpp
    src/default_planner.cpp
    src/arrival_checker.cpp
    src/mission_planner.cpp
    src/route_handler.cpp
)

target_include_directories(mission_planner PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/external/lanelet2
)

# ----------------------------------------------------------------------
# 3) lanelet2 라이브러리 링크
# ----------------------------------------------------------------------
target_link_libraries(mission_planner PRIVATE
    lanelet2_core
    lanelet2_io
    lanelet2_projection
    lanelet2_routing
    lanelet2_traffic_rules
    # validation은 필요없으면 빼도 OK
)
