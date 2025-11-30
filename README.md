# Standalone Mission Planner

본 프로젝트는 Autoware Universe Mission Planner의 핵심 기능을 ROS2 없이 단독 실행 환경(C++17)에서 재구성한 Standalone 버전입니다.
- Autoware Mission Planner의 핵심 알고리즘을 ROS 없이 단독 실행 가능하게 분리
- Autoware /planning/mission_planning/route 와 동일한 Route 메시지 구조 출력
- TF, Executor, Node 없이도 C++ 프로그램에서 단독 경로 생성 가능
- ROS 의존성 제거한 Lanelet2 오픈소스 사용

## 주요 기능
####  1. OSM → Lanelet2 Map 로딩

- .osm 파일을 읽어 lanelet 맵을 구성
- Traffic Rules / RoutingGraph 생성

#### 2. 경로 생성(Route Planning)
- 시작점(Start Pose)과 목표점(Goal Pose)을 입력받아 가장 가까운 drivable lanelet을 찾고
- Lanelet2 RoutingGraph 기반 Shortest Path 계산
- Autoware /planning/mission_planning/route 메시지 구조와 동일한 Route 타입 생성
- **TO DO: 현재는 start/goal pose main에 하드코드. 추후에 AUTOSAR 에서이벤트 형태로 수신받아야...**

#### 3. Goal 유효성 검사 

- Goal이 속한 lanelet의 진행 방향과 goal yaw 차이를 비교
- goal_angle_threshold_rad 파라미터로 허용 범위 설정

#### 4. 도착 판정(Arrival Checker) (🔧검증필요)

- 거리(distance), 각도(angle), 정지시간(duration), frame_id 비교를 통해 “도착(Arrived)” 여부 판단
-  **TO DO: 추후 odometry 값 받으면 여기에 넣어서 도착판정 정상적으로 이루어지는지 검증 필요**


#### 5. ROS/Autoware 없이 단독 실행

- TF, ROS2, rclcpp 없이도 작동하도록 MissionPlanner / RouteHandler / DefaultPlanner 로 완전 분리
- standalone C++ 프로그램에서 route 생성 가능

## 코드구조

```
standalone_planner/
├── build/                          # CMake 빌드 산출물 (생성됨)
│   └── mission_planner             # 빌드된 실행 파일
│
├── CMakeLists.txt                  # 프로젝트 빌드 설정
├── README.md│
├── external/
│   └── Rosless-Lanelet2/           # ROS 의존성 제거한 Lanelet2 소스
│       ├── CMakeLists.txt
│       ├── cmake/                  # FindGeographicLib.cmake, FindPugiXml.cmake 등
│       ├── lanelet2_core/
│       ├── lanelet2_io/
│       ├── lanelet2_projection/
│       ├── lanelet2_routing/
│       ├── lanelet2_traffic_rules/
│       ├── lanelet2_validation/
│       └── lanelet2_examples/      # Lanelet2 공식 예제
│
├── include/
│   ├── mission_planner_types.hpp   # Route, Segment, Primitive, Pose, UUID 정의
│   ├── mission_planner.hpp         
│   ├── default_planner.hpp        
│   ├── arrival_checker.hpp       
│   └── route_handler.hpp          
│
├── src/
│   ├── mission_planner.cpp         # MissionPlanner 구현
│   ├── default_planner.cpp         # DefaultPlanner 구현
│   ├── route_handler.cpp           # RouteHandler 구현
│   ├── arrival_checker.cpp         # ArrivalChecker 구현
│   └── main.cpp                    # Standalone 엔트리 포인트
│
└── sample-map-planning/
    ├── lanelet2_map.osm            # 샘플 Lanelet2 OSM 맵
    ├── map_config.yaml             # Autoware 스타일 맵 설정 (origin 등)
    ├── map_projector_info.yaml     # projector 설정 (UTM 등)
    └── pointcloud_map.pcd          # 샘플 포인트클라우드 맵


```

## 🔧 주요 모듈 설명


![Standalone Planner flow diagram](flow.png)
### 1. route_handler.hpp / route_handler.cpp

: Lanelet2 기반 경로 생성기 (Autoware RouteHandler기반)

**입력**

- 맵
- RoutingGraph
- TrafficRules
- RouteHandlerParam(goal angle threshold, search radius)

**출력**
- LaneletRoute 

**핵심 기능**
- drivable lanelet 탐색
- lanelet 진행 방향 계산
- goal yaw 유효성 판단
- shortest path 계산
- lanelet 시퀀스 → RouteSegments 생성


###  2. default_planner.hpp / default_planner.cpp

: OSM 파일 로딩 + RouteHandler 호출 플래너

**입력**

- DefaultPlannerParam
- SimpleVehicleInfo
- start/goal poses

**주요 기능**
- OSM → LaneletMap 로딩
- TrafficRules / RoutingGraph 생성
- RouteHandler로 경로 생성 요청


### 3. mission_planner.hpp / mission_planner.cpp

: 외부에서 사용하는 Planner 인터페이스

**입력**

- header(frame_id)
- start/goal pose
- optional waypoints
- TransformPoseFn (TF 함수)

**출력**

LaneletRoute

**역할**

- pose transform 적용

- DefaultPlanner::plan() 호출


#### 4. arrival_checker.hpp / arrival_checker.cpp

: 도착 여부 판단 모듈

**조건**

- distance ≤ threshold
- yaw difference ≤ threshold
- frame_id 동일
- stop_checker(duration) true




