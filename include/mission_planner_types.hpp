#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace autoware::mission_planner_universe
{

// 간단한 타임스탬프
struct TimeStamp
{
  std::int64_t sec{0};
  std::int64_t nsec{0};
};

struct Header
{
  TimeStamp stamp{};
  std::string frame_id;
};

// Pose 대체 (geometry_msgs::msg::Pose 비슷하게)
struct Position
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Orientation
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

struct Pose
{
  Position position;
  Orientation orientation;
};

// UUID
using UUID = std::array<std::uint8_t, 16>;

// lane 하나의 고유 ID, type(lane) 만 담는 구조
struct LaneletPrimitive
{
  std::int64_t id{};              // lanelet::Id
  std::string primitive_type;     // 보통 "lane" 하나만 써도 충분
};

// LaneletSegment: "차선 변경 가능한 구간 하나" = 여러 개의 lanelet 묶음
struct LaneletSegment
{
  LaneletPrimitive preferred_primitive;          // 실제 주행 lane 예) referred = lane 10
  std::vector<LaneletPrimitive> primitives;      // 예) primitives = {10,11} (여차하면 11로 lane change 가능)
};

// 경로 전체
struct LaneletRoute
{
  Header header;
  Pose start_pose;
  Pose goal_pose;
  std::vector<LaneletSegment> segments;
  UUID uuid{};                  // Request-Response 매칭용 ID
  bool allow_modification{true};
};

// PoseWithUuidStamped
struct PoseWithUuidStamped
{
  Header header;
  Pose pose;
  UUID uuid{};
};

}  // namespace autoware::mission_planner_universe
