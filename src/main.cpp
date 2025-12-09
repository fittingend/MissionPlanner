#include "default_planner.hpp"
#include "mission_planner.hpp"

#include <array>
#include <cctype>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <lanelet2_core/primitives/GPSPoint.h>
#include <lanelet2_projection/UTM.h>

using namespace autoware::mission_planner_universe;

namespace
{
// Helper to represent a key path inside the simple YAML parser.
struct YamlPathEntry
{
  std::string key;
  bool is_list_item{false};
  std::size_t list_index{0};
  std::string list_name;
};
using YamlPath = std::vector<YamlPathEntry>;
using YamlCallback = std::function<void(const YamlPath &, const std::string &)>;

std::string trim(const std::string & str)
{
  const auto begin = str.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return {};
  }
  const auto end = str.find_last_not_of(" \t\r\n");
  return str.substr(begin, end - begin + 1);
}

int countLeadingSpaces(const std::string & line)
{
  int count = 0;
  while (count < static_cast<int>(line.size()) && line[count] == ' ') {
    ++count;
  }
  return count;
}

bool fileExists(const std::string & path)
{
  std::ifstream file(path);
  return static_cast<bool>(file);
}

std::string stripQuotes(const std::string & value)
{
  if (value.size() >= 2) {
    if ((value.front() == '\'' && value.back() == '\'') || (value.front() == '"' && value.back() == '"')) {
      return value.substr(1, value.size() - 2);
    }
  }
  return value;
}

bool parseIntLiteral(const std::string & text, std::int64_t & out)
{
  try {
    out = std::stoll(text);
    return true;
  } catch (...) {
    return false;
  }
}

bool parseDoubleLiteral(const std::string & text, double & out)
{
  try {
    out = std::stod(text);
    return true;
  } catch (...) {
    return false;
  }
}

bool parseBoolLiteral(const std::string & text, bool & out)
{
  auto lowered = text;
  for (auto & ch : lowered) {
    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  }
  if (lowered == "true") {
    out = true;
    return true;
  }
  if (lowered == "false") {
    out = false;
    return true;
  }
  return false;
}

std::string formatDouble(double value)
{
  std::ostringstream oss;
  oss << std::setprecision(16) << value;
  return oss.str();
}

std::vector<std::string> flattenKeys(const YamlPath & path)
{
  std::vector<std::string> keys;
  keys.reserve(path.size());
  for (const auto & entry : path) {
    keys.push_back(entry.key);
  }
  return keys;
}

bool pathEndsWith(const std::vector<std::string> & keys, const std::vector<std::string> & suffix)
{
  if (keys.size() < suffix.size()) {
    return false;
  }
  const std::size_t offset = keys.size() - suffix.size();
  for (std::size_t i = 0; i < suffix.size(); ++i) {
    if (keys[offset + i] != suffix[i]) {
      return false;
    }
  }
  return true;
}

std::optional<std::size_t> findListIndex(const YamlPath & path, const std::string & list_name)
{
  std::optional<std::size_t> index;
  for (const auto & entry : path) {
    if (entry.is_list_item && entry.list_name == list_name) {
      index = entry.list_index;
    }
  }
  return index;
}

class MinimalYamlParser
{
public:
  static bool parseFile(const std::string & path, const YamlCallback & callback)
  {
    std::ifstream file(path);
    if (!file) {
      std::cerr << "[parser] cannot open yaml file: " << path << std::endl;
      return false;
    }

    std::vector<StackEntry> stack;
    std::unordered_map<std::string, std::size_t> list_counters;
    std::string line;
    while (std::getline(file, line)) {
      processLine(line, stack, list_counters, callback);
    }
    return true;
  }

private:
  struct StackEntry
  {
    int indent{0};
    std::string key;
    bool is_list_item{false};
    std::size_t list_index{0};
    std::string list_name;
  };

  static void processLine(
    const std::string & raw_line,
    std::vector<StackEntry> & stack,
    std::unordered_map<std::string, std::size_t> & list_counters,
    const YamlCallback & callback)
  {
    const std::string trimmed = trim(raw_line);
    if (trimmed.empty() || trimmed.front() == '#') {
      return;
    }

    const int indent = countLeadingSpaces(raw_line);
    if (trimmed.front() == '-') {
      std::string rest = trimmed.substr(1);
      if (!rest.empty() && std::isspace(static_cast<unsigned char>(rest.front()))) {
        rest.erase(0, 1);
      }

      popForListEntry(indent, stack);
      const std::string parent_path = buildParentPath(stack);
      const std::size_t current_index = list_counters[parent_path]++;

      stack.push_back(
        StackEntry{indent, "<list_item>", true, current_index, parent_path});

      if (!rest.empty()) {
        processKeyValueLine(rest, indent + 2, stack, callback);
      }
      return;
    }

    processKeyValueLine(trimmed, indent, stack, callback);
  }

  static void processKeyValueLine(
    const std::string & line,
    int indent,
    std::vector<StackEntry> & stack,
    const YamlCallback & callback)
  {
    const auto colon = line.find(':');
    if (colon == std::string::npos) {
      return;
    }

    const std::string key = trim(line.substr(0, colon));
    std::string value;
    if (colon + 1 < line.size()) {
      value = trim(line.substr(colon + 1));
    }

    if (value.empty()) {
      popForContainer(indent, stack);
      stack.push_back(StackEntry{indent, key, false, 0, {}});
      return;
    }

    YamlPath path = buildCallbackPath(stack, key);
    callback(path, value);
  }

  static void popForContainer(int indent, std::vector<StackEntry> & stack)
  {
    while (!stack.empty() && indent <= stack.back().indent) {
      stack.pop_back();
    }
  }

  static void popForListEntry(int indent, std::vector<StackEntry> & stack)
  {
    while (!stack.empty()) {
      if (stack.back().indent > indent) {
        stack.pop_back();
        continue;
      }
      if (stack.back().is_list_item && stack.back().indent == indent) {
        stack.pop_back();
        continue;
      }
      break;
    }
  }

  static std::string buildParentPath(const std::vector<StackEntry> & stack)
  {
    std::string result;
    for (const auto & entry : stack) {
      if (entry.is_list_item) {
        continue;
      }
      if (!result.empty()) {
        result.push_back('.');
      }
      result += entry.key;
    }
    return result;
  }

  static YamlPath buildCallbackPath(const std::vector<StackEntry> & stack, const std::string & key)
  {
    YamlPath path;
    path.reserve(stack.size() + 1);
    for (const auto & entry : stack) {
      path.push_back(
        YamlPathEntry{entry.key, entry.is_list_item, entry.list_index, entry.list_name});
    }
    path.push_back(YamlPathEntry{key, false, 0, {}});
    return path;
  }
};

struct LocalizationInput
{
  Header header{};
  Pose pose{};
};

struct WaypointRequest
{
  Header header{};
  Pose goal_pose{};
  std::vector<Pose> waypoints{};
  UUID uuid{};
  bool allow_modification{true};
};

bool parseUuidArray(const std::string & text, UUID & uuid)
{
  std::string contents = trim(text);
  if (!contents.empty() && contents.front() == '[') {
    contents.erase(0, 1);
  }
  if (!contents.empty() && contents.back() == ']') {
    contents.pop_back();
  }

  std::istringstream iss(contents);
  std::string segment;
  std::size_t index = 0;

  while (std::getline(iss, segment, ',')) {
    const std::string token = trim(segment);
    if (token.empty()) {
      continue;
    }
    int value = 0;
    try {
      value = std::stoi(token);
    } catch (...) {
      return false;
    }
    if (index < uuid.size()) {
      uuid[index] = static_cast<std::uint8_t>(value);
    }
    ++index;
  }
  for (; index < uuid.size(); ++index) {
    uuid[index] = 0;
  }
  return true;
}

bool loadLocalizationData(const std::string & path, LocalizationInput & out)
{
  if (!fileExists(path)) {
    std::cerr << "[mission_planner] localization file missing: " << path << std::endl;
    return false;
  }

  bool got_pos_x = false;
  bool got_pos_y = false;
  bool got_pos_z = false;
  bool got_ori_x = false;
  bool got_ori_y = false;
  bool got_ori_z = false;
  bool got_ori_w = false;

  auto callback = [&](const YamlPath & yaml_path, const std::string & raw_value) {
    const std::string value = stripQuotes(raw_value);
    const auto keys = flattenKeys(yaml_path);

    if (pathEndsWith(keys, {"header", "stamp", "sec"})) {
      parseIntLiteral(value, out.header.stamp.sec);
      return;
    }
    if (pathEndsWith(keys, {"header", "stamp", "nanosec"})) {
      parseIntLiteral(value, out.header.stamp.nsec);
      return;
    }
    if (pathEndsWith(keys, {"header", "frame_id"})) {
      out.header.frame_id = value;
      return;
    }

    if (pathEndsWith(keys, {"pose", "pose", "position", "x"})) {
      parseDoubleLiteral(value, out.pose.position.x);
      got_pos_x = true;
      return;
    }
    if (pathEndsWith(keys, {"pose", "pose", "position", "y"})) {
      parseDoubleLiteral(value, out.pose.position.y);
      got_pos_y = true;
      return;
    }
    if (pathEndsWith(keys, {"pose", "pose", "position", "z"})) {
      parseDoubleLiteral(value, out.pose.position.z);
      got_pos_z = true;
      return;
    }
    if (pathEndsWith(keys, {"pose", "pose", "orientation", "x"})) {
      parseDoubleLiteral(value, out.pose.orientation.x);
      got_ori_x = true;
      return;
    }
    if (pathEndsWith(keys, {"pose", "pose", "orientation", "y"})) {
      parseDoubleLiteral(value, out.pose.orientation.y);
      got_ori_y = true;
      return;
    }
    if (pathEndsWith(keys, {"pose", "pose", "orientation", "z"})) {
      parseDoubleLiteral(value, out.pose.orientation.z);
      got_ori_z = true;
      return;
    }
    if (pathEndsWith(keys, {"pose", "pose", "orientation", "w"})) {
      parseDoubleLiteral(value, out.pose.orientation.w);
      got_ori_w = true;
      return;
    }
  };

  if (!MinimalYamlParser::parseFile(path, callback)) {
    return false;
  }

  const bool has_position = got_pos_x && got_pos_y && got_pos_z;
  const bool has_orientation = got_ori_x && got_ori_y && got_ori_z && got_ori_w;

  if (!has_position || !has_orientation) {
    std::cerr << "[mission_planner] incomplete localization pose data\n";
    return false;
  }
  return true;
}

bool loadWaypointRequest(const std::string & path, WaypointRequest & out)
{
  if (!fileExists(path)) {
    std::cerr << "[mission_planner] waypoint file missing: " << path << std::endl;
    return false;
  }

  bool got_goal_pos_x = false;
  bool got_goal_pos_y = false;
  bool got_goal_pos_z = false;
  bool got_goal_ori_x = false;
  bool got_goal_ori_y = false;
  bool got_goal_ori_z = false;
  bool got_goal_ori_w = false;
  bool got_uuid = false;

  auto callback = [&](const YamlPath & yaml_path, const std::string & raw_value) {
    const std::string value = stripQuotes(raw_value);
    const auto keys = flattenKeys(yaml_path);

    if (pathEndsWith(keys, {"header", "stamp", "sec"})) {
      parseIntLiteral(value, out.header.stamp.sec);
      return;
    }
    if (pathEndsWith(keys, {"header", "stamp", "nanosec"})) {
      parseIntLiteral(value, out.header.stamp.nsec);
      return;
    }
    if (pathEndsWith(keys, {"header", "frame_id"})) {
      out.header.frame_id = value;
      return;
    }

    if (pathEndsWith(keys, {"goal_pose", "position", "x"})) {
      parseDoubleLiteral(value, out.goal_pose.position.x);
      got_goal_pos_x = true;
      return;
    }
    if (pathEndsWith(keys, {"goal_pose", "position", "y"})) {
      parseDoubleLiteral(value, out.goal_pose.position.y);
      got_goal_pos_y = true;
      return;
    }
    if (pathEndsWith(keys, {"goal_pose", "position", "z"})) {
      parseDoubleLiteral(value, out.goal_pose.position.z);
      got_goal_pos_z = true;
      return;
    }
    if (pathEndsWith(keys, {"goal_pose", "orientation", "x"})) {
      parseDoubleLiteral(value, out.goal_pose.orientation.x);
      got_goal_ori_x = true;
      return;
    }
    if (pathEndsWith(keys, {"goal_pose", "orientation", "y"})) {
      parseDoubleLiteral(value, out.goal_pose.orientation.y);
      got_goal_ori_y = true;
      return;
    }
    if (pathEndsWith(keys, {"goal_pose", "orientation", "z"})) {
      parseDoubleLiteral(value, out.goal_pose.orientation.z);
      got_goal_ori_z = true;
      return;
    }
    if (pathEndsWith(keys, {"goal_pose", "orientation", "w"})) {
      parseDoubleLiteral(value, out.goal_pose.orientation.w);
      got_goal_ori_w = true;
      return;
    }

    if (const auto index = findListIndex(yaml_path, "waypoints")) {
      if (index.value() >= out.waypoints.size()) {
        out.waypoints.resize(index.value() + 1);
      }
      Pose & target = out.waypoints[index.value()];

      if (pathEndsWith(keys, {"waypoints", "<list_item>", "position", "x"})) {
        parseDoubleLiteral(value, target.position.x);
        return;
      }
      if (pathEndsWith(keys, {"waypoints", "<list_item>", "position", "y"})) {
        parseDoubleLiteral(value, target.position.y);
        return;
      }
      if (pathEndsWith(keys, {"waypoints", "<list_item>", "position", "z"})) {
        parseDoubleLiteral(value, target.position.z);
        return;
      }
      if (pathEndsWith(keys, {"waypoints", "<list_item>", "orientation", "x"})) {
        parseDoubleLiteral(value, target.orientation.x);
        return;
      }
      if (pathEndsWith(keys, {"waypoints", "<list_item>", "orientation", "y"})) {
        parseDoubleLiteral(value, target.orientation.y);
        return;
      }
      if (pathEndsWith(keys, {"waypoints", "<list_item>", "orientation", "z"})) {
        parseDoubleLiteral(value, target.orientation.z);
        return;
      }
      if (pathEndsWith(keys, {"waypoints", "<list_item>", "orientation", "w"})) {
        parseDoubleLiteral(value, target.orientation.w);
        return;
      }
    }

    if (pathEndsWith(keys, {"uuid", "uuid"})) {
      if (parseUuidArray(value, out.uuid)) {
        got_uuid = true;
      }
      return;
    }

    if (pathEndsWith(keys, {"allow_modification"})) {
      parseBoolLiteral(value, out.allow_modification);
      return;
    }
  };

  if (!MinimalYamlParser::parseFile(path, callback)) {
    return false;
  }

  const bool have_goal_position = got_goal_pos_x && got_goal_pos_y && got_goal_pos_z;
  const bool have_goal_orientation =
    got_goal_ori_x && got_goal_ori_y && got_goal_ori_z && got_goal_ori_w;
  if (!have_goal_position || !have_goal_orientation) {
    std::cerr << "[mission_planner] incomplete goal pose in waypoint file\n";
    return false;
  }
  if (!got_uuid) {
    std::cerr << "[mission_planner] uuid not found in waypoint file\n";
    return false;
  }

  return true;
}

void writePoseBlock(std::ostream & out, const std::string & label, const Pose & pose)
{
  out << label << ":\n";
  out << "  position:\n";
  out << "    x: " << formatDouble(pose.position.x) << "\n";
  out << "    y: " << formatDouble(pose.position.y) << "\n";
  out << "    z: " << formatDouble(pose.position.z) << "\n";
  out << "  orientation:\n";
  out << "    x: " << formatDouble(pose.orientation.x) << "\n";
  out << "    y: " << formatDouble(pose.orientation.y) << "\n";
  out << "    z: " << formatDouble(pose.orientation.z) << "\n";
  out << "    w: " << formatDouble(pose.orientation.w) << "\n";
}

bool writeRouteOutput(const LaneletRoute & route, const std::string & path)
{
  std::ofstream out(path, std::ios::trunc);
  if (!out) {
    std::cerr << "[mission_planner] failed to open output file: " << path << std::endl;
    return false;
  }

  out << "header:\n";
  out << "  stamp:\n";
  out << "    sec: " << route.header.stamp.sec << "\n";
  out << "    nanosec: " << route.header.stamp.nsec << "\n";
  out << "  frame_id: " << route.header.frame_id << "\n";

  writePoseBlock(out, "start_pose", route.start_pose);
  writePoseBlock(out, "goal_pose", route.goal_pose);

  out << "segments:\n";
  for (const auto & seg : route.segments) {
    out << "- preferred_primitive:\n";
    out << "    id: " << seg.preferred_primitive.id << "\n";
    if (seg.preferred_primitive.primitive_type.empty()) {
      out << "    primitive_type: ''\n";
    } else {
      out << "    primitive_type: " << seg.preferred_primitive.primitive_type << "\n";
    }
    out << "  primitives:\n";
    for (const auto & prim : seg.primitives) {
      out << "  - id: " << prim.id << "\n";
      out << "    primitive_type: " << prim.primitive_type << "\n";
    }
  }

  out << "uuid:\n";
  out << "  uuid:\n";
  for (const auto & byte : route.uuid) {
    out << "  - " << static_cast<int>(byte) << "\n";
  }
  out << "allow_modification: " << (route.allow_modification ? "true" : "false") << "\n";
  out << "---\n";

  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string io_root = "../reference_IO";
  std::string map_root = "../sample-map-planning";

  if (!fileExists(io_root + "/localization_kinematicstate.txt") ||
      !fileExists(io_root + "/setwaypointroute.txt")) {
    io_root = "reference_IO";
  }
  if (!fileExists(map_root + "/lanelet2_map.osm")) {
    map_root = "sample-map-planning";
  }

  if (argc > 1) {
    io_root = argv[1];
  }
  if (argc > 2) {
    map_root = argv[2];
  }

  const std::string localization_file = io_root + "/localization_kinematicstate.txt";
  const std::string waypoint_file = io_root + "/setwaypointroute.txt";
  const std::string osm_file = map_root + "/lanelet2_map.osm";
  constexpr const char * output_file = "mission_planner.txt";

  LocalizationInput ego_state{};
  if (!loadLocalizationData(localization_file, ego_state)) {
    return 1;
  }

  WaypointRequest request{};
  if (!loadWaypointRequest(waypoint_file, request)) {
    return 1;
  }

  if (!fileExists(osm_file)) {
    std::cerr << "[mission_planner] map file not found: " << osm_file << std::endl;
    return 1;
  }

  DefaultPlannerParam param{};
  SimpleVehicleInfo vehicle_info{};
  auto planner = std::make_shared<DefaultPlanner>(param, vehicle_info);

  lanelet::GPSPoint origin_gps;
  origin_gps.lat = 35.23808753540768;
  origin_gps.lon = 139.9009591876285;
  origin_gps.ele = 0.0;
  lanelet::Origin origin(origin_gps);
  planner->loadOsmMap(osm_file, origin);

  TransformPoseFn tf_fn = [](const Pose & p, const Header &) {
    return p;
  };

  MissionPlanner core("map", planner, tf_fn);
  const auto route = core.makeRouteFromWaypoints(
    request.header,
    request.waypoints,
    ego_state.pose,
    request.goal_pose,
    request.uuid,
    request.allow_modification);

  if (route.segments.empty()) {
    std::cerr << "[mission_planner] generated route is empty\n";
    return 1;
  }

  if (!writeRouteOutput(route, output_file)) {
    return 1;
  }

  std::cout << "mission planner output written to " << output_file << std::endl;
  return 0;
}
