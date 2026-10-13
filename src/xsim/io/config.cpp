#include "config.hpp"
#include <algorithm>
#include <cerrno>
#include <fcntl.h>
#include <filesystem>
#include <set>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>
#include <yaml-cpp/yaml.h>
namespace xsim {
namespace {
Eigen::Vector3d vec3(const Json &v) {
  if (!v.is_array() || v.size() != 3)
    throw std::invalid_argument("expected three numbers");
  Eigen::Vector3d r(v[0].get<double>(), v[1].get<double>(), v[2].get<double>());
  if (!r.allFinite())
    throw std::invalid_argument("nonfinite vector");
  return r;
}
Json yaml_json(const YAML::Node &n) {
  if(n.IsNull())return nullptr;
  if(n.IsSequence()){Json j=Json::array();for(const auto& x:n)j.push_back(yaml_json(x));return j;}
  if(n.IsMap()){Json j=Json::object();for(const auto& x:n)j[x.first.as<std::string>()]=yaml_json(x.second);return j;}
  auto text=n.as<std::string>();
  // Quoted and explicitly tagged strings keep their YAML type. Scene IDs may
  // look numeric without being numbers.
  if(n.Tag()=="!" || n.Tag()=="tag:yaml.org,2002:str")return text;
  if(text=="true")return true;
  if(text=="false")return false;
  try {size_t end=0;double value=std::stod(text,&end);if(end==text.size())return value;}catch(const std::exception&){}
  return text;
}
} // namespace
Config parse_entity(const Json &j) {
  Config c;
  c.name = j.at("name").get<std::string>();
  if (c.name.empty() ||
      c.name.find_first_not_of(
          "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_") !=
          std::string::npos)
    throw std::invalid_argument("name must be a ROS namespace segment");
  const auto kind = j.at("kind").get<std::string>();
  if (kind == "fs150")
    c.kind = Kind::FS150;
  else if (kind == "scout")
    c.kind = Kind::Scout;
  else if (kind == "mecanum")
    c.kind = Kind::Mecanum;
  else
    throw std::invalid_argument("unknown kind");
  if (j.contains("position"))
    c.initial = vec3(j.at("position"));
  c.yaw = j.value("yaw", 0.0);
  c.ground_z = j.value("ground_z", 0.0);
  if (!std::isfinite(c.yaw) || !std::isfinite(c.ground_z))
    throw std::invalid_argument("nonfinite pose");
  if (j.contains("local_origin"))
    c.local_origin = vec3(j.at("local_origin"));
  if (j.contains("fcu_parameters")) {
    if (c.kind != Kind::FS150)
      throw std::invalid_argument("FCU parameters require fs150");
    for (auto i = j.at("fcu_parameters").begin();
         i != j.at("fcu_parameters").end(); ++i)
      if (!c.fcu.set(i.key(), i.value().get<double>()))
        throw std::invalid_argument("invalid FCU parameter: " + i.key());
  }
  const auto ros = j.value("ros", Json::object());
  for (const char *key : {"provider_service", "truth_topic", "reset_service",
                          "mocap_topic", "mocap_velocity_topic"})
    if (ros.contains(key))
      throw std::invalid_argument("unsupported ROS configuration: " + std::string(key));
  if (c.kind != Kind::FS150)
    for (const char *key : {"pose_topic", "velocity_topic", "odometry_topic"})
      if (ros.contains(key))
        throw std::invalid_argument("UGV ROS configuration is not supported: " + std::string(key));
  return c;
}
Json load_json_object(const std::string &path) {
  constexpr std::size_t limit = 1024 * 1024;
  struct File {
    int fd;
    ~File() { if (fd >= 0) close(fd); }
  } file{open(path.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW | O_NONBLOCK)};
  if (file.fd < 0) throw std::runtime_error("cannot read configuration without symbolic links");
  struct stat metadata{};
  if (fstat(file.fd, &metadata) != 0 || !S_ISREG(metadata.st_mode) ||
      metadata.st_size <= 0 || metadata.st_size > static_cast<off_t>(limit))
    throw std::invalid_argument("configuration must be a nonempty regular file <=1 MiB");
  std::string contents;
  contents.reserve(static_cast<std::size_t>(metadata.st_size));
  char buffer[8192];
  for (;;) {
    const auto count = read(file.fd, buffer, std::min(sizeof(buffer), limit + 1 - contents.size()));
    if (count < 0 && errno == EINTR) continue;
    if (count < 0) throw std::runtime_error("cannot read configuration");
    if (count == 0) break;
    contents.append(buffer, static_cast<std::size_t>(count));
    if (contents.size() > limit) throw std::invalid_argument("configuration exceeds 1 MiB");
  }
  return parse_json_object(contents);
}
Json parse_json_object(const std::string &contents) {
  std::vector<std::set<std::string>> object_keys;
  const auto unique_keys = [&object_keys](int, Json::parse_event_t event, Json &value) {
    if (event == Json::parse_event_t::object_start) object_keys.emplace_back();
    else if (event == Json::parse_event_t::object_end) object_keys.pop_back();
    else if (event == Json::parse_event_t::key &&
             !object_keys.back().insert(value.get<std::string>()).second)
      throw std::invalid_argument("duplicate configuration key: " + value.get<std::string>());
    return true;
  };
  Json j = Json::parse(contents, unique_keys);
  if (!j.is_object()) throw std::invalid_argument("configuration must be one JSON object");
  return j;
}
Json resolve_scene_file(Json j, const std::string &scene_file) {
  if (!scene_file.empty()) {
    const auto configured = j.value("scene_file", std::string{});
    if (!configured.empty() && configured != scene_file)
      throw std::invalid_argument("--scene-file conflicts with configuration scene_file");
    j["scene_file"] = scene_file;
  }
  if(j.contains("scene_file") && !j.at("scene_file").get<std::string>().empty()) {
    const auto path = j.at("scene_file").get<std::string>();
    j["scene"]["document"] = yaml_json(YAML::LoadFile(path));
    const auto manifest = std::filesystem::path(path).parent_path() / "manifest.json";
    // Saved point-cloud scenes are sampled by the shared scene fleet on both
    // simulators. Do not publish a second empty geometry cloud on these topics.
    if (std::filesystem::is_regular_file(manifest) &&
        load_json_object(manifest.string()).contains("pointCloud")) {
      j.erase("reference_cloud_topic");
      for (auto& entity : j["entities"]) entity.erase("sensor");
    }
  }
  return j;
}
Json load_config(const std::string &path, const std::string &scene_file) {
 return resolve_scene_file(load_json_object(path), scene_file);
}
} // namespace xsim

namespace xsim {
namespace {
Json inline_artifact(const Json& asset, const char* media_type) {
  const auto& realization = asset.at("realization");
  if (asset.at("id").get<std::string>().empty()) throw std::invalid_argument("asset ID required");
  if (realization.at("media_type") != media_type) throw std::invalid_argument("unsupported asset realization media_type");
  if (!realization.contains("content") || realization.contains("uri"))
    throw std::invalid_argument("this xsim realization requires inline content; URI resolution is not configured");
  const auto& content = realization.at("content");
  const auto value = content.is_string() ? Json::parse(content.get<std::string>()) : content;
  if (!value.is_object()) throw std::invalid_argument("asset content must be a JSON object");
  return value;
}
void public_pose(Json& native, const Json& pose) {
  const auto position = vec3(pose.at("position"));
  const auto& q = pose.at("orientation");
  if (!q.is_array() || q.size() != 4) throw std::invalid_argument("orientation must be an xyzw unit quaternion");
  Eigen::Quaterniond orientation(q[3].get<double>(), q[0].get<double>(), q[1].get<double>(), q[2].get<double>());
  if (!orientation.coeffs().allFinite() || std::abs(orientation.norm() - 1.0) > 1e-6)
    throw std::invalid_argument("orientation must be a finite unit quaternion");
  if (std::abs(orientation.x()) > 1e-8 || std::abs(orientation.y()) > 1e-8)
    throw std::invalid_argument("xsim robot assets support yaw-only initial orientation");
  native["position"] = {position.x(), position.y(), position.z()};
  native["yaw"] = 2.0 * std::atan2(orientation.z(), orientation.w());
}
}
Json entity_artifact(const Json& specification) {
  for (auto i = specification.begin(); i != specification.end(); ++i)
    if (i.key() != "id" && i.key() != "role" && i.key() != "asset" && i.key() != "pose" && i.key() != "parameters" && i.key() != "sensors")
      throw std::invalid_argument("unknown entity specification field: " + i.key());
  if (specification.at("role") != "robot") throw std::invalid_argument("this xsim realization supports robot entities");
  if ((specification.contains("parameters") && !specification.at("parameters").empty()) ||
      (specification.contains("sensors") && !specification.at("sensors").empty()))
    throw std::invalid_argument("entity extension parameters and sensor specs are not supported by this artifact schema");
  auto native = inline_artifact(specification.at("asset"), "application/vnd.xgc2.xsim.entity+json");
  public_pose(native, specification.at("pose"));
  (void)parse_entity(native);
  return native;
}
Json load_manifest(const std::string& path) {
  const auto manifest = load_config(path);
  if (manifest.at("api_version") != 1) throw std::invalid_argument("manifest api_version must be 1");
  for (auto i = manifest.begin(); i != manifest.end(); ++i)
    if (i.key() != "api_version" && i.key() != "world" && i.key() != "entities") throw std::invalid_argument("unknown manifest field: " + i.key());
  auto config = inline_artifact(manifest.at("world").at("asset"), "application/vnd.xgc2.xsim.world+json");
  if (config.contains("instance_id") || config.contains("entities")) throw std::invalid_argument("world artifact cannot supply instance_id or entities; manifest owns membership");
  if (!config.contains("epoch_ns")) throw std::invalid_argument("world artifact requires an explicit epoch_ns");
  config["entities"] = Json::array();
  for (const auto& specification : manifest.value("entities", Json::array())) {
    auto native = entity_artifact(specification);
    native["public_id"] = specification.value("id", native.at("name").get<std::string>());
    native["specification"] = specification;
    config["entities"].push_back(std::move(native));
  }
  if (config["entities"].size() > 256) throw std::invalid_argument("manifest entity limit exceeded");
  return config;
}
} // namespace xsim
