#include "io/config.hpp"
#include <cassert>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <sys/stat.h>
#include <unistd.h>

namespace {
template <class Function> void rejects(Function function, const std::string &message = "") {
  bool rejected = false;
  try { function(); }
  catch (const std::exception &error) {
    rejected = true;
    if (!message.empty()) assert(std::string(error.what()).find(message) != std::string::npos);
  }
  assert(rejected);
}
}

int main() {
  char pattern[] = "/tmp/xsim-config-test-XXXXXX";
  const char *created = mkdtemp(pattern);
  assert(created);
  const std::filesystem::path root(created);
  struct Remove { std::filesystem::path root; ~Remove() { std::filesystem::remove_all(root); } } remove{root};
  const auto configuration = (root / "world.json").string();
  const auto scene = (root / "scene.yaml").string();
  const auto other = (root / "other.yaml").string();
  std::ofstream(scene) << "name: original-scene\nenabled: true\norigin: [1, 2.5, -3]\nobjects:\n  - id: '1'\n    name: \"false\"\n    label: !!str 42\n    vertices: [[0, 0, 0], [1, 2, 3]]\n";
  std::ofstream(other) << "name: other-scene\n";
  xsim::Json original{{"instance_id", "frozen-instance"}, {"epoch_ns", int64_t(1791250000000000000)},
                      {"entities", xsim::Json::array()}, {"scene", {{"keep", "source-value"}}}};
  auto write = [&](const xsim::Json &value) { std::ofstream(configuration) << value.dump(); };
  write(original);
  assert(xsim::load_config(configuration) == original);
  assert(xsim::load_config(configuration, "") == original);
  auto from_flag = xsim::load_config(configuration, scene);
  auto configured = original;
  configured["scene_file"] = scene;
  write(configured);
  const auto from_json = xsim::load_config(configuration);
  assert(from_flag == from_json);
  assert(xsim::load_config(configuration, scene) == from_json);
  assert(xsim::load_config(configuration, "") == from_json);
  assert(from_flag.at("epoch_ns") == original.at("epoch_ns"));
  assert(from_flag.at("instance_id") == original.at("instance_id"));
  assert(from_flag.at("entities") == original.at("entities"));
  assert(from_flag.at("scene").at("keep") == "source-value");
  const auto &document = from_flag.at("scene").at("document");
  assert(document.at("name") == "original-scene");
  assert(document.at("enabled") == true);
  assert(document.at("origin").at(1) == 2.5);
  assert(document.at("objects").at(0).at("id").get<std::string>() == "1");
  assert(document.at("objects").at(0).at("name").get<std::string>() == "false");
  assert(document.at("objects").at(0).at("label").get<std::string>() == "42");
  rejects([&] { xsim::load_config(configuration, other); }, "conflicts");
  // Conflict precedes loading either file; do not silently prefer a new path.
  configured["scene_file"] = (root / "missing-from-config.yaml").string();
  write(configured);
  rejects([&] { xsim::load_config(configuration, other); }, "conflicts");
  configured["scene_file"] = "";
  write(configured);
  assert(xsim::load_config(configuration, scene) == from_json);
  rejects([&] { xsim::load_config(configuration, (root / "missing.yaml").string()); });
  std::ofstream(other) << "objects: [not-closed\n";
  rejects([&] { xsim::load_config(configuration, other); });
  configured["scene_file"] = 5;
  write(configured);
  rejects([&] { xsim::load_config(configuration, scene); });
  assert(configured.at("epoch_ns") == original.at("epoch_ns"));
  // Native fields retain their original types/values; only the JSON boundary
  // is stricter, not a second limited Core-side product DTO.
  auto native = original;
  native["native_extension"] = {{"weights", {1, 2.5, -3}}, {"label", "source"}};
  write(native);
  assert(xsim::load_config(configuration) == native);
  for (const auto &text : {"{} {}", "{} trailing", "{\"epoch_ns\":1,\"epoch_ns\":2}",
                          "{\"outer\":[{\"x\":1,\"x\":2}]}", "[]", "{\"x\":NaN}"}) {
    std::ofstream(configuration) << text;
    rejects([&] { xsim::load_config(configuration); });
  }
  std::ofstream(configuration) << "{\"a\":{\"x\":1},\"b\":{\"x\":2}} \n\t";
  assert(xsim::load_config(configuration).at("b").at("x") == 2);
  xsim::Json boundary{{"padding", ""}};
  boundary["padding"] = std::string(1024 * 1024 - boundary.dump().size(), 'x');
  assert(boundary.dump().size() == 1024 * 1024);
  write(boundary);
  assert(xsim::load_config(configuration) == boundary);
  std::ofstream(configuration) << std::string(1024 * 1024 + 1, ' ');
  rejects([&] { xsim::load_config(configuration); }, "1 MiB");
  std::ofstream(configuration) << "";
  rejects([&] { xsim::load_config(configuration); }, "nonempty regular");
  rejects([&] { xsim::load_config(root.string()); }, "regular");
  const auto fifo = (root / "fifo").string();
  assert(mkfifo(fifo.c_str(), 0600) == 0);
  rejects([&] { xsim::load_config(fifo); }, "regular");
  const auto link = (root / "linked.json").string();
  assert(symlink(configuration.c_str(), link.c_str()) == 0);
  rejects([&] { xsim::load_config(link); }, "symbolic links");
  std::cout << "scene flag/config parity, empty flag, conflict, parser errors and frozen epoch passed\n";
}
