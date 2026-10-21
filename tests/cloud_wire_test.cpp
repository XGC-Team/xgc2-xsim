#include "io/ros/cloud_layout.hpp"
#include "io/ros/point_cloud_view.hpp"
#include <cassert>
#include <cstdint>
#include <cstring>
#include <string>

namespace {
void expectField(const sensor_msgs::PointField &field, const std::string &name,
                 uint32_t offset, uint8_t datatype) {
  assert(field.name == name);
  assert(field.offset == offset);
  assert(field.datatype == datatype);
  assert(field.count == 1);
}

void roundTrip(sensor_msgs::PointCloud2 metadata, uint32_t width) {
  metadata.header.seq = 23;
  metadata.header.stamp.fromNSec(1700000000123456789LL);
  metadata.header.frame_id = "custom_world_frame";
  metadata.width = width;
  metadata.row_step = metadata.width * metadata.point_step;
  std::vector<uint8_t> payload(metadata.row_step);
  for (size_t i = 0; i < payload.size(); ++i) payload[i] = uint8_t(i * 31);
  const xsim::PointCloudView view{metadata, payload};
  auto normal = metadata;
  normal.data = payload;
  const auto length = ros::serialization::serializationLength(normal);
  assert(ros::serialization::serializationLength(view) == length);
  std::vector<uint8_t> expected(length), actual(length);
  ros::serialization::OStream normal_stream(expected.data(), length);
  ros::serialization::OStream view_stream(actual.data(), length);
  ros::serialization::serialize(normal_stream, normal);
  ros::serialization::serialize(view_stream, view);
  assert(actual == expected);
  sensor_msgs::PointCloud2 decoded;
  ros::serialization::IStream input(actual.data(), length);
  ros::serialization::deserialize(input, decoded);
  assert(decoded.data == payload);
  assert(decoded.header.frame_id == metadata.header.frame_id);
  assert(decoded.header.stamp == metadata.header.stamp);
  assert(decoded.width == metadata.width && decoded.point_step == metadata.point_step);
  assert(decoded.fields.size() == metadata.fields.size());
  for (size_t i = 0; i < metadata.fields.size(); ++i) {
    assert(decoded.fields[i].name == metadata.fields[i].name);
    assert(decoded.fields[i].datatype == metadata.fields[i].datatype);
    assert(decoded.fields[i].offset == metadata.fields[i].offset);
    assert(decoded.fields[i].count == metadata.fields[i].count);
  }
  assert(metadata.data.empty());
}
} // namespace

int main() {
  sensor_msgs::PointCloud2 metadata;
  metadata.height = 1;
  metadata.is_dense = true;
  metadata.is_bigendian = false;
  const char *names[] = {"x", "y", "z", "vehicle_id"};
  for (unsigned n : {0u, 9u}) {
    metadata.fields.clear();
    for (unsigned i = 0; i < n; ++i) {
      sensor_msgs::PointField field;
      field.name = names[i % 4];
      field.offset = i * 4;
      field.datatype = i == 3 ? sensor_msgs::PointField::INT32
                            : sensor_msgs::PointField::FLOAT32;
      field.count = 1;
      metadata.fields.push_back(field);
    }
    metadata.point_step = n * 4;
    roundTrip(metadata, n ? 17 : 0);
  }

  const auto plain = xsim::cloudPointLayout(false);
  assert(plain.fields.size() == 3 && plain.point_step == 12);
  expectField(plain.fields[0], "x", 0, sensor_msgs::PointField::FLOAT32);
  expectField(plain.fields[1], "y", 4, sensor_msgs::PointField::FLOAT32);
  expectField(plain.fields[2], "z", 8, sensor_msgs::PointField::FLOAT32);
  for (const auto &field : plain.fields) assert(field.name != "intensity");
  roundTrip(plain, 17);

  const auto body = xsim::cloudPointLayout(true);
  assert(body.fields.size() == 4 && body.point_step == 16);
  expectField(body.fields[3], "vehicle_id", 12, sensor_msgs::PointField::INT32);
  for (unsigned a = 0; a < 3; ++a)
    expectField(body.fields[a], std::string(1, "xyz"[a]), a * 4,
                sensor_msgs::PointField::FLOAT32);
  roundTrip(body, 17);
}
