#pragma once
#include <sensor_msgs/PointCloud2.h>
#include <string>

namespace xsim {
// Point-field metadata only. Payload bytes, topics, and sensor defaults stay
// with the caller. Without bodies the cloud is XYZ. With bodies the fourth
// field is vehicle_id:INT32. The GPU module ABI is not described here.
inline sensor_msgs::PointCloud2 cloudPointLayout(bool with_bodies) {
  sensor_msgs::PointCloud2 cloud;
  cloud.fields.resize(with_bodies ? 4 : 3);
  for (unsigned a = 0; a < cloud.fields.size(); ++a) {
    auto &field = cloud.fields[a];
    field.offset = a * 4;
    field.count = 1;
    if (a < 3) {
      field.name = std::string(1, "xyz"[a]);
      field.datatype = sensor_msgs::PointField::FLOAT32;
    } else {
      field.name = "vehicle_id";
      field.datatype = sensor_msgs::PointField::INT32;
    }
  }
  cloud.point_step = with_bodies ? 16 : 12;
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.is_bigendian = false;
  return cloud;
}
} // namespace xsim
