#pragma once

#include "agnocast/agnocast_public_api.hpp"

#include <cstddef>
#include <optional>
#include <string>

namespace agnocast
{

// Byte offsets of header.stamp fields within a message's in-memory C++ struct image.
// Used by hz/delay-style CLI tools to read timestamps from a payload without typed access.
struct StampOffsets
{
  size_t sec_offset;      // int32_t in builtin_interfaces::msg::Time
  size_t nanosec_offset;  // uint32_t in builtin_interfaces::msg::Time
};

// Resolves the offsets of header.stamp.sec / header.stamp.nanosec inside a message struct,
// by runtime-dlopen'ing the message package's rosidl_typesupport_introspection_cpp library.
//
// type_name format: "<pkg>/msg/<MsgName>" (e.g. "sensor_msgs/msg/Imu").
//
// Returns std::nullopt when:
// - the type does not contain a `std_msgs::msg::Header header` field,
// - or the introspection library cannot be loaded / looked up.
AGNOCAST_PUBLIC
std::optional<StampOffsets> resolve_header_stamp_offsets(const std::string & type_name);

// Metadata of a top-level field inside a message struct.
struct FieldInfo
{
  size_t offset;     // byte offset from the start of the struct
  uint8_t type_id;   // rosidl_typesupport_introspection_cpp::ROS_TYPE_*
  bool is_array;
};

// Looks up a top-level field by name. Returns std::nullopt when the type cannot be loaded
// or when the field is absent. Nested-field traversal is not supported here; callers that
// need dotted paths should compose multiple lookups manually.
AGNOCAST_PUBLIC
std::optional<FieldInfo> resolve_field_offset(
  const std::string & type_name, const std::string & field_name);

// Renders a payload (the C++ struct image returned by GenericSubscription::drain) into a
// YAML-like multi-line string using rosidl_typesupport_introspection_cpp. Supports nested
// messages, fixed arrays, and bounded/unbounded sequences for all builtin field types.
//
// Returns std::nullopt on the same failure modes as resolve_header_stamp_offsets
// (introspection lib not loadable, malformed type name).
//
// This is the generic counterpart of the `MessageT::to_yaml()` trait used by typed `ros2
// topic echo`. The caller passes only the type name at runtime, so no per-type plugin is
// required.
AGNOCAST_PUBLIC
std::optional<std::string> dump_message_yaml(
  const std::string & type_name, const void * payload);

}  // namespace agnocast
