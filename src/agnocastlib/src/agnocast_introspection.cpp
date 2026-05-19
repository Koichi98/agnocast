#include "agnocast/agnocast_introspection.hpp"

#include "dynmsg/message_reading.hpp"
#include "dynmsg/typesupport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <utility>

// This file is a thin wrapper over dynmsg (osrf/dynamic_message_introspection): dynmsg owns the
// runtime type resolution (dlopen of the rosidl_typesupport_introspection_cpp library) and the
// payload -> YAML walker. agnocast only adds the small glue that the GenericSubscription-based
// CLI tools need: parsing the "<pkg>/msg/<MsgName>" string and resolving named-field offsets.

namespace agnocast
{

namespace
{

using rosidl_typesupport_introspection_cpp::MessageMember;
using rosidl_typesupport_introspection_cpp::MessageMembers;

// Parse "<pkg>/msg/<MsgName>" into a dynmsg InterfaceTypeName (pkg, MsgName).
std::optional<InterfaceTypeName> parse_type_name(const std::string & type_name)
{
  const std::string sep = "/msg/";
  const auto pos = type_name.find(sep);
  std::string pkg;
  std::string msg_name;
  if (pos != std::string::npos) {
    pkg = type_name.substr(0, pos);
    msg_name = type_name.substr(pos + sep.size());
  }
  if (pkg.empty() || msg_name.empty()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("agnocast.introspection"),
      "Malformed type name '%s' (expected <pkg>/msg/<MsgName>)", type_name.c_str());
    return std::nullopt;
  }
  return InterfaceTypeName{std::move(pkg), std::move(msg_name)};
}

// Resolve the top-level introspection metadata via dynmsg, which dlopen's the package's
// rosidl_typesupport_introspection_cpp library. Returns nullptr on failure (dynmsg logs the
// underlying reason, e.g. library not loadable).
const MessageMembers * load_message_members(const std::string & type_name)
{
  auto interface = parse_type_name(type_name);
  if (!interface) return nullptr;
  return dynmsg::cpp::get_type_info(*interface);
}

// Find a top-level member by name. Returns nullptr if absent.
const MessageMember * find_member(const MessageMembers * members, const char * name)
{
  if (members == nullptr) return nullptr;
  for (uint32_t i = 0; i < members->member_count_; ++i) {
    if (std::strcmp(members->members_[i].name_, name) == 0) {
      return &members->members_[i];
    }
  }
  return nullptr;
}

// For ROS_TYPE_MESSAGE fields, follow the type-support nesting to its MessageMembers.
const MessageMembers * nested_members(const MessageMember * m)
{
  if (m == nullptr || m->members_ == nullptr) return nullptr;
  return static_cast<const MessageMembers *>(m->members_->data);
}

}  // namespace

std::optional<StampOffsets> resolve_header_stamp_offsets(const std::string & type_name)
{
  const MessageMembers * members = load_message_members(type_name);
  if (members == nullptr) return std::nullopt;

  const MessageMember * header_m = find_member(members, "header");
  if (header_m == nullptr) return std::nullopt;
  const MessageMembers * header_members = nested_members(header_m);
  if (header_members == nullptr) return std::nullopt;

  const MessageMember * stamp_m = find_member(header_members, "stamp");
  if (stamp_m == nullptr) return std::nullopt;
  const MessageMembers * stamp_members = nested_members(stamp_m);
  if (stamp_members == nullptr) return std::nullopt;

  const MessageMember * sec_m = find_member(stamp_members, "sec");
  const MessageMember * nanosec_m = find_member(stamp_members, "nanosec");
  if (sec_m == nullptr || nanosec_m == nullptr) return std::nullopt;

  StampOffsets out{};
  out.sec_offset = static_cast<size_t>(header_m->offset_) + static_cast<size_t>(stamp_m->offset_) +
                   static_cast<size_t>(sec_m->offset_);
  out.nanosec_offset = static_cast<size_t>(header_m->offset_) +
                       static_cast<size_t>(stamp_m->offset_) +
                       static_cast<size_t>(nanosec_m->offset_);
  return out;
}

std::optional<FieldInfo> resolve_field_offset(
  const std::string & type_name, const std::string & field_name)
{
  const MessageMembers * members = load_message_members(type_name);
  if (members == nullptr) return std::nullopt;

  const MessageMember * m = find_member(members, field_name.c_str());
  if (m == nullptr) return std::nullopt;

  FieldInfo out{};
  out.offset = static_cast<size_t>(m->offset_);
  out.type_id = m->type_id_;
  out.is_array = m->is_array_;
  return out;
}

std::optional<std::string> dump_message_yaml(const std::string & type_name, const void * payload)
{
  if (payload == nullptr) return std::nullopt;
  const MessageMembers * members = load_message_members(type_name);
  if (members == nullptr) return std::nullopt;

  // dynmsg walks the payload (the in-memory C++ struct image returned by
  // GenericSubscription) against the introspection metadata and builds a YAML tree;
  // YAML::Dump renders it to text. RosMessage_Cpp::data is non-const, but message_to_yaml
  // only reads it.
  RosMessage_Cpp ros_msg{members, const_cast<uint8_t *>(static_cast<const uint8_t *>(payload))};
  const YAML::Node node = dynmsg::cpp::message_to_yaml(ros_msg);
  return YAML::Dump(node);
}

}  // namespace agnocast
