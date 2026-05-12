#include "agnocast/agnocast_introspection.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

#include <dlfcn.h>

#include <cstdint>
#include <cstring>
#include <sstream>
#include <string>

namespace agnocast
{

namespace
{

using rosidl_typesupport_introspection_cpp::MessageMember;
using rosidl_typesupport_introspection_cpp::MessageMembers;
namespace tsi = rosidl_typesupport_introspection_cpp;

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

// Parses "<pkg>/msg/<MsgName>" into (pkg, MsgName). Returns false on malformed input.
bool parse_type_name(const std::string & type_name, std::string & pkg, std::string & msg_name)
{
  const std::string sep = "/msg/";
  const auto pos = type_name.find(sep);
  if (pos == std::string::npos) return false;
  pkg = type_name.substr(0, pos);
  msg_name = type_name.substr(pos + sep.size());
  return !pkg.empty() && !msg_name.empty();
}

// Open the message package's introspection .so and return its top-level MessageMembers.
const MessageMembers * load_message_members(const std::string & type_name)
{
  std::string pkg;
  std::string msg_name;
  if (!parse_type_name(type_name, pkg, msg_name)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("agnocast.introspection"),
      "Malformed type name '%s' (expected <pkg>/msg/<MsgName>)", type_name.c_str());
    return nullptr;
  }

  const std::string lib_name = "lib" + pkg + "__rosidl_typesupport_introspection_cpp.so";
  void * handle = dlopen(lib_name.c_str(), RTLD_LAZY | RTLD_NODELETE);
  if (handle == nullptr) {
    RCLCPP_ERROR(
      rclcpp::get_logger("agnocast.introspection"), "dlopen('%s') failed: %s", lib_name.c_str(),
      dlerror());
    return nullptr;
  }

  const std::string sym =
    "rosidl_typesupport_introspection_cpp__get_message_type_support_handle__" + pkg + "__msg__" +
    msg_name;
  using Getter = const rosidl_message_type_support_t * (*)();
  auto * getter = reinterpret_cast<Getter>(dlsym(handle, sym.c_str()));
  if (getter == nullptr) {
    RCLCPP_ERROR(
      rclcpp::get_logger("agnocast.introspection"), "dlsym('%s') failed: %s", sym.c_str(),
      dlerror());
    return nullptr;
  }

  const rosidl_message_type_support_t * ts = getter();
  if (ts == nullptr || ts->data == nullptr) {
    RCLCPP_ERROR(
      rclcpp::get_logger("agnocast.introspection"),
      "Type support for '%s' returned null payload", type_name.c_str());
    return nullptr;
  }
  return static_cast<const MessageMembers *>(ts->data);
}

void indent_to(std::ostringstream & out, int depth)
{
  for (int i = 0; i < depth; ++i) out << "  ";
}

// Format one scalar at element_ptr according to type_id. For ROS_TYPE_MESSAGE, descend.
void format_scalar(
  std::ostringstream & out, const uint8_t * element_ptr, uint8_t type_id,
  const MessageMembers * sub_members, int depth);

void dump_message(
  std::ostringstream & out, const uint8_t * base, const MessageMembers * members, int depth);

void format_scalar(
  std::ostringstream & out, const uint8_t * p, uint8_t type_id,
  const MessageMembers * sub_members, int depth)
{
  switch (type_id) {
    case tsi::ROS_TYPE_FLOAT32:
      out << *reinterpret_cast<const float *>(p);
      break;
    case tsi::ROS_TYPE_FLOAT64:
      out << *reinterpret_cast<const double *>(p);
      break;
    case tsi::ROS_TYPE_LONG_DOUBLE:
      out << static_cast<double>(*reinterpret_cast<const long double *>(p));
      break;
    case tsi::ROS_TYPE_CHAR:
      out << static_cast<int>(*reinterpret_cast<const char *>(p));
      break;
    case tsi::ROS_TYPE_WCHAR:
      out << static_cast<unsigned>(*reinterpret_cast<const uint16_t *>(p));
      break;
    case tsi::ROS_TYPE_BOOLEAN:
      out << (*reinterpret_cast<const bool *>(p) ? "true" : "false");
      break;
    case tsi::ROS_TYPE_OCTET:
    case tsi::ROS_TYPE_UINT8:
      out << static_cast<unsigned>(*reinterpret_cast<const uint8_t *>(p));
      break;
    case tsi::ROS_TYPE_INT8:
      out << static_cast<int>(*reinterpret_cast<const int8_t *>(p));
      break;
    case tsi::ROS_TYPE_UINT16:
      out << *reinterpret_cast<const uint16_t *>(p);
      break;
    case tsi::ROS_TYPE_INT16:
      out << *reinterpret_cast<const int16_t *>(p);
      break;
    case tsi::ROS_TYPE_UINT32:
      out << *reinterpret_cast<const uint32_t *>(p);
      break;
    case tsi::ROS_TYPE_INT32:
      out << *reinterpret_cast<const int32_t *>(p);
      break;
    case tsi::ROS_TYPE_UINT64:
      out << *reinterpret_cast<const uint64_t *>(p);
      break;
    case tsi::ROS_TYPE_INT64:
      out << *reinterpret_cast<const int64_t *>(p);
      break;
    case tsi::ROS_TYPE_STRING:
      out << "'" << *reinterpret_cast<const std::string *>(p) << "'";
      break;
    case tsi::ROS_TYPE_WSTRING:
      out << "<wstring>";  // not bothered for PoC
      break;
    case tsi::ROS_TYPE_MESSAGE: {
      if (sub_members == nullptr) {
        out << "<null-message>";
      } else {
        out << "\n";
        dump_message(out, p, sub_members, depth + 1);
      }
      break;
    }
    default:
      out << "<type_id=" << static_cast<unsigned>(type_id) << ">";
      break;
  }
}

void dump_member(
  std::ostringstream & out, const uint8_t * base, const MessageMember & m, int depth)
{
  indent_to(out, depth);
  out << m.name_ << ":";

  const uint8_t * field_base = base + m.offset_;
  const MessageMembers * sub = (m.type_id_ == tsi::ROS_TYPE_MESSAGE) ? nested_members(&m) : nullptr;

  if (!m.is_array_) {
    // Singular field
    if (m.type_id_ == tsi::ROS_TYPE_MESSAGE) {
      format_scalar(out, field_base, m.type_id_, sub, depth);
    } else {
      out << " ";
      format_scalar(out, field_base, m.type_id_, sub, depth);
      out << "\n";
    }
    return;
  }

  // Array or sequence: use size_function / get_const_function so we work for both
  // std::array<T,N> (fixed) and std::vector<T> (sequence) layouts uniformly.
  size_t size = 0;
  if (m.size_function != nullptr) {
    size = m.size_function(field_base);
  } else {
    size = m.array_size_;  // fixed array fallback
  }

  if (size == 0) {
    out << " []\n";
    return;
  }

  out << "\n";
  for (size_t i = 0; i < size; ++i) {
    const void * elem = nullptr;
    if (m.get_const_function != nullptr) {
      elem = m.get_const_function(field_base, i);
    }
    if (elem == nullptr) {
      indent_to(out, depth + 1);
      out << "- <null>\n";
      continue;
    }
    indent_to(out, depth + 1);
    out << "-";
    if (m.type_id_ == tsi::ROS_TYPE_MESSAGE) {
      out << "\n";
      dump_message(out, static_cast<const uint8_t *>(elem), sub, depth + 2);
    } else {
      out << " ";
      format_scalar(out, static_cast<const uint8_t *>(elem), m.type_id_, sub, depth + 1);
      out << "\n";
    }
  }
}

void dump_message(
  std::ostringstream & out, const uint8_t * base, const MessageMembers * members, int depth)
{
  if (members == nullptr) {
    indent_to(out, depth);
    out << "<unknown>\n";
    return;
  }
  for (uint32_t i = 0; i < members->member_count_; ++i) {
    dump_member(out, base, members->members_[i], depth);
  }
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
  out.sec_offset = static_cast<size_t>(header_m->offset_) +
                   static_cast<size_t>(stamp_m->offset_) + static_cast<size_t>(sec_m->offset_);
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

std::optional<std::string> dump_message_yaml(
  const std::string & type_name, const void * payload)
{
  const MessageMembers * members = load_message_members(type_name);
  if (members == nullptr || payload == nullptr) return std::nullopt;
  std::ostringstream oss;
  dump_message(oss, static_cast<const uint8_t *>(payload), members, 0);
  return oss.str();
}

}  // namespace agnocast
