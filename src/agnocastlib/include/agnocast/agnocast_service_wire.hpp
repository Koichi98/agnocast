#pragma once

#include "agnocast/agnocast_ioctl.hpp"  // NODE_NAME_BUFFER_SIZE
#include "agnocast/agnocast_smart_pointer.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>
#include <utility>

namespace agnocast
{

// On-the-wire layout for Agnocast service request/response messages.
//
// The correlation metadata (`_sequence_number`, and for requests the client `_node_name`) is
// carried *out of band* with respect to the user payload: the payload is the FIRST member, so the
// first `sizeof(Payload)` bytes of the wire object are byte-for-byte a plain user message that has
// its own rosidl typesupport. The fixed-size metadata trailer lives at a generically-computable
// offset (`align_up(sizeof(Payload), alignof(int64_t))`), which lets a type-erased (generic)
// bridge construct/parse the wire without per-type generated code.
//
// Composition (payload as the first member) is used instead of inheritance so that:
//   * the payload sits at offset 0 (a pointer to the wrapper is pointer-interconvertible with a
//     pointer to the payload), keeping the user-facing handle and the publish/release address
//     identical, and
//   * the metadata offset follows `sizeof(Payload)` (well-defined) rather than the ABI-specific
//     tail-padding-reuse rule that governs derived-class member layout.
//
// `_node_name` is a fixed-length buffer (NODE_NAME_BUFFER_SIZE) rather than a std::string so the
// whole wire object is a flat, self-contained block with no nested shared-memory allocations.

template <typename Payload>
struct ServiceRequestWrapper
{
  using PayloadType = Payload;

  Payload payload;  // MUST be the first member (offset 0).
  int64_t _sequence_number;
  char _node_name[NODE_NAME_BUFFER_SIZE];
};

template <typename Payload>
struct ServiceResponseWrapper
{
  using PayloadType = Payload;

  Payload payload;  // MUST be the first member (offset 0).
  int64_t _sequence_number;
};

// Copy a node's fully-qualified name into the fixed-length wire buffer (always NUL-terminated,
// truncating names that do not fit). NODE_NAME_BUFFER_SIZE (256) is far above any realistic FQN.
inline void set_wire_node_name(char (&dst)[NODE_NAME_BUFFER_SIZE], const std::string & src)
{
  const std::size_t n = std::min<std::size_t>(src.size(), NODE_NAME_BUFFER_SIZE - 1);
  std::memcpy(dst, src.data(), n);
  dst[n] = '\0';
}

// --- Generic (type-erased) wire layout helpers ---------------------------------------------------
//
// A type-erased bridge does not have the C++ wrapper type, only the payload's runtime size
// (rosidl_typesupport_introspection_cpp::MessageMembers::size_of_). These constexpr functions
// reproduce the byte offsets of the wrapper members from that size so the bridge can read/write the
// trailing metadata at exactly the offsets a typed peer expects.
//
// This relies on the wrapper layout: payload at offset 0, then an int64 aligned to alignof(int64_t)
// (8). ROS message alignment never exceeds 8, so the wrapper alignment is 8 and these offsets match
// the compiler-generated layout. (Verified against the real struct layout in the unit tests.)

inline constexpr std::size_t wire_align_up(std::size_t n, std::size_t a)
{
  return (n + a - 1) & ~(a - 1);
}

// Offset of `_sequence_number` (both request and response wrappers).
inline constexpr std::size_t wire_sequence_number_offset(std::size_t payload_size)
{
  return wire_align_up(payload_size, alignof(int64_t));
}

// Offset of `_node_name` (request wrapper only).
inline constexpr std::size_t wire_request_node_name_offset(std::size_t payload_size)
{
  return wire_sequence_number_offset(payload_size) + sizeof(int64_t);
}

// Total allocation size of a request / response wire object given the payload size.
inline constexpr std::size_t wire_request_size(std::size_t payload_size)
{
  return wire_request_node_name_offset(payload_size) + NODE_NAME_BUFFER_SIZE;
}
inline constexpr std::size_t wire_response_size(std::size_t payload_size)
{
  return wire_sequence_number_offset(payload_size) + sizeof(int64_t);
}

// Expose the payload of a wrapper handle as a payload-typed handle that shares the same control
// block. The payload is at offset 0, so the address is unchanged; only the static type differs.
template <typename Wrapper>
ipc_shared_ptr<typename Wrapper::PayloadType> wire_to_payload(ipc_shared_ptr<Wrapper> && w)
{
  typename Wrapper::PayloadType * p = &w.get()->payload;
  return ipc_shared_ptr<typename Wrapper::PayloadType>(std::move(w), p);
}

template <typename Wrapper>
ipc_shared_ptr<typename Wrapper::PayloadType> wire_to_payload(const ipc_shared_ptr<Wrapper> & w)
{
  typename Wrapper::PayloadType * p = &w.get()->payload;
  return ipc_shared_ptr<typename Wrapper::PayloadType>(w, p);
}

// Recover the wrapper handle from a payload-typed handle. Valid because the payload is the first
// member (offset 0), so the wrapper and payload pointers are interconvertible. The wrapper address
// is what must be published/released, so this restores the correct base pointer and type.
template <typename Wrapper, typename Payload>
ipc_shared_ptr<Wrapper> wire_to_wrapper(ipc_shared_ptr<Payload> && p)
{
  Wrapper * w = reinterpret_cast<Wrapper *>(p.get());
  return ipc_shared_ptr<Wrapper>(std::move(p), w);
}

template <typename Wrapper, typename Payload>
ipc_shared_ptr<Wrapper> wire_to_wrapper(const ipc_shared_ptr<Payload> & p)
{
  Wrapper * w = reinterpret_cast<Wrapper *>(p.get());
  return ipc_shared_ptr<Wrapper>(p, w);
}

}  // namespace agnocast
