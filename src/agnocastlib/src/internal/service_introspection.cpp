#include "agnocast/internal/service_introspection.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace agnocast
{

namespace internal
{

namespace
{

constexpr uint64_t k_fnv_offset_basis = 14695981039346656037ULL;
constexpr uint64_t k_fnv_prime = 1099511628211ULL;

uint64_t fnv1a_64(const std::string & value, uint64_t basis)
{
  uint64_t hash = basis;
  for (const char c : value) {
    hash ^= static_cast<uint64_t>(static_cast<unsigned char>(c));
    hash *= k_fnv_prime;
  }
  return hash;
}

}  // namespace

std::string create_service_event_topic_name(const std::string & resolved_service_name)
{
  return resolved_service_name + k_service_event_topic_postfix;
}

std::array<uint8_t, 16> make_service_client_gid(const std::string & node_fqn)
{
  // Two FNV-1a runs with different offset bases give 16 deterministic bytes without pulling in a
  // hash library. This is an identifier only; it is never used as a security or uniqueness
  // guarantee.
  const uint64_t low = fnv1a_64(node_fqn, k_fnv_offset_basis);
  const uint64_t high = fnv1a_64(node_fqn, ~k_fnv_offset_basis);

  std::array<uint8_t, 16> gid{};
  for (size_t i = 0; i < 8; i++) {
    gid[i] = static_cast<uint8_t>((low >> (8 * i)) & 0xFF);
    gid[8 + i] = static_cast<uint8_t>((high >> (8 * i)) & 0xFF);
  }
  return gid;
}

}  // namespace internal

}  // namespace agnocast
