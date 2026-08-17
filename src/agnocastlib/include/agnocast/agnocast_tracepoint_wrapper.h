#pragma once
#include "tracetools/tracetools.h"

// Compatibility: Jazzy uses _DECLARE_TRACEPOINT, Humble uses DECLARE_TRACEPOINT
#ifndef DECLARE_TRACEPOINT
#define DECLARE_TRACEPOINT(...) _DECLARE_TRACEPOINT(__VA_ARGS__)
#endif

#ifdef __cplusplus
extern "C" {
#endif

// clang-format off

DECLARE_TRACEPOINT(
  agnocast_init,
  const void * context_handle)

DECLARE_TRACEPOINT(
  agnocast_node_init,
  const void * node_handle,
  const char * node_name,
  const char * namespace_arg)

DECLARE_TRACEPOINT(
  agnocast_publisher_init,
  const void * publisher_handle,
  const void * node_handle,
  const char * topic_name,
  const size_t queue_depth)

DECLARE_TRACEPOINT(
  agnocast_subscription_init,
  const void * subscription_handle,
  const void * node_handle,
  const void * callback,
  const void * callback_group,
  const char * function_symbol,
  const char * topic_name,
  const size_t queue_depth,
  const uint64_t pid_callback_info_id)

DECLARE_TRACEPOINT(
  agnocast_timer_init,
  const void * timer_handle,
  const void * node_handle,
  const void * callback,
  const void * callback_group,
  const char * function_symbol,
  int64_t period)

DECLARE_TRACEPOINT(
  agnocast_add_callback_group,
  const void * executor_addr,
  const void * node_handle,
  const void * callback_group_addr,
  const char * group_type_name)

DECLARE_TRACEPOINT(
  agnocast_publish,
  const void * publisher_handle,
  const int64_t entry_id)

DECLARE_TRACEPOINT(
  agnocast_create_callable,
  const void * callable,
  const int64_t entry_id,
  const uint64_t pid_callback_info_id)

DECLARE_TRACEPOINT(
  agnocast_create_timer_callable,
  const void * callable,
  const void * timer_handle)

DECLARE_TRACEPOINT(
  agnocast_callable_start,
  const void * callable)

DECLARE_TRACEPOINT(
  agnocast_callable_end,
  const void * callable)

DECLARE_TRACEPOINT(
  agnocast_take,
  const void * subscription_handle,
  const void * message,
  const int64_t entry_id)

DECLARE_TRACEPOINT(
  agnocast_construct_executor,
  const void * executor_addr,
  const char * executor_type_name)

// clang-format on

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "tracetools/utils.hpp"

#include <cstdlib>
#include <string>
#include <type_traits>

namespace agnocast
{

/// Get the symbol of a callback, for use as a tracepoint argument.
/**
 * This wraps `tracetools::get_symbol` to hide two portability problems:
 *
 * - It is only declared when tracing is compiled in, so calling it unconditionally breaks the
 *   build with `TRACETOOLS_DISABLED`.
 * - Its ownership contract differs across distributions. Humble returns a pointer that must not
 *   be freed, while Jazzy returns heap memory that the caller must free, and may return nullptr.
 *
 * Returning by value keeps the ownership handling in one place and gives the call sites a symbol
 * that is always valid to pass to `ctf_string()`.
 *
 * \param[in] callback the callback to resolve
 * \return the symbol, or an empty string if it is unavailable
 */
template <typename CallbackT>
inline std::string get_callback_symbol([[maybe_unused]] const CallbackT & callback)
{
#ifndef TRACETOOLS_DISABLED
  auto * symbol = tracetools::get_symbol(callback);
  if (symbol == nullptr) {
    return {};
  }
  std::string result{symbol};
  if constexpr (!std::is_const_v<std::remove_pointer_t<decltype(symbol)>>) {
    // Jazzy onwards: the caller owns the demangled string.
    std::free(symbol);
  }
  return result;
#else
  return {};
#endif
}

}  // namespace agnocast

#endif  // __cplusplus
