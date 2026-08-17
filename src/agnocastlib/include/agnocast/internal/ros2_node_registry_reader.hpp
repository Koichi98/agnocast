// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
//
// Reads the DDS-side node list that the per-IPC-namespace
// `ros2agnocast_discovery_agent` publishes to
// `/dev/shm/agnocast_ros2_nodes/<ipc_ns_inode>/<domain_id>`.
//
// A pure-Agnocast process has no DDS participant, so `agnocast::Node` cannot
// see the ROS 2 graph on its own. The discovery agent does have one, and it
// already runs once per (IPC namespace, ROS_DOMAIN_ID) -- exactly the scope
// `NodeGraph::get_node_names()` reports on -- so it writes what it sees there
// for the Agnocast processes to read back.
//
// Each line is tab-separated and `\n`-terminated:
//
//   <node_namespace>\t<node_name>\n
//
// with the fully qualified name composed by the reader. As in
// `type_registry_writer.hpp`, an unterminated trailing line (writer died
// mid-write) is dropped, and fields beyond the required two are ignored so the
// writer can extend the format additively.
//
// The file is rewritten once per agent tick (1 s). A file older than
// `kStaleAfter` is treated as absent, which covers an agent killed without the
// chance to unlink its file: reporting no DDS-side list makes
// `get_node_names()` fall back to reporting every Agnocast node, rather than
// serving names of nodes that may be long gone.

#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace agnocast::internal
{

// An agent writing at 1 Hz; five missed ticks is a dead agent, not a slow one.
constexpr std::chrono::seconds kStaleAfter{5};

// Returns the fully qualified names of the ROS 2 nodes in this IPC namespace and domain, or
// std::nullopt when no discovery agent has left a usable list (not installed, not started yet, or
// its snapshot went stale). Duplicates are preserved: two nodes sharing a name are two nodes.
std::optional<std::vector<std::string>> read_ros2_node_names(
  uint64_t ipc_ns_inode, uint32_t domain_id);

// Test seam: override the tmpfs base directory (default `/dev/shm/agnocast_ros2_nodes`).
void set_ros2_node_registry_base_dir_for_test(const std::string & dir);

// Test seam: undo the override above.
void reset_ros2_node_registry_base_dir_for_test();

}  // namespace agnocast::internal
