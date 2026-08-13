#include "agnocast_cie_thread_configurator/thread_config.hpp"

#include <linux/sched.h>
#include <sched.h>

#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>

namespace agnocast_cie_thread_configurator
{

namespace
{

// Unset = the attribute must not be applied: YAML null / absent key (the
// user's opt-out) or the UNMANAGEABLE sentinel (a kernel/tool constraint).
bool is_unset(const YAML::Node & node)
{
  if (!node || node.IsNull()) {
    return true;
  }
  return node.IsScalar() && node.Scalar() == k_unmanageable;
}

}  // namespace

const std::unordered_map<std::string, int> policy_to_sched_const = {
  {"SCHED_OTHER", SCHED_OTHER}, {"SCHED_BATCH", SCHED_BATCH}, {"SCHED_IDLE", SCHED_IDLE},
  {"SCHED_FIFO", SCHED_FIFO},   {"SCHED_RR", SCHED_RR},       {"SCHED_DEADLINE", SCHED_DEADLINE},
};

bool ThreadConfig::is_wildcard() const noexcept
{
  static constexpr std::string_view suffix = "/*";
  return thread_str.size() >= suffix.size() &&
         thread_str.compare(thread_str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::string ThreadConfig::wildcard_prefix() const
{
  return thread_str.substr(0, thread_str.size() - 2);
}

std::string extract_node_part(const std::string & callback_group_id)
{
  return callback_group_id.substr(0, callback_group_id.find('@'));
}

void parse_yaml(
  const YAML::Node & yaml, size_t default_domain_id,
  std::vector<ThreadConfig> & callback_groups_out, std::vector<ThreadConfig> & non_ros_threads_out)
{
  YAML::Node callback_groups = yaml["callback_groups"];
  YAML::Node non_ros_threads = yaml["non_ros_threads"];

  callback_groups_out.clear();
  non_ros_threads_out.clear();
  callback_groups_out.resize(callback_groups.size());
  non_ros_threads_out.resize(non_ros_threads.size());

  for (size_t i = 0; i < callback_groups.size(); ++i) {
    const auto & cg = callback_groups[i];
    auto & cfg = callback_groups_out[i];

    cfg.thread_str = cg["id"].as<std::string>();
    if (cfg.thread_str.find('*') != std::string::npos) {
      // A typo'd pattern silently treated as an exact id would never match,
      // so any id containing '*' must be a well-formed "<node name>/*".
      if (!cfg.is_wildcard()) {
        throw std::runtime_error(
          "Invalid id '" + cfg.thread_str +
          "': '*' is only allowed as a trailing \"/*\" wildcard (e.g. /my_node/*)");
      }
      const std::string prefix = cfg.wildcard_prefix();
      if (prefix.empty() || prefix.find('*') != std::string::npos) {
        throw std::runtime_error(
          "Invalid wildcard id '" + cfg.thread_str +
          "': the part before \"/*\" must be a non-empty node name without '*'");
      }
      if (prefix.find('@') != std::string::npos) {
        throw std::runtime_error(
          "Invalid wildcard id '" + cfg.thread_str +
          "': the part before \"/*\" must be a plain node name, not a full callback-group id "
          "containing '@'");
      }
    }
    cfg.domain_id = cg["domain_id"] ? cg["domain_id"].as<size_t>() : default_domain_id;
    for (auto & cpu : cg["affinity"]) cfg.affinity.push_back(cpu.as<int>());
    cfg.policy = cg["policy"].as<std::string>();

    if (policy_to_sched_const.count(cfg.policy) == 0) {
      throw std::runtime_error(
        "Unknown scheduling policy '" + cfg.policy + "' for id=" + cfg.thread_str +
        ". Valid policies: SCHED_OTHER, SCHED_BATCH, SCHED_IDLE, SCHED_FIFO, SCHED_RR, "
        "SCHED_DEADLINE");
    }

    if (cfg.policy == "SCHED_DEADLINE") {
      cfg.runtime = cg["runtime"].as<unsigned int>();
      cfg.period = cg["period"].as<unsigned int>();
      cfg.deadline = cg["deadline"].as<unsigned int>();
    } else {
      cfg.priority = cg["priority"].as<int>();
    }
  }

  for (size_t i = 0; i < non_ros_threads.size(); ++i) {
    const auto & nrt = non_ros_threads[i];
    auto & cfg = non_ros_threads_out[i];

    cfg.thread_str = nrt["name"].as<std::string>();
    for (auto & cpu : nrt["affinity"]) cfg.affinity.push_back(cpu.as<int>());
    cfg.policy = nrt["policy"].as<std::string>();

    if (policy_to_sched_const.count(cfg.policy) == 0) {
      throw std::runtime_error(
        "Unknown scheduling policy '" + cfg.policy + "' for name=" + cfg.thread_str +
        ". Valid policies: SCHED_OTHER, SCHED_BATCH, SCHED_IDLE, SCHED_FIFO, SCHED_RR, "
        "SCHED_DEADLINE");
    }

    if (cfg.policy == "SCHED_DEADLINE") {
      cfg.runtime = nrt["runtime"].as<unsigned int>();
      cfg.period = nrt["period"].as<unsigned int>();
      cfg.deadline = nrt["deadline"].as<unsigned int>();
    } else {
      cfg.priority = nrt["priority"].as<int>();
    }
  }

  // Reject duplicates: id_to_*_config_ would silently collapse them to the
  // last-inserted entry, dropping earlier YAML lines without warning.
  // The std::find_if rewrite cppcheck suggests would hide a side-effecting
  // predicate inside the algorithm; a plain loop is clearer here.
  std::unordered_set<std::string> seen_cb;
  for (const auto & c : callback_groups_out) {
    // cppcheck-suppress useStlAlgorithm
    if (!seen_cb.insert(std::to_string(c.domain_id) + ":" + c.thread_str).second) {
      throw std::runtime_error(
        "Duplicate callback_group entry: domain_id=" + std::to_string(c.domain_id) +
        ", id=" + c.thread_str);
    }
  }
  std::unordered_set<std::string> seen_nrt;
  for (const auto & c : non_ros_threads_out) {
    // cppcheck-suppress useStlAlgorithm
    if (!seen_nrt.insert(c.thread_str).second) {
      throw std::runtime_error("Duplicate non_ros_thread entry: name=" + c.thread_str);
    }
  }
}

bool KernelThreadConfig::is_managed() const noexcept
{
  return policy.has_value() || !affinity.empty();
}

bool IrqConfig::is_managed() const noexcept
{
  return !affinity.empty();
}

std::vector<KernelThreadConfig> parse_kernel_threads(const YAML::Node & yaml)
{
  YAML::Node section = yaml["kernel_threads"];
  std::vector<KernelThreadConfig> result;
  if (!section || section.IsNull()) {
    return result;
  }
  result.resize(section.size());

  for (size_t i = 0; i < section.size(); ++i) {
    const auto & kt = section[i];
    auto & cfg = result[i];

    if (is_unset(kt["comm"])) {
      throw std::runtime_error("A kernel_threads entry is missing a non-empty 'comm'");
    }
    cfg.comm = kt["comm"].as<std::string>();
    if (cfg.comm.empty()) {
      throw std::runtime_error("A kernel_threads entry is missing a non-empty 'comm'");
    }
    if (cfg.comm.compare(0, 8, "kworker/") == 0) {
      throw std::runtime_error(
        "kernel_threads entry '" + cfg.comm +
        "' is not manageable: kworker comms are ephemeral and mutate at runtime, so they cannot "
        "be matched reliably");
    }

    if (!is_unset(kt["affinity"])) {
      for (auto & cpu : kt["affinity"]) cfg.affinity.push_back(cpu.as<int>());
    }

    const bool has_policy = !is_unset(kt["policy"]);
    const bool has_priority = !is_unset(kt["priority"]);
    if (!has_policy) {
      if (has_priority) {
        throw std::runtime_error(
          "'priority' requires 'policy' for comm=" + cfg.comm + ": set both or leave both unset");
      }
      continue;
    }

    cfg.policy = kt["policy"].as<std::string>();
    if (policy_to_sched_const.count(*cfg.policy) == 0) {
      throw std::runtime_error(
        "Unknown scheduling policy '" + *cfg.policy + "' for comm=" + cfg.comm +
        ". Valid policies: SCHED_OTHER, SCHED_BATCH, SCHED_IDLE, SCHED_FIFO, SCHED_RR, "
        "SCHED_DEADLINE");
    }

    if (*cfg.policy == "SCHED_DEADLINE") {
      // Explicit check for a clear message: these fields are always
      // hand-written (prerun never emits DEADLINE) and easy to forget.
      if (is_unset(kt["runtime"]) || is_unset(kt["period"]) || is_unset(kt["deadline"])) {
        throw std::runtime_error(
          "SCHED_DEADLINE requires 'runtime', 'period' and 'deadline' for comm=" + cfg.comm);
      }
      cfg.runtime = kt["runtime"].as<unsigned int>();
      cfg.period = kt["period"].as<unsigned int>();
      cfg.deadline = kt["deadline"].as<unsigned int>();
    } else {
      if (!has_priority) {
        throw std::runtime_error(
          "Policy '" + *cfg.policy + "' requires 'priority' for comm=" + cfg.comm);
      }
      cfg.priority = kt["priority"].as<int>();
    }
  }

  std::unordered_set<std::string> seen;
  for (const auto & c : result) {
    // cppcheck-suppress useStlAlgorithm
    if (!seen.insert(c.comm).second) {
      throw std::runtime_error("Duplicate kernel_thread entry: comm=" + c.comm);
    }
  }
  return result;
}

std::vector<IrqConfig> parse_irqs(const YAML::Node & yaml)
{
  YAML::Node section = yaml["irqs"];
  std::vector<IrqConfig> result;
  if (!section || section.IsNull()) {
    return result;
  }
  result.resize(section.size());

  for (size_t i = 0; i < section.size(); ++i) {
    const auto & iq = section[i];
    auto & cfg = result[i];

    if (is_unset(iq["irq"]) || !iq["irq"].IsScalar()) {
      throw std::runtime_error("An irqs entry is missing a non-negative integer 'irq'");
    }
    try {
      cfg.irq = iq["irq"].as<int>();
    } catch (const YAML::Exception &) {
      throw std::runtime_error("An irqs entry is missing a non-negative integer 'irq'");
    }
    if (cfg.irq < 0) {
      throw std::runtime_error("An irqs entry is missing a non-negative integer 'irq'");
    }

    if (!is_unset(iq["name"])) {
      cfg.name = iq["name"].as<std::string>();
    }
    if (!is_unset(iq["affinity"])) {
      for (auto & cpu : iq["affinity"]) cfg.affinity.push_back(cpu.as<int>());
    }
  }

  std::unordered_set<int> seen;
  for (const auto & c : result) {
    // cppcheck-suppress useStlAlgorithm
    if (!seen.insert(c.irq).second) {
      throw std::runtime_error("Duplicate irq entry: irq=" + std::to_string(c.irq));
    }
  }
  return result;
}

}  // namespace agnocast_cie_thread_configurator
