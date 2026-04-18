#!/usr/bin/env python3
"""Verify that agnocast_cie_thread_configurator YAML settings are correctly applied.

Uses Linux /proc filesystem and syscalls to check each thread's actual scheduling
policy, priority/nice, and CPU affinity against the YAML config.

Run while both agnocast_cie_thread_configurator and the target application are running.

Note: Reading scheduling parameters of threads owned by another user requires
root privileges or CAP_SYS_PTRACE/CAP_SYS_NICE. Run with 'sudo' if the target
process is owned by a different user.

Usage:
    python3 verify_thread_config.py <config.yaml> [--pid <pid>]
"""

import argparse
import ctypes
import ctypes.util
import os
import platform
import re
import sys

import yaml

# ---------------------------------------------------------------------------
# Scheduling policy constants
# ---------------------------------------------------------------------------
SCHED_POLICY_NAMES = {
    0: "SCHED_OTHER",
    1: "SCHED_FIFO",
    2: "SCHED_RR",
    3: "SCHED_BATCH",
    5: "SCHED_IDLE",
    6: "SCHED_DEADLINE",
}

# ---------------------------------------------------------------------------
# sched_getattr via syscall -- needed to read SCHED_DEADLINE params
# ---------------------------------------------------------------------------
_ARCH = platform.machine()
_SYS_SCHED_GETATTR = {"x86_64": 315, "aarch64": 275}.get(_ARCH)

_libc = None
if _SYS_SCHED_GETATTR is not None:
    _lib_name = ctypes.util.find_library("c")
    if _lib_name:
        _libc = ctypes.CDLL(_lib_name, use_errno=True)


class _SchedAttr(ctypes.Structure):
    _fields_ = [
        ("size", ctypes.c_uint32),
        ("sched_policy", ctypes.c_uint32),
        ("sched_flags", ctypes.c_uint64),
        ("sched_nice", ctypes.c_int32),
        ("sched_priority", ctypes.c_uint32),
        ("sched_runtime", ctypes.c_uint64),
        ("sched_deadline", ctypes.c_uint64),
        ("sched_period", ctypes.c_uint64),
    ]


def _sched_getattr(tid):
    if _libc is None:
        return None
    attr = _SchedAttr()
    attr.size = ctypes.sizeof(_SchedAttr)
    ret = _libc.syscall(
        _SYS_SCHED_GETATTR, tid, ctypes.byref(attr), ctypes.sizeof(_SchedAttr), 0
    )
    if ret != 0:
        errno = ctypes.get_errno()
        raise OSError(errno, os.strerror(errno))
    return attr


# ---------------------------------------------------------------------------
# Read actual thread scheduling state from Linux
# ---------------------------------------------------------------------------
def get_thread_sched_info(tid):
    """Return dict with actual scheduling info for *tid*, or None if the thread is gone."""
    try:
        attr = _sched_getattr(tid)
    except OSError:
        return None

    if attr is None:
        # Fallback when sched_getattr syscall number is unknown
        try:
            policy_int = os.sched_getscheduler(tid)
            priority = os.sched_getparam(tid).sched_priority
            nice = os.getpriority(os.PRIO_PROCESS, tid)
        except OSError:
            return None
        return {
            "policy": SCHED_POLICY_NAMES.get(policy_int, f"UNKNOWN({policy_int})"),
            "priority": priority,
            "nice": nice,
        }

    return {
        "policy": SCHED_POLICY_NAMES.get(
            attr.sched_policy, f"UNKNOWN({attr.sched_policy})"
        ),
        "priority": attr.sched_priority,
        "nice": attr.sched_nice,
        "runtime": attr.sched_runtime,
        "deadline": attr.sched_deadline,
        "period": attr.sched_period,
    }


def get_thread_affinity(tid):
    """Return the set of CPU indices the thread is allowed to run on, or None if unavailable."""
    try:
        return os.sched_getaffinity(tid)
    except OSError:
        return None


def get_thread_comm(tid):
    """Read the thread's comm name from /proc."""
    try:
        with open(f"/proc/{tid}/comm") as f:
            return f.read().strip()
    except (IOError, PermissionError):
        return None


# ---------------------------------------------------------------------------
# Process / thread discovery
# ---------------------------------------------------------------------------
def find_pids_by_node_name(node_name):
    """Find PIDs whose cmdline contains the node name."""
    pids = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            with open(f"/proc/{entry}/cmdline", "rb") as f:
                cmdline = f.read().decode("utf-8", errors="replace")
            # cmdline has null-separated arguments
            args = cmdline.split("\0")
            for arg in args:
                if node_name in arg:
                    pids.append(int(entry))
                    break
        except (IOError, PermissionError):
            continue
    return pids


def get_all_tids(pid):
    """Get all thread TIDs for a given PID."""
    task_dir = f"/proc/{pid}/task"
    try:
        return [int(tid) for tid in os.listdir(task_dir) if tid.isdigit()]
    except (IOError, PermissionError):
        return []


# ---------------------------------------------------------------------------
# YAML helpers
# ---------------------------------------------------------------------------
def remove_trailing_waitable(s):
    suffix = "@Waitable"
    while s.endswith(suffix):
        s = s[: -len(suffix)]
    return s


def extract_node_names_from_config(yaml_config):
    """Extract unique node names from callback group IDs.

    Callback group IDs have format: /<node_name>@<entries...>
    """
    names = set()
    for cbg in yaml_config.get("callback_groups") or []:
        cbg_id = remove_trailing_waitable(cbg["id"])
        match = re.match(r"^/([^@/]+)", cbg_id)
        if match:
            names.add(match.group(1))
    return names


# ---------------------------------------------------------------------------
# Matching logic
# ---------------------------------------------------------------------------
def config_matches_thread(expected, actual_info, actual_affinity):
    """Check if an expected config matches an actual thread's state."""
    expected_policy = expected["policy"]
    actual_policy = actual_info["policy"]

    if actual_policy != expected_policy:
        return False

    if expected_policy in ("SCHED_OTHER", "SCHED_BATCH", "SCHED_IDLE"):
        exp_nice = expected.get("priority", 0)
        act_nice = actual_info.get("nice", 0)
        if act_nice != exp_nice:
            return False
    elif expected_policy in ("SCHED_FIFO", "SCHED_RR"):
        if actual_info.get("priority", 0) != expected.get("priority", 0):
            return False
    elif expected_policy == "SCHED_DEADLINE":
        for param in ("runtime", "deadline", "period"):
            if actual_info.get(param, 0) != expected.get(param, 0):
                return False

    expected_affinity = expected.get("affinity") or []
    if expected_affinity:
        if actual_affinity is None:
            return False
        if set(expected_affinity) != actual_affinity:
            return False

    return True


def describe_config(expected):
    """Human-readable summary of expected config."""
    policy = expected["policy"]
    parts = [policy]
    if policy in ("SCHED_OTHER", "SCHED_BATCH", "SCHED_IDLE"):
        parts.append(f"nice={expected.get('priority', 0)}")
    elif policy in ("SCHED_FIFO", "SCHED_RR"):
        parts.append(f"priority={expected.get('priority', 0)}")
    elif policy == "SCHED_DEADLINE":
        parts.append(f"runtime={expected.get('runtime', 0)}")
        parts.append(f"deadline={expected.get('deadline', 0)}")
        parts.append(f"period={expected.get('period', 0)}")
    aff = expected.get("affinity") or []
    if aff:
        parts.append(f"affinity={sorted(aff)}")
    return ", ".join(parts)


def describe_actual(info, affinity):
    """Human-readable summary of actual thread state."""
    policy = info["policy"]
    parts = [policy]
    if policy in ("SCHED_OTHER", "SCHED_BATCH", "SCHED_IDLE"):
        parts.append(f"nice={info.get('nice', 0)}")
    elif policy in ("SCHED_FIFO", "SCHED_RR"):
        parts.append(f"priority={info.get('priority', 0)}")
    elif policy == "SCHED_DEADLINE":
        parts.append(f"runtime={info.get('runtime', 0)}")
        parts.append(f"deadline={info.get('deadline', 0)}")
        parts.append(f"period={info.get('period', 0)}")
    if affinity is not None:
        parts.append(f"affinity={sorted(affinity)}")
    return ", ".join(parts)


def get_mismatch_details(expected, actual_info, actual_affinity):
    """Return list of mismatch details between expected and actual."""
    errors = []
    expected_policy = expected["policy"]
    actual_policy = actual_info["policy"]

    if actual_policy != expected_policy:
        errors.append(f"policy: expected={expected_policy}, actual={actual_policy}")
        return errors

    if expected_policy in ("SCHED_OTHER", "SCHED_BATCH", "SCHED_IDLE"):
        exp_nice = expected.get("priority", 0)
        act_nice = actual_info.get("nice", 0)
        if act_nice != exp_nice:
            errors.append(f"nice: expected={exp_nice}, actual={act_nice}")
    elif expected_policy in ("SCHED_FIFO", "SCHED_RR"):
        exp_prio = expected.get("priority", 0)
        act_prio = actual_info.get("priority", 0)
        if act_prio != exp_prio:
            errors.append(f"priority: expected={exp_prio}, actual={act_prio}")
    elif expected_policy == "SCHED_DEADLINE":
        for param in ("runtime", "deadline", "period"):
            exp_val = expected.get(param, 0)
            act_val = actual_info.get(param, 0)
            if act_val != exp_val:
                errors.append(f"{param}: expected={exp_val}, actual={act_val}")

    expected_affinity = expected.get("affinity") or []
    if expected_affinity:
        if actual_affinity is None:
            errors.append(f"affinity: expected={sorted(expected_affinity)}, actual=unavailable")
        elif set(expected_affinity) != actual_affinity:
            errors.append(
                f"affinity: expected={sorted(expected_affinity)}, "
                f"actual={sorted(actual_affinity)}"
            )

    return errors


def _find_best_mismatch(expected, all_threads):
    """Find the thread with fewest mismatches and return a diagnostic message."""
    if not all_threads:
        return "No threads found in target process(es)"
    best_errors = None
    best_tid = None
    best_comm = None
    for tid, info, affinity, comm in all_threads:
        errors = get_mismatch_details(expected, info, affinity)
        if best_errors is None or len(errors) < len(best_errors):
            best_errors = errors
            best_tid = tid
            best_comm = comm
    if best_errors:
        return (
            f"Closest thread: tid={best_tid} comm='{best_comm}' — "
            + "; ".join(best_errors)
        )
    return "No thread found matching expected configuration"


# ---------------------------------------------------------------------------
# Verification
# ---------------------------------------------------------------------------
def verify(yaml_config, pids):
    """Verify YAML config against actual thread states found in the given PIDs."""
    # Collect all thread info
    all_threads = []  # list of (tid, sched_info, affinity, comm)
    for pid in pids:
        for tid in get_all_tids(pid):
            info = get_thread_sched_info(tid)
            if info is None:
                continue
            affinity = get_thread_affinity(tid)
            comm = get_thread_comm(tid)
            all_threads.append((tid, info, affinity, comm))

    results = []
    available_threads = list(all_threads)

    # Check callback groups
    for cbg in yaml_config.get("callback_groups") or []:
        expected = dict(cbg)
        expected["id"] = remove_trailing_waitable(expected["id"])
        entry = {
            "type": "callback_group",
            "id": expected["id"],
            "domain_id": expected.get("domain_id", 0),
            "expected": describe_config(expected),
        }

        # Find a matching thread
        matched_idx = None
        for i, (tid, info, affinity, _comm) in enumerate(available_threads):
            if config_matches_thread(expected, info, affinity):
                matched_idx = i
                break

        if matched_idx is not None:
            tid, info, affinity, comm = available_threads.pop(matched_idx)
            entry["status"] = "OK"
            entry["tid"] = tid
            entry["comm"] = comm
            entry["actual"] = describe_actual(info, affinity)
        else:
            entry["status"] = "FAIL"
            # Find the closest thread to show mismatch details
            best_details = _find_best_mismatch(expected, all_threads)
            entry["message"] = best_details
        results.append(entry)

    # Check non-ROS threads
    for nrt in yaml_config.get("non_ros_threads") or []:
        expected = dict(nrt)
        expected_name = expected["name"]
        entry = {
            "type": "non_ros_thread",
            "name": expected_name,
            "expected": describe_config(expected),
        }

        matched_idx = None
        for i, (tid, info, affinity, comm) in enumerate(available_threads):
            if comm == expected_name and config_matches_thread(expected, info, affinity):
                matched_idx = i
                break

        if matched_idx is not None:
            tid, info, affinity, comm = available_threads.pop(matched_idx)
            entry["status"] = "OK"
            entry["tid"] = tid
            entry["comm"] = comm
            entry["actual"] = describe_actual(info, affinity)
        else:
            entry["status"] = "FAIL"
            # Check if any thread has the right name but wrong params
            name_matched = [
                (tid, info, aff) for tid, info, aff, c in all_threads if c == expected_name
            ]
            if name_matched:
                tid, info, aff = name_matched[0]
                details = get_mismatch_details(expected, info, aff)
                entry["message"] = (
                    f"Thread '{expected_name}' (tid={tid}) found but mismatched: "
                    + "; ".join(details)
                )
            else:
                entry["message"] = f"No thread with comm name '{expected_name}' found"
        results.append(entry)

    return results, all_threads


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------
def print_results(results, all_threads):
    all_ok = True

    print("Verification results:")
    for r in results:
        label = r.get("id") or r.get("name")
        domain_str = f" [domain={r['domain_id']}]" if "domain_id" in r else ""
        tid_str = f" (tid={r['tid']})" if "tid" in r else ""

        if r["status"] == "OK":
            print(f"  OK    {r['type']}: {label}{domain_str}{tid_str}")
            print(f"          config: {r['expected']}")
        else:
            print(f"  FAIL  {r['type']}: {label}{domain_str}")
            print(f"          config: {r['expected']}")
            print(f"          {r.get('message', 'Unknown error')}")
            all_ok = False

    # Show all threads for debugging
    print(f"\nAll threads ({len(all_threads)} total):")
    for tid, info, affinity, comm in sorted(all_threads, key=lambda t: t[0]):
        print(f"  tid={tid:>7}  comm={comm or '?':<20}  {describe_actual(info, affinity)}")

    return all_ok


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Verify thread configurator YAML against actual thread state"
    )
    parser.add_argument("config_file", help="Path to thread configurator YAML config")
    parser.add_argument(
        "--pid",
        type=int,
        action="append",
        default=None,
        help="PID of the target process (can be specified multiple times). "
        "If not specified, auto-detected from node names in the YAML config.",
    )
    args = parser.parse_args()

    with open(args.config_file) as f:
        yaml_config = yaml.safe_load(f)

    if args.pid:
        pids = args.pid
        for pid in pids:
            if not os.path.isdir(f"/proc/{pid}"):
                print(f"Error: PID {pid} does not exist.", file=sys.stderr)
                return 1
        print(f"Using specified PID(s): {pids}")
    else:
        node_names = extract_node_names_from_config(yaml_config)
        if not node_names:
            print("Error: No node names found in YAML config and no --pid specified.", file=sys.stderr)
            return 1

        pids = []
        for name in sorted(node_names):
            found = find_pids_by_node_name(name)
            if not found:
                print(f"Warning: No process found for node '{name}'", file=sys.stderr)
            else:
                pids.extend(found)
                print(f"Found PID(s) {found} for node '{name}'")

        # Deduplicate
        pids = sorted(set(pids))
        if not pids:
            print("Error: No target processes found.", file=sys.stderr)
            return 1

    print(f"Checking {len(pids)} process(es)...\n")

    results, all_threads = verify(yaml_config, pids)
    all_ok = print_results(results, all_threads)
    print()

    if all_ok:
        print("All configurations verified successfully.")
        return 0
    else:
        print("Some configurations could not be verified or have mismatches.")
        return 1


if __name__ == "__main__":
    sys.exit(main())
