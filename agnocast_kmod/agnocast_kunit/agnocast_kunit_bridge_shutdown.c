// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_bridge_shutdown.h"

#include "../agnocast.h"

#include <kunit/test.h>

static pid_t pid_bs = 8000;

// Registering as a bridge manager should succeed and record the role
// so that subsequent processes see the manager as alive
void test_case_bridge_manager_flag_set_on_registration(struct kunit * test)
{
  // Register bridge manager
  pid_t bridge_pid = pid_bs++;
  union ioctl_add_process_args bridge_args = {};
  int ret = agnocast_ioctl_add_process(
    bridge_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_BRIDGE_MANAGER, 0, NULL, &bridge_args);
  KUNIT_EXPECT_EQ(test, ret, 0);

  // Verify the flag was set by checking a new process sees it
  pid_t normal_pid = pid_bs++;
  union ioctl_add_process_args normal_args = {};
  ret = agnocast_ioctl_add_process(
    normal_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, NULL, &normal_args);
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_TRUE(
    test, agnocast_daemon_alive(AGNOCAST_SPAWN_BRIDGE_MANAGER, current->nsproxy->ipc_ns, 0));
}

// When a bridge manager is already registered, a new process must see it as alive
void test_case_bridge_manager_detected_by_new_process(struct kunit * test)
{
  // Register bridge manager
  pid_t bridge_pid = pid_bs++;
  union ioctl_add_process_args bridge_args = {};
  int ret = agnocast_ioctl_add_process(
    bridge_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_BRIDGE_MANAGER, 0, NULL, &bridge_args);
  KUNIT_EXPECT_EQ(test, ret, 0);

  // Register normal process - should see bridge manager exists
  pid_t normal_pid = pid_bs++;
  union ioctl_add_process_args normal_args = {};
  ret = agnocast_ioctl_add_process(
    normal_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, NULL, &normal_args);
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_TRUE(
    test, agnocast_daemon_alive(AGNOCAST_SPAWN_BRIDGE_MANAGER, current->nsproxy->ipc_ns, 0));
}

// notify_bridge_shutdown clears the bridge manager role, so the manager stops reading as alive
void test_case_notify_bridge_shutdown_clears_flag(struct kunit * test)
{
  // Register bridge manager
  pid_t bridge_pid = pid_bs++;
  union ioctl_add_process_args bridge_args = {};
  int ret = agnocast_ioctl_add_process(
    bridge_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_BRIDGE_MANAGER, 0, NULL, &bridge_args);
  KUNIT_EXPECT_EQ(test, ret, 0);

  // Notify shutdown
  agnocast_ioctl_notify_bridge_shutdown(bridge_pid);

  // Register normal process - should see no bridge manager
  pid_t normal_pid = pid_bs++;
  union ioctl_add_process_args normal_args = {};
  ret = agnocast_ioctl_add_process(
    normal_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, NULL, &normal_args);
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_FALSE(
    test, agnocast_daemon_alive(AGNOCAST_SPAWN_BRIDGE_MANAGER, current->nsproxy->ipc_ns, 0));
}
