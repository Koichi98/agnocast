// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_add_process.h"

#include "../agnocast.h"
#include "../agnocast_memory_allocator.h"

#include <kunit/test.h>
#include <linux/delay.h>

static pid_t pid = 1000;

// ipc_eq() compares pointers and nothing dereferences an ipc_ns, so a distinct address is a
// distinct namespace here. init_ipc_ns is not exported to modules.
static struct ipc_namespace other_ns;

static int add_process(
  const pid_t pid, const struct ipc_namespace * ipc_ns, const enum process_role role,
  const uint32_t domain_id, struct agnocast_spawn_grants * spawn)
{
  union ioctl_add_process_args args = {};
  return agnocast_ioctl_add_process(pid, ipc_ns, role, domain_id, spawn, &args);
}

// Asks for every kind on behalf of an application process, the way agnocastlib does when nothing
// in its own configuration rules a daemon out.
static struct agnocast_spawn_grants request_all(
  struct kunit * test, const struct ipc_namespace * ipc_ns, const uint32_t domain_id)
{
  struct agnocast_spawn_grants spawn = {.requested_mask = AGNOCAST_SPAWN_MASK_ALL};
  KUNIT_ASSERT_EQ(test, add_process(pid++, ipc_ns, PROCESS_ROLE_APPLICATION, domain_id, &spawn), 0);
  return spawn;
}

static void release_all(struct agnocast_spawn_grants * spawn)
{
  for (enum agnocast_spawn_kind kind = 0; kind < AGNOCAST_SPAWN_KIND_NUM; kind++) {
    agnocast_spawn_grant_release(spawn->granted[kind]);
    spawn->granted[kind] = NULL;
  }
}

void test_case_add_process_normal(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 0);

  uint64_t local_pid = pid++;
  union ioctl_add_process_args args;
  int ret = agnocast_ioctl_add_process(
    local_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, NULL, &args);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), 1);
  KUNIT_EXPECT_FALSE(test, agnocast_is_proc_exited(local_pid));
}

void test_case_add_process_many(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 0);

  // ================================================
  // Act

  pid_t local_pid_start = pid;
  for (int i = 0; i < mempool_num - 1; i++) {
    uint64_t local_pid = pid++;
    union ioctl_add_process_args args;
    agnocast_ioctl_add_process(
      local_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, NULL, &args);
  }

  uint64_t local_pid = pid++;
  union ioctl_add_process_args args;
  int ret = agnocast_ioctl_add_process(
    local_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, NULL, &args);

  // ================================================
  // Assert

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), mempool_num);
  for (int i = 0; i < mempool_num; i++) {
    KUNIT_EXPECT_FALSE(test, agnocast_is_proc_exited(local_pid_start + i));
  }
}

void test_case_add_process_twice(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 0);

  pid_t local_pid = pid++;
  union ioctl_add_process_args args;
  int ret1 = agnocast_ioctl_add_process(
    local_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, NULL, &args);
  int ret2 = agnocast_ioctl_add_process(
    local_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, NULL, &args);

  KUNIT_EXPECT_EQ(test, ret1, 0);
  KUNIT_EXPECT_EQ(test, ret2, -EINVAL);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), 1);
  KUNIT_EXPECT_FALSE(test, agnocast_is_proc_exited(local_pid));
}

// A bridge manager is gated per-(ipc_ns, domain): a manager in one
// domain must not suppress spawning a manager in another domain, while a second
// manager in the same domain is suppressed.
void test_case_add_process_bridge_manager_per_domain(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 0);

  union ioctl_add_process_args args_d0;
  int ret_d0 = agnocast_ioctl_add_process(
    pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_BRIDGE_MANAGER, 0, NULL, &args_d0);
  KUNIT_EXPECT_EQ(test, ret_d0, 0);
  KUNIT_EXPECT_FALSE(test, args_d0.ret_role_already_taken);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), 1);

  // Different domain: not suppressed, so it is added and sees no existing manager.
  union ioctl_add_process_args args_d1;
  int ret_d1 = agnocast_ioctl_add_process(
    pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_BRIDGE_MANAGER, 1, NULL, &args_d1);
  KUNIT_EXPECT_EQ(test, ret_d1, 0);
  KUNIT_EXPECT_FALSE(test, args_d1.ret_role_already_taken);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), 2);

  // Same domain as the first: a manager already exists, so it is suppressed.
  union ioctl_add_process_args args_d0_again;
  int ret_d0_again = agnocast_ioctl_add_process(
    pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_BRIDGE_MANAGER, 0, NULL, &args_d0_again);
  KUNIT_EXPECT_EQ(test, ret_d0_again, 0);
  KUNIT_EXPECT_TRUE(test, args_d0_again.ret_role_already_taken);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), 2);
}

// A registered daemon, not a non-empty namespace, is what makes the daemon read as alive.
void test_case_add_process_unlink_daemon_registration(struct kunit * test)
{
  // Arrange
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 0);
  union ioctl_add_process_args app_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_process(
      pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, NULL, &app_args),
    0);
  KUNIT_ASSERT_FALSE(
    test, agnocast_daemon_alive(
            AGNOCAST_SPAWN_UNLINK_DAEMON, current->nsproxy->ipc_ns, AGNOCAST_DOMAIN_ID_NONE));

  // Act
  union ioctl_add_process_args daemon_args;
  int ret = agnocast_ioctl_add_process(
    pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, 0, NULL, &daemon_args);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_FALSE(test, daemon_args.ret_role_already_taken);
  union ioctl_add_process_args later_args;
  KUNIT_EXPECT_EQ(
    test,
    agnocast_ioctl_add_process(
      pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, NULL, &later_args),
    0);
  KUNIT_EXPECT_TRUE(
    test, agnocast_daemon_alive(
            AGNOCAST_SPAWN_UNLINK_DAEMON, current->nsproxy->ipc_ns, AGNOCAST_DOMAIN_ID_NONE));
}

// Two processes starting at once can both decide to spawn a daemon; only one may register.
void test_case_add_process_unlink_daemon_duplicate_refused(struct kunit * test)
{
  // Arrange
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 0);
  union ioctl_add_process_args first_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_process(
      pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, 0, NULL, &first_args),
    0);
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 1);

  // Act
  union ioctl_add_process_args second_args;
  int ret = agnocast_ioctl_add_process(
    pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, 0, NULL, &second_args);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_TRUE(test, second_args.ret_role_already_taken);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), 1);
}

// A dead daemon is observable, so the exited entries it left behind must not stand in for it.
void test_case_add_process_unlink_daemon_respawns_after_death(struct kunit * test)
{
  // Arrange
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 0);
  const pid_t app_pid = pid++;
  union ioctl_add_process_args app_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_process(
      app_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, NULL, &app_args),
    0);
  const pid_t daemon_pid = pid++;
  union ioctl_add_process_args daemon_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_process(
      daemon_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, 0, NULL, &daemon_args),
    0);
  agnocast_process_exit_cleanup(app_pid);
  agnocast_process_exit_cleanup(daemon_pid);
  KUNIT_ASSERT_TRUE(test, agnocast_is_proc_exited(app_pid));

  // Act
  union ioctl_add_process_args next_args;
  int ret = agnocast_ioctl_add_process(
    pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, NULL, &next_args);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_FALSE(
    test, agnocast_daemon_alive(
            AGNOCAST_SPAWN_UNLINK_DAEMON, current->nsproxy->ipc_ns, AGNOCAST_DOMAIN_ID_NONE));
}

// The daemon is what drains the table, so on its own it has to be told the namespace is done.
void test_case_add_process_unlink_daemon_is_not_counted_as_work(struct kunit * test)
{
  // Arrange
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 0);
  union ioctl_add_process_args daemon_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_process(
      pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, 0, NULL, &daemon_args),
    0);

  // Act
  bool daemon_should_exit = false;
  agnocast_commit_exit_process(current->nsproxy->ipc_ns, -1, -1, &daemon_should_exit);

  // Assert
  KUNIT_EXPECT_TRUE(test, daemon_should_exit);
}

// An application's entry is left pending for the daemon to drain; the daemon's own is not.
void test_case_add_process_unlink_daemon_removed_on_death(struct kunit * test)
{
  // Arrange
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 0);
  const pid_t app_pid = pid++;
  union ioctl_add_process_args app_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_process(
      app_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, NULL, &app_args),
    0);
  const pid_t daemon_pid = pid++;
  union ioctl_add_process_args daemon_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_process(
      daemon_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, 0, NULL, &daemon_args),
    0);

  // Act
  agnocast_process_exit_cleanup(app_pid);
  agnocast_process_exit_cleanup(daemon_pid);

  // Assert
  struct ioctl_get_exit_process_args exit_args = {};
  KUNIT_EXPECT_EQ(
    test, agnocast_ioctl_get_exit_process(current->nsproxy->ipc_ns, &exit_args), app_pid);
  bool daemon_should_exit = false;
  agnocast_commit_exit_process(current->nsproxy->ipc_ns, app_pid, -1, &daemon_should_exit);
  KUNIT_EXPECT_EQ(test, agnocast_ioctl_get_exit_process(current->nsproxy->ipc_ns, &exit_args), -1);
}

// poll_for_unlink()'s drain loop discards the flag from a commit that returned a pid, so
// deregistering there would leave the daemon polling on as an unregistered ghost.
void test_case_add_process_unlink_daemon_stays_registered_while_draining(struct kunit * test)
{
  // Arrange
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 0);
  const pid_t app_pid = pid++;
  union ioctl_add_process_args app_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_process(
      app_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, NULL, &app_args),
    0);
  const pid_t daemon_pid = pid++;
  union ioctl_add_process_args daemon_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_process(
      daemon_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, 0, NULL, &daemon_args),
    0);
  agnocast_process_exit_cleanup(app_pid);

  // Act
  bool daemon_should_exit = false;
  agnocast_commit_exit_process(current->nsproxy->ipc_ns, app_pid, daemon_pid, &daemon_should_exit);

  // Assert
  KUNIT_EXPECT_TRUE(test, daemon_should_exit);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), 1);
  union ioctl_add_process_args next_args;
  KUNIT_EXPECT_EQ(
    test,
    agnocast_ioctl_add_process(
      pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, NULL, &next_args),
    0);
  KUNIT_EXPECT_TRUE(
    test, agnocast_daemon_alive(
            AGNOCAST_SPAWN_UNLINK_DAEMON, current->nsproxy->ipc_ns, AGNOCAST_DOMAIN_ID_NONE));
}

// A process starting between the exit decision and the daemon's death spawns a replacement.
void test_case_add_process_unlink_daemon_deregisters_when_told_to_exit(struct kunit * test)
{
  // Arrange
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 0);
  const pid_t daemon_pid = pid++;
  union ioctl_add_process_args daemon_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_process(
      daemon_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, 0, NULL, &daemon_args),
    0);

  // Act
  bool daemon_should_exit = false;
  agnocast_commit_exit_process(current->nsproxy->ipc_ns, -1, daemon_pid, &daemon_should_exit);

  // Assert
  KUNIT_EXPECT_TRUE(test, daemon_should_exit);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), 0);
  // The daemon process is still running at this point.
  union ioctl_add_process_args next_args;
  KUNIT_EXPECT_EQ(
    test,
    agnocast_ioctl_add_process(
      pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, NULL, &next_args),
    0);
  KUNIT_EXPECT_FALSE(
    test, agnocast_daemon_alive(
            AGNOCAST_SPAWN_UNLINK_DAEMON, current->nsproxy->ipc_ns, AGNOCAST_DOMAIN_ID_NONE));
}

// Deregistering the daemon early must release its mempool slot too, which nothing else will:
// its exit no longer reaches agnocast_process_exit_cleanup().
void test_case_add_process_unlink_daemon_deregistration_frees_its_mempool_slot(struct kunit * test)
{
  // Arrange
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 0);
  const pid_t daemon_pid = pid++;
  union ioctl_add_process_args daemon_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_process(
      daemon_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, 0, NULL, &daemon_args),
    0);
  bool daemon_should_exit = false;
  agnocast_commit_exit_process(current->nsproxy->ipc_ns, -1, daemon_pid, &daemon_should_exit);
  KUNIT_ASSERT_TRUE(test, daemon_should_exit);

  // Act
  int ret = 0;
  for (int i = 0; i < mempool_num; i++) {
    union ioctl_add_process_args args;
    ret = agnocast_ioctl_add_process(
      pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, NULL, &args);
    if (ret != 0) break;
  }

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), mempool_num);
}

void test_case_add_process_rejects_unknown_role(struct kunit * test)
{
  // Arrange
  union ioctl_add_process_args args;

  // Act
  int ret = agnocast_ioctl_add_process(
    pid++, current->nsproxy->ipc_ns, (enum process_role)(PROCESS_ROLE_UNLINK_DAEMON + 1), 0, NULL,
    &args);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), 0);
}

void test_case_add_process_rejects_the_daemon_domain_id(struct kunit * test)
{
  // Arrange
  union ioctl_add_process_args args;

  // Act
  int ret = agnocast_ioctl_add_process(
    pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, AGNOCAST_DOMAIN_ID_NONE, NULL,
    &args);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), 0);
}

// The daemon's domain_id is assigned by role, so whatever it sends is accepted and ignored.
void test_case_add_process_daemon_may_send_the_daemon_domain_id(struct kunit * test)
{
  // Arrange
  union ioctl_add_process_args args;

  // Act
  int ret = agnocast_ioctl_add_process(
    pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, AGNOCAST_DOMAIN_ID_NONE, NULL,
    &args);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), 1);
}

void test_case_add_process_too_many(struct kunit * test)
{
  KUNIT_ASSERT_EQ(test, agnocast_get_alive_proc_num(), 0);

  // ================================================
  // Act

  for (int i = 0; i < mempool_num; i++) {
    uint64_t local_pid = pid++;
    union ioctl_add_process_args args;
    agnocast_ioctl_add_process(
      local_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, NULL, &args);
  }
  uint64_t local_pid = pid++;
  union ioctl_add_process_args args;
  int ret = agnocast_ioctl_add_process(
    local_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, NULL, &args);

  // ================================================
  // Assert

  KUNIT_EXPECT_EQ(test, ret, -ENOMEM);
  KUNIT_EXPECT_EQ(test, agnocast_get_alive_proc_num(), mempool_num);
  KUNIT_EXPECT_FALSE(test, agnocast_is_proc_exited(local_pid));
}

// The point of the whole mechanism: the second process to start is not told to fork a daemon of
// its own while the first process's child is still on its way to registering.
void test_case_add_process_spawn_right_granted_once(struct kunit * test)
{
  // Arrange
  struct agnocast_spawn_grants first = request_all(test, current->nsproxy->ipc_ns, 0);
  KUNIT_ASSERT_NOT_NULL(test, first.granted[AGNOCAST_SPAWN_UNLINK_DAEMON]);

  // Act
  struct agnocast_spawn_grants second = request_all(test, current->nsproxy->ipc_ns, 0);

  // Assert
  KUNIT_EXPECT_NULL(test, second.granted[AGNOCAST_SPAWN_UNLINK_DAEMON]);

  release_all(&first);
  release_all(&second);
}

// What the fd's ->release does when a child dies before registering.
void test_case_add_process_spawn_right_released_right_is_regrantable(struct kunit * test)
{
  // Arrange
  struct agnocast_spawn_grants first = request_all(test, current->nsproxy->ipc_ns, 0);
  KUNIT_ASSERT_NOT_NULL(test, first.granted[AGNOCAST_SPAWN_UNLINK_DAEMON]);

  // Act
  release_all(&first);

  // Assert
  struct agnocast_spawn_grants second = request_all(test, current->nsproxy->ipc_ns, 0);
  KUNIT_EXPECT_NOT_NULL(test, second.granted[AGNOCAST_SPAWN_UNLINK_DAEMON]);

  release_all(&second);
}

// Once the daemon registers, proc_info_htable is authoritative and the right stops being
// outstanding, so a later process is refused because the daemon is alive rather than because
// somebody still holds the right.
void test_case_add_process_spawn_right_settled_by_daemon_registration(struct kunit * test)
{
  // Arrange
  struct agnocast_spawn_grants spawn = request_all(test, current->nsproxy->ipc_ns, 0);
  KUNIT_ASSERT_NOT_NULL(test, spawn.granted[AGNOCAST_SPAWN_UNLINK_DAEMON]);

  // Act
  int ret = add_process(pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, 0, NULL);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_FALSE(
    test, agnocast_spawn_grant_outstanding(
            AGNOCAST_SPAWN_UNLINK_DAEMON, current->nsproxy->ipc_ns, AGNOCAST_DOMAIN_ID_NONE));
  KUNIT_EXPECT_TRUE(
    test, agnocast_daemon_alive(
            AGNOCAST_SPAWN_UNLINK_DAEMON, current->nsproxy->ipc_ns, AGNOCAST_DOMAIN_ID_NONE));

  release_all(&spawn);
}

// The bridge manager settles its own right on registration, like the unlink daemon but scoped to
// its domain.
void test_case_add_process_spawn_right_settled_by_bridge_manager_registration(struct kunit * test)
{
  // Arrange
  struct agnocast_spawn_grants spawn = request_all(test, current->nsproxy->ipc_ns, 3);
  KUNIT_ASSERT_NOT_NULL(test, spawn.granted[AGNOCAST_SPAWN_BRIDGE_MANAGER]);

  // Act
  int ret = add_process(pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_BRIDGE_MANAGER, 3, NULL);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_FALSE(
    test,
    agnocast_spawn_grant_outstanding(AGNOCAST_SPAWN_BRIDGE_MANAGER, current->nsproxy->ipc_ns, 3));

  release_all(&spawn);
}

// agnocast_commit_exit_process() deregisters the daemon while it is still running, so that a
// process starting in that window spawns the replacement. Holding the right for the daemon's
// lifetime instead of settling it at registration would leave that process with nothing to do.
void test_case_add_process_spawn_right_regranted_after_early_deregistration(struct kunit * test)
{
  // Arrange: only the daemon, since get_process_num_except_unlink_daemon() counts every other
  // entry in the namespace and a live one would veto the exit this hangs off.
  const pid_t daemon_pid = pid++;
  KUNIT_ASSERT_EQ(
    test, add_process(daemon_pid, current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, 0, NULL),
    0);

  // Act
  bool daemon_should_exit = false;
  agnocast_commit_exit_process(current->nsproxy->ipc_ns, -1, daemon_pid, &daemon_should_exit);

  // Assert
  KUNIT_EXPECT_TRUE(test, daemon_should_exit);
  struct agnocast_spawn_grants next = request_all(test, current->nsproxy->ipc_ns, 0);
  KUNIT_EXPECT_NOT_NULL(test, next.granted[AGNOCAST_SPAWN_UNLINK_DAEMON]);

  release_all(&next);
}

void test_case_add_process_spawn_right_not_granted_while_daemon_alive(struct kunit * test)
{
  // Arrange
  KUNIT_ASSERT_EQ(
    test, add_process(pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, 0, NULL), 0);

  // Act
  struct agnocast_spawn_grants spawn = request_all(test, current->nsproxy->ipc_ns, 0);

  // Assert
  KUNIT_EXPECT_NULL(test, spawn.granted[AGNOCAST_SPAWN_UNLINK_DAEMON]);

  release_all(&spawn);
}

// The bridge manager is one per (namespace, domain), so its right must be too.
void test_case_add_process_spawn_right_bridge_is_per_domain(struct kunit * test)
{
  // Arrange
  struct agnocast_spawn_grants d0 = request_all(test, current->nsproxy->ipc_ns, 0);
  KUNIT_ASSERT_NOT_NULL(test, d0.granted[AGNOCAST_SPAWN_BRIDGE_MANAGER]);

  // Act
  struct agnocast_spawn_grants d1 = request_all(test, current->nsproxy->ipc_ns, 1);
  struct agnocast_spawn_grants d0_again = request_all(test, current->nsproxy->ipc_ns, 0);

  // Assert
  KUNIT_EXPECT_NOT_NULL(test, d1.granted[AGNOCAST_SPAWN_BRIDGE_MANAGER]);
  KUNIT_EXPECT_NULL(test, d0_again.granted[AGNOCAST_SPAWN_BRIDGE_MANAGER]);
  // The unlink daemon is namespace-scoped, so the other domain does not get a second one.
  KUNIT_EXPECT_NULL(test, d1.granted[AGNOCAST_SPAWN_UNLINK_DAEMON]);

  release_all(&d0);
  release_all(&d1);
  release_all(&d0_again);
}

void test_case_add_process_spawn_right_unlink_is_per_namespace(struct kunit * test)
{
  // Arrange
  struct agnocast_spawn_grants here = request_all(test, current->nsproxy->ipc_ns, 0);
  KUNIT_ASSERT_NOT_NULL(test, here.granted[AGNOCAST_SPAWN_UNLINK_DAEMON]);

  // Act
  struct agnocast_spawn_grants there = request_all(test, &other_ns, 0);

  // Assert
  KUNIT_EXPECT_NOT_NULL(test, there.granted[AGNOCAST_SPAWN_UNLINK_DAEMON]);

  release_all(&here);
  release_all(&there);
}

// A process that cannot fork a daemon must not be handed the right to, or nobody would.
void test_case_add_process_spawn_right_only_requested_kinds_are_granted(struct kunit * test)
{
  // Arrange
  struct agnocast_spawn_grants spawn = {
    .requested_mask = AGNOCAST_SPAWN_MASK(AGNOCAST_SPAWN_BRIDGE_MANAGER)};

  // Act
  int ret = add_process(pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, &spawn);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_NOT_NULL(test, spawn.granted[AGNOCAST_SPAWN_BRIDGE_MANAGER]);
  KUNIT_EXPECT_NULL(test, spawn.granted[AGNOCAST_SPAWN_UNLINK_DAEMON]);
  KUNIT_EXPECT_NULL(test, spawn.granted[AGNOCAST_SPAWN_DISCOVERY_AGENT]);
  // Still ungranted, so the next process gets it.
  struct agnocast_spawn_grants next = request_all(test, current->nsproxy->ipc_ns, 0);
  KUNIT_EXPECT_NOT_NULL(test, next.granted[AGNOCAST_SPAWN_UNLINK_DAEMON]);

  release_all(&spawn);
  release_all(&next);
}

// The daemon holds its fd past its own registration, so ->release runs on a right that is already
// settled. It must free that grant without disturbing the entries still on the list.
void test_case_add_process_spawn_right_release_is_idempotent_after_settle(struct kunit * test)
{
  // Arrange
  struct agnocast_spawn_grants spawn = request_all(test, current->nsproxy->ipc_ns, 0);
  KUNIT_ASSERT_NOT_NULL(test, spawn.granted[AGNOCAST_SPAWN_UNLINK_DAEMON]);
  KUNIT_ASSERT_NOT_NULL(test, spawn.granted[AGNOCAST_SPAWN_BRIDGE_MANAGER]);
  KUNIT_ASSERT_EQ(
    test, add_process(pid++, current->nsproxy->ipc_ns, PROCESS_ROLE_UNLINK_DAEMON, 0, NULL), 0);

  // Act
  agnocast_spawn_grant_release(spawn.granted[AGNOCAST_SPAWN_UNLINK_DAEMON]);
  spawn.granted[AGNOCAST_SPAWN_UNLINK_DAEMON] = NULL;

  // Assert
  KUNIT_EXPECT_TRUE(
    test,
    agnocast_spawn_grant_outstanding(AGNOCAST_SPAWN_BRIDGE_MANAGER, current->nsproxy->ipc_ns, 0));
  KUNIT_EXPECT_TRUE(
    test,
    agnocast_spawn_grant_outstanding(AGNOCAST_SPAWN_DISCOVERY_AGENT, current->nsproxy->ipc_ns, 0));

  release_all(&spawn);
}

// Why the right has to be an fd: the exit path filters on is_agnocast_pid(), so a child that dies
// between fork() and its own registration is invisible to it. Only the file's ->release returns
// the right.
void test_case_add_process_spawn_right_survives_exit_of_an_unregistered_process(struct kunit * test)
{
  // Arrange
  struct agnocast_spawn_grants spawn = request_all(test, current->nsproxy->ipc_ns, 0);
  KUNIT_ASSERT_NOT_NULL(test, spawn.granted[AGNOCAST_SPAWN_UNLINK_DAEMON]);

  // Act
  agnocast_process_exit_cleanup(pid++);

  // Assert
  KUNIT_EXPECT_TRUE(
    test, agnocast_spawn_grant_outstanding(
            AGNOCAST_SPAWN_UNLINK_DAEMON, current->nsproxy->ipc_ns, AGNOCAST_DOMAIN_ID_NONE));

  release_all(&spawn);
}
