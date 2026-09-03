// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_exit_free_data.h"

#include "../agnocast.h"
#include "agnocast_kunit_eventfd.h"

#include <kunit/test.h>

static const pid_t PID_BASE = 1000;
static const char * TOPIC_NAME = "/kunit_test_topic";
static const char * NODE_NAME = "/kunit_test_node";
static const uint32_t QOS_DEPTH = 1;
#define QOS_IS_TRANSIENT_LOCAL false
#define QOS_IS_RELIABLE true
#define IS_TAKE_SUB false
#define IGNORE_LOCAL_PUBLICATIONS false
#define IS_BRIDGE false

static void setup_one_process(struct kunit * test, const pid_t pid)
{
  union ioctl_add_process_args add_process_args;
  int ret = agnocast_ioctl_add_process(
    pid, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, NULL, &add_process_args);

  KUNIT_ASSERT_EQ(test, ret, 0);
}

static void setup_one_subscriber(struct kunit * test, const pid_t pid, const int eventfd)
{
  union ioctl_add_subscriber_args add_subscriber_args;
  int ret = agnocast_ioctl_add_subscriber(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, pid, QOS_DEPTH, QOS_IS_TRANSIENT_LOCAL,
    QOS_IS_RELIABLE, IS_TAKE_SUB, IGNORE_LOCAL_PUBLICATIONS, IS_BRIDGE, eventfd,
    &add_subscriber_args);

  KUNIT_ASSERT_EQ(test, ret, 0);
}

// Module unload tears topics down with their subscribers still registered, so this is the only
// path on which agnocast_release_topic_wrapper() sees live subscriber_info entries.
void test_case_exit_free_data_releases_notify_context(struct kunit * test)
{
  // Arrange
  agnocast_kunit_eventfd_reset();
  const int subscriber_num = 3;
  setup_one_process(test, PID_BASE);
  for (int eventfd = 0; eventfd < subscriber_num; eventfd++) {
    setup_one_subscriber(test, PID_BASE, eventfd);
  }
  KUNIT_ASSERT_EQ(test, agnocast_kunit_eventfd_outstanding(), (int64_t)subscriber_num);
  KUNIT_ASSERT_TRUE(test, agnocast_is_in_topic_htable(TOPIC_NAME, current->nsproxy->ipc_ns));

  // Act
  agnocast_exit_free_data();

  // Assert
  KUNIT_EXPECT_EQ(test, agnocast_get_topic_num(current->nsproxy->ipc_ns), 0);
  for (int eventfd = 0; eventfd < subscriber_num; eventfd++) {
    const struct agnocast_kunit_eventfd_slot * slot = agnocast_kunit_eventfd_slot_of(eventfd);
    KUNIT_ASSERT_NOT_NULL(test, slot);
    KUNIT_EXPECT_EQ(test, slot->put_count, 1);
  }
  KUNIT_EXPECT_EQ(test, agnocast_kunit_eventfd_outstanding(), (int64_t)0);
}

// A grant is normally freed by its file's ->release, so unload is the one path that has to drain
// the list itself. In the KUnit build it is also what keeps one test case from leaking a grant
// into the next.
void test_case_exit_free_data_drains_outstanding_spawn_grants(struct kunit * test)
{
  // Arrange
  struct agnocast_spawn_grants spawn = {
    .requested_mask = AGNOCAST_SPAWN_MASK(AGNOCAST_SPAWN_UNLINK_DAEMON)};
  union ioctl_add_process_args add_process_args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_process(
      PID_BASE, current->nsproxy->ipc_ns, PROCESS_ROLE_APPLICATION, 0, &spawn, &add_process_args),
    0);
  KUNIT_ASSERT_NOT_NULL(test, spawn.granted[AGNOCAST_SPAWN_UNLINK_DAEMON]);

  // Act
  agnocast_exit_free_data();

  // Assert: the grant is freed, so it must not be dereferenced again from here.
  KUNIT_EXPECT_FALSE(
    test, agnocast_spawn_grant_outstanding(
            AGNOCAST_SPAWN_UNLINK_DAEMON, current->nsproxy->ipc_ns, AGNOCAST_DOMAIN_ID_NONE));
}
