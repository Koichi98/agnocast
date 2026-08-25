// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_get_node_names.h"

#include "../agnocast.h"

#include <kunit/test.h>

static const char * TOPIC_NAME = "/kunit_test_topic";
static const char * TOPIC_NAME2 = "/kunit_test_topic2";
static const char * NODE_NAME = "/kunit_test_node";
static const char * NODE_NAME2 = "/kunit_test_node2";
static const pid_t PID = 1000;
static const pid_t PID2 = 2000;
static const uint32_t QOS_DEPTH = 1;
static const uint32_t DOMAIN_ID = 0;
static const bool EXCLUDE_ROS2_NODES = true;
static const bool INCLUDE_ROS2_NODES = false;

static void setup_process(struct kunit * test, const pid_t pid)
{
  union ioctl_add_process_args add_process_args;
  int ret =
    agnocast_ioctl_add_process(pid, current->nsproxy->ipc_ns, false, DOMAIN_ID, &add_process_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
}

static void add_publisher(
  struct kunit * test, const char * topic_name, const char * node_name, const pid_t pid,
  const bool is_bridge, const bool is_ros2_node)
{
  union ioctl_add_publisher_args add_pub_args;
  int ret = agnocast_ioctl_add_publisher(
    topic_name, current->nsproxy->ipc_ns, node_name, pid, QOS_DEPTH, false, is_bridge, is_ros2_node,
    &add_pub_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
}

static void add_subscriber(
  struct kunit * test, const char * topic_name, const char * node_name, const pid_t pid,
  const bool is_bridge, const bool is_ros2_node)
{
  union ioctl_add_subscriber_args add_sub_args;
  int ret = agnocast_ioctl_add_subscriber(
    topic_name, current->nsproxy->ipc_ns, node_name, pid, QOS_DEPTH, false, true, false, false,
    is_bridge, is_ros2_node, -1, &add_sub_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
}

// Returns true if `name` is among the `num` NUL-terminated strings packed into `buf`.
static bool contains_name(const char * buf, const uint32_t num, const char * name)
{
  const char * p = buf;
  uint32_t i;

  for (i = 0; i < num; i++) {
    if (strcmp(p, name) == 0) return true;
    p += strlen(p) + 1;
  }

  return false;
}

// Returns how many of the `num` NUL-terminated strings packed into `buf` equal `name`.
static uint32_t count_name(const char * buf, const uint32_t num, const char * name)
{
  const char * p = buf;
  uint32_t count = 0;
  uint32_t i;

  for (i = 0; i < num; i++) {
    if (strcmp(p, name) == 0) count++;
    p += strlen(p) + 1;
  }

  return count;
}

void test_case_get_node_names_no_node(struct kunit * test)
{
  char buf[NODE_NAME_BUFFER_SIZE];
  size_t used = SIZE_MAX;
  uint32_t node_num = UINT_MAX;

  int ret = agnocast_ioctl_get_node_names(
    current->nsproxy->ipc_ns, DOMAIN_ID, EXCLUDE_ROS2_NODES, buf, sizeof(buf), &used, &node_num);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, node_num, 0);
  KUNIT_EXPECT_EQ(test, used, 0);
}

void test_case_get_node_names_multiple_nodes(struct kunit * test)
{
  char buf[2 * NODE_NAME_BUFFER_SIZE];
  size_t used = 0;
  uint32_t node_num = 0;

  setup_process(test, PID);
  add_publisher(test, TOPIC_NAME, NODE_NAME, PID, false, false);
  add_subscriber(test, TOPIC_NAME2, NODE_NAME2, PID, false, false);

  int ret = agnocast_ioctl_get_node_names(
    current->nsproxy->ipc_ns, DOMAIN_ID, EXCLUDE_ROS2_NODES, buf, sizeof(buf), &used, &node_num);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, node_num, 2);
  KUNIT_EXPECT_EQ(test, used, strlen(NODE_NAME) + 1 + strlen(NODE_NAME2) + 1);
  KUNIT_EXPECT_TRUE(test, contains_name(buf, node_num, NODE_NAME));
  KUNIT_EXPECT_TRUE(test, contains_name(buf, node_num, NODE_NAME2));
}

// A node owning several endpoints is reported exactly once.
void test_case_get_node_names_deduplicates(struct kunit * test)
{
  char buf[2 * NODE_NAME_BUFFER_SIZE];
  size_t used = 0;
  uint32_t node_num = 0;

  setup_process(test, PID);
  add_publisher(test, TOPIC_NAME, NODE_NAME, PID, false, false);
  add_publisher(test, TOPIC_NAME2, NODE_NAME, PID, false, false);
  add_subscriber(test, TOPIC_NAME, NODE_NAME, PID, false, false);

  int ret = agnocast_ioctl_get_node_names(
    current->nsproxy->ipc_ns, DOMAIN_ID, EXCLUDE_ROS2_NODES, buf, sizeof(buf), &used, &node_num);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, node_num, 1);
  KUNIT_EXPECT_STREQ(test, buf, NODE_NAME);
}

// Two processes running a node of the same name are two nodes, matching what rclcpp reports for
// the DDS graph. Only the dedup within one process may collapse them.
void test_case_get_node_names_same_name_in_two_processes(struct kunit * test)
{
  char buf[2 * NODE_NAME_BUFFER_SIZE];
  size_t used = 0;
  uint32_t node_num = 0;

  setup_process(test, PID);
  setup_process(test, PID2);
  add_publisher(test, TOPIC_NAME, NODE_NAME, PID, false, false);
  add_publisher(test, TOPIC_NAME, NODE_NAME, PID2, false, false);

  int ret = agnocast_ioctl_get_node_names(
    current->nsproxy->ipc_ns, DOMAIN_ID, EXCLUDE_ROS2_NODES, buf, sizeof(buf), &used, &node_num);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, node_num, 2);
  KUNIT_EXPECT_EQ(test, count_name(buf, node_num, NODE_NAME), 2);
}

// Endpoints created by a bridge do not introduce a node of their own.
void test_case_get_node_names_excludes_bridge(struct kunit * test)
{
  char buf[2 * NODE_NAME_BUFFER_SIZE];
  size_t used = 0;
  uint32_t node_num = 0;

  setup_process(test, PID);
  add_publisher(test, TOPIC_NAME, NODE_NAME, PID, false, false);
  add_subscriber(test, TOPIC_NAME, NODE_NAME2, PID, true, false);

  int ret = agnocast_ioctl_get_node_names(
    current->nsproxy->ipc_ns, DOMAIN_ID, EXCLUDE_ROS2_NODES, buf, sizeof(buf), &used, &node_num);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, node_num, 1);
  KUNIT_EXPECT_STREQ(test, buf, NODE_NAME);
}

// A node that DDS also announces is left to the discovery agent's side of the graph.
void test_case_get_node_names_excludes_ros2_node(struct kunit * test)
{
  char buf[2 * NODE_NAME_BUFFER_SIZE];
  size_t used = 0;
  uint32_t node_num = 0;

  setup_process(test, PID);
  add_publisher(test, TOPIC_NAME, NODE_NAME, PID, false, false);
  add_publisher(test, TOPIC_NAME, NODE_NAME2, PID, false, true);

  int ret = agnocast_ioctl_get_node_names(
    current->nsproxy->ipc_ns, DOMAIN_ID, EXCLUDE_ROS2_NODES, buf, sizeof(buf), &used, &node_num);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, node_num, 1);
  KUNIT_EXPECT_STREQ(test, buf, NODE_NAME);
}

// The subscriber table is filtered on the same condition as the publisher table above.
void test_case_get_node_names_excludes_ros2_node_subscriber(struct kunit * test)
{
  char buf[2 * NODE_NAME_BUFFER_SIZE];
  size_t used = 0;
  uint32_t node_num = 0;

  setup_process(test, PID);
  add_subscriber(test, TOPIC_NAME, NODE_NAME, PID, false, false);
  add_subscriber(test, TOPIC_NAME, NODE_NAME2, PID, false, true);

  int ret = agnocast_ioctl_get_node_names(
    current->nsproxy->ipc_ns, DOMAIN_ID, EXCLUDE_ROS2_NODES, buf, sizeof(buf), &used, &node_num);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, node_num, 1);
  KUNIT_EXPECT_STREQ(test, buf, NODE_NAME);
}

// Without a DDS-side list to merge with, the caller asks for every Agnocast node instead.
void test_case_get_node_names_includes_ros2_node(struct kunit * test)
{
  char buf[2 * NODE_NAME_BUFFER_SIZE];
  size_t used = 0;
  uint32_t node_num = 0;

  setup_process(test, PID);
  add_publisher(test, TOPIC_NAME, NODE_NAME, PID, false, false);
  add_publisher(test, TOPIC_NAME, NODE_NAME2, PID, false, true);

  int ret = agnocast_ioctl_get_node_names(
    current->nsproxy->ipc_ns, DOMAIN_ID, INCLUDE_ROS2_NODES, buf, sizeof(buf), &used, &node_num);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, node_num, 2);
  KUNIT_EXPECT_TRUE(test, contains_name(buf, node_num, NODE_NAME));
  KUNIT_EXPECT_TRUE(test, contains_name(buf, node_num, NODE_NAME2));
}

// Nodes belonging to another ROS_DOMAIN_ID are not visible.
void test_case_get_node_names_other_domain(struct kunit * test)
{
  char buf[2 * NODE_NAME_BUFFER_SIZE];
  size_t used = 0;
  uint32_t node_num = 0;

  setup_process(test, PID);
  add_publisher(test, TOPIC_NAME, NODE_NAME, PID, false, false);

  int ret = agnocast_ioctl_get_node_names(
    current->nsproxy->ipc_ns, DOMAIN_ID + 1, EXCLUDE_ROS2_NODES, buf, sizeof(buf), &used,
    &node_num);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, node_num, 0);
}

void test_case_get_node_names_buffer_too_small(struct kunit * test)
{
  char buf[4];
  size_t used = 0;
  uint32_t node_num = 0;

  setup_process(test, PID);
  add_publisher(test, TOPIC_NAME, NODE_NAME, PID, false, false);

  int ret = agnocast_ioctl_get_node_names(
    current->nsproxy->ipc_ns, DOMAIN_ID, EXCLUDE_ROS2_NODES, buf, sizeof(buf), &used, &node_num);

  KUNIT_EXPECT_EQ(test, ret, -ENOBUFS);
}

// Past MAX_NODE_NUM the caller gets an error instead of a silently truncated graph. Split across
// two topics because MAX_PUBLISHER_NUM caps a single one at MAX_NODE_NUM publishers.
void test_case_get_node_names_too_many_nodes(struct kunit * test)
{
  const size_t buf_size = (size_t)(MAX_NODE_NUM + 1) * NODE_NAME_BUFFER_SIZE;
  char * buf = kunit_kzalloc(test, buf_size, GFP_KERNEL);
  KUNIT_ASSERT_NOT_NULL(test, buf);
  size_t used = 0;
  uint32_t node_num = 0;
  char node_name[NODE_NAME_BUFFER_SIZE];
  int i;

  setup_process(test, PID);
  for (i = 0; i < MAX_NODE_NUM; i++) {
    snprintf(node_name, sizeof(node_name), "/kunit_test_node%d", i);
    add_publisher(test, TOPIC_NAME, node_name, PID, false, false);
  }
  snprintf(node_name, sizeof(node_name), "/kunit_test_node%d", MAX_NODE_NUM);
  add_publisher(test, TOPIC_NAME2, node_name, PID, false, false);

  int ret = agnocast_ioctl_get_node_names(
    current->nsproxy->ipc_ns, DOMAIN_ID, EXCLUDE_ROS2_NODES, buf, buf_size, &used, &node_num);

  KUNIT_EXPECT_EQ(test, ret, -ENOBUFS);
}
