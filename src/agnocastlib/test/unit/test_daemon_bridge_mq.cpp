#include "agnocast/agnocast_mq.hpp"
#include "agnocast/agnocast_utils.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdlib>

// The discovery daemon (Python) packs MqMsgDaemonBridge by hand, mirroring
// this layout. These checks fail loudly if the C++ struct drifts from the
// daemon's `_MSG_PACK_FORMAT` ('=256s256sIIBB2x', 524 bytes).
TEST(DaemonBridgeMqTest, WireLayoutMatchesDaemonPackFormat)
{
  using agnocast::MqMsgDaemonBridge;
  EXPECT_EQ(sizeof(MqMsgDaemonBridge), 524u);
  EXPECT_EQ(offsetof(MqMsgDaemonBridge, topic_name), 0u);
  EXPECT_EQ(offsetof(MqMsgDaemonBridge, type_name), 256u);
  EXPECT_EQ(offsetof(MqMsgDaemonBridge, direction), 512u);
  EXPECT_EQ(offsetof(MqMsgDaemonBridge, qos_depth), 516u);
  EXPECT_EQ(offsetof(MqMsgDaemonBridge, qos_is_transient_local), 520u);
  EXPECT_EQ(offsetof(MqMsgDaemonBridge, qos_is_reliable), 521u);
}

TEST(DaemonBridgeMqTest, StandardMqNameIsKeyedByPid)
{
  EXPECT_EQ(agnocast::create_mq_name_for_daemon_bridge(4242), "/agnocast_daemon_bridge@4242");
}

TEST(DaemonBridgeMqTest, PerformanceMqNameIsPerNamespace)
{
  unsetenv("ROS_DOMAIN_ID");
  EXPECT_EQ(
    agnocast::create_mq_name_for_daemon_bridge(agnocast::PERFORMANCE_BRIDGE_VIRTUAL_PID),
    "/agnocast_daemon_bridge_perf");
}

TEST(DaemonBridgeMqTest, PerformanceMqNameAppendsDomainId)
{
  setenv("ROS_DOMAIN_ID", "7", 1);
  EXPECT_EQ(
    agnocast::create_mq_name_for_daemon_bridge(agnocast::PERFORMANCE_BRIDGE_VIRTUAL_PID),
    "/agnocast_daemon_bridge_perf_d7");
  unsetenv("ROS_DOMAIN_ID");
}
