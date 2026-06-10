#include "agnocast/agnocast_mq.hpp"
#include "agnocast/agnocast_utils.hpp"
#include "agnocast/bridge/agnocast_bridge_utils.hpp"

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

// Performance-mode daemon bridges have no local endpoint to query, so the QoS
// must be rebuilt faithfully from the request's explicit fields.
TEST(DaemonBridgeMqTest, DaemonRequestQosReliableTransientLocal)
{
  agnocast::MqMsgDaemonBridge req{};
  req.qos_depth = 10;
  req.qos_is_reliable = true;
  req.qos_is_transient_local = true;

  const rclcpp::QoS qos = agnocast::daemon_request_qos(req);
  EXPECT_EQ(qos.depth(), 10u);
  EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(qos.durability(), rclcpp::DurabilityPolicy::TransientLocal);
}

TEST(DaemonBridgeMqTest, DaemonRequestQosBestEffortVolatile)
{
  agnocast::MqMsgDaemonBridge req{};
  req.qos_depth = 1;
  req.qos_is_reliable = false;
  req.qos_is_transient_local = false;

  const rclcpp::QoS qos = agnocast::daemon_request_qos(req);
  EXPECT_EQ(qos.depth(), 1u);
  EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::BestEffort);
  EXPECT_EQ(qos.durability(), rclcpp::DurabilityPolicy::Volatile);
}
