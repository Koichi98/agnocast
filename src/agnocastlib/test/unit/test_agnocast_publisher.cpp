#include "agnocast/agnocast.hpp"
#include "agnocast/agnocast_publisher.hpp"
#include "agnocast/node/agnocast_node.hpp"

#include "std_msgs/msg/string.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>

// Defined in test_mocked_agnocast.cpp, which mocks initialize_publisher() so a publisher can
// be constructed without a kernel module.
extern size_t initialize_publisher_mock_last_qos_depth;
extern bool initialize_publisher_mock_last_is_ros2_node;

using StringMsg = std_msgs::msg::String;

class PublisherGetActualQosTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(PublisherGetActualQosTest, get_actual_qos_reports_the_qos_passed_at_construction)
{
  // Arrange
  auto node = std::make_shared<rclcpp::Node>("test_pub_actual_qos");
  const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(7)).transient_local();

  // Act
  auto pub = agnocast::create_publisher<StringMsg>(node.get(), "/test_pub_actual_qos", qos);

  // Assert
  EXPECT_EQ(pub->get_actual_qos(), qos);
}

TEST_F(PublisherGetActualQosTest, get_actual_qos_reports_the_qos_of_a_generic_publisher)
{
  // Arrange
  auto node = std::make_shared<rclcpp::Node>("test_generic_pub_actual_qos");
  const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(3));

  // Act
  agnocast::GenericPublisher pub(
    node.get(), "/test_generic_pub_actual_qos", "std_msgs/msg/String", qos);

  // Assert
  EXPECT_EQ(pub.get_actual_qos(), qos);
}

TEST_F(PublisherGetActualQosTest, get_actual_qos_reflects_a_qos_parameter_override)
{
  // Arrange
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(
    {rclcpp::Parameter("qos_overrides./test_pub_actual_qos_override.publisher.depth", 9)});
  auto node = std::make_shared<rclcpp::Node>("test_pub_actual_qos_override", node_options);

  agnocast::PublisherOptions options;
  options.qos_overriding_options = rclcpp::QosOverridingOptions({rclcpp::QosPolicyKind::Depth});

  // Act: depth 1 is expected to be replaced by the override.
  auto pub = agnocast::create_publisher<StringMsg>(
    node.get(), "/test_pub_actual_qos_override", rclcpp::QoS{1}, options);

  // Assert
  EXPECT_EQ(pub->get_actual_qos().depth(), 9u);
}

TEST_F(PublisherGetActualQosTest, the_resolved_qos_reaches_the_kernel_registration)
{
  // Arrange
  auto node = std::make_shared<rclcpp::Node>("test_pub_actual_qos_registration");
  initialize_publisher_mock_last_qos_depth = 0;

  // Act
  auto pub = agnocast::create_publisher<StringMsg>(
    node.get(), "/test_pub_actual_qos_registration", rclcpp::QoS(rclcpp::KeepLast(6)));

  // Assert
  EXPECT_EQ(initialize_publisher_mock_last_qos_depth, 6u);
}

// An agnocast::Node is invisible to DDS while every other node type carries a participant, and
// NodeGraph::get_node_names() splits the graph on exactly this flag: getting it backwards either
// reports an rclcpp::Node twice or drops an agnocast::Node entirely.
class PublisherIsRos2NodeTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }

  static agnocast::Node::SharedPtr make_agnocast_node(const std::string & name)
  {
    rclcpp::NodeOptions options;
    options.start_parameter_services(false);
    return std::make_shared<agnocast::Node>(name, "/", options);
  }
};

TEST_F(PublisherIsRos2NodeTest, a_publisher_of_an_rclcpp_node_registers_as_a_ros2_node)
{
  auto node = std::make_shared<rclcpp::Node>("test_pub_is_ros2_rclcpp");
  initialize_publisher_mock_last_is_ros2_node = false;

  auto pub = agnocast::create_publisher<StringMsg>(node.get(), "/test_pub_is_ros2_rclcpp", 1);

  EXPECT_TRUE(initialize_publisher_mock_last_is_ros2_node);
}

TEST_F(PublisherIsRos2NodeTest, a_publisher_of_an_agnocast_node_does_not_register_as_a_ros2_node)
{
  auto node = make_agnocast_node("test_pub_is_ros2_agnocast");
  initialize_publisher_mock_last_is_ros2_node = true;

  auto pub = agnocast::create_publisher<StringMsg>(node.get(), "/test_pub_is_ros2_agnocast", 1);

  EXPECT_FALSE(initialize_publisher_mock_last_is_ros2_node);
}

namespace
{
class DerivedAgnocastNode : public agnocast::Node
{
public:
  DerivedAgnocastNode(const std::string & name, const rclcpp::NodeOptions & options)
  : agnocast::Node(name, "/", options)
  {
  }
};
}  // namespace

// create_publisher() accepts a node derived from either base, so the flag must follow the base and
// not the concrete type the caller happened to pass.
TEST_F(PublisherIsRos2NodeTest, a_publisher_of_a_node_derived_from_agnocast_node_follows_the_base)
{
  rclcpp::NodeOptions options;
  options.start_parameter_services(false);
  auto node = std::make_shared<DerivedAgnocastNode>("test_pub_is_ros2_derived", options);
  initialize_publisher_mock_last_is_ros2_node = true;

  auto pub = agnocast::create_publisher<StringMsg>(node.get(), "/test_pub_is_ros2_derived", 1);

  EXPECT_FALSE(initialize_publisher_mock_last_is_ros2_node);
}
