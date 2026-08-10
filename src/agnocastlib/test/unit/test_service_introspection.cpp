#include "agnocast/internal/service_introspection.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <set>
#include <string>
#include <vector>

TEST(ServiceIntrospectionTest, EventTopicNameFollowsRos2Convention)
{
  EXPECT_EQ(
    agnocast::internal::create_service_event_topic_name("/sample/service"),
    "/sample/service/_service_event");
  EXPECT_EQ(
    agnocast::internal::create_service_event_topic_name("/ns/node/srv"),
    "/ns/node/srv/_service_event");
}

TEST(ServiceIntrospectionTest, ClientGidIsDeterministic)
{
  const auto first = agnocast::internal::make_service_client_gid("/ns/talker");
  const auto second = agnocast::internal::make_service_client_gid("/ns/talker");

  EXPECT_EQ(first, second)
    << "the GID must be reproducible so the client and the server derive the same value";
}

TEST(ServiceIntrospectionTest, ClientGidDiffersBetweenNodes)
{
  const std::vector<std::string> node_names = {"/talker",      "/listener", "/ns/talker",
                                               "/ns/listener", "/talker2",  ""};

  std::set<std::array<uint8_t, 16>> gids;
  for (const auto & name : node_names) {
    gids.insert(agnocast::internal::make_service_client_gid(name));
  }

  EXPECT_EQ(gids.size(), node_names.size()) << "distinct node names must produce distinct GIDs";
}

TEST(ServiceIntrospectionTest, ClientGidUsesAllSixteenBytes)
{
  const auto gid = agnocast::internal::make_service_client_gid("/ns/talker");

  // Both halves come from separate hash runs; neither may be left as zero padding.
  EXPECT_TRUE(std::any_of(gid.begin(), gid.begin() + 8, [](uint8_t b) { return b != 0; }));
  EXPECT_TRUE(std::any_of(gid.begin() + 8, gid.end(), [](uint8_t b) { return b != 0; }));
}
