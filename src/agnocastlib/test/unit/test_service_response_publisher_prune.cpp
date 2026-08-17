// Selection rules of detail::prune_departed_response_publishers. The predicate is injected, so
// these run without a kernel module; what is exercised is which entries may be erased.
#include "agnocast/agnocast_service.hpp"

#include <gtest/gtest.h>

#include <set>
#include <string>
#include <unordered_map>

namespace
{

// Stands in for a response publisher: the prune only moves these around, never uses them.
using FakePublishers = std::unordered_map<std::string, int>;

FakePublishers three_callers()
{
  return {{"/res_a", 1}, {"/res_b", 2}, {"/res_c", 3}};
}

std::set<std::string> keys_of(const FakePublishers & publishers)
{
  std::set<std::string> keys;
  for (const auto & [name, _] : publishers) {
    keys.insert(name);
  }
  return keys;
}

auto subscribed(std::set<std::string> alive)
{
  return [alive = std::move(alive)](const std::string & name) { return alive.count(name) > 0; };
}

}  // namespace

TEST(ServiceResponsePublisherPrune, erases_only_callers_without_a_subscriber)
{
  auto publishers = three_callers();
  const std::unordered_map<std::string, uint32_t> pending;

  agnocast::detail::prune_departed_response_publishers(
    publishers, "/res_a", pending, subscribed({"/res_a", "/res_c"}));

  EXPECT_EQ(keys_of(publishers), std::set<std::string>({"/res_a", "/res_c"}));
}

TEST(ServiceResponsePublisherPrune, keeps_the_caller_being_served)
{
  auto publishers = three_callers();
  const std::unordered_map<std::string, uint32_t> pending;

  // /res_b has no subscriber yet -- it is being served right now, so it must survive.
  agnocast::detail::prune_departed_response_publishers(
    publishers, "/res_b", pending, subscribed({}));

  EXPECT_EQ(keys_of(publishers), std::set<std::string>({"/res_b"}));
}

TEST(ServiceResponsePublisherPrune, keeps_callers_with_a_borrowed_response)
{
  auto publishers = three_callers();
  // A deferred response was borrowed for /res_c and not sent yet. Erasing it would make
  // send_response() resolve to a different publisher than the response came from.
  const std::unordered_map<std::string, uint32_t> pending{{"/res_c", 1}};

  agnocast::detail::prune_departed_response_publishers(
    publishers, "/res_a", pending, subscribed({}));

  EXPECT_EQ(keys_of(publishers), std::set<std::string>({"/res_a", "/res_c"}));
}

TEST(ServiceResponsePublisherPrune, keeps_everything_while_all_callers_are_alive)
{
  auto publishers = three_callers();
  const std::unordered_map<std::string, uint32_t> pending;

  agnocast::detail::prune_departed_response_publishers(
    publishers, "/res_a", pending, subscribed({"/res_a", "/res_b", "/res_c"}));

  EXPECT_EQ(keys_of(publishers), keys_of(three_callers()));
}

TEST(ServiceResponsePublisherPrune, empty_map_is_a_no_op)
{
  FakePublishers publishers;
  const std::unordered_map<std::string, uint32_t> pending;

  agnocast::detail::prune_departed_response_publishers(
    publishers, "/res_a", pending, subscribed({}));

  EXPECT_TRUE(publishers.empty());
}
