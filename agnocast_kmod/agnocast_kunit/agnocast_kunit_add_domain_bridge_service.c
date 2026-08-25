// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_add_domain_bridge_service.h"

#include "../agnocast.h"

#include <kunit/test.h>

// The names below are spelled out rather than built from the SRV_* macros, so a change to the
// naming scheme shows up here as a failing test.
//
// SVC is the name the clients call; SVC_RENAMED is what the server offers when the two differ.
static const char * SVC = "/kunit_svc";
static const char * SVC_RENAMED = "/kunit_svc_renamed";
static const char * SVC_OTHER = "/kunit_svc_other";
// A name no ROS node can resolve to, so it only reaches the ioctl from an unvalidated caller.
static const char * SVC_WITH_SEP = "/kunit_svc%nested";

static const char * REQ = "/AGNOCAST_SRV_REQUEST/kunit_svc";
static const char * REQ_RENAMED = "/AGNOCAST_SRV_REQUEST/kunit_svc_renamed";
// A response topic of one client: the prefix plus the per-client suffix the client appends.
static const char * RES_A = "/AGNOCAST_SRV_RESPONSE/kunit_svc%/nodeA%0";
// Beyond the response prefix by its last component only.
static const char * OUTSIDE = "/AGNOCAST_SRV_RESPONSE/kunit_svc_outside%/nodeA%0";

static int add_service(
  const char * from, const char * to, const uint32_t from_domain, const uint32_t to_domain)
{
  return agnocast_ioctl_add_domain_bridge_service(
    from, to, from_domain, to_domain, current->nsproxy->ipc_ns);
}

static bool has_rule(const char * topic_name, const uint32_t domain)
{
  uint32_t domain_a = 0, domain_b = 0;
  bool a_to_b = false, b_to_a = false;
  return agnocast_get_domain_rule(
    topic_name, current->nsproxy->ipc_ns, domain, &domain_a, &domain_b, &a_to_b, &b_to_a);
}

static void setup_process_in_domain(struct kunit * test, const pid_t pid, const uint32_t domain_id)
{
  union ioctl_add_process_args args;
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_process(pid, current->nsproxy->ipc_ns, false, domain_id, &args), 0);
}

static void add_publisher_named(struct kunit * test, const pid_t pid, const char * topic_name)
{
  union ioctl_add_publisher_args args;
  KUNIT_ASSERT_EQ(
    test,
    agnocast_ioctl_add_publisher(
      topic_name, current->nsproxy->ipc_ns, "/kunit_node", pid, 1, false, false, &args),
    0);
}

void test_case_add_domain_bridge_service_normal(struct kunit * test)
{
  // Act: clients in domain 1, server in domain 2.
  const int ret = add_service(SVC, SVC, 1, 2);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);

  // The request travels from the clients to the server.
  uint32_t domain_a = 0, domain_b = 0;
  bool a_to_b = false, b_to_a = false;
  KUNIT_ASSERT_TRUE(
    test, agnocast_get_domain_rule(
            REQ, current->nsproxy->ipc_ns, 1, &domain_a, &domain_b, &a_to_b, &b_to_a));
  KUNIT_EXPECT_EQ(test, domain_a, 1);
  KUNIT_EXPECT_EQ(test, domain_b, 2);
  KUNIT_EXPECT_TRUE(test, a_to_b);
  KUNIT_EXPECT_FALSE(test, b_to_a);

  // The response travels back, so the same canonical pair carries the other direction.
  domain_a = 0, domain_b = 0, a_to_b = false, b_to_a = false;
  KUNIT_ASSERT_TRUE(
    test, agnocast_get_domain_rule(
            RES_A, current->nsproxy->ipc_ns, 2, &domain_a, &domain_b, &a_to_b, &b_to_a));
  KUNIT_EXPECT_EQ(test, domain_a, 1);
  KUNIT_EXPECT_EQ(test, domain_b, 2);
  KUNIT_EXPECT_FALSE(test, a_to_b);
  KUNIT_EXPECT_TRUE(test, b_to_a);
}

void test_case_add_domain_bridge_service_renames_only_the_request(struct kunit * test)
{
  // Act
  const int ret = add_service(SVC, SVC_RENAMED, 1, 2);

  // Assert: the request rule pairs the two different names.
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_TRUE(test, has_rule(REQ, 1));
  KUNIT_EXPECT_TRUE(test, has_rule(REQ_RENAMED, 2));

  // The response prefix keeps the caller-side name on both sides, so the server's domain reaches
  // it under the *client's* service name and not under the renamed one.
  KUNIT_EXPECT_TRUE(test, has_rule(RES_A, 2));
  KUNIT_EXPECT_FALSE(test, has_rule("/AGNOCAST_SRV_RESPONSE/kunit_svc_renamed%/nodeA%0", 2));
}

void test_case_add_domain_bridge_service_same_domain_rejected(struct kunit * test)
{
  // Act
  const int ret = add_service(SVC, SVC, 3, 3);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

void test_case_add_domain_bridge_service_empty_rejected(struct kunit * test)
{
  // Act
  const int ret = add_service("", "", 1, 2);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

void test_case_add_domain_bridge_service_root_rejected(struct kunit * test)
{
  // Act
  const int ret = add_service("/", "/", 1, 2);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

void test_case_add_domain_bridge_service_relative_rejected(struct kunit * test)
{
  // Act: the same service name without its leading '/'.
  const int ret = add_service(SVC + 1, SVC, 1, 2);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

void test_case_add_domain_bridge_service_relative_target_rejected(struct kunit * test)
{
  // Act: the rename target is the one missing its leading '/'.
  const int ret = add_service(SVC, SVC_RENAMED + 1, 1, 2);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

void test_case_add_domain_bridge_service_separator_in_name_rejected(struct kunit * test)
{
  // Act
  const int ret = add_service(SVC_WITH_SEP, SVC_WITH_SEP, 1, 2);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

void test_case_add_domain_bridge_service_too_long_rejected(struct kunit * test)
{
  // Arrange: a name a name_info can still carry, but not once a topic prefix is on it.
  char svc[TOPIC_NAME_BUFFER_SIZE];
  svc[0] = '/';
  memset(svc + 1, 'a', TOPIC_NAME_BUFFER_SIZE - 2);
  svc[TOPIC_NAME_BUFFER_SIZE - 1] = '\0';

  // Act
  const int ret = add_service(svc, svc, 1, 2);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -ENAMETOOLONG);
}

void test_case_add_domain_bridge_service_redeclaration_is_idempotent(struct kunit * test)
{
  // Arrange: re-running the registration tool over an unchanged config must keep working, even
  // once clients are up.
  KUNIT_ASSERT_EQ(test, add_service(SVC, SVC, 1, 2), 0);
  setup_process_in_domain(test, 1000, 1);
  add_publisher_named(test, 1000, REQ);

  // Act
  const int ret = add_service(SVC, SVC, 1, 2);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
}

void test_case_add_domain_bridge_service_reverse_direction(struct kunit * test)
{
  // Arrange: no endpoint yet, so the mirrored registration is still free to be added.
  KUNIT_ASSERT_EQ(test, add_service(SVC, SVC, 1, 2), 0);

  // Act: the same service declared with the two domains swapped.
  const int ret = add_service(SVC, SVC, 2, 1);

  // Assert: both rules now carry both directions.
  KUNIT_EXPECT_EQ(test, ret, 0);
  uint32_t domain_a = 0, domain_b = 0;
  bool a_to_b = false, b_to_a = false;
  KUNIT_ASSERT_TRUE(
    test, agnocast_get_domain_rule(
            REQ, current->nsproxy->ipc_ns, 1, &domain_a, &domain_b, &a_to_b, &b_to_a));
  KUNIT_EXPECT_TRUE(test, a_to_b);
  KUNIT_EXPECT_TRUE(test, b_to_a);
}

void test_case_add_domain_bridge_service_accepted_beside_another_service(struct kunit * test)
{
  // Arrange: neither service's names fall under the other's.
  KUNIT_ASSERT_EQ(test, add_service(SVC_OTHER, SVC_OTHER, 1, 2), 0);

  // Act
  const int ret = add_service(SVC, SVC, 1, 2);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
}

void test_case_add_domain_bridge_service_accepted_beside_an_exact_rule(struct kunit * test)
{
  // Arrange: a plain topic rule on a name no service covers.
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge(OUTSIDE, OUTSIDE, 1, 2, current->nsproxy->ipc_ns), 0);

  // Act
  const int ret = add_service(SVC, SVC, 1, 2);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
}

void test_case_add_domain_bridge_service_accepted_with_a_topic_outside_it(struct kunit * test)
{
  // Arrange: an endpoint has joined, but on a name neither rule covers.
  setup_process_in_domain(test, 1000, 1);
  add_publisher_named(test, 1000, OUTSIDE);

  // Act
  const int ret = add_service(SVC, SVC, 1, 2);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
}

void test_case_add_domain_bridge_service_accepted_with_a_client_elsewhere(struct kunit * test)
{
  // Arrange: a covered name has an endpoint, but in a domain this service does not bridge.
  setup_process_in_domain(test, 1000, 3);
  add_publisher_named(test, 1000, RES_A);

  // Act
  const int ret = add_service(SVC, SVC, 1, 2);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
}

void test_case_add_domain_bridge_service_accepted_over_a_disjoint_pair(struct kunit * test)
{
  // Arrange
  KUNIT_ASSERT_EQ(test, add_service(SVC, SVC, 1, 2), 0);

  // Act: the same service between two other domains. A lookup filters by domain before the name,
  // so the two registrations never compete.
  const int ret = add_service(SVC, SVC, 3, 4);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, 0);
}

void test_case_add_domain_bridge_service_repointed_pair_rejected(struct kunit * test)
{
  // Arrange
  KUNIT_ASSERT_EQ(test, add_service(SVC, SVC, 1, 2), 0);

  // Act: the same service, pointed at a different partner domain.
  const int ret = add_service(SVC, SVC, 1, 3);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -EBUSY);
}

void test_case_add_domain_bridge_service_late_reverse_direction_rejected(struct kunit * test)
{
  // Arrange: a client joined while only one direction was declared.
  KUNIT_ASSERT_EQ(test, add_service(SVC, SVC, 1, 2), 0);
  setup_process_in_domain(test, 1000, 1);
  add_publisher_named(test, 1000, REQ);

  // Act
  const int ret = add_service(SVC, SVC, 2, 1);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -EBUSY);
}

void test_case_add_domain_bridge_service_rejected_when_client_joined(struct kunit * test)
{
  // Arrange: a client's request publisher joined before any rule was registered.
  setup_process_in_domain(test, 1000, 1);
  add_publisher_named(test, 1000, REQ);

  // Act
  const int ret = add_service(SVC, SVC, 1, 2);

  // Assert: the request half is rejected, so neither half is registered.
  KUNIT_EXPECT_EQ(test, ret, -EBUSY);
  KUNIT_EXPECT_FALSE(test, has_rule(RES_A, 2));
}

void test_case_add_domain_bridge_service_rejected_when_response_joined(struct kunit * test)
{
  // Arrange: a response topic already has an endpoint, so the prefix half is the one rejected.
  setup_process_in_domain(test, 1000, 2);
  add_publisher_named(test, 1000, RES_A);

  // Act
  const int ret = add_service(SVC, SVC, 1, 2);

  // Assert: rejected, and the request rule the request half validated is never inserted.
  KUNIT_EXPECT_EQ(test, ret, -EBUSY);
  KUNIT_EXPECT_FALSE(test, has_rule(REQ, 1));
}

void test_case_add_domain_bridge_service_over_exact_response_rejected(struct kunit * test)
{
  // Arrange: an exact rule already covers a name the response prefix would shadow.
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge(RES_A, RES_A, 1, 2, current->nsproxy->ipc_ns), 0);

  // Act
  const int ret = add_service(SVC, SVC, 1, 2);

  // Assert: rejected, and the request rule is never inserted.
  KUNIT_EXPECT_EQ(test, ret, -EBUSY);
  KUNIT_EXPECT_FALSE(test, has_rule(REQ, 1));
}

void test_case_add_domain_bridge_service_over_exact_request_rejected(struct kunit * test)
{
  // Arrange: the request topic is already paired with a different cell.
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge(REQ, OUTSIDE, 1, 2, current->nsproxy->ipc_ns), 0);

  // Act
  const int ret = add_service(SVC, SVC, 1, 2);

  // Assert: the request half is rejected, so the response prefix is never registered.
  KUNIT_EXPECT_EQ(test, ret, -EBUSY);
  KUNIT_EXPECT_FALSE(test, has_rule(RES_A, 2));
}

void test_case_add_domain_bridge_service_keeps_a_pre_existing_request_rule(struct kunit * test)
{
  // Arrange: the request topic is already bridged on its own, and a response endpoint makes the
  // prefix half fail. The request half accepts a declaration that enables nothing new.
  setup_process_in_domain(test, 1000, 2);
  add_publisher_named(test, 1000, RES_A);
  KUNIT_ASSERT_EQ(
    test, agnocast_ioctl_add_domain_bridge(REQ, REQ, 1, 2, current->nsproxy->ipc_ns), 0);

  // Act
  const int ret = add_service(SVC, SVC, 1, 2);

  // Assert
  KUNIT_EXPECT_EQ(test, ret, -EBUSY);
  KUNIT_EXPECT_TRUE(test, has_rule(REQ, 1));
}

void test_case_add_domain_bridge_service_does_not_add_the_direction(struct kunit * test)
{
  // Arrange: one direction is registered, then a client joins a response topic, which is what
  // makes the prefix half reject the second direction while the request half still accepts it.
  KUNIT_ASSERT_EQ(test, add_service(SVC, SVC, 1, 2), 0);
  setup_process_in_domain(test, 1000, 2);
  add_publisher_named(test, 1000, RES_A);

  // Act
  const int ret = add_service(SVC, SVC, 2, 1);

  // Assert: the direction the request half would have added stays off.
  KUNIT_EXPECT_EQ(test, ret, -EBUSY);
  uint32_t domain_a = 0, domain_b = 0;
  bool a_to_b = false, b_to_a = false;
  KUNIT_ASSERT_TRUE(
    test, agnocast_get_domain_rule(
            REQ, current->nsproxy->ipc_ns, 1, &domain_a, &domain_b, &a_to_b, &b_to_a));
  KUNIT_EXPECT_TRUE(test, a_to_b);
  KUNIT_EXPECT_FALSE(test, b_to_a);
}
