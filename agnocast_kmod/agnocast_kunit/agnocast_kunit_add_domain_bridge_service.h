/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#pragma once
#include <kunit/test.h>

#define TEST_CASES_ADD_DOMAIN_BRIDGE_SERVICE                                           \
  KUNIT_CASE(test_case_add_domain_bridge_service_normal),                              \
    KUNIT_CASE(test_case_add_domain_bridge_service_renames_only_the_request),          \
    KUNIT_CASE(test_case_add_domain_bridge_service_same_domain_rejected),              \
    KUNIT_CASE(test_case_add_domain_bridge_service_empty_rejected),                    \
    KUNIT_CASE(test_case_add_domain_bridge_service_root_rejected),                     \
    KUNIT_CASE(test_case_add_domain_bridge_service_relative_rejected),                 \
    KUNIT_CASE(test_case_add_domain_bridge_service_relative_target_rejected),          \
    KUNIT_CASE(test_case_add_domain_bridge_service_too_long_rejected),                 \
    KUNIT_CASE(test_case_add_domain_bridge_service_redeclaration_is_idempotent),       \
    KUNIT_CASE(test_case_add_domain_bridge_service_reverse_direction),                 \
    KUNIT_CASE(test_case_add_domain_bridge_service_accepted_beside_another_service),   \
    KUNIT_CASE(test_case_add_domain_bridge_service_accepted_beside_an_exact_rule),     \
    KUNIT_CASE(test_case_add_domain_bridge_service_accepted_with_a_topic_outside_it),  \
    KUNIT_CASE(test_case_add_domain_bridge_service_accepted_with_a_client_elsewhere),  \
    KUNIT_CASE(test_case_add_domain_bridge_service_nested_over_disjoint_domains),      \
    KUNIT_CASE(test_case_add_domain_bridge_service_repointed_pair_rejected),           \
    KUNIT_CASE(test_case_add_domain_bridge_service_nested_rejected),                   \
    KUNIT_CASE(test_case_add_domain_bridge_service_nested_rejected_either_order),      \
    KUNIT_CASE(test_case_add_domain_bridge_service_late_reverse_direction_rejected),   \
    KUNIT_CASE(test_case_add_domain_bridge_service_rejected_when_client_joined),       \
    KUNIT_CASE(test_case_add_domain_bridge_service_rejected_when_response_joined),     \
    KUNIT_CASE(test_case_add_domain_bridge_service_over_exact_response_rejected),      \
    KUNIT_CASE(test_case_add_domain_bridge_service_over_exact_request_rejected),       \
    KUNIT_CASE(test_case_add_domain_bridge_service_keeps_a_pre_existing_request_rule), \
    KUNIT_CASE(test_case_add_domain_bridge_service_does_not_add_the_direction)

void test_case_add_domain_bridge_service_normal(struct kunit * test);
void test_case_add_domain_bridge_service_renames_only_the_request(struct kunit * test);
void test_case_add_domain_bridge_service_same_domain_rejected(struct kunit * test);
void test_case_add_domain_bridge_service_empty_rejected(struct kunit * test);
void test_case_add_domain_bridge_service_root_rejected(struct kunit * test);
void test_case_add_domain_bridge_service_relative_rejected(struct kunit * test);
void test_case_add_domain_bridge_service_relative_target_rejected(struct kunit * test);
void test_case_add_domain_bridge_service_too_long_rejected(struct kunit * test);
void test_case_add_domain_bridge_service_redeclaration_is_idempotent(struct kunit * test);
void test_case_add_domain_bridge_service_reverse_direction(struct kunit * test);
void test_case_add_domain_bridge_service_accepted_beside_another_service(struct kunit * test);
void test_case_add_domain_bridge_service_accepted_beside_an_exact_rule(struct kunit * test);
void test_case_add_domain_bridge_service_accepted_with_a_topic_outside_it(struct kunit * test);
void test_case_add_domain_bridge_service_accepted_with_a_client_elsewhere(struct kunit * test);
void test_case_add_domain_bridge_service_nested_over_disjoint_domains(struct kunit * test);
void test_case_add_domain_bridge_service_repointed_pair_rejected(struct kunit * test);
void test_case_add_domain_bridge_service_nested_rejected(struct kunit * test);
void test_case_add_domain_bridge_service_nested_rejected_either_order(struct kunit * test);
void test_case_add_domain_bridge_service_late_reverse_direction_rejected(struct kunit * test);
void test_case_add_domain_bridge_service_rejected_when_client_joined(struct kunit * test);
void test_case_add_domain_bridge_service_rejected_when_response_joined(struct kunit * test);
void test_case_add_domain_bridge_service_over_exact_response_rejected(struct kunit * test);
void test_case_add_domain_bridge_service_over_exact_request_rejected(struct kunit * test);
void test_case_add_domain_bridge_service_keeps_a_pre_existing_request_rule(struct kunit * test);
void test_case_add_domain_bridge_service_does_not_add_the_direction(struct kunit * test);
