/* SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause */
#pragma once
#include <kunit/test.h>

#define TEST_CASES_ADD_PROCESS                                                             \
  KUNIT_CASE(test_case_add_process_normal), KUNIT_CASE(test_case_add_process_many),        \
    KUNIT_CASE(test_case_add_process_twice),                                               \
    KUNIT_CASE(test_case_add_process_bridge_manager_per_domain),                           \
    KUNIT_CASE(test_case_add_process_too_many),                                            \
    KUNIT_CASE(test_case_add_process_unlink_daemon_registration),                          \
    KUNIT_CASE(test_case_add_process_unlink_daemon_duplicate_refused),                     \
    KUNIT_CASE(test_case_add_process_unlink_daemon_respawns_after_death),                  \
    KUNIT_CASE(test_case_add_process_unlink_daemon_is_not_counted_as_work),                \
    KUNIT_CASE(test_case_add_process_unlink_daemon_removed_on_death),                      \
    KUNIT_CASE(test_case_add_process_unlink_daemon_deregisters_when_told_to_exit),         \
    KUNIT_CASE(test_case_add_process_rejects_unknown_role),                                \
    KUNIT_CASE(test_case_add_process_rejects_the_daemon_domain_id),                        \
    KUNIT_CASE(test_case_add_process_daemon_may_send_the_daemon_domain_id),                \
    KUNIT_CASE(test_case_add_process_unlink_daemon_deregistration_frees_its_mempool_slot), \
    KUNIT_CASE(test_case_add_process_unlink_daemon_stays_registered_while_draining),       \
    KUNIT_CASE(test_case_add_process_spawn_right_granted_once),                            \
    KUNIT_CASE(test_case_add_process_spawn_right_released_right_is_regrantable),           \
    KUNIT_CASE(test_case_add_process_spawn_right_settled_by_daemon_registration),          \
    KUNIT_CASE(test_case_add_process_spawn_right_settled_by_bridge_manager_registration),  \
    KUNIT_CASE(test_case_add_process_spawn_right_regranted_after_early_deregistration),    \
    KUNIT_CASE(test_case_add_process_spawn_right_not_granted_while_daemon_alive),          \
    KUNIT_CASE(test_case_add_process_spawn_right_bridge_is_per_domain),                    \
    KUNIT_CASE(test_case_add_process_spawn_right_unlink_is_per_namespace),                 \
    KUNIT_CASE(test_case_add_process_spawn_right_only_requested_kinds_are_granted),        \
    KUNIT_CASE(test_case_add_process_spawn_right_release_is_idempotent_after_settle),      \
    KUNIT_CASE(test_case_add_process_spawn_right_survives_exit_of_an_unregistered_process)

void test_case_add_process_normal(struct kunit * test);
void test_case_add_process_many(struct kunit * test);
void test_case_add_process_twice(struct kunit * test);
void test_case_add_process_bridge_manager_per_domain(struct kunit * test);
void test_case_add_process_too_many(struct kunit * test);
void test_case_add_process_unlink_daemon_registration(struct kunit * test);
void test_case_add_process_unlink_daemon_duplicate_refused(struct kunit * test);
void test_case_add_process_unlink_daemon_respawns_after_death(struct kunit * test);
void test_case_add_process_unlink_daemon_is_not_counted_as_work(struct kunit * test);
void test_case_add_process_unlink_daemon_removed_on_death(struct kunit * test);
void test_case_add_process_unlink_daemon_deregisters_when_told_to_exit(struct kunit * test);
void test_case_add_process_rejects_unknown_role(struct kunit * test);
void test_case_add_process_rejects_the_daemon_domain_id(struct kunit * test);
void test_case_add_process_daemon_may_send_the_daemon_domain_id(struct kunit * test);
void test_case_add_process_unlink_daemon_deregistration_frees_its_mempool_slot(struct kunit * test);
void test_case_add_process_unlink_daemon_stays_registered_while_draining(struct kunit * test);
void test_case_add_process_spawn_right_granted_once(struct kunit * test);
void test_case_add_process_spawn_right_released_right_is_regrantable(struct kunit * test);
void test_case_add_process_spawn_right_settled_by_daemon_registration(struct kunit * test);
void test_case_add_process_spawn_right_settled_by_bridge_manager_registration(struct kunit * test);
void test_case_add_process_spawn_right_regranted_after_early_deregistration(struct kunit * test);
void test_case_add_process_spawn_right_not_granted_while_daemon_alive(struct kunit * test);
void test_case_add_process_spawn_right_bridge_is_per_domain(struct kunit * test);
void test_case_add_process_spawn_right_unlink_is_per_namespace(struct kunit * test);
void test_case_add_process_spawn_right_only_requested_kinds_are_granted(struct kunit * test);
void test_case_add_process_spawn_right_release_is_idempotent_after_settle(struct kunit * test);
void test_case_add_process_spawn_right_survives_exit_of_an_unregistered_process(
  struct kunit * test);
