"""Unit tests for parsing the domain_bridge YAML into kmod rule tuples.

These exercise the pure parser only; no kmod, DDS, or file I/O is involved.
"""

import pytest

from ros2agnocast_discovery_agent.domain_bridge_config import parse_domain_bridge_config


def test_top_level_domains_apply_to_each_topic():
    text = """
from_domain: 1
to_domain: 2
topics:
  chatter:
    type: std_msgs/msg/String
"""
    assert parse_domain_bridge_config(text) == ([('chatter', 'chatter', 1, 2)], [], [])


def test_per_topic_domains_override_top_level():
    text = """
from_domain: 1
to_domain: 2
topics:
  chatter:
    type: std_msgs/msg/String
  special:
    from_domain: 3
    to_domain: 4
"""
    rules, _services, skipped = parse_domain_bridge_config(text)
    assert ('chatter', 'chatter', 1, 2) in rules
    assert ('special', 'special', 3, 4) in rules
    assert skipped == []


def test_remap_sets_the_target_topic_name():
    # The `remap` field becomes the to_topic; the source name stays the from_topic.
    text = """
from_domain: 1
to_domain: 2
topics:
  /in_sub/chatter:
    type: std_msgs/msg/String
    remap: /chatter
"""
    assert parse_domain_bridge_config(text) == ([('/in_sub/chatter', '/chatter', 1, 2)], [], [])


def test_absent_remap_reuses_the_source_name():
    text = """
from_domain: 1
to_domain: 2
topics:
  chatter:
    type: std_msgs/msg/String
"""
    assert parse_domain_bridge_config(text) == ([('chatter', 'chatter', 1, 2)], [], [])


def test_non_string_topic_key_without_remap_is_coerced():
    # A non-string YAML key (here an integer) with no `remap` must not trip the remap
    # string check; both names default to the coerced source name.
    text = """
from_domain: 1
to_domain: 2
topics:
  123:
"""
    assert parse_domain_bridge_config(text) == ([('123', '123', 1, 2)], [], [])


def test_non_string_remap_raises():
    text = """
from_domain: 1
to_domain: 2
topics:
  chatter:
    remap: [not, a, string]
"""
    with pytest.raises((ValueError, TypeError)):
        parse_domain_bridge_config(text)


def test_topic_without_resolvable_domain_pair_is_reported_as_skipped():
    text = """
topics:
  chatter:
    type: std_msgs/msg/String
"""
    rules, _services, skipped = parse_domain_bridge_config(text)
    assert rules == []
    assert skipped == ['chatter']


def test_empty_or_topicless_config_yields_no_rules():
    assert parse_domain_bridge_config('') == ([], [], [])
    assert parse_domain_bridge_config('topics:') == ([], [], [])
    assert parse_domain_bridge_config('services:') == ([], [], [])


def test_non_integer_domain_raises():
    # A non-integer domain raises; the agent catches this and runs without rules
    # rather than crashing at startup.
    text = """
from_domain: not_a_number
to_domain: 2
topics:
  chatter:
    type: std_msgs/msg/String
"""
    with pytest.raises(ValueError):
        parse_domain_bridge_config(text)


def test_out_of_range_domain_raises():
    # uint32 overflow would wrap silently at the ioctl boundary, so reject it.
    text = """
from_domain: 1
to_domain: 4294967296
topics:
  chatter:
    type: std_msgs/msg/String
"""
    with pytest.raises(ValueError):
        parse_domain_bridge_config(text)


def test_negative_domain_raises():
    text = """
from_domain: -1
to_domain: 2
topics:
  chatter:
    type: std_msgs/msg/String
"""
    with pytest.raises(ValueError):
        parse_domain_bridge_config(text)


# Malformed structure must raise a caught exception (ValueError/TypeError), not
# AttributeError, so the daemon runs without rules instead of crashing.

def test_non_mapping_root_raises():
    with pytest.raises((ValueError, TypeError)):
        parse_domain_bridge_config('- a\n- b\n')


def test_non_mapping_topics_raises():
    with pytest.raises((ValueError, TypeError)):
        parse_domain_bridge_config('topics:\n  - chatter\n  - special\n')


def test_non_mapping_topic_spec_raises():
    with pytest.raises((ValueError, TypeError)):
        parse_domain_bridge_config('from_domain: 1\nto_domain: 2\ntopics:\n  chatter: oops\n')


def test_non_mapping_services_raises():
    with pytest.raises((ValueError, TypeError)):
        parse_domain_bridge_config('services:\n  - add_two_ints\n')


def test_services_are_parsed_into_their_own_list():
    # from_domain is the clients' side and to_domain the server's; the caller turns each
    # entry into the request and response topic rules.
    text = """
from_domain: 2
to_domain: 1
topics:
  chatter:
services:
  /add_two_ints:
    type: example_interfaces/srv/AddTwoInts
"""
    topics, services, skipped = parse_domain_bridge_config(text)
    assert topics == [('chatter', 'chatter', 2, 1)]
    assert services == [('/add_two_ints', '/add_two_ints', 2, 1)]
    assert skipped == []


def test_service_remap_and_per_entry_domains_are_honored():
    text = """
from_domain: 2
to_domain: 1
services:
  /add_two_ints:
    remap: /renamed_add
  /other:
    from_domain: 4
    to_domain: 3
"""
    _topics, services, skipped = parse_domain_bridge_config(text)
    assert ('/add_two_ints', '/renamed_add', 2, 1) in services
    assert ('/other', '/other', 4, 3) in services
    assert skipped == []


def test_service_without_resolvable_domain_pair_is_reported_as_skipped():
    _topics, services, skipped = parse_domain_bridge_config('services:\n  /add_two_ints:\n')
    assert services == []
    assert skipped == ['/add_two_ints']


def test_null_topic_spec_with_top_level_domains_is_used():
    # `chatter:` with no body is a None spec; it should fall back to the
    # top-level domains, not crash.
    text = """
from_domain: 1
to_domain: 2
topics:
  chatter:
"""
    assert parse_domain_bridge_config(text) == ([('chatter', 'chatter', 1, 2)], [], [])
