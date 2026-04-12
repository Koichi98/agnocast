from types import SimpleNamespace

import pytest

from ros2agnocast.verb.list_agnocast import (
    BridgeStatus,
    determine_bridge_status,
    get_topic_suffix,
    remove_service_topic,
)


# =========================================
# remove_service_topic tests
# =========================================

class TestRemoveServiceTopic:
    def test_removes_service_topics(self):
        topics = ['/AGNOCAST_SRV_foo', '/normal_topic', '/AGNOCAST_SRV_bar']
        assert remove_service_topic(topics) == ['/normal_topic']

    def test_keeps_all_non_service_topics(self):
        topics = ['/topic_a', '/topic_b']
        assert remove_service_topic(topics) == ['/topic_a', '/topic_b']

    def test_empty_list(self):
        assert remove_service_topic([]) == []

    def test_all_service_topics(self):
        topics = ['/AGNOCAST_SRV_a', '/AGNOCAST_SRV_b']
        assert remove_service_topic(topics) == []


# =========================================
# determine_bridge_status tests
# =========================================

def make_node(is_bridge):
    return SimpleNamespace(is_bridge=is_bridge)


class TestDetermineBridgeStatus:
    def test_no_nodes(self):
        status, has_pub, has_sub = determine_bridge_status([], [])
        assert status == BridgeStatus.NONE
        assert has_pub is False
        assert has_sub is False

    def test_only_agnocast_sub(self):
        sub_nodes = [make_node(is_bridge=False)]
        status, has_pub, has_sub = determine_bridge_status(sub_nodes, [])
        assert status == BridgeStatus.NONE
        assert has_pub is False
        assert has_sub is True

    def test_only_agnocast_pub(self):
        pub_nodes = [make_node(is_bridge=False)]
        status, has_pub, has_sub = determine_bridge_status([], pub_nodes)
        assert status == BridgeStatus.NONE
        assert has_pub is True
        assert has_sub is False

    def test_sub_bridge_only(self):
        """sub_bridge=True means Agnocast->ROS2 bridge direction."""
        sub_nodes = [make_node(is_bridge=True)]
        status, has_pub, has_sub = determine_bridge_status(sub_nodes, [])
        assert status == BridgeStatus.AGNOCAST_TO_ROS2

    def test_pub_bridge_only(self):
        """pub_bridge=True means ROS2->Agnocast bridge direction."""
        pub_nodes = [make_node(is_bridge=True)]
        status, has_pub, has_sub = determine_bridge_status([], pub_nodes)
        assert status == BridgeStatus.ROS2_TO_AGNOCAST

    def test_bidirectional_bridge(self):
        sub_nodes = [make_node(is_bridge=True)]
        pub_nodes = [make_node(is_bridge=True)]
        status, has_pub, has_sub = determine_bridge_status(sub_nodes, pub_nodes)
        assert status == BridgeStatus.BIDIRECTION

    def test_mixed_sub_nodes(self):
        """Both bridge and non-bridge subscriber nodes."""
        sub_nodes = [make_node(is_bridge=True), make_node(is_bridge=False)]
        status, has_pub, has_sub = determine_bridge_status(sub_nodes, [])
        assert status == BridgeStatus.AGNOCAST_TO_ROS2
        assert has_sub is True

    def test_mixed_pub_nodes(self):
        """Both bridge and non-bridge publisher nodes."""
        pub_nodes = [make_node(is_bridge=True), make_node(is_bridge=False)]
        status, has_pub, has_sub = determine_bridge_status([], pub_nodes)
        assert status == BridgeStatus.ROS2_TO_AGNOCAST
        assert has_pub is True


# =========================================
# get_topic_suffix tests
# =========================================

class TestGetTopicSuffix:
    def test_agnocast_only_topic(self):
        """Topic exists only in Agnocast, not in ROS2."""
        suffix = get_topic_suffix(
            '/agnocast_topic',
            agnocast_topics_set={'/agnocast_topic'},
            ros2_topics_set=set(),
            ros2_pub_topics=[],
            ros2_sub_topics=[],
            bridge_status_func=lambda t: None,
        )
        assert suffix == " (Agnocast enabled)"

    def test_ros2_only_topic(self):
        """Topic exists only in ROS2, not in Agnocast."""
        suffix = get_topic_suffix(
            '/ros2_topic',
            agnocast_topics_set=set(),
            ros2_topics_set={'/ros2_topic'},
            ros2_pub_topics=['/ros2_topic'],
            ros2_sub_topics=[],
            bridge_status_func=lambda t: None,
        )
        assert suffix == ""

    def test_both_with_bidirectional_bridge(self):
        """Topic in both Agnocast and ROS2 with bidirectional bridge."""
        suffix = get_topic_suffix(
            '/topic',
            agnocast_topics_set={'/topic'},
            ros2_topics_set={'/topic'},
            ros2_pub_topics=['/topic'],
            ros2_sub_topics=['/topic'],
            bridge_status_func=lambda t: (BridgeStatus.BIDIRECTION, True, True),
        )
        assert suffix == " (Agnocast enabled, bridged)"

    def test_both_with_r2a_bridge_no_a2r_needed(self):
        """ROS2->Agnocast bridge active, no Agnocast->ROS2 needed."""
        suffix = get_topic_suffix(
            '/topic',
            agnocast_topics_set={'/topic'},
            ros2_topics_set={'/topic'},
            ros2_pub_topics=['/topic'],
            ros2_sub_topics=[],
            bridge_status_func=lambda t: (BridgeStatus.ROS2_TO_AGNOCAST, False, True),
        )
        assert suffix == " (Agnocast enabled, bridged)"

    def test_both_with_r2a_bridge_but_a2r_needed(self):
        """ROS2->Agnocast bridge active, but Agnocast->ROS2 also needed (not bridged)."""
        suffix = get_topic_suffix(
            '/topic',
            agnocast_topics_set={'/topic'},
            ros2_topics_set={'/topic'},
            ros2_pub_topics=[],
            ros2_sub_topics=['/topic'],
            bridge_status_func=lambda t: (BridgeStatus.ROS2_TO_AGNOCAST, True, False),
        )
        assert suffix == " (WARN: Agnocast and ROS2 endpoints exist but bridge is not active)"

    def test_both_with_a2r_bridge_no_r2a_needed(self):
        """Agnocast->ROS2 bridge active, no ROS2->Agnocast needed."""
        suffix = get_topic_suffix(
            '/topic',
            agnocast_topics_set={'/topic'},
            ros2_topics_set={'/topic'},
            ros2_pub_topics=[],
            ros2_sub_topics=['/topic'],
            bridge_status_func=lambda t: (BridgeStatus.AGNOCAST_TO_ROS2, True, False),
        )
        assert suffix == " (Agnocast enabled, bridged)"

    def test_both_with_a2r_bridge_but_r2a_needed(self):
        """Agnocast->ROS2 bridge active, but ROS2->Agnocast also needed (not bridged)."""
        suffix = get_topic_suffix(
            '/topic',
            agnocast_topics_set={'/topic'},
            ros2_topics_set={'/topic'},
            ros2_pub_topics=['/topic'],
            ros2_sub_topics=[],
            bridge_status_func=lambda t: (BridgeStatus.AGNOCAST_TO_ROS2, False, True),
        )
        assert suffix == " (WARN: Agnocast and ROS2 endpoints exist but bridge is not active)"

    def test_both_no_bridge_no_need(self):
        """Topic in both but no bridge needed (no cross endpoints)."""
        suffix = get_topic_suffix(
            '/topic',
            agnocast_topics_set={'/topic'},
            ros2_topics_set={'/topic'},
            ros2_pub_topics=[],
            ros2_sub_topics=[],
            bridge_status_func=lambda t: (BridgeStatus.NONE, True, False),
        )
        assert suffix == " (Agnocast enabled)"

    def test_both_no_bridge_but_r2a_needed(self):
        """No bridge active, but ROS2 pub -> Agnocast sub needs bridging."""
        suffix = get_topic_suffix(
            '/topic',
            agnocast_topics_set={'/topic'},
            ros2_topics_set={'/topic'},
            ros2_pub_topics=['/topic'],
            ros2_sub_topics=[],
            bridge_status_func=lambda t: (BridgeStatus.NONE, False, True),
        )
        assert suffix == " (WARN: Agnocast and ROS2 endpoints exist but bridge is not active)"

    def test_both_no_bridge_but_a2r_needed(self):
        """No bridge active, but Agnocast pub -> ROS2 sub needs bridging."""
        suffix = get_topic_suffix(
            '/topic',
            agnocast_topics_set={'/topic'},
            ros2_topics_set={'/topic'},
            ros2_pub_topics=[],
            ros2_sub_topics=['/topic'],
            bridge_status_func=lambda t: (BridgeStatus.NONE, True, False),
        )
        assert suffix == " (WARN: Agnocast and ROS2 endpoints exist but bridge is not active)"
