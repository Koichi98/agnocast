"""Unit tests for the bridge decider.

These need neither the kmod, DDS, nor a POSIX MQ: ``decide_bridges`` is pure
logic, and the wire format is checked against the hand-built byte layout that
mirrors ``sizeof(MqMsgDaemonBridge) == 524`` in ``agnocast_mq.hpp``.
"""

import struct

from ros2agnocast_discovery_agent.bridge_decider import (
    BridgeRequest,
    decide_bridges,
    DIRECTION_AGNOCAST_TO_ROS2,
    DIRECTION_ROS2_TO_AGNOCAST,
    dispatch_requests,
    serialize_request,
)
from ros2agnocast_discovery_agent.type_registry import RegistryEntry
from ros2agnocast_discovery_msgs.msg import (
    AgnocastDaemonState,
    AgnocastEndpoint,
    AgnocastTopic,
)


def _endpoint(node, depth=10, transient=False, reliable=True, is_bridge=False):
    ep = AgnocastEndpoint()
    ep.node_name = node
    ep.qos_depth = depth
    ep.qos_is_transient_local = transient
    ep.qos_is_reliable = reliable
    ep.is_bridge = is_bridge
    return ep


def _topic(name, type_name='std_msgs/msg/Int32', pubs=None, subs=None):
    t = AgnocastTopic()
    t.topic_name = name
    t.type_name = type_name
    t.domain_id = 0
    t.publishers = pubs or []
    t.subscribers = subs or []
    return t


def _state(host_uuid='HOST', ipc_ns=111, topics=None):
    s = AgnocastDaemonState()
    s.schema_version = 1
    s.host_uuid = host_uuid
    s.ipc_ns_inode = ipc_ns
    s.topics = topics or []
    return s


class _FakeRegistry:

    def __init__(self, entries):
        self._entries = entries

    def lookup(self, topic_name, role, node_name):
        return self._entries.get((topic_name, role, node_name))


def test_decide_emits_a2r_when_local_pub_remote_sub():
    local = _state(topics=[_topic('/x', pubs=[_endpoint('/pub')])])
    remote = _state(host_uuid='OTHER', ipc_ns=222,
                    topics=[_topic('/x', subs=[_endpoint('/sub')])])

    reqs = decide_bridges(local, {('OTHER', 222): remote})
    assert len(reqs) == 1
    assert reqs[0].topic_name == '/x'
    assert reqs[0].direction == DIRECTION_AGNOCAST_TO_ROS2
    assert reqs[0].type_name == 'std_msgs/msg/Int32'


def test_decide_emits_r2a_when_local_sub_remote_pub():
    local = _state(topics=[_topic('/x', subs=[_endpoint('/sub')])])
    remote = _state(host_uuid='OTHER', ipc_ns=222,
                    topics=[_topic('/x', pubs=[_endpoint('/pub')])])

    reqs = decide_bridges(local, {('OTHER', 222): remote})
    assert len(reqs) == 1
    assert reqs[0].direction == DIRECTION_ROS2_TO_AGNOCAST


def test_decide_emits_both_directions_when_both_sides_have_both():
    local = _state(topics=[_topic('/x', pubs=[_endpoint('/lp')], subs=[_endpoint('/ls')])])
    remote = _state(host_uuid='OTHER', ipc_ns=222,
                    topics=[_topic('/x', pubs=[_endpoint('/rp')], subs=[_endpoint('/rs')])])

    reqs = decide_bridges(local, {('OTHER', 222): remote})
    directions = sorted(r.direction for r in reqs)
    assert directions == sorted([DIRECTION_ROS2_TO_AGNOCAST, DIRECTION_AGNOCAST_TO_ROS2])


def test_decide_skips_bridge_only_endpoints():
    local = _state(topics=[_topic('/x', pubs=[_endpoint('/lp', is_bridge=True)])])
    remote = _state(host_uuid='OTHER', ipc_ns=222,
                    topics=[_topic('/x', subs=[_endpoint('/rs')])])

    assert decide_bridges(local, {('OTHER', 222): remote}) == []


def test_decide_skips_self_namespace():
    local = _state(topics=[_topic('/x', pubs=[_endpoint('/lp')], subs=[_endpoint('/ls')])])
    assert decide_bridges(local, {('HOST', 111): local}) == []


def test_decide_skips_when_no_topic_overlap():
    local = _state(topics=[_topic('/x', pubs=[_endpoint('/lp')])])
    remote = _state(host_uuid='OTHER', ipc_ns=222,
                    topics=[_topic('/y', subs=[_endpoint('/rs')])])
    assert decide_bridges(local, {('OTHER', 222): remote}) == []


def test_decide_skips_when_type_unknown_on_both_sides():
    local = _state(topics=[_topic('/x', type_name='', pubs=[_endpoint('/lp')])])
    remote = _state(host_uuid='OTHER', ipc_ns=222,
                    topics=[_topic('/x', type_name='', subs=[_endpoint('/rs')])])
    assert decide_bridges(local, {('OTHER', 222): remote}) == []


def test_decide_uses_remote_type_when_local_missing():
    local = _state(topics=[_topic('/x', type_name='', pubs=[_endpoint('/lp')])])
    remote = _state(host_uuid='OTHER', ipc_ns=222,
                    topics=[_topic('/x', type_name='std_msgs/msg/String',
                                   subs=[_endpoint('/rs')])])
    reqs = decide_bridges(local, {('OTHER', 222): remote})
    assert len(reqs) == 1
    assert reqs[0].type_name == 'std_msgs/msg/String'


def test_decide_collapses_duplicates_across_remotes():
    local = _state(topics=[_topic('/x', pubs=[_endpoint('/lp')])])
    remote_a = _state(host_uuid='A', ipc_ns=1, topics=[_topic('/x', subs=[_endpoint('/sa')])])
    remote_b = _state(host_uuid='B', ipc_ns=2, topics=[_topic('/x', subs=[_endpoint('/sb')])])

    reqs = decide_bridges(local, {('A', 1): remote_a, ('B', 2): remote_b})
    assert len(reqs) == 1
    assert reqs[0].direction == DIRECTION_AGNOCAST_TO_ROS2


def test_decide_targets_local_publisher_owner_pid_for_a2r():
    local = _state(topics=[_topic('/x', pubs=[_endpoint('/pub')])])
    remote = _state(host_uuid='OTHER', ipc_ns=222,
                    topics=[_topic('/x', subs=[_endpoint('/sub')])])
    registry = _FakeRegistry(
        {('/x', 'pub', '/pub'): RegistryEntry(pid=4242, type_name='std_msgs/msg/Int32')})

    reqs = decide_bridges(local, {('OTHER', 222): remote}, registry)
    assert len(reqs) == 1
    assert reqs[0].target_pid == 4242


def test_decide_targets_local_subscriber_owner_pid_for_r2a():
    local = _state(topics=[_topic('/x', subs=[_endpoint('/sub')])])
    remote = _state(host_uuid='OTHER', ipc_ns=222,
                    topics=[_topic('/x', pubs=[_endpoint('/pub')])])
    registry = _FakeRegistry(
        {('/x', 'sub', '/sub'): RegistryEntry(pid=7777, type_name='std_msgs/msg/Int32')})

    reqs = decide_bridges(local, {('OTHER', 222): remote}, registry)
    assert len(reqs) == 1
    assert reqs[0].target_pid == 7777


def test_decide_falls_back_to_zero_pid_without_registry():
    local = _state(topics=[_topic('/x', pubs=[_endpoint('/pub')])])
    remote = _state(host_uuid='OTHER', ipc_ns=222,
                    topics=[_topic('/x', subs=[_endpoint('/sub')])])
    reqs = decide_bridges(local, {('OTHER', 222): remote})
    assert len(reqs) == 1
    assert reqs[0].target_pid == 0


def test_serialize_matches_cpp_struct_size():
    req = BridgeRequest('/x', 'std_msgs/msg/Int32', DIRECTION_AGNOCAST_TO_ROS2,
                        10, False, True, 0)
    assert len(serialize_request(req)) == 524


def test_serialize_nul_terminates_truncated_topic():
    req = BridgeRequest('/' + 'a' * 1000, 'T', DIRECTION_AGNOCAST_TO_ROS2, 10, False, True, 0)
    assert serialize_request(req)[255] == 0


def test_serialize_packs_direction_qos_at_expected_offsets():
    req = BridgeRequest('/x', 'T', DIRECTION_ROS2_TO_AGNOCAST, 7, True, True, 0)
    payload = serialize_request(req)
    direction, depth = struct.unpack_from('=II', payload, 512)
    transient, reliable = struct.unpack_from('=BB', payload, 520)
    assert (direction, depth, transient, reliable) == (DIRECTION_ROS2_TO_AGNOCAST, 7, 1, 1)


def test_dispatch_targets_perf_and_owner_standard_mq(monkeypatch):
    from ros2agnocast_discovery_agent import bridge_decider as bd
    sent = []
    monkeypatch.setattr(bd, 'send_request', lambda mq, payload: sent.append(mq) or None)

    req = BridgeRequest('/x', 'T', DIRECTION_AGNOCAST_TO_ROS2, 1, False, True, 12345)
    dispatch_requests([req])

    assert any(name.startswith('/agnocast_daemon_bridge_perf') for name in sent)
    standard = [n for n in sent if n.startswith('/agnocast_daemon_bridge@')]
    assert standard == ['/agnocast_daemon_bridge@12345']


def test_dispatch_skips_standard_mq_when_pid_unknown(monkeypatch):
    from ros2agnocast_discovery_agent import bridge_decider as bd
    sent = []
    monkeypatch.setattr(bd, 'send_request', lambda mq, payload: sent.append(mq) or None)

    req = BridgeRequest('/x', 'T', DIRECTION_AGNOCAST_TO_ROS2, 1, False, True, 0)
    dispatch_requests([req])

    assert any(name.startswith('/agnocast_daemon_bridge_perf') for name in sent)
    assert [n for n in sent if n.startswith('/agnocast_daemon_bridge@')] == []
