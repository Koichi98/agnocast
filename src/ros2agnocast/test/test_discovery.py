"""Unit tests for ros2agnocast.discovery helpers.

These tests do not require DDS; they exercise the projection helpers with
hand-built AgnocastDaemonState messages.
"""

import time

from builtin_interfaces.msg import Time

from ros2agnocast_discovery_msgs.msg import (
    AgnocastDaemonState,
    AgnocastEndpoint,
    AgnocastTopic,
)

from ros2agnocast.discovery import (
    all_nodes,
    all_topic_names,
    filter_fresh,
    is_stale,
    topic_endpoints,
    topics_of_node,
)


def _endpoint(node_name: str, *, is_bridge: bool = False, qos_depth: int = 10) -> AgnocastEndpoint:
    ep = AgnocastEndpoint()
    ep.node_name = node_name
    ep.pid = 0
    ep.qos_depth = qos_depth
    ep.qos_is_transient_local = False
    ep.qos_is_reliable = True
    ep.is_bridge = is_bridge
    return ep


def _topic(topic_name: str, *, pubs=None, subs=None, type_name: str = '') -> AgnocastTopic:
    topic = AgnocastTopic()
    topic.topic_name = topic_name
    topic.type_name = type_name
    topic.domain_id = 0
    topic.publishers = pubs or []
    topic.subscribers = subs or []
    return topic


def _state(host: str, ipc_ns: int, *, topics=None, ts_sec: int = 0) -> AgnocastDaemonState:
    state = AgnocastDaemonState()
    state.schema_version = 1
    state.agnocast_version = ''
    state.host_uuid = host
    state.host_hostname = host
    state.timestamp = Time(sec=ts_sec, nanosec=0) if ts_sec else Time(sec=int(time.time()), nanosec=0)
    state.ipc_ns_inode = ipc_ns
    state.topics = topics or []
    return state


def test_all_topic_names_unions_across_snapshots():
    snap_a = _state('a', 1, topics=[_topic('/foo'), _topic('/bar')])
    snap_b = _state('b', 2, topics=[_topic('/bar'), _topic('/baz')])
    assert all_topic_names([snap_a, snap_b]) == {'/foo', '/bar', '/baz'}


def test_all_topic_names_empty_when_no_snapshots():
    assert all_topic_names([]) == set()


def test_all_nodes_includes_both_publishers_and_subscribers():
    pub = _endpoint('/talker')
    sub = _endpoint('/listener')
    snap = _state('a', 1, topics=[_topic('/foo', pubs=[pub], subs=[sub])])
    assert all_nodes([snap]) == {'/talker', '/listener'}


def test_topic_endpoints_returns_pubs_and_subs_for_named_topic():
    pub1 = _endpoint('/talker_a')
    pub2 = _endpoint('/talker_b')
    sub = _endpoint('/listener')
    snap_a = _state('a', 1, topics=[_topic('/foo', pubs=[pub1])])
    snap_b = _state('b', 2, topics=[_topic('/foo', subs=[sub]), _topic('/bar', pubs=[pub2])])
    pubs, subs = topic_endpoints([snap_a, snap_b], '/foo')
    assert [p.node_name for p in pubs] == ['/talker_a']
    assert [s.node_name for s in subs] == ['/listener']


def test_topic_endpoints_returns_empty_for_unknown_topic():
    snap = _state('a', 1, topics=[_topic('/foo')])
    pubs, subs = topic_endpoints([snap], '/missing')
    assert pubs == []
    assert subs == []


def test_topics_of_node_collects_pub_and_sub_topics():
    pub = _endpoint('/talker')
    sub = _endpoint('/talker')  # same node also subscribes elsewhere
    snap = _state('a', 1, topics=[
        _topic('/foo', pubs=[pub], type_name='std_msgs/msg/String'),
        _topic('/bar', subs=[sub]),
    ])
    pubs, subs = topics_of_node([snap], '/talker')
    assert pubs == [{'topic_name': '/foo', 'type_name': 'std_msgs/msg/String'}]
    assert subs == [{'topic_name': '/bar', 'type_name': ''}]


def test_is_stale_detects_old_timestamp():
    msg = _state('a', 1, ts_sec=100)
    assert is_stale(msg, now_sec=200, stale_after_sec=10) is True
    assert is_stale(msg, now_sec=105, stale_after_sec=10) is False


def test_filter_fresh_drops_old_snapshots():
    fresh = _state('a', 1, ts_sec=int(time.time()))
    old = _state('b', 2, ts_sec=int(time.time()) - 999)
    kept = filter_fresh([fresh, old], stale_after_sec=10)
    assert kept == [fresh]
