"""Unit tests for the discovery agent's ioctl-to-msg conversion and gossip wiring.

These tests do not require the kmod or DDS; they exercise the conversion
helpers directly with a mock ctypes library.
"""

import threading
from unittest.mock import MagicMock

from ros2agnocast_discovery_agent.agent import (
    SnapshotEndpointRet,
    SnapshotTopicRet,
    _read_host_uuid,
    _snapshot_to_endpoint,
    read_local_topics,
)


def _make_endpoint(node_name: str, pid: int = 0, qos_depth: int = 10,
                   qos_is_transient_local: bool = False,
                   qos_is_reliable: bool = True,
                   is_bridge: bool = False) -> SnapshotEndpointRet:
    ep = SnapshotEndpointRet()
    # Assigning bytes shorter than the array pads the remainder with NUL.
    ep.node_name = node_name.encode('utf-8')
    ep.pid = pid
    ep.qos_depth = qos_depth
    ep.qos_is_transient_local = qos_is_transient_local
    ep.qos_is_reliable = qos_is_reliable
    ep.is_bridge = is_bridge
    return ep


def test_snapshot_to_endpoint_copies_all_fields():
    ep_ret = _make_endpoint(
        '/talker_node',
        pid=4242,
        qos_depth=7,
        qos_is_transient_local=True,
        qos_is_reliable=False,
        is_bridge=True,
    )
    ep = _snapshot_to_endpoint(ep_ret)
    assert ep.node_name == '/talker_node'
    # pid comes straight from the kmod snapshot (no registry join).
    assert ep.pid == 4242
    assert ep.qos_depth == 7
    assert ep.qos_is_transient_local is True
    assert ep.qos_is_reliable is False
    assert ep.is_bridge is True


def test_snapshot_to_endpoint_handles_short_name():
    ep = _snapshot_to_endpoint(_make_endpoint('/x'))
    assert ep.node_name == '/x'


def _make_mock_lib(topic_to_endpoints: dict) -> MagicMock:
    """Build a ctypes-flavoured mock of ``get_agnocast_discovery_snapshot``.

    ``topic_to_endpoints`` maps topic name -> {'pub': [SnapshotEndpointRet], 'sub': [...]}.
    The endpoint array is laid out per topic as publishers-then-subscribers, matching
    the kmod contract that ``read_local_topics`` slices via publisher_num/subscriber_num.
    """
    lib = MagicMock()

    topic_structs = []
    endpoints = []
    for name, ends in topic_to_endpoints.items():
        pubs = ends.get('pub', [])
        subs = ends.get('sub', [])
        ts = SnapshotTopicRet()
        ts.topic_name = name.encode('utf-8')
        ts.publisher_num = len(pubs)
        ts.subscriber_num = len(subs)
        ts.ros2_publisher_num = 0
        ts.ros2_subscriber_num = 0
        topic_structs.append(ts)
        endpoints.extend(pubs)
        endpoints.extend(subs)

    topics_arr = (SnapshotTopicRet * len(topic_structs))(*topic_structs) if topic_structs else None
    eps_arr = (SnapshotEndpointRet * len(endpoints))(*endpoints) if endpoints else None
    # Keep the backing arrays alive for the lifetime of the mock so the pointers stay valid.
    lib._keepalive = (topics_arr, eps_arr)

    def snapshot(topics_pp, topic_count_p, eps_pp, ep_count_p):
        topic_count_p._obj.value = len(topic_structs)
        ep_count_p._obj.value = len(endpoints)
        if topics_arr is not None:
            topics_pp._obj.contents = topics_arr[0]
        if eps_arr is not None:
            eps_pp._obj.contents = eps_arr[0]
        return 0

    lib.get_agnocast_discovery_snapshot = MagicMock(side_effect=snapshot)
    lib.free_agnocast_discovery_snapshot = MagicMock()

    return lib


def test_read_local_topics_combines_pub_and_sub():
    pub = _make_endpoint('/talker_node', pid=11, qos_depth=3)
    sub = _make_endpoint('/listener_node', pid=22, qos_depth=5)

    lib = _make_mock_lib({
        '/chatter': {
            'pub': [pub],
            'sub': [sub],
        },
    })

    topics = read_local_topics(lib)
    assert len(topics) == 1
    topic = topics[0]
    assert topic.topic_name == '/chatter'
    # No registry passed => type stays empty.
    assert topic.type_name == ''
    assert topic.domain_id == 0
    assert len(topic.publishers) == 1
    assert topic.publishers[0].node_name == '/talker_node'
    assert topic.publishers[0].qos_depth == 3
    assert topic.publishers[0].pid == 11
    assert len(topic.subscribers) == 1
    assert topic.subscribers[0].node_name == '/listener_node'
    assert topic.subscribers[0].pid == 22


def test_read_local_topics_resolves_type_from_registry():
    """A registry entry for any endpoint on the topic populates `type_name`.

    pid no longer comes from the registry: it is carried by the snapshot itself.
    """
    from ros2agnocast_discovery_agent.type_registry import RegistryEntry

    pub = _make_endpoint('/talker_node', pid=99)
    lib = _make_mock_lib({'/chatter': {'pub': [pub], 'sub': []}})

    class FakeRegistry:
        def lookup(self, topic, role, node):
            if (topic, role, node) == ('/chatter', 'pub', '/talker_node'):
                return RegistryEntry(pid=99, type_name='std_msgs/msg/Int32')
            return None

    topics = read_local_topics(lib, FakeRegistry())
    assert len(topics) == 1
    assert topics[0].type_name == 'std_msgs/msg/Int32'
    assert topics[0].publishers[0].pid == 99


def test_read_local_topics_falls_back_to_subscriber_type_when_pub_missing():
    """If the registry only knows the subscriber side, `type_name` still resolves.

    Mirrors the case where the publisher process exited (or was on another NS
    and its registry file isn't local) but a local subscriber is still active.
    """
    from ros2agnocast_discovery_agent.type_registry import RegistryEntry

    pub = _make_endpoint('/talker_node', pid=11)
    sub = _make_endpoint('/listener_node', pid=22)
    lib = _make_mock_lib({'/chatter': {'pub': [pub], 'sub': [sub]}})

    class SubOnlyRegistry:
        def lookup(self, topic, role, node):
            if (topic, role, node) == ('/chatter', 'sub', '/listener_node'):
                return RegistryEntry(pid=22, type_name='std_msgs/msg/Int32')
            return None

    topics = read_local_topics(lib, SubOnlyRegistry())
    assert len(topics) == 1
    assert topics[0].type_name == 'std_msgs/msg/Int32'
    # pids come from the snapshot regardless of registry coverage.
    assert topics[0].subscribers[0].pid == 22
    assert topics[0].publishers[0].pid == 11


def test_read_local_topics_returns_empty_when_no_topics():
    lib = _make_mock_lib({})
    assert read_local_topics(lib) == []


def test_read_local_topics_handles_topic_without_subscribers():
    pub = _make_endpoint('/orphan_pub_node')
    lib = _make_mock_lib({
        '/orphan_topic': {'pub': [pub], 'sub': []},
    })
    topics = read_local_topics(lib)
    assert len(topics) == 1
    assert topics[0].topic_name == '/orphan_topic'
    assert len(topics[0].publishers) == 1
    assert topics[0].subscribers == []


def test_read_host_uuid_returns_uuid_string():
    host_uuid = _read_host_uuid()
    assert isinstance(host_uuid, str)
    assert len(host_uuid) > 0
    # Either a valid UUID from /proc/sys/kernel/random/boot_id or a random
    # fallback; both should parse as UUIDs.
    import uuid
    uuid.UUID(host_uuid)


# ---------------------------------------------------------------------------
# Singleton lock
# ---------------------------------------------------------------------------


def test_singleton_lock_path_honors_tmpfs_dir(monkeypatch, tmp_path):
    monkeypatch.setenv('AGNOCAST_TMPFS_DIR', str(tmp_path))
    from ros2agnocast_discovery_agent.agent import _singleton_lock_path
    assert _singleton_lock_path(42) == str(tmp_path / 'agnocast_discovery_agent_42.lock')


def test_singleton_lock_path_defaults_to_dev_shm(monkeypatch):
    monkeypatch.delenv('AGNOCAST_TMPFS_DIR', raising=False)
    from ros2agnocast_discovery_agent.agent import _singleton_lock_path
    assert _singleton_lock_path(42) == '/dev/shm/agnocast_discovery_agent_42.lock'


def test_acquire_singleton_lock_succeeds_when_free(monkeypatch, tmp_path):
    monkeypatch.setenv('AGNOCAST_TMPFS_DIR', str(tmp_path))
    from ros2agnocast_discovery_agent.agent import LockStatus, _try_acquire_singleton_lock
    attempt = _try_acquire_singleton_lock(123)
    assert attempt.status == LockStatus.ACQUIRED
    attempt.file.close()


def test_acquire_singleton_lock_blocks_second_attempt(monkeypatch, tmp_path):
    """A second acquire in the same process reports HELD while the first is alive."""
    monkeypatch.setenv('AGNOCAST_TMPFS_DIR', str(tmp_path))
    from ros2agnocast_discovery_agent.agent import LockStatus, _try_acquire_singleton_lock
    first = _try_acquire_singleton_lock(456)
    assert first.status == LockStatus.ACQUIRED
    assert _try_acquire_singleton_lock(456).status == LockStatus.HELD
    first.file.close()
    # After releasing, a new acquire succeeds.
    third = _try_acquire_singleton_lock(456)
    assert third.status == LockStatus.ACQUIRED
    third.file.close()


def test_acquire_singleton_lock_independent_per_ipc_ns(monkeypatch, tmp_path):
    """Different IPC NS inodes get independent locks."""
    monkeypatch.setenv('AGNOCAST_TMPFS_DIR', str(tmp_path))
    from ros2agnocast_discovery_agent.agent import LockStatus, _try_acquire_singleton_lock
    lock_a = _try_acquire_singleton_lock(111)
    lock_b = _try_acquire_singleton_lock(222)
    assert lock_a.status == LockStatus.ACQUIRED
    assert lock_b.status == LockStatus.ACQUIRED
    lock_a.file.close()
    lock_b.file.close()


def test_acquire_singleton_lock_reports_error_on_unwritable_dir(monkeypatch):
    """When the lock-file directory is not writable we report ERROR (distinct
    from HELD) so the caller can surface a non-zero exit code."""
    monkeypatch.setenv('AGNOCAST_TMPFS_DIR', '/nonexistent_path_for_agnocast_test')
    from ros2agnocast_discovery_agent.agent import LockStatus, _try_acquire_singleton_lock
    assert _try_acquire_singleton_lock(789).status == LockStatus.ERROR


def test_main_exits_promptly_when_another_agent_holds_the_lock(monkeypatch, tmp_path):
    """A duplicate (lock already held) returns 0 promptly instead of idling.

    Runs main() in a thread so a regression to the old ``signal.pause()``
    (which would block forever here) is caught as a non-returning thread
    rather than hanging the whole test run. main() returns before any DDS /
    ioctl bring-up, so this needs neither the kmod nor rclpy.
    """
    monkeypatch.setenv('AGNOCAST_TMPFS_DIR', str(tmp_path))
    from ros2agnocast_discovery_agent.agent import (
        LockStatus, _read_ipc_ns_inode, _try_acquire_singleton_lock, main)

    holder = _try_acquire_singleton_lock(_read_ipc_ns_inode())
    assert holder.status == LockStatus.ACQUIRED
    try:
        result = {}
        t = threading.Thread(target=lambda: result.__setitem__('rc', main(argv=[])))
        t.start()
        t.join(timeout=5.0)
        assert not t.is_alive(), 'main() did not return (regressed to signal.pause()?)'
        assert result['rc'] == 0
    finally:
        holder.file.close()
