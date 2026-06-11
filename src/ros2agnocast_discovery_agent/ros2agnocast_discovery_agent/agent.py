"""Per-IPC-namespace discovery agent.

Reads the local Agnocast state via the existing NS-scoped ioctl wrapper
(libagnocast_ioctl_wrapper.so) and publishes it as AgnocastDaemonState on
``/_agnocast_discovery`` so other namespaces and ECUs running ros2agnocast
tooling can observe and make bridge generation decisions.

One daemon process is intended to run per IPC namespace. To make duplicate
launches (e.g. one node spawning the agent and another ``ros2 launch``
session also trying to spawn one) safe, the agent acquires an
``flock(2)``-based singleton lock on a per-IPC-namespace file before
starting; subsequent instances detect the held lock and exit cleanly
(code 0), leaving exactly one live agent per namespace. Emitting a single
agent per launch tree is the launch side's responsibility; this lock is
the cross-invocation safety net for when separate launches target the
same namespace.
"""

import ctypes
from dataclasses import dataclass
from enum import Enum
import fcntl
import importlib.metadata
import logging
import os
import socket
import sys
from typing import IO
import uuid

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, LivelinessPolicy, QoSProfile, ReliabilityPolicy
from rclpy.duration import Duration

from ros2agnocast_discovery_msgs.msg import (
    AgnocastDaemonState,
    AgnocastEndpoint,
    AgnocastTopic,
)

from .type_registry import TypeRegistryReader


GOSSIP_TOPIC = '/_agnocast_discovery'
SCHEMA_VERSION = 1
PUBLISH_INTERVAL_SEC = 1.0
LIVELINESS_LEASE_SEC = 30.0
# Preferred over /etc/machine-id: avoids the systemd dep and the
# baked-into-image case where every ECU collides on one host_uuid.
BOOT_ID_PATH = '/proc/sys/kernel/random/boot_id'
SELF_IPC_NS_PATH = '/proc/self/ns/ipc'
# Kept separate from TOPIC_NAME_BUFFER_SIZE because the C side (`agnocast_kmod`)
# defines NODE_NAME_BUFFER_SIZE and TOPIC_NAME_BUFFER_SIZE as independent
# constants — they happen to share the value 256 today.
NODE_NAME_BUFFER_SIZE = 256
TOPIC_NAME_BUFFER_SIZE = 256
# How long a remote daemon's last snapshot may sit in _remote_states without
# being refreshed before we drop it. Matches the gossip Liveliness lease so
# DDS-side liveliness loss and local prune happen on the same timescale.
REMOTE_STATE_STALE_SEC = 30.0


class SnapshotTopicRet(ctypes.Structure):
    """Mirror of ``struct snapshot_topic_ret`` in agnocast.h / agnocast_ioctl.hpp."""

    _fields_ = [
        ('topic_name', ctypes.c_char * TOPIC_NAME_BUFFER_SIZE),
        ('publisher_num', ctypes.c_uint32),
        ('subscriber_num', ctypes.c_uint32),
        ('ros2_publisher_num', ctypes.c_uint32),
        ('ros2_subscriber_num', ctypes.c_uint32),
    ]


class SnapshotEndpointRet(ctypes.Structure):
    """Mirror of ``struct snapshot_endpoint_ret``. ``pid`` is pid_t (== int32)."""

    _fields_ = [
        ('node_name', ctypes.c_char * NODE_NAME_BUFFER_SIZE),
        ('pid', ctypes.c_int32),
        ('qos_depth', ctypes.c_uint32),
        ('qos_is_transient_local', ctypes.c_bool),
        ('qos_is_reliable', ctypes.c_bool),
        ('is_bridge', ctypes.c_bool),
    ]


def _load_ioctl_wrapper():
    """Load libagnocast_ioctl_wrapper.so and set argtypes for the symbols we use."""
    lib = ctypes.CDLL('libagnocast_ioctl_wrapper.so')

    lib.get_agnocast_discovery_snapshot.argtypes = [
        ctypes.POINTER(ctypes.POINTER(SnapshotTopicRet)), ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.POINTER(SnapshotEndpointRet)), ctypes.POINTER(ctypes.c_int),
    ]
    lib.get_agnocast_discovery_snapshot.restype = ctypes.c_int
    lib.free_agnocast_discovery_snapshot.argtypes = [
        ctypes.POINTER(SnapshotTopicRet), ctypes.POINTER(SnapshotEndpointRet)]
    lib.free_agnocast_discovery_snapshot.restype = None

    return lib


def _snapshot_to_endpoint(ep_ret: SnapshotEndpointRet) -> AgnocastEndpoint:
    """Convert one ``snapshot_endpoint_ret`` row to an AgnocastEndpoint msg.

    Unlike the old per-topic ioctl, the snapshot carries ``pid`` straight from
    the kmod (the registered process), so no tmpfs registry join is needed for
    it. The registry is still consulted for ``type_name`` (see
    ``_resolve_topic_type``), which the kmod does not track.
    """
    ep = AgnocastEndpoint()
    ep.node_name = ep_ret.node_name.decode('utf-8', errors='replace')
    ep.pid = ep_ret.pid
    ep.qos_depth = ep_ret.qos_depth
    ep.qos_is_transient_local = ep_ret.qos_is_transient_local
    ep.qos_is_reliable = ep_ret.qos_is_reliable
    ep.is_bridge = ep_ret.is_bridge
    return ep


def read_local_topics(lib, registry: TypeRegistryReader | None = None) -> list:
    """Snapshot the current namespace's Agnocast topics via the ioctl wrapper.

    Returns a list of AgnocastTopic msgs. A single ``get_agnocast_discovery_snapshot``
    call returns every topic plus its pub/sub endpoints for the caller's IPC
    namespace; the endpoint array is laid out in topic order as "publishers then
    subscribers", sliced via publisher_num / subscriber_num. The optional
    ``registry`` argument supplies the type names that the ioctl does not expose.
    """
    topics_ptr = ctypes.POINTER(SnapshotTopicRet)()
    topic_count = ctypes.c_int()
    endpoints_ptr = ctypes.POINTER(SnapshotEndpointRet)()
    endpoint_count = ctypes.c_int()
    rc = lib.get_agnocast_discovery_snapshot(
        ctypes.byref(topics_ptr), ctypes.byref(topic_count),
        ctypes.byref(endpoints_ptr), ctypes.byref(endpoint_count))

    topics = []
    if rc != 0 or topic_count.value == 0:
        return topics

    try:
        cursor = 0
        for i in range(topic_count.value):
            topic_ret = topics_ptr[i]

            agnocast_topic = AgnocastTopic()
            agnocast_topic.topic_name = topic_ret.topic_name.decode('utf-8', errors='replace')
            agnocast_topic.type_name = ''
            agnocast_topic.domain_id = 0
            agnocast_topic.publishers = [
                _snapshot_to_endpoint(endpoints_ptr[cursor + j])
                for j in range(topic_ret.publisher_num)]
            cursor += topic_ret.publisher_num
            agnocast_topic.subscribers = [
                _snapshot_to_endpoint(endpoints_ptr[cursor + j])
                for j in range(topic_ret.subscriber_num)]
            cursor += topic_ret.subscriber_num
            # Type name comes from the tmpfs registry; any registered
            # endpoint on this topic carries the same type (ROS 2
            # invariant), so the first non-empty one wins.
            if registry is not None:
                resolved = _resolve_topic_type(agnocast_topic, registry)
                if resolved:
                    agnocast_topic.type_name = resolved
            topics.append(agnocast_topic)
    finally:
        lib.free_agnocast_discovery_snapshot(topics_ptr, endpoints_ptr)

    return topics


def _resolve_topic_type(
        agnocast_topic: AgnocastTopic, registry: TypeRegistryReader) -> str:
    for ep in agnocast_topic.publishers:
        entry = registry.lookup(agnocast_topic.topic_name, 'pub', ep.node_name)
        if entry is not None and entry.type_name:
            return entry.type_name
    for ep in agnocast_topic.subscribers:
        entry = registry.lookup(agnocast_topic.topic_name, 'sub', ep.node_name)
        if entry is not None and entry.type_name:
            return entry.type_name
    return ''


def _read_host_uuid() -> str:
    """Return the boot UUID; fall back to a random UUID with a WARN log."""
    try:
        with open(BOOT_ID_PATH) as fp:
            return str(uuid.UUID(fp.read().strip()))
    except (OSError, ValueError) as exc:
        fallback = str(uuid.uuid4())
        logging.warning(
            'agnocast_discovery_agent: failed to read %s (%s); '
            'falling back to random host_uuid=%s',
            BOOT_ID_PATH, exc, fallback)
        return fallback


def _read_ipc_ns_inode() -> int:
    """Return the inode number of the daemon's own IPC namespace."""
    return os.stat(SELF_IPC_NS_PATH).st_ino


def _singleton_lock_path(ipc_ns_inode: int) -> str:
    """Return the singleton lock-file path for this IPC namespace.

    Co-located with the rest of Agnocast tmpfs state so a container
    overriding ``AGNOCAST_TMPFS_DIR`` keeps the lock under the same root.
    """
    root = os.environ.get('AGNOCAST_TMPFS_DIR') or '/dev/shm'
    return os.path.join(root, f'agnocast_discovery_agent_{ipc_ns_inode}.lock')


class LockStatus(Enum):
    ACQUIRED = 'acquired'   # we got the lock
    HELD = 'held'           # another agent in this NS holds it — caller should idle
    ERROR = 'error'         # could not even open the lock file — caller should exit non-zero


@dataclass
class SingletonLockAttempt:
    """Result of trying to acquire the per-NS singleton lock.

    ``file`` is set only when ``status == ACQUIRED`` — the caller keeps the
    reference alive so the ``flock(2)`` outlives the spin loop.
    """
    status: LockStatus
    file: IO | None = None


def _try_acquire_singleton_lock(ipc_ns_inode: int) -> SingletonLockAttempt:
    """Try to take an exclusive ``flock(2)`` on the per-IPC-namespace lock file."""
    lock_path = _singleton_lock_path(ipc_ns_inode)
    try:
        fd = os.open(lock_path, os.O_RDWR | os.O_CREAT | os.O_CLOEXEC, 0o644)
    except OSError as e:
        sys.stderr.write(
            f'agnocast_discovery_agent: cannot open singleton lock {lock_path}: {e}\n')
        return SingletonLockAttempt(LockStatus.ERROR)
    lock_file = os.fdopen(fd, 'r+')
    try:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        # Contention: another agent in this NS holds the lock.
        lock_file.close()
        return SingletonLockAttempt(LockStatus.HELD)
    except OSError as e:
        # Real failure (permission denied, ENOTSUP on the FS, etc.) — surface
        # it rather than masking it as "already running".
        sys.stderr.write(
            f'agnocast_discovery_agent: flock({lock_path}) failed: {e}\n')
        lock_file.close()
        return SingletonLockAttempt(LockStatus.ERROR)
    return SingletonLockAttempt(LockStatus.ACQUIRED, file=lock_file)


def _gossip_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        # AUTOMATIC: rclpy (Humble) does not expose ``assert_liveliness`` on
        # Publisher, so MANUAL_BY_TOPIC cannot be driven reliably from Python.
        # AUTOMATIC keeps the publisher alive as long as the participant is up.
        liveliness=LivelinessPolicy.AUTOMATIC,
        liveliness_lease_duration=Duration(seconds=LIVELINESS_LEASE_SEC),
    )


def _read_agent_version() -> str:
    """Return this package's installed version, or '' if running from source."""
    try:
        return importlib.metadata.version('ros2agnocast_discovery_agent')
    except importlib.metadata.PackageNotFoundError:
        return ''


class DiscoveryAgent(Node):
    """rclpy Node that publishes the local Agnocast state every PUBLISH_INTERVAL_SEC.

    Also subscribes to its own gossip topic and caches the latest snapshot per
    ``(host_uuid, ipc_ns_inode)`` — exposed through ``remote_states`` for
    future consumers (e.g. cross-NS decision logic added in a follow-up PR).
    """

    def __init__(self, registry: TypeRegistryReader | None = None):
        self._host_uuid = _read_host_uuid()
        self._host_hostname = socket.gethostname()
        self._ipc_ns_inode = _read_ipc_ns_inode()
        # Multiple agents may run on one ECU (one per IPC NS); disambiguate.
        host_short = self._host_uuid.replace('-', '')[:8]
        # Force use_sim_time=False so the 1 Hz timer + clock reads are
        # wall-clock regardless of /clock; sim-time playback would otherwise
        # stretch the publish cadence past the liveliness lease window.
        super().__init__(
            f'agnocast_discovery_agent_{host_short}_{self._ipc_ns_inode}',
            parameter_overrides=[Parameter('use_sim_time', value=False)],
        )
        self._lib = _load_ioctl_wrapper()
        self._agnocast_version = _read_agent_version()
        # Independent wall-clock for prune / receive timestamps so they stay
        # consistent even if a future change accidentally enables sim time.
        self._clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self._registry = registry if registry is not None else TypeRegistryReader(
            self._ipc_ns_inode, logger=self.get_logger())

        qos = _gossip_qos()
        self._pub = self.create_publisher(AgnocastDaemonState, GOSSIP_TOPIC, qos)
        self._sub = self.create_subscription(
            AgnocastDaemonState, GOSSIP_TOPIC, self._on_remote_state, qos)
        self._remote_states = {}

        self._timer = self.create_timer(PUBLISH_INTERVAL_SEC, self._on_tick)

        self.get_logger().info(
            f'agnocast_discovery_agent up: host_uuid={self._host_uuid} '
            f'hostname={self._host_hostname} ipc_ns_inode={self._ipc_ns_inode} '
            f'version={self._agnocast_version}')

    def _on_tick(self) -> None:
        self._registry.rebuild()
        self._registry.cleanup_dead_pids()
        self._prune_stale_remote_states()
        self.publish_snapshot()

    def _prune_stale_remote_states(
            self, now_sec: float | None = None,
            stale_after_sec: float = REMOTE_STATE_STALE_SEC) -> None:
        """Drop ``_remote_states`` entries whose local arrival time is too old.

        Mirrors the DDS Liveliness lease so dead-peer snapshots don't leak
        memory. Cross-ECU clocks aren't synced, so local arrival time is
        the freshness reference rather than the publisher's stamp.
        """
        if now_sec is None:
            now_sec = self._clock.now().nanoseconds / 1e9
        stale_keys = [
            key for key, (_msg, received_at) in self._remote_states.items()
            if now_sec - received_at > stale_after_sec
        ]
        for key in stale_keys:
            del self._remote_states[key]

    def publish_snapshot(self) -> None:
        """Build and publish the current local AgnocastDaemonState."""
        msg = self.build_state()
        self._pub.publish(msg)

    def build_state(self) -> AgnocastDaemonState:
        msg = AgnocastDaemonState()
        msg.schema_version = SCHEMA_VERSION
        msg.agnocast_version = self._agnocast_version
        msg.host_uuid = self._host_uuid
        msg.host_hostname = self._host_hostname
        msg.ipc_ns_inode = self._ipc_ns_inode
        msg.topics = read_local_topics(self._lib, self._registry)
        return msg

    def _on_remote_state(self, msg: AgnocastDaemonState) -> None:
        if msg.host_uuid == self._host_uuid and msg.ipc_ns_inode == self._ipc_ns_inode:
            return
        received_at = self._clock.now().nanoseconds / 1e9
        self._remote_states[(msg.host_uuid, msg.ipc_ns_inode)] = (msg, received_at)

    @property
    def remote_states(self) -> dict:
        """Map of ``(host_uuid, ipc_ns_inode)`` to ``(msg, received_at_sec)``."""
        return self._remote_states


def main(argv=None) -> int:
    # Take the per-IPC-namespace singleton lock before any DDS / ioctl work.
    # If another agent holds it, exit cleanly (0).
    #
    # A duplicate that exits early *could* make launch_test treat it as a
    # crashed node and tear down the launch tree. That's not a risk here: the
    # launch emits one agent per tree, so a held lock always means a separate
    # launch/run — never a same-tree sibling — which exiting can't affect.
    ipc_ns_inode = _read_ipc_ns_inode()
    lock_attempt = _try_acquire_singleton_lock(ipc_ns_inode)
    if lock_attempt.status == LockStatus.HELD:
        sys.stderr.write(
            'agnocast_discovery_agent: another instance is already running in this '
            f'IPC namespace (inode={ipc_ns_inode}); exiting.\n')
        return 0
    if lock_attempt.status == LockStatus.ERROR:
        # Don't masquerade as "already running" — propagate failure so
        # supervisors can detect it.
        return 1

    rclpy.init(args=argv)
    node = DiscoveryAgent()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        # Keep `lock_attempt` referenced through shutdown so the flock
        # outlives the spin() loop.
        del lock_attempt
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
