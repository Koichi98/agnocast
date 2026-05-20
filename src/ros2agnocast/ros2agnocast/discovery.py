"""Shared helper for subscribing to /_agnocast_discovery and aggregating gossip.

CLI verbs use this to collect AgnocastDaemonState snapshots from every
per-IPC-NS daemon reachable on the same ROS_DOMAIN_ID and to fold the
combined view into the existing local-ioctl / DDS merge.
"""

import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, HistoryPolicy, LivelinessPolicy, QoSProfile, ReliabilityPolicy

from ros2agnocast_discovery_msgs.msg import AgnocastDaemonState


GOSSIP_TOPIC = '/_agnocast_discovery'
DEFAULT_COLLECT_TIMEOUT_SEC = 2.0
DEFAULT_STALE_AFTER_SEC = 10.0
LIVELINESS_LEASE_SEC = 30.0


def gossip_qos() -> QoSProfile:
    """QoS profile matching the discovery agent's publisher.

    Subscribers must match these settings to receive the late-joiner snapshot
    via TransientLocal.
    """
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=64,
        liveliness=LivelinessPolicy.AUTOMATIC,
        liveliness_lease_duration=Duration(seconds=LIVELINESS_LEASE_SEC),
    )


def collect_announcements(
    node,
    timeout_sec: float = DEFAULT_COLLECT_TIMEOUT_SEC,
) -> list:
    """Subscribe to the gossip topic and collect snapshots for `timeout_sec`.

    Returns a list of AgnocastDaemonState messages, one (latest) per (host_uuid,
    ipc_ns_inode) pair seen during the window.
    """
    snapshots = {}

    def on_msg(msg: AgnocastDaemonState) -> None:
        snapshots[(msg.host_uuid, msg.ipc_ns_inode)] = msg

    sub = node.create_subscription(
        AgnocastDaemonState, GOSSIP_TOPIC, on_msg, gossip_qos())
    try:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_subscription(sub)

    return list(snapshots.values())


def warn_if_no_announcements(snapshots: list, timeout_sec: float) -> None:
    """Print a stderr hint when zero gossip snapshots were collected.

    A non-zero timeout that yielded no AgnocastDaemonState messages usually
    means the per-IPC-namespace discovery agent is not running in this IPC
    namespace, so the CLI sees only its own local ioctl view. This warning
    is best-effort and does not change exit codes.
    """
    if timeout_sec <= 0:
        return
    if snapshots:
        return
    print(
        'WARNING: no /_agnocast_discovery announcements received within '
        f'{timeout_sec:.1f}s — cross-namespace / cross-ECU Agnocast endpoints '
        "will not appear. Start the discovery agent in this IPC namespace via "
        '`ros2 run ros2agnocast_discovery_agent discovery_agent`, or pass '
        '`--gossip-timeout 0` to skip this check.',
        file=sys.stderr)


def is_stale(msg: AgnocastDaemonState, now_sec: float,
             stale_after_sec: float = DEFAULT_STALE_AFTER_SEC) -> bool:
    """Return True if the snapshot is older than `stale_after_sec` seconds."""
    msg_sec = msg.timestamp.sec + msg.timestamp.nanosec * 1e-9
    return (now_sec - msg_sec) > stale_after_sec


def filter_fresh(snapshots: list,
                 stale_after_sec: float = DEFAULT_STALE_AFTER_SEC) -> list:
    """Drop snapshots older than `stale_after_sec` seconds (best-effort)."""
    now_sec = time.time()
    return [m for m in snapshots if not is_stale(m, now_sec, stale_after_sec)]


def all_topic_names(snapshots: list) -> set:
    """Union of topic names across all snapshots."""
    return {topic.topic_name for snap in snapshots for topic in snap.topics}


def all_nodes(snapshots: list) -> set:
    """Union of (node_name) seen as a publisher or subscriber across all snapshots.

    Node identity is just name here because the CLI's existing presentation is
    name-based; pid is best-effort empty in the gossip schema.
    """
    nodes = set()
    for snap in snapshots:
        for topic in snap.topics:
            for endpoint in topic.publishers:
                nodes.add(endpoint.node_name)
            for endpoint in topic.subscribers:
                nodes.add(endpoint.node_name)
    return nodes


def topic_endpoints(snapshots: list, topic_name: str) -> tuple:
    """Return (publishers, subscribers) across all snapshots for `topic_name`.

    Each element is an AgnocastEndpoint message. The same (node_name) appearing
    in multiple snapshots is preserved (the caller is responsible for dedup).
    """
    publishers = []
    subscribers = []
    for snap in snapshots:
        for topic in snap.topics:
            if topic.topic_name != topic_name:
                continue
            publishers.extend(topic.publishers)
            subscribers.extend(topic.subscribers)
    return publishers, subscribers


def topics_of_node(snapshots: list, node_name: str) -> tuple:
    """Return (publish_topics, subscribe_topics) for `node_name`.

    Each list contains AgnocastTopic-like dicts with topic_name and type_name.
    """
    pubs, subs = [], []
    for snap in snapshots:
        for topic in snap.topics:
            for endpoint in topic.publishers:
                if endpoint.node_name == node_name:
                    pubs.append({'topic_name': topic.topic_name,
                                 'type_name': topic.type_name})
            for endpoint in topic.subscribers:
                if endpoint.node_name == node_name:
                    subs.append({'topic_name': topic.topic_name,
                                 'type_name': topic.type_name})
    return pubs, subs
