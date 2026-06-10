"""Long-lived daemon verb: gate A2R bridge activation on rosbag2 recorder presence.

`ros2 agnocast a2r-activate` runs forever. It watches the ROS 2 graph for an active
rosbag2 recorder (the one logpacker spawns while recording). While a recorder is present
it runs an :class:`A2rBridgeActivator` so that Agnocast publisher topics are bridged to
ROS 2 and become recordable; when the recorder disappears it tears the activator down so
no A2R bridge keeps copying SHM->DDS while nothing is recording.

This is the production counterpart of `ros2 bag record_agnocast`: record_agnocast couples
bridge activation to a single CLI recording session, whereas logpacker records via its own
rosbag2 recorder (not the `ros2 bag` CLI), so we gate on the recorder's lifecycle instead.

Topic scope is the same allowlist logpacker records (pass --topic-regex / --topic-list-file)
so we only bridge what will actually be recorded.
"""

import os
import re
import time

import rclpy
from ros2cli.verb import VerbExtension

from ros2agnocast._a2r_bridge_activator import A2rBridgeActivator

DEFAULT_POLL_INTERVAL = 2.0          # seconds between graph polls
DEFAULT_RECORDER_SERVICE_TYPE_PREFIX = 'rosbag2_interfaces/'


def _load_topic_regexes(args):
    """Build the list of compiled allowlist regexes from CLI args.

    --topic-regex may be repeated; --topic-list-file is a newline-separated file
    (used to share logpacker's topic_list verbatim). Returns None when no allowlist
    is given, meaning "activate every Agnocast topic".
    """
    patterns = list(args.topic_regex or [])
    if args.topic_list_file:
        with open(args.topic_list_file) as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#'):
                    patterns.append(line)
    if not patterns:
        return None
    compiled = []
    for p in patterns:
        try:
            compiled.append(re.compile(p))
        except re.error as e:
            print("WARN: [a2r-activate] ignoring invalid topic regex '%s': %s" % (p, e))
    return compiled or None


class A2rActivateVerb(VerbExtension):
    """Gate A2R bridge activation on rosbag2 recorder presence (long-lived daemon)."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            '--topic-regex', action='append', metavar='REGEX',
            help='Allowlist regex of topics to bridge. Repeatable. '
                 'Omit (and omit --topic-list-file) to bridge every Agnocast topic.')
        parser.add_argument(
            '--topic-list-file', metavar='PATH',
            help='File with one topic regex per line (share logpacker topic_list here).')
        parser.add_argument(
            '--poll-interval', type=float, default=DEFAULT_POLL_INTERVAL, metavar='SEC',
            help='Seconds between ROS 2 graph polls (default: %.1f).' % DEFAULT_POLL_INTERVAL)
        parser.add_argument(
            '--recorder-service-type-prefix', default=DEFAULT_RECORDER_SERVICE_TYPE_PREFIX,
            metavar='PREFIX',
            help="Detect the recorder by any advertised service whose type starts with this "
                 "prefix (default: '%s'). These services exist only while recording."
                 % DEFAULT_RECORDER_SERVICE_TYPE_PREFIX)
        parser.add_argument(
            '--recorder-node-regex', metavar='REGEX',
            help='Optional: also treat a graph node whose fully-qualified name matches this '
                 'regex as "recording active". Use when the recorder does not advertise '
                 'rosbag2_interfaces services.')
        parser.add_argument(
            '--always-on', action='store_true',
            help='Skip recorder gating; keep bridges up unconditionally (wasteful while idle).')
        parser.add_argument(
            '--log-level', default='info',
            help='Log level passed to A2rBridgeActivator (default: info).')

    def main(self, *, args):
        topic_regexes = _load_topic_regexes(args)

        def should_activate(topic_name, type_name):
            if topic_regexes is None:
                return True
            return any(rx.search(topic_name) for rx in topic_regexes)

        if args.always_on:
            print('[a2r-activate] --always-on: activating bridges unconditionally.')
            with A2rBridgeActivator(log_level=args.log_level, should_activate=should_activate):
                self._sleep_forever(args.poll_interval)
            return 0

        recorder_node_regex = re.compile(args.recorder_node_regex) if args.recorder_node_regex else None
        ctx = rclpy.Context()
        rclpy.init(context=ctx)
        node = rclpy.create_node('_ros2agnocast_recorder_gate_%d' % os.getpid(), context=ctx)
        activator = None
        print('[a2r-activate] watching for rosbag2 recorder (service prefix=%r, node regex=%r)'
              % (args.recorder_service_type_prefix, args.recorder_node_regex))
        try:
            while rclpy.ok(context=ctx):
                recording = self._recorder_present(
                    node, args.recorder_service_type_prefix, recorder_node_regex)
                if recording and activator is None:
                    print('[a2r-activate] recorder detected -> activating A2R bridges')
                    activator = A2rBridgeActivator(
                        log_level=args.log_level, should_activate=should_activate)
                    activator.start()
                elif not recording and activator is not None:
                    print('[a2r-activate] recorder gone -> deactivating A2R bridges')
                    activator.stop()
                    activator = None
                time.sleep(args.poll_interval)
        except KeyboardInterrupt:
            pass
        finally:
            if activator is not None:
                activator.stop()
            try:
                node.destroy_node()
            except Exception:
                pass
            rclpy.try_shutdown(context=ctx)
        return 0

    @staticmethod
    def _recorder_present(node, service_type_prefix, recorder_node_regex):
        """Return True if a rosbag2 recorder appears to be active in the ROS 2 graph."""
        if service_type_prefix:
            for _name, types in node.get_service_names_and_types():
                if any(t.startswith(service_type_prefix) for t in types):
                    return True
        if recorder_node_regex is not None:
            for name, namespace in node.get_node_names_and_namespaces():
                fq = (namespace.rstrip('/') + '/' + name) if namespace != '/' else '/' + name
                if recorder_node_regex.search(fq):
                    return True
        return False

    @staticmethod
    def _sleep_forever(interval):
        try:
            while True:
                time.sleep(interval)
        except KeyboardInterrupt:
            pass
