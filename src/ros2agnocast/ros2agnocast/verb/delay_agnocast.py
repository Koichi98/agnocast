import subprocess
import sys

from ros2topic.verb import VerbExtension


class DelayAgnocastVerb(VerbExtension):
    """Print the wall-clock delay between header.stamp and reception of an agnocast topic.

    The consumer side uses agnocast::GenericSubscription with runtime-resolved field offsets
    (rosidl_typesupport_introspection_cpp dlopen), so no per-type plugin needs to be built
    for the topic's message type.
    """

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            'topic_name',
            help='Name of the agnocast topic to monitor (e.g. /imu).')
        parser.add_argument(
            '--type', required=True, dest='type_name',
            help='Message type name, e.g. sensor_msgs/msg/Imu. '
                 'Required because hz/delay tools need to resolve header.stamp offsets.')
        parser.add_argument(
            '--window', type=float, default=5.0,
            help='Sliding window size in seconds for the statistics (default: 5.0).')

    def main(self, *, args):
        # See HzAgnocastVerb for the rationale behind subprocess vs execvp.
        argv = ['ros2', 'run', 'agnocast_stats', 'agnocast_stats_runner',
                '--topic', args.topic_name,
                '--mode', 'delay',
                '--type', args.type_name,
                '--window', str(args.window)]
        try:
            proc = subprocess.run(argv, check=False)
            return proc.returncode
        except KeyboardInterrupt:
            return 130
        except FileNotFoundError as exc:
            print(f'failed to launch agnocast_stats_runner: {exc}', file=sys.stderr)
            return 1
