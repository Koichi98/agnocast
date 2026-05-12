import subprocess
import sys

from ros2topic.verb import VerbExtension


class EchoAgnocastVerb(VerbExtension):
    """Dump agnocast messages as YAML using a generic introspection walker.

    The consumer side uses agnocast::GenericSubscription with no per-type plugin: the
    message type is supplied at the command line and resolved at runtime via
    rosidl_typesupport_introspection_cpp dlopen.
    """

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            'topic_name',
            help='Name of the agnocast topic to dump (e.g. /imu).')
        parser.add_argument(
            '--type', required=True, dest='type_name',
            help='Message type name, e.g. sensor_msgs/msg/Imu.')

    def main(self, *, args):
        # See HzAgnocastVerb for the rationale behind subprocess vs execvp.
        argv = ['ros2', 'run', 'agnocast_stats', 'agnocast_stats_runner',
                '--topic', args.topic_name,
                '--mode', 'echo',
                '--type', args.type_name]
        try:
            proc = subprocess.run(argv, check=False)
            return proc.returncode
        except KeyboardInterrupt:
            return 130
        except FileNotFoundError as exc:
            print(f'failed to launch agnocast_stats_runner: {exc}', file=sys.stderr)
            return 1
