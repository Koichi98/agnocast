import subprocess
import sys

from ros2topic.verb import VerbExtension


class HzAgnocastVerb(VerbExtension):
    """Print rate at which an agnocast topic is published.

    Subscribes via agnocast::GenericSubscription (no per-type plugin required at the consumer
    side) and counts arrivals. Works for any agnocast publisher reachable on the current
    IPC namespace.
    """

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            'topic_name',
            help='Name of the agnocast topic to monitor (e.g. /my_topic).')
        parser.add_argument(
            '--window', type=float, default=5.0,
            help='Sliding window size in seconds for the statistics (default: 5.0).')

    def main(self, *, args):
        # We spawn via subprocess (fork+exec) rather than execvp. The agnocast heaphook
        # (LD_PRELOAD) issues AGNOCAST_ADD_PROCESS_CMD on load, which the kmod rejects when
        # called twice for the same PID; execvp keeps the PID and trips that path.
        argv = ['ros2', 'run', 'agnocast_stats', 'agnocast_stats_runner',
                '--topic', args.topic_name,
                '--mode', 'hz',
                '--window', str(args.window)]
        try:
            proc = subprocess.run(argv, check=False)
            return proc.returncode
        except KeyboardInterrupt:
            return 130
        except FileNotFoundError as exc:
            print(f'failed to launch agnocast_stats_runner: {exc}', file=sys.stderr)
            return 1
