import os
import time
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts

# End-to-end 1-to-1 test for agnocast::Node (the pure-Agnocast node).
# One publisher node, driven by an Agnocast wall timer, publishes PUB_NUM
# messages to a single subscriber node. Both are standalone executables that
# spin an AgnocastOnlySingleThreadedExecutor.
#
# When STRESS_TEST_TIMEOUT is set, the nodes are launched with --forever and
# keep running under load; the soak is spent in setUpClass before the output
# assertions run (mirrors test_1to1.py).

TOPIC = '/agnocast_node_1to1'
PUB_NUM = 5
STRESS_TEST_TIMEOUT = os.environ.get('STRESS_TEST_TIMEOUT')
FOREVER = STRESS_TEST_TIMEOUT is not None


def generate_test_description():
    additional_env = {
        'LD_PRELOAD': f"libagnocast_heaphook.so:{os.getenv('LD_PRELOAD', '')}",
    }

    forever_arg = ['--forever'] if FOREVER else []

    subscriber = launch_ros.actions.Node(
        package='agnocast_e2e_test',
        executable='test_agnocast_node_subscriber',
        arguments=[
            '--node-name', 'agnocast_node_sub',
            '--topic', TOPIC,
            '--target-end-id', str(PUB_NUM - 1),
            '--target-end-count', '1',
        ] + forever_arg,
        output='screen',
        additional_env=additional_env,
    )

    publisher = launch_ros.actions.Node(
        package='agnocast_e2e_test',
        executable='test_agnocast_node_publisher',
        arguments=[
            '--node-name', 'agnocast_node_pub',
            '--topic', TOPIC,
            '--pub-num', str(PUB_NUM),
            '--planned-sub-count', '1',
        ] + forever_arg,
        output='screen',
        additional_env=additional_env,
    )

    return (
        launch.LaunchDescription([
            launch.actions.SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0'),
            subscriber,
            # The publisher waits for the subscriber to connect before publishing,
            # but launch it slightly later so the subscriber is up first.
            launch.actions.TimerAction(period=2.0, actions=[publisher]),
            launch.actions.TimerAction(
                period=5.0, actions=[launch_testing.actions.ReadyToTest()]),
        ]),
        {
            'publisher': publisher,
            'subscriber': subscriber,
        }
    )


class TestAgnocastNode1To1(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # In stress mode the nodes run with --forever under load. ReadyToTest
        # fires promptly and the soak period is spent here, before the assertions.
        if STRESS_TEST_TIMEOUT:
            time.sleep(float(STRESS_TEST_TIMEOUT))

    def test_publisher_output(self, proc_output, publisher):
        proc_output.assertWaitFor(
            'All messages published. Shutting down.', timeout=20.0, process=publisher)

        output_text = "".join(
            output.text.decode('utf-8') for output in proc_output[publisher]
        )
        for i in range(PUB_NUM):
            self.assertEqual(output_text.count(f"Publishing {i}."), 1)

    def test_subscriber_output(self, proc_output, subscriber):
        proc_output.assertWaitFor(
            'All messages received. Shutting down.', timeout=20.0, process=subscriber)

        output_text = "".join(
            output.text.decode('utf-8') for output in proc_output[subscriber]
        )
        for i in range(PUB_NUM):
            self.assertEqual(output_text.count(f"Receiving {i}."), 1)


@launch_testing.post_shutdown_test()
class TestAgnocastNode1To1Shutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)


# In forever mode launch_testing terminates the still-running nodes, so their
# exit codes are not clean; skip the exit-code assertion in that case.
if FOREVER:
    del TestAgnocastNode1To1Shutdown.test_exit_codes
