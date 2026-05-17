import os
import time
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts

# End-to-end 2-to-2 test for agnocast::Node (the pure-Agnocast node).
# Two publisher nodes, each driven by an Agnocast wall timer, publish PUB_NUM
# messages to a shared topic. Two subscriber nodes each receive every message
# from both publishers. All four are standalone executables that spin an
# AgnocastOnlySingleThreadedExecutor.
#
# When STRESS_TEST_TIMEOUT is set, the nodes are launched with --forever and
# keep running under load; the soak is spent in setUpClass before the output
# assertions run (mirrors test_2to2.py).

TOPIC = '/agnocast_node_2to2'
PUB_NUM = 5
NUM_PUB = 2
NUM_SUB = 2
STRESS_TEST_TIMEOUT = os.environ.get('STRESS_TEST_TIMEOUT')
FOREVER = STRESS_TEST_TIMEOUT is not None


def generate_test_description():
    additional_env = {
        'LD_PRELOAD': f"libagnocast_heaphook.so:{os.getenv('LD_PRELOAD', '')}",
    }

    forever_arg = ['--forever'] if FOREVER else []

    subscribers = [
        launch_ros.actions.Node(
            package='agnocast_e2e_test',
            executable='test_agnocast_node_subscriber',
            arguments=[
                '--node-name', f'agnocast_node_sub_{i}',
                '--topic', TOPIC,
                '--target-end-id', str(PUB_NUM - 1),
                # The terminating id arrives once from each publisher.
                '--target-end-count', str(NUM_PUB),
            ] + forever_arg,
            output='screen',
            additional_env=additional_env,
        )
        for i in range(NUM_SUB)
    ]

    publishers = [
        launch_ros.actions.Node(
            package='agnocast_e2e_test',
            executable='test_agnocast_node_publisher',
            arguments=[
                '--node-name', f'agnocast_node_pub_{i}',
                '--topic', TOPIC,
                '--pub-num', str(PUB_NUM),
                '--planned-sub-count', str(NUM_SUB),
            ] + forever_arg,
            output='screen',
            additional_env=additional_env,
        )
        for i in range(NUM_PUB)
    ]

    actions = [launch.actions.SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0')]
    actions += subscribers
    # The publishers wait for both subscribers to connect, but launch them
    # slightly later so the subscribers are up first.
    actions.append(launch.actions.TimerAction(period=2.0, actions=publishers))
    actions.append(launch.actions.TimerAction(
        period=5.0, actions=[launch_testing.actions.ReadyToTest()]))

    context = {}
    for i, publisher in enumerate(publishers):
        context[f'publisher{i}'] = publisher
    for i, subscriber in enumerate(subscribers):
        context[f'subscriber{i}'] = subscriber

    return launch.LaunchDescription(actions), context


class TestAgnocastNode2To2(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # In stress mode the nodes run with --forever under load. ReadyToTest
        # fires promptly and the soak period is spent here, before the assertions.
        if STRESS_TEST_TIMEOUT:
            time.sleep(float(STRESS_TEST_TIMEOUT))

    def test_publisher_output(self, proc_output, publisher0, publisher1):
        for publisher in (publisher0, publisher1):
            proc_output.assertWaitFor(
                'All messages published. Shutting down.', timeout=25.0, process=publisher)

            output_text = "".join(
                output.text.decode('utf-8') for output in proc_output[publisher]
            )
            for i in range(PUB_NUM):
                self.assertEqual(output_text.count(f"Publishing {i}."), 1)

    def test_subscriber_output(self, proc_output, subscriber0, subscriber1):
        for subscriber in (subscriber0, subscriber1):
            proc_output.assertWaitFor(
                'All messages received. Shutting down.', timeout=25.0, process=subscriber)

            output_text = "".join(
                output.text.decode('utf-8') for output in proc_output[subscriber]
            )
            # Each id is received once per publisher.
            for i in range(PUB_NUM):
                self.assertEqual(output_text.count(f"Receiving {i}."), NUM_PUB)


@launch_testing.post_shutdown_test()
class TestAgnocastNode2To2Shutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)


# In forever mode launch_testing terminates the still-running nodes, so their
# exit codes are not clean; skip the exit-code assertion in that case.
if FOREVER:
    del TestAgnocastNode2To2Shutdown.test_exit_codes
