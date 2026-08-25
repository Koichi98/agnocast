"""Unit tests for the agent's Ros2NodeRegistryWriter.

The writer publishes the DDS node list to a tmpfs file that
`agnocastlib::internal::read_ros2_node_names()` reads back; the reader's own
tests live in agnocastlib. These tests use a temporary directory as the base --
no kmod, no rclpy.
"""

import os
import tempfile

from ros2agnocast_discovery_agent.ros2_node_registry import Ros2NodeRegistryWriter


def _make_writer(tmpdir: str, ns_inode: int = 1234, domain_id: int = 0):
    return Ros2NodeRegistryWriter(
        ipc_ns_inode=ns_inode, domain_id=domain_id, base_dir=tmpdir)


def _read(path: str) -> str:
    with open(path, encoding='utf-8') as fp:
        return fp.read()


def test_write_creates_the_namespace_directory_and_file():
    with tempfile.TemporaryDirectory() as tmpdir:
        writer = _make_writer(tmpdir, ns_inode=42, domain_id=7)

        assert writer.write([('talker', '/')])

        assert writer.path == os.path.join(tmpdir, '42', '7')
        assert _read(writer.path) == '/\ttalker\n'


def test_write_emits_namespace_then_name_per_line():
    with tempfile.TemporaryDirectory() as tmpdir:
        writer = _make_writer(tmpdir)

        writer.write([('talker', '/'), ('listener', '/sensing')])

        assert _read(writer.path) == '/\ttalker\n/sensing\tlistener\n'


# Two nodes sharing a name are two nodes; the reader relies on the writer not
# collapsing them.
def test_write_keeps_duplicate_names():
    with tempfile.TemporaryDirectory() as tmpdir:
        writer = _make_writer(tmpdir)

        writer.write([('talker', '/'), ('talker', '/')])

        assert _read(writer.path) == '/\ttalker\n/\ttalker\n'


def test_write_truncates_the_previous_snapshot():
    with tempfile.TemporaryDirectory() as tmpdir:
        writer = _make_writer(tmpdir)
        writer.write([('talker', '/'), ('listener', '/')])

        writer.write([('talker', '/')])

        assert _read(writer.path) == '/\ttalker\n'


def test_write_leaves_no_temporary_file_behind():
    with tempfile.TemporaryDirectory() as tmpdir:
        writer = _make_writer(tmpdir, ns_inode=42)
        writer.write([('talker', '/')])

        assert sorted(os.listdir(os.path.join(tmpdir, '42'))) == ['0']


# The reading process may run as another user than the agent.
def test_write_makes_the_file_world_readable():
    with tempfile.TemporaryDirectory() as tmpdir:
        writer = _make_writer(tmpdir)

        writer.write([('talker', '/')])

        assert os.stat(writer.path).st_mode & 0o044 == 0o044


def test_write_of_an_empty_graph_writes_an_empty_file():
    with tempfile.TemporaryDirectory() as tmpdir:
        writer = _make_writer(tmpdir)

        assert writer.write([])

        assert _read(writer.path) == ''


def test_cleanup_removes_the_file():
    with tempfile.TemporaryDirectory() as tmpdir:
        writer = _make_writer(tmpdir)
        writer.write([('talker', '/')])

        writer.cleanup()

        assert not os.path.exists(writer.path)


def test_cleanup_is_a_no_op_when_nothing_was_written():
    with tempfile.TemporaryDirectory() as tmpdir:
        writer = _make_writer(tmpdir)

        writer.cleanup()  # must not raise


class _RecordingLogger:
    def __init__(self):
        self.warnings = []

    def warn(self, message):
        self.warnings.append(message)


# A tmpfs the agent cannot write to degrades get_node_names() to Agnocast-only
# reporting; it must not take the agent down, and it must not re-log every tick.
def test_write_warns_once_and_keeps_going_when_the_base_dir_is_unusable():
    with tempfile.TemporaryDirectory() as tmpdir:
        blocking_file = os.path.join(tmpdir, 'blocked')
        with open(blocking_file, 'w', encoding='utf-8'):
            pass
        logger = _RecordingLogger()
        writer = Ros2NodeRegistryWriter(
            ipc_ns_inode=1234, domain_id=0, base_dir=blocking_file, logger=logger)

        assert not writer.write([('talker', '/')])
        assert not writer.write([('talker', '/')])

        assert len(logger.warnings) == 1


# The agent unlinks before it frees its kmod slot, so the shutdown path runs a second
# cleanup() once a successor may already own the path. That one must not delete it.
def test_cleanup_twice_leaves_a_successors_file_alone():
    with tempfile.TemporaryDirectory() as tmpdir:
        writer = _make_writer(tmpdir)
        writer.write([('talker', '/')])
        writer.cleanup()

        successor = _make_writer(tmpdir)
        successor.write([('successor', '/')])
        writer.cleanup()

        assert _read(successor.path) == '/\tsuccessor\n'


# A vetoed exit keeps the agent running, so the next tick's write must make the file
# removable again.
def test_cleanup_works_again_after_a_rewrite():
    with tempfile.TemporaryDirectory() as tmpdir:
        writer = _make_writer(tmpdir)
        writer.write([('talker', '/')])
        writer.cleanup()

        writer.write([('talker', '/')])
        writer.cleanup()

        assert not os.path.exists(writer.path)
