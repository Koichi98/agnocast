"""Publish the DDS-side node list to tmpfs for Agnocast-only processes to read.

An ``agnocast::Node`` process creates no DDS participant, so
``NodeGraph::get_node_names()`` cannot see the ROS 2 graph from the inside. This
agent does have a participant, and it already runs once per (IPC namespace,
ROS_DOMAIN_ID) -- the same scope ``get_node_names()`` reports on -- so each tick
it writes what ``rclpy`` sees to
``${AGNOCAST_TMPFS_DIR:-/dev/shm}/agnocast_ros2_nodes/<ipc_ns_inode>/<domain_id>``.

This is the mirror image of :mod:`type_registry` (there agnocastlib writes and
this agent reads), and the line format follows the same contract:

* Lines are tab-separated and ``\\n``-terminated::

      <node_namespace>\\t<node_name>\\n

  The reader composes the fully qualified name, so it matches the names the kmod
  holds for Agnocast endpoints.
* The first two fields are required; extra fields are ignored, so the format can
  be extended additively without a lockstep reader update.

The file is replaced atomically (write to a temporary, then ``rename``) so a
reader never observes a half-written list, and it is unlinked on graceful
shutdown. After a crash the file survives, so the reader also treats one that has
stopped being refreshed as absent (see ``kStaleAfter`` in
``ros2_node_registry_reader.hpp``); keep the write cadence well inside that
window.
"""

import os
import tempfile
from typing import Iterable, Tuple


def _default_base_dir() -> str:
    """Return the tmpfs root, honouring the same override as agnocastlib."""
    root = os.environ.get('AGNOCAST_TMPFS_DIR') or '/dev/shm'
    return os.path.join(root, 'agnocast_ros2_nodes')


BASE_DIR = _default_base_dir()

# World-readable so an Agnocast process owned by another user can read the list;
# /dev/shm itself is 1777, which is what lets an unelevated agent create these.
DIR_MODE = 0o755
FILE_MODE = 0o644


class Ros2NodeRegistryWriter:
    """Writes one file per (IPC namespace, domain) with the local DDS node list.

    Errors are logged once and then swallowed: a missing or full tmpfs degrades
    ``get_node_names()`` back to Agnocast-only reporting, which is not worth
    taking the agent down for.
    """

    def __init__(
            self, ipc_ns_inode: int, domain_id: int, base_dir: str = BASE_DIR, logger=None):
        self._ns_dir = os.path.join(base_dir, str(ipc_ns_inode))
        self._path = os.path.join(self._ns_dir, str(domain_id))
        self._logger = logger
        self._warned = False

    @property
    def path(self) -> str:
        return self._path

    def write(self, node_names_and_namespaces: Iterable[Tuple[str, str]]) -> bool:
        """Replace the file with ``(name, namespace)`` pairs as rclpy returns them.

        Duplicates are written as-is: two nodes sharing a name are two nodes, and
        collapsing them here would lose that.

        Returns True when the file was replaced.
        """
        lines = ''.join(
            f'{namespace}\t{name}\n' for name, namespace in node_names_and_namespaces)
        try:
            os.makedirs(self._ns_dir, mode=DIR_MODE, exist_ok=True)
            # Same directory as the target, so the rename below cannot cross a filesystem.
            fd, tmp_path = tempfile.mkstemp(dir=self._ns_dir, prefix='.tmp-')
            try:
                with os.fdopen(fd, 'w') as fp:
                    fp.write(lines)
                os.chmod(tmp_path, FILE_MODE)
                os.replace(tmp_path, self._path)
            except OSError:
                # mkstemp already created the file, so remove it before propagating.
                try:
                    os.unlink(tmp_path)
                except OSError:
                    pass
                raise
        except OSError as exc:
            self._warn(f'ros2_node_registry: failed to write {self._path}: {exc}')
            return False
        return True

    def cleanup(self) -> None:
        """Remove the file so readers stop reporting this agent's snapshot."""
        try:
            os.unlink(self._path)
        except FileNotFoundError:
            pass
        except OSError as exc:
            self._warn(f'ros2_node_registry: failed to unlink {self._path}: {exc}')

    def _warn(self, message: str) -> None:
        if self._warned or self._logger is None:
            return
        self._logger.warn(message)
        self._warned = True
