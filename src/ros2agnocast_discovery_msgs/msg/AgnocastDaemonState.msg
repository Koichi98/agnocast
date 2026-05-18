# Snapshot of the Agnocast state of a single IPC namespace, published by
# the discovery agent. One message per IPC namespace.
#
# Schema is versioned: v1 is the initial form; future bumps add fields
# without breaking existing readers (CLI checks `schema_version` and
# degrades gracefully on unknown values).

uint32 schema_version
# Agnocast library / runtime version of the publisher. Redundant with
# `schema_version` for correctness but useful for debugging mixed-version
# deployments.
string agnocast_version
string host_uuid                   # ECU identity; /etc/machine-id recommended
string host_hostname               # Human-readable hostname
builtin_interfaces/Time timestamp  # For stale detection
uint64 ipc_ns_inode                # NS this message describes
AgnocastTopic[] topics
