# Registering domain bridge rules at boot

Agnocast domain bridge rules must be registered **before any publisher or
subscriber for the bridged topics exists** — the kernel module rejects a rule
once a domain has allocated endpoint ids. Registration is therefore a one-time
boot step, independent of the discovery agent (which is observability-only and
never registers rules).

`register_domain_bridge` (a console script in this package) reads a ROS 2
`domain_bridge` YAML and registers each rule:

```bash
ros2 run ros2agnocast_discovery_agent register_domain_bridge --config /etc/agnocast/domain_bridge.yaml
# or set AGNOCAST_DOMAIN_BRIDGE_CONFIG and run it with no arguments
```

Run it once, ordered so that it:

1. runs **after** the Agnocast kernel module is loaded (so `/dev/agnocast` exists),
2. runs **after** the filesystem holding the config is mounted, and
3. completes **before** any application node for the bridged topics starts.

`agnocast-domain-bridge.service.example` is a reference systemd one-shot that
expresses exactly this ordering (`After=` the kmod, `RequiresMountsFor=` the
config, `Before=` your application target). Agnocast does not ship an installed
unit or assume systemd — an init script or a container entrypoint that
satisfies the same ordering works just as well.

The tool is idempotent (the kmod folds duplicate rules) and exits non-zero if
any rule is rejected, so a misordering — a node started first — fails loudly
instead of silently leaving topics unbridged.

## Changing the config without reloading the kernel module

Registering the *same* config again always succeeds, so a plain restart of the
stack needs no extra step. But rules are otherwise append-only for the module's
lifetime: re-pointing a topic at a different domain, or adding the reverse
direction, is rejected while the old rule stands. `--unregister` removes the
rules a config names, so the corrected config can be applied in place:

```bash
# 1. stop every node in the bridged domains
# 2. drop the old rules, naming the config they were registered from
ros2 run ros2agnocast_discovery_agent register_domain_bridge --config /etc/agnocast/old.yaml --unregister
# 3. register the corrected config
ros2 run ros2agnocast_discovery_agent register_domain_bridge --config /etc/agnocast/new.yaml
```

Step 1 is required: removal is rejected with `EBUSY` while an endpoint in either
domain still exists, for the same reason registration must precede those nodes —
a rule merges the two domains' id spaces, and regrouping them is only safe while
neither side has any allocated. A rule that is already absent is reported as
`already absent` and does not fail, so `--unregister` is itself idempotent and
safe to run in a teardown path.

Without this step the only way to undo a rule is to reload the kernel module.
