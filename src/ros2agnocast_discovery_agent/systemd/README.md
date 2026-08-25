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

## Services

A `services:` entry has the same shape as a `topics:` one, with `from_domain`
naming the side the **clients** are on and `to_domain` the side the **server** is
on:

```yaml
from_domain: 2   # clients
to_domain: 1     # server
services:
  "/srv/sum_int_array":
    # remap: "/srv/renamed"   # optional; the name on the server's side
```

Agnocast carries a service over a request topic plus one response topic per
client, so one entry becomes **two** rules:

| rule | name | direction |
| --- | --- | --- |
| request | `/AGNOCAST_SRV_REQUEST<svc>` | clients → server |
| response | `/AGNOCAST_SRV_RESPONSE<svc>%` (a **prefix** rule) | server → clients |

Each client appends its own node name and request-publisher id to the response
topic, so those full names exist only at runtime; the prefix rule covers them all,
whatever clients turn up later. Under a `remap` both response cells keep the
**client-side** name, because the client dictates it and the server publishes to
whatever the request asked for.

The config names the *service*, not those two topics. The kernel module expands it
and applies both rules under one lock, all or nothing: the response rule is only
safe alongside its request rule, whose merged id space is what keeps two clients
that share a node name off one response topic.

### A `services:` block is not shared with the external `domain_bridge` node

That node creates its ROS 2 **client** in `from_domain` and exports the relay
**service** into `to_domain`, so there `from_domain` is the *server's* side and
`remap` renames the name the *callers* see — the reverse of both rules above.
Give the two a config file each.
