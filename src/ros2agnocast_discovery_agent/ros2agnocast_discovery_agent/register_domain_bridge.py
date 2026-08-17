"""Register Agnocast domain bridge rules with the kernel module.

Reads a ROS 2 ``domain_bridge`` YAML and registers each rule through the ioctl
wrapper. A ``topics:`` entry becomes one rule; a ``services:`` entry becomes two,
because Agnocast carries a service over a request topic plus one response topic
per client:

* ``/AGNOCAST_SRV_REQUEST<svc>``, client domain -> server domain, an exact rule.
* ``/AGNOCAST_SRV_RESPONSE<svc>_SEP_``, server domain -> client domain, a *prefix*
  rule. Each client appends its own node name and domain, so the full names only
  exist at runtime and cannot be enumerated here.

Run this once, before any application node for the bridged topics starts: the
kmod rejects a rule once an endpoint exists in either domain. The tool is
standalone and idempotent (the kmod folds duplicate rules), so it can run from
a boot-time one-shot, a launch file, or by hand. It is independent of the
discovery agent, which is observability-only and never registers rules.
"""
import argparse
import ctypes
import os
import sys

import yaml

from . import domain_bridge_config

# Must match create_service_{request,response}_topic_name in
# src/agnocastlib/src/agnocast_utils.cpp, which owns the naming.
_SRV_REQUEST_PREFIX = '/AGNOCAST_SRV_REQUEST'
_SRV_RESPONSE_PREFIX = '/AGNOCAST_SRV_RESPONSE'
_SRV_RESPONSE_SEP = '_SEP_'


def _load_add_rule_symbol():
    """Load the ioctl wrapper and return the bound add_agnocast_domain_bridge_rule."""
    lib = ctypes.CDLL('libagnocast_ioctl_wrapper.so')
    lib.add_agnocast_domain_bridge_rule.argtypes = [
        ctypes.c_char_p, ctypes.c_char_p, ctypes.c_uint32, ctypes.c_uint32]
    lib.add_agnocast_domain_bridge_rule.restype = ctypes.c_int
    return lib.add_agnocast_domain_bridge_rule


def _load_add_prefix_rule_symbol():
    """Load the ioctl wrapper and return the bound prefix-rule entry point."""
    lib = ctypes.CDLL('libagnocast_ioctl_wrapper.so')
    lib.add_agnocast_domain_bridge_prefix_rule.argtypes = [
        ctypes.c_char_p, ctypes.c_uint32, ctypes.c_uint32]
    lib.add_agnocast_domain_bridge_prefix_rule.restype = ctypes.c_int
    return lib.add_agnocast_domain_bridge_prefix_rule


def _service_rules(service_rules):
    """Expand ``services:`` entries into the topic rules the kmod needs.

    Yields ``(label, is_prefix, from_name, to_name, from_domain, to_domain)``. The
    response prefix uses the *client-side* service name on both sides: the client
    dictates the response topic name and the server publishes to whatever the
    request asked for, so the two cells share one name even under a remap.
    """
    for from_svc, to_svc, from_domain, to_domain in service_rules:
        yield ('request', False,
               _SRV_REQUEST_PREFIX + from_svc, _SRV_REQUEST_PREFIX + to_svc,
               from_domain, to_domain)
        response_prefix = _SRV_RESPONSE_PREFIX + from_svc + _SRV_RESPONSE_SEP
        yield ('response prefix', True,
               response_prefix, response_prefix, to_domain, from_domain)


def main(argv=None) -> int:
    """Register every rule in the config; return non-zero if any is rejected."""
    parser = argparse.ArgumentParser(
        description='Register Agnocast domain bridge rules with the kernel module.')
    parser.add_argument(
        '--config',
        default=os.environ.get(domain_bridge_config.CONFIG_ENV),
        help='path to the domain_bridge YAML '
             f'(default: ${domain_bridge_config.CONFIG_ENV})')
    args = parser.parse_args(argv)

    if not args.config:
        parser.error(
            f'no config given; pass --config or set {domain_bridge_config.CONFIG_ENV}')

    try:
        topic_rules, service_rules, skipped = domain_bridge_config.load_domain_bridge_rules(
            args.config)
    except (OSError, yaml.YAMLError, ValueError, TypeError) as e:
        print(f'error: cannot load {args.config}: {e}', file=sys.stderr)
        return 1

    for name in skipped:
        print(f'warning: skipping {name}: no from_domain/to_domain resolved '
              '(set them at the top level or on the entry)', file=sys.stderr)

    add_rule = _load_add_rule_symbol()
    # A topic-only config must not require the prefix-rule symbol, so resolve it lazily.
    add_prefix_rule = _load_add_prefix_rule_symbol() if service_rules else None

    failures = 0
    total = 0
    for from_topic, to_topic, from_domain, to_domain in topic_rules:
        total += 1
        label = f'{from_topic}@{from_domain} -> {to_topic}@{to_domain}'
        if add_rule(
                from_topic.encode('utf-8'), to_topic.encode('utf-8'),
                from_domain, to_domain) == 0:
            print(f'registered: {label}')
            continue
        # The wrapper prints the specific errno to stderr just above; the usual
        # cause is that an endpoint already exists, since a rule must precede
        # every node in either domain.
        failures += 1
        print(f'error: failed to register {label}', file=sys.stderr)

    for kind, is_prefix, from_name, to_name, from_domain, to_domain in _service_rules(
            service_rules):
        total += 1
        label = f'{from_name}@{from_domain} -> {to_name}@{to_domain} ({kind})'
        if is_prefix:
            ret = add_prefix_rule(from_name.encode('utf-8'), from_domain, to_domain)
        else:
            ret = add_rule(
                from_name.encode('utf-8'), to_name.encode('utf-8'), from_domain, to_domain)
        if ret == 0:
            print(f'registered: {label}')
            continue
        failures += 1
        print(f'error: failed to register {label}', file=sys.stderr)

    if failures:
        print(f'error: {failures} of {total} rule(s) rejected', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
