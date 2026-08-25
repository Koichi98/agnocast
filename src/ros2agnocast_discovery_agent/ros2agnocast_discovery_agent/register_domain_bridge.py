"""Register Agnocast domain bridge rules with the kernel module.

Reads a ROS 2 ``domain_bridge`` YAML and registers each entry through the ioctl
wrapper. A ``topics:`` entry becomes one rule; a ``services:`` entry goes through
its own ioctl, which takes the *service* name and expands it into the request and
response rules -- the topic naming rule stays in the kmod and in agnocastlib.

Run this once, before any application node for the bridged topics starts: the kmod
rejects a rule once an endpoint exists in either domain. It is standalone and
idempotent (the kmod folds duplicate rules), and independent of the discovery
agent, which is observability-only and never registers rules.
"""
import argparse
import ctypes
import os
import sys

import yaml

from . import domain_bridge_config


def _load_symbol(name):
    """Load the ioctl wrapper and return its bound ``name`` entry point."""
    lib = ctypes.CDLL('libagnocast_ioctl_wrapper.so')
    fn = getattr(lib, name)
    fn.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_uint32, ctypes.c_uint32]
    fn.restype = ctypes.c_int
    return fn


def _load_add_rule_symbol():
    """Return the bound topic-rule entry point."""
    return _load_symbol('add_agnocast_domain_bridge_rule')


def _load_add_service_rule_symbol():
    """Return the bound service-rule entry point."""
    return _load_symbol('add_agnocast_domain_bridge_service_rule')


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
    # Lazily resolved: an older wrapper has no service symbol, and a topic-only
    # config must not need it.
    add_service_rule = _load_add_service_rule_symbol() if service_rules else None

    failures = 0
    total = len(topic_rules) + len(service_rules)
    for from_topic, to_topic, from_domain, to_domain in topic_rules:
        label = f'{from_topic}@{from_domain} -> {to_topic}@{to_domain}'
        if add_rule(
                from_topic.encode('utf-8'), to_topic.encode('utf-8'),
                from_domain, to_domain) == 0:
            print(f'registered: {label}')
            continue
        # The wrapper printed the errno just above; the usual cause is an endpoint
        # that already exists, since a rule must precede every node in either domain.
        failures += 1
        print(f'error: failed to register {label}', file=sys.stderr)

    for from_svc, to_svc, from_domain, to_domain in service_rules:
        label = f'{from_svc}@{from_domain} -> {to_svc}@{to_domain} (service)'
        # One ioctl covers both rules; the kmod applies them together or not at all.
        if add_service_rule(
                from_svc.encode('utf-8'), to_svc.encode('utf-8'),
                from_domain, to_domain) == 0:
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
