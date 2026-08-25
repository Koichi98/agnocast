"""Register Agnocast domain bridge rules with the kernel module.

Reads a ROS 2 ``domain_bridge`` YAML and registers each
``(from_topic, to_topic, from_domain, to_domain)`` rule through the ioctl wrapper
(``to_topic`` is the per-topic ``remap`` target, or the source name if absent).

Run this once, before any application node for the bridged topics starts: the
kmod rejects a rule once an endpoint exists in either domain. The tool is
standalone and idempotent (the kmod folds duplicate rules), so it can run from
a boot-time one-shot, a launch file, or by hand. It is independent of the
discovery agent, which is observability-only and never registers rules.

``--unregister`` removes the rules the same config names, so a changed config can
be applied without reloading the kmod (rules are otherwise append-only for the
module's lifetime). It requires every bridged node to have exited first, for the
same reason registration must precede them: the rule groups the two domains'
id spaces, and regrouping is only safe while neither side has any allocated.
"""
import argparse
import ctypes
import errno
import os
import sys

import yaml

from . import domain_bridge_config


def _load_add_rule_symbol():
    """Load the ioctl wrapper and return the bound add_agnocast_domain_bridge_rule."""
    lib = ctypes.CDLL('libagnocast_ioctl_wrapper.so')
    lib.add_agnocast_domain_bridge_rule.argtypes = [
        ctypes.c_char_p, ctypes.c_char_p, ctypes.c_uint32, ctypes.c_uint32]
    lib.add_agnocast_domain_bridge_rule.restype = ctypes.c_int
    return lib.add_agnocast_domain_bridge_rule


def _load_remove_rule_symbol():
    """Load the ioctl wrapper and return the bound remove_agnocast_domain_bridge_rule.

    ``use_errno`` is required so ctypes captures the wrapper's errno into its own
    slot; ``_unregister`` reads it to tell "already absent" (ENOENT) apart from a
    real failure.
    """
    lib = ctypes.CDLL('libagnocast_ioctl_wrapper.so', use_errno=True)
    lib.remove_agnocast_domain_bridge_rule.argtypes = [ctypes.c_char_p, ctypes.c_uint32]
    lib.remove_agnocast_domain_bridge_rule.restype = ctypes.c_int
    return lib.remove_agnocast_domain_bridge_rule


def _unregister(rules) -> int:
    """Remove every rule the config names; return the number that failed.

    A rule is removed by naming one of its two cells, so only the ``from`` side is
    passed. ENOENT is not a failure: the rule being already absent is the intended
    end state, which keeps repeated ``--unregister`` runs idempotent.
    """
    remove_rule = _load_remove_rule_symbol()
    failures = 0
    for from_topic, to_topic, from_domain, to_domain in rules:
        label = f'{from_topic}@{from_domain} -> {to_topic}@{to_domain}'
        if remove_rule(from_topic.encode('utf-8'), from_domain) == 0:
            print(f'unregistered: {label}')
            continue
        if ctypes.get_errno() == errno.ENOENT:
            print(f'already absent: {label}')
            continue
        # The wrapper prints the specific errno to stderr just above; the usual
        # cause is EBUSY, meaning a node in either domain is still running.
        failures += 1
        print(f'error: failed to unregister {label}', file=sys.stderr)
    return failures


def main(argv=None) -> int:
    """Register every rule in the config; return non-zero if any is rejected."""
    parser = argparse.ArgumentParser(
        description='Register Agnocast domain bridge rules with the kernel module.')
    parser.add_argument(
        '--config',
        default=os.environ.get(domain_bridge_config.CONFIG_ENV),
        help='path to the domain_bridge YAML '
             f'(default: ${domain_bridge_config.CONFIG_ENV})')
    parser.add_argument(
        '--unregister',
        action='store_true',
        help='remove the rules this config names instead of adding them; '
             'requires every node in the bridged domains to have exited')
    args = parser.parse_args(argv)

    if not args.config:
        parser.error(
            f'no config given; pass --config or set {domain_bridge_config.CONFIG_ENV}')

    try:
        rules, skipped = domain_bridge_config.load_domain_bridge_rules(args.config)
    except (OSError, yaml.YAMLError, ValueError, TypeError) as e:
        print(f'error: cannot load {args.config}: {e}', file=sys.stderr)
        return 1

    for topic in skipped:
        print(f'warning: skipping {topic}: no from_domain/to_domain resolved '
              '(set them at the top level or on the topic)', file=sys.stderr)

    if args.unregister:
        failures = _unregister(rules)
        if failures:
            print(f'error: {failures} of {len(rules)} rule(s) not removed', file=sys.stderr)
            return 1
        return 0

    add_rule = _load_add_rule_symbol()
    failures = 0
    for from_topic, to_topic, from_domain, to_domain in rules:
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

    if failures:
        print(f'error: {failures} of {len(rules)} rule(s) rejected', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
