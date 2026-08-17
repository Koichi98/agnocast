"""Parse a ROS 2 ``domain_bridge`` YAML into kmod rule tuples.

Each rule is ``(from_name, to_name, from_domain, to_domain)``. The same YAML
drives both the external ``domain_bridge`` node (cross-ECU, via DDS) and the kmod
rule injection that opens same-IPC-namespace zero-copy cross-domain delivery. The
name, its ``remap`` target, and the domain pair matter here; ``type`` and
other fields are ignored.

``topics:`` and ``services:`` have the same shape. A service is not a kmod
concept, so its entry is translated into topic rules by the caller: Agnocast
carries a service over a request topic and one response topic per client.
"""
import yaml

# Operators point the daemon at the config by setting this to the YAML path.
CONFIG_ENV = 'AGNOCAST_DOMAIN_BRIDGE_CONFIG'

# Domain ids cross the ioctl boundary as ctypes.c_uint32, so an out-of-range
# value would wrap silently; reject it here instead.
_UINT32_MAX = 0xFFFFFFFF


def _as_domain_id(value):
    """Coerce a YAML domain value to a uint32, raising ``ValueError`` if invalid."""
    domain = int(value)  # ValueError on non-numeric, TypeError on a list/dict
    if not 0 <= domain <= _UINT32_MAX:
        raise ValueError(f'domain id {domain} out of range [0, {_UINT32_MAX}]')
    return domain


def _parse_section(doc, section, default_from, default_to):
    """Return ``(rules, skipped)`` for the ``section`` mapping of ``doc``."""
    entries = doc.get(section)
    if entries is None:
        entries = {}
    if not isinstance(entries, dict):
        raise ValueError(f"'{section}' must be a mapping")

    rules = []
    skipped = []
    for name, spec in entries.items():
        if spec is None:
            spec = {}
        elif not isinstance(spec, dict):
            raise ValueError(f'spec for {name!r} must be a mapping')
        from_domain = spec.get('from_domain', default_from)
        to_domain = spec.get('to_domain', default_to)
        if from_domain is None or to_domain is None:
            skipped.append(str(name))
            continue
        # Default to the source name (coerced like from_name below), so a non-string YAML key
        # without a remap doesn't trip the "'remap' must be a string" check.
        to_name = spec.get('remap', str(name))
        if not isinstance(to_name, str):
            raise ValueError(f"'remap' for {name!r} must be a string")
        rules.append(
            (str(name), to_name, _as_domain_id(from_domain), _as_domain_id(to_domain)))
    return rules, skipped


def parse_domain_bridge_config(text):
    """Return ``(topic_rules, service_rules, skipped)``.

    Each rule list holds ``(from_name, to_name, from_domain, to_domain)`` tuples.
    ``to_name`` is the per-entry ``remap`` target (the same ``domain_bridge`` field
    the external node honors), or the source name when ``remap`` is absent.
    ``skipped`` lists the topic and service names dropped for lack of a resolvable
    domain pair, so the caller can surface them instead of dropping them silently.
    ``from_domain`` / ``to_domain`` are taken from the top level and may be
    overridden per entry.

    For a service, ``from_domain`` is the side the clients are on and ``to_domain``
    the side the server is on; the request flows from the former to the latter.

    Raises ``ValueError`` / ``TypeError`` on a structurally malformed document
    (non-mapping root, ``topics``, ``services``, or entry spec), a non-string
    ``remap``, or an out-of-range domain id. The caller catches these and skips the
    config rather than crashing.
    """
    doc = yaml.safe_load(text) or {}
    if not isinstance(doc, dict):
        raise ValueError('domain bridge config root must be a mapping')

    default_from = doc.get('from_domain')
    default_to = doc.get('to_domain')

    topic_rules, topic_skipped = _parse_section(doc, 'topics', default_from, default_to)
    service_rules, service_skipped = _parse_section(doc, 'services', default_from, default_to)
    return topic_rules, service_rules, topic_skipped + service_skipped


def load_domain_bridge_rules(path):
    """Read and parse the ``domain_bridge`` YAML at ``path``.

    Returns ``(topic_rules, service_rules, skipped)``.
    """
    with open(path, encoding='utf-8') as f:
        return parse_domain_bridge_config(f.read())
