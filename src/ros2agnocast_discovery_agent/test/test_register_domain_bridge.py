"""Tests for the standalone domain bridge rule injector."""
import pytest

from ros2agnocast_discovery_agent import register_domain_bridge
from ros2agnocast_discovery_agent.domain_bridge_config import CONFIG_ENV


class FakeAddRule:
    """Stand-in for an ioctl wrapper rule symbol: records calls, returns a code per source name."""

    def __init__(self, codes=None):
        self.calls = []
        self._codes = codes or {}

    def __call__(self, from_name, to_name, from_domain, to_domain):
        from_name = from_name.decode('utf-8')
        to_name = to_name.decode('utf-8')
        self.calls.append((from_name, to_name, from_domain, to_domain))
        return self._codes.get(from_name, 0)


def _write_config(tmp_path, text):
    path = tmp_path / 'bridge.yaml'
    path.write_text(text)
    return str(path)


def test_no_config_is_usage_error(monkeypatch):
    monkeypatch.delenv(CONFIG_ENV, raising=False)
    with pytest.raises(SystemExit):
        register_domain_bridge.main([])


def test_registers_every_rule(tmp_path, monkeypatch):
    fake = FakeAddRule()
    monkeypatch.setattr(register_domain_bridge, '_load_add_rule_symbol', lambda: fake)
    cfg = _write_config(tmp_path, 'from_domain: 1\nto_domain: 2\ntopics:\n  chatter:\n')
    assert register_domain_bridge.main(['--config', cfg]) == 0
    assert fake.calls == [('chatter', 'chatter', 1, 2)]


def test_registers_remapped_rule_with_both_names(tmp_path, monkeypatch):
    fake = FakeAddRule()
    monkeypatch.setattr(register_domain_bridge, '_load_add_rule_symbol', lambda: fake)
    cfg = _write_config(
        tmp_path,
        'from_domain: 1\nto_domain: 2\ntopics:\n  /in_sub/chatter:\n    remap: /chatter\n')
    assert register_domain_bridge.main(['--config', cfg]) == 0
    assert fake.calls == [('/in_sub/chatter', '/chatter', 1, 2)]


def test_returns_nonzero_when_a_rule_is_rejected(tmp_path, monkeypatch):
    fake = FakeAddRule(codes={'chatter': -16})  # -EBUSY: an endpoint already exists
    monkeypatch.setattr(register_domain_bridge, '_load_add_rule_symbol', lambda: fake)
    cfg = _write_config(tmp_path, 'from_domain: 1\nto_domain: 2\ntopics:\n  chatter:\n')
    assert register_domain_bridge.main(['--config', cfg]) == 1


def test_skipped_topic_is_reported(tmp_path, monkeypatch, capsys):
    fake = FakeAddRule()
    monkeypatch.setattr(register_domain_bridge, '_load_add_rule_symbol', lambda: fake)
    # No domains resolve for 'chatter', so it is skipped rather than registered.
    cfg = _write_config(tmp_path, 'topics:\n  chatter:\n')
    assert register_domain_bridge.main(['--config', cfg]) == 0
    assert fake.calls == []
    assert 'skipping chatter' in capsys.readouterr().err


def test_returns_nonzero_on_unreadable_config(tmp_path, monkeypatch):
    monkeypatch.setattr(
        register_domain_bridge, '_load_add_rule_symbol',
        lambda: pytest.fail('the wrapper must not load when the config is unreadable'))
    missing = str(tmp_path / 'does_not_exist.yaml')
    assert register_domain_bridge.main(['--config', missing]) == 1


def test_config_path_falls_back_to_env(tmp_path, monkeypatch):
    fake = FakeAddRule()
    monkeypatch.setattr(register_domain_bridge, '_load_add_rule_symbol', lambda: fake)
    cfg = _write_config(tmp_path, 'from_domain: 3\nto_domain: 4\ntopics:\n  image:\n')
    monkeypatch.setenv(CONFIG_ENV, cfg)
    assert register_domain_bridge.main([]) == 0
    assert fake.calls == [('image', 'image', 3, 4)]


def test_service_is_registered_through_the_service_ioctl(tmp_path, monkeypatch):
    # The tool passes the service name straight through; it never builds /AGNOCAST_SRV_*.
    fake = FakeAddRule()
    fake_service = FakeAddRule()
    monkeypatch.setattr(register_domain_bridge, '_load_add_rule_symbol', lambda: fake)
    monkeypatch.setattr(
        register_domain_bridge, '_load_add_service_rule_symbol', lambda: fake_service)
    cfg = _write_config(
        tmp_path, 'from_domain: 2\nto_domain: 1\nservices:\n  /add_two_ints:\n')
    assert register_domain_bridge.main(['--config', cfg]) == 0
    # from_domain is the clients' domain, to_domain the server's.
    assert fake_service.calls == [('/add_two_ints', '/add_two_ints', 2, 1)]
    assert fake.calls == []


def test_remapped_service_passes_both_names(tmp_path, monkeypatch):
    # The clients call the from-name, the server offers the to-name.
    fake = FakeAddRule()
    fake_service = FakeAddRule()
    monkeypatch.setattr(register_domain_bridge, '_load_add_rule_symbol', lambda: fake)
    monkeypatch.setattr(
        register_domain_bridge, '_load_add_service_rule_symbol', lambda: fake_service)
    cfg = _write_config(
        tmp_path,
        'from_domain: 2\nto_domain: 1\nservices:\n  /add_two_ints:\n    remap: /renamed_add\n')
    assert register_domain_bridge.main(['--config', cfg]) == 0
    assert fake_service.calls == [('/add_two_ints', '/renamed_add', 2, 1)]


def test_returns_nonzero_when_a_service_rule_is_rejected(tmp_path, monkeypatch):
    fake = FakeAddRule()
    fake_service = FakeAddRule(codes={'/add_two_ints': -16})  # -EBUSY
    monkeypatch.setattr(register_domain_bridge, '_load_add_rule_symbol', lambda: fake)
    monkeypatch.setattr(
        register_domain_bridge, '_load_add_service_rule_symbol', lambda: fake_service)
    cfg = _write_config(
        tmp_path, 'from_domain: 2\nto_domain: 1\nservices:\n  /add_two_ints:\n')
    assert register_domain_bridge.main(['--config', cfg]) == 1


def test_topic_only_config_does_not_load_the_service_symbol(tmp_path, monkeypatch):
    # An older wrapper has no service symbol at all.
    fake = FakeAddRule()
    monkeypatch.setattr(register_domain_bridge, '_load_add_rule_symbol', lambda: fake)
    monkeypatch.setattr(
        register_domain_bridge, '_load_add_service_rule_symbol',
        lambda: pytest.fail('a topic-only config must not need the service symbol'))
    cfg = _write_config(tmp_path, 'from_domain: 1\nto_domain: 2\ntopics:\n  chatter:\n')
    assert register_domain_bridge.main(['--config', cfg]) == 0
