#!/bin/bash
# Same-IPC-namespace cross-domain zero-copy e2e for a *service* (two ROS domains, one mempool).
#
# A service rides on two topics, so one `services:` entry becomes two kmod rules:
#
#   /AGNOCAST_SRV_REQUEST<svc>        client domain -> server domain   (exact)
#   /AGNOCAST_SRV_RESPONSE<svc>_SEP_  server domain -> client domain   (prefix)
#
# The response rule has to be a prefix rule: each client appends its own node name and domain,
# so the full response topic names only exist at runtime and cannot be listed in the config.
#
# The script runs the sample server in one domain and the sample client in the other as
# SEPARATE launches, and asserts the client got its responses -- a cross-domain service call
# through the kmod with no DDS / bridge node.
#
# It then repeats the exchange with a *second* client of the same node name in the server's own
# domain. Response topics are per client, and the client's domain is part of the name; were it
# not, a domain bridge rule would merge the two same-named clients' response cells and each
# would consume the other's responses (they match only on a per-client seqno), so one of them
# would report "bad entry id" or hang instead of printing a result.
#
# Two separate launches (not one launch file) are required: each `ros2 launch` runs entirely
# under one ROS_DOMAIN_ID.
#
# Each launch is run under `timeout` so it self-terminates: the sample client keeps spinning
# after its last response, and Agnocast shutdown can stall on SIGINT.
#
# Run on a freshly loaded kmod -- a rule must be registered before any endpoint for its cells
# joins. This path uses no bridge, so AGNOCAST_BRIDGE_MODE=off.

set -uo pipefail

if ! grep -q "^agnocast " /proc/modules; then
    echo "ERROR: agnocast kernel module is not loaded." >&2
    echo "Load it first: sudo insmod agnocast_kmod/agnocast.ko" >&2
    exit 1
fi

if ! ros2 pkg prefix ros2agnocast_discovery_agent >/dev/null 2>&1; then
    echo "ERROR: ros2agnocast_discovery_agent not found -- source the workspace first:" >&2
    echo "  source /opt/ros/<distro>/setup.bash && source install/setup.bash" >&2
    exit 1
fi

# The sample launches remap /sum_int_array to this name.
SERVICE="${E2E_SERVICE_NAME:-/srv/sum_int_array}"
CLIENT_DOMAIN="${E2E_CLIENT_DOMAIN:-2}"
SERVER_DOMAIN="${E2E_SERVER_DOMAIN:-1}"
RUN_SECONDS="${E2E_RUN_SECONDS:-10}"

SERVER_SECS=$((5 + RUN_SECONDS + 3))   # server: startup + run + drain
CLIENT_SECS=$((RUN_SECONDS + 3))       # client: starts 5 s later

cfg="$(mktemp --suffix=.yaml)"
{
    echo "from_domain: ${CLIENT_DOMAIN}"
    echo "to_domain: ${SERVER_DOMAIN}"
    echo "services:"
    echo "  \"${SERVICE}\":"
} > "$cfg"

echo ">>> registering rules for ${SERVICE} (clients@${CLIENT_DOMAIN}, server@${SERVER_DOMAIN})"
# Register through the same tool production uses, not an inline ioctl call.
if ! ros2 run ros2agnocast_discovery_agent register_domain_bridge --config "$cfg"; then
    echo "Rule registration failed (fresh kmod? a covered endpoint already exists?)." >&2
    rm -f "$cfg"; exit 1
fi
rm -f "$cfg"

serverlog="$(mktemp)"
echo ">>> server: ${SERVICE}@${SERVER_DOMAIN} (~${SERVER_SECS}s)"
timeout -s INT -k 5 "$SERVER_SECS" \
    env AGNOCAST_BRIDGE_MODE=off ROS_DOMAIN_ID="$SERVER_DOMAIN" \
    ros2 launch agnocast_sample_application server.launch.xml > "$serverlog" 2>&1 &
server_pid=$!
sleep 5

# run_client <domain> <label>; leaves its log path in CLIENT_LOG.
run_client() {
    local domain="$1" label="$2"
    CLIENT_LOG="$(mktemp)"
    echo ">>> client (${label}): calling ${SERVICE} from domain ${domain} (~${CLIENT_SECS}s)"
    timeout -s INT -k 5 "$CLIENT_SECS" \
        env AGNOCAST_BRIDGE_MODE=off ROS_DOMAIN_ID="$domain" \
        ros2 launch agnocast_sample_application client.launch.xml > "$CLIENT_LOG" 2>&1
}

# check_client <log> <label>; 0 if both responses arrived and nothing was mismatched.
check_client() {
    local log="$1" label="$2" ret=0
    local result_count bad_entry
    result_count=$(grep -cE "Result[12]: " "$log" || true)
    bad_entry=$(grep -c "bad entry id" "$log" || true)
    echo "  ${label}: responses=${result_count} bad_entry_id=${bad_entry}"
    if [ "$result_count" -lt 2 ]; then
        echo "  FAIL (${label}): expected 2 responses, got ${result_count}." >&2
        ret=1
    fi
    if [ "$bad_entry" -ne 0 ]; then
        echo "  FAIL (${label}): a response could not be matched to its request." >&2
        ret=1
    fi
    [ "$ret" -ne 0 ] && sed 's/^/    /' "$log" >&2
    return "$ret"
}

failures=0

run_client "$CLIENT_DOMAIN" "cross-domain"
cross_log="$CLIENT_LOG"
check_client "$cross_log" "cross-domain" || failures=$((failures + 1))

# Same node name, this time in the server's own domain: it must not disturb, or be disturbed by,
# the cross-domain client above.
run_client "$SERVER_DOMAIN" "same-domain twin"
twin_log="$CLIENT_LOG"
check_client "$twin_log" "same-domain twin" || failures=$((failures + 1))

run_client "$CLIENT_DOMAIN" "cross-domain again"
again_log="$CLIENT_LOG"
check_client "$again_log" "cross-domain again" || failures=$((failures + 1))

wait "$server_pid" 2>/dev/null || true
sleep 1

served=$(grep -c "Sending back response: \[" "$serverlog" || true)
echo "  server: responses sent=${served}"
if [ "$served" -lt 6 ]; then
    echo "  FAIL: server sent ${served} responses, expected 6 (2 per client)." >&2
    sed 's/^/    /' "$serverlog" >&2
    failures=$((failures + 1))
fi

rm -f "$serverlog" "$cross_log" "$twin_log" "$again_log"

if [ "$failures" -ne 0 ]; then
    echo "FAIL: cross-domain service call not working (${failures} check(s) failed)." >&2
    exit 1
fi
echo "PASS: ${SERVICE}@${SERVER_DOMAIN} served clients in domain ${CLIENT_DOMAIN} and ${SERVER_DOMAIN} (cross-domain zero-copy)."
