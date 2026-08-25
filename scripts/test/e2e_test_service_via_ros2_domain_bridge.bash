#!/bin/bash
# Cross-domain Agnocast service call relayed by the external ROS 2 `domain_bridge` node -- the
# non-zero-copy path, with the kernel module bridging nothing:
#
#   agnocast client @CLIENT_DOMAIN
#     -> A2R service bridge      (Agnocast service + ROS 2 client)
#       -> domain_bridge         (ROS 2 service @CLIENT_DOMAIN <- client @SERVER_DOMAIN)
#         -> R2A service bridge  (ROS 2 service + Agnocast client)
#           -> agnocast server @SERVER_DOMAIN
#
# Both bridges rise on demand from the service name, so the YAML is the only configuration. Its
# `from_domain` is the *server's* side -- the reverse of a `topics:` entry and of `services:`.
#
# One IPC namespace does not weaken the test: kmod topics are keyed by domain as well, and no
# domain bridge rule is registered here, so DDS stays the only route. A real split needs
# `unshare`, and privileges this script lacks.
#
# Needs AGNOCAST_BRIDGE_MODE on. Skipped when domain_bridge lacks generic service support.

set -uo pipefail

SERVICE="${E2E_SERVICE_NAME:-/srv/sum_int_array}"
SERVICE_TYPE="${E2E_SERVICE_TYPE:-agnocast_sample_interfaces/srv/SumIntArray}"
CLIENT_DOMAIN="${E2E_CLIENT_DOMAIN:-1}"
SERVER_DOMAIN="${E2E_SERVER_DOMAIN:-2}"

BRIDGE_SECS="${E2E_BRIDGE_SECONDS:-40}"
SERVER_SECS="${E2E_SERVER_SECONDS:-32}"
CLIENT_SECS="${E2E_CLIENT_SECONDS:-20}"

if ! grep -q "^agnocast " /proc/modules; then
    echo "ERROR: agnocast kernel module is not loaded." >&2
    echo "Load it first: sudo insmod agnocast_kmod/agnocast.ko" >&2
    exit 1
fi

if ! ros2 pkg prefix agnocast_sample_application >/dev/null 2>&1; then
    echo "ERROR: agnocast_sample_application not found -- source the workspace first:" >&2
    echo "  source /opt/ros/<distro>/setup.bash && source install/setup.bash" >&2
    exit 1
fi

domain_bridge_prefix="$(ros2 pkg prefix domain_bridge 2>/dev/null)"
if [ -z "$domain_bridge_prefix" ]; then
    echo "SKIP: domain_bridge is not installed."
    exit 0
fi
# Relaying a service from a YAML config needs generic (type-erased) service support, which a stock
# build lacks; without this probe the run would fail on the assertions instead.
if [ ! -f "${domain_bridge_prefix}/include/domain_bridge/generic_service.hpp" ]; then
    echo "SKIP: domain_bridge at ${domain_bridge_prefix} was built without service support."
    exit 0
fi

cfg="$(mktemp --suffix=.yaml)"
bridgelog="$(mktemp)"; serverlog="$(mktemp)"; clientlog="$(mktemp)"
cleanup() { rm -f "$cfg" "$bridgelog" "$serverlog" "$clientlog"; }
trap cleanup EXIT

{
    echo "name: agnocast_e2e_service_bridge"
    echo "from_domain: ${SERVER_DOMAIN}"
    echo "to_domain: ${CLIENT_DOMAIN}"
    echo "services:"
    echo "  \"${SERVICE}\":"
    echo "    type: ${SERVICE_TYPE}"
} > "$cfg"

# Started first so its ROS 2 client is already in the server's domain when the server appears:
# that client is what a demand-gated R2A bridge waits for.
echo ">>> domain_bridge: ${SERVICE} from domain ${SERVER_DOMAIN} to domain ${CLIENT_DOMAIN}"
timeout -s INT -k 5 "$BRIDGE_SECS" ros2 run domain_bridge domain_bridge "$cfg" \
    > "$bridgelog" 2>&1 &
bridge_pid=$!
sleep 4

echo ">>> server: ${SERVICE}@${SERVER_DOMAIN}"
timeout -s INT -k 5 "$SERVER_SECS" env ROS_DOMAIN_ID="$SERVER_DOMAIN" \
    ros2 launch agnocast_sample_application server.launch.xml > "$serverlog" 2>&1 &
server_pid=$!
sleep 8

# The relay service in the caller's domain is what distinguishes this route from the two Agnocast
# ends having found each other some other way, and is what the caller-side A2R bridge keys off.
exported=0
for d in "$SERVER_DOMAIN" "$CLIENT_DOMAIN"; do
    seen=$(ROS_DOMAIN_ID="$d" timeout 10 ros2 service list 2>/dev/null | grep -c "^${SERVICE}$")
    echo "  ${SERVICE} visible in domain ${d}: ${seen}"
    [ "$d" = "$CLIENT_DOMAIN" ] && exported="$seen"
done
if [ "$exported" -eq 0 ]; then
    echo "FAIL: domain_bridge did not export ${SERVICE} into domain ${CLIENT_DOMAIN}." >&2
    echo "--- domain_bridge:"; sed 's/^/  /' "$bridgelog" >&2
    echo "--- server:"; sed 's/^/  /' "$serverlog" >&2
    exit 1
fi

echo ">>> client: calling ${SERVICE} from domain ${CLIENT_DOMAIN}"
timeout -s INT -k 5 "$CLIENT_SECS" env ROS_DOMAIN_ID="$CLIENT_DOMAIN" \
    ros2 launch agnocast_sample_application client.launch.xml > "$clientlog" 2>&1
sleep 1

wait "$server_pid" "$bridge_pid" 2>/dev/null || true

# The sample sums 1..100 and 0..99. Counting each answer apart rules out a response reaching the
# wrong caller or being reordered by the relay; one combined count would accept a duplicate.
got1=$(grep -c "Result1: 5050" "$clientlog" || true)
got2=$(grep -c "Result2: 4950" "$clientlog" || true)
served=$(grep -c "Sending back response: \[" "$serverlog" || true)
echo "  Result1=${got1} Result2=${got2}  responses sent=${served}"

if [ "$got1" -lt 1 ] || [ "$got2" -lt 1 ] || [ "$served" -lt 2 ]; then
    echo "FAIL: the call did not complete across the domain bridge." >&2
    echo "--- client:"; sed 's/^/  /' "$clientlog" >&2
    echo "--- server:"; sed 's/^/  /' "$serverlog" >&2
    echo "--- domain_bridge:"; sed 's/^/  /' "$bridgelog" >&2
    exit 1
fi

echo "PASS: ${SERVICE}@${SERVER_DOMAIN} answered a caller in domain ${CLIENT_DOMAIN} via domain_bridge."
