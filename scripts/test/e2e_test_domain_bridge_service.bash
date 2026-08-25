#!/bin/bash
# Cross-domain zero-copy e2e for a *service*: sample server in one domain, sample clients in the
# other, one IPC namespace, no DDS and no bridge node. One `services:` entry becomes an exact rule
# for /AGNOCAST_SRV_REQUEST<svc> and a prefix rule for /AGNOCAST_SRV_RESPONSE<svc>%, whose
# per-client names exist only at runtime.
#
# The cross-domain client and a same-named client in the server's domain run CONCURRENTLY: only
# while both are live can a shared response topic show itself, as "bad entry id" in both logs.
#
# Two exchanges run in turn, plain and renamed, with different service names so neither rule
# claims a cell of the other. Run on a freshly loaded kmod: a rule must precede its endpoints.

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

SERVICE="${E2E_SERVICE_NAME:-/srv/sum_int_array}"      # plain exchange: one name for both sides
RENAMED_FROM=/srv/renamed_call                         # rename exchange: what the clients call
RENAMED_TO=/srv/renamed_offer                          # ... and what the server offers
CLIENT_DOMAIN="${E2E_CLIENT_DOMAIN:-2}"
SERVER_DOMAIN="${E2E_SERVER_DOMAIN:-1}"
RUN_SECONDS="${E2E_RUN_SECONDS:-10}"

# minimal_client.cpp sends 1..100 and 0..99 every run. Assert the sums, not just the count, so a
# response delivered to the wrong client cannot pass as this one's own.
SUM1=5050
SUM2=4950

CLIENT_SECS=$((RUN_SECONDS + 3))       # one client: run + drain
CLIENT_RUNS=3                          # cross-domain, its same-domain twin, cross-domain again
CLIENT_PHASES=2                        # the twin shares its phase with the first cross-domain run
SERVER_SECS=$((5 + CLIENT_PHASES * CLIENT_SECS + 3))

# Both services in one config, the way an operator would write it, and registered before any node
# starts: the kmod rejects a rule once an endpoint for its cells exists.
cfg="$(mktemp --suffix=.yaml)"
{
    echo "from_domain: ${CLIENT_DOMAIN}"
    echo "to_domain: ${SERVER_DOMAIN}"
    echo "services:"
    echo "  \"${SERVICE}\":"
    echo "  \"${RENAMED_FROM}\":"
    echo "    remap: \"${RENAMED_TO}\""
} > "$cfg"

echo ">>> registering ${SERVICE} and ${RENAMED_FROM} -> ${RENAMED_TO}" \
     "(clients@${CLIENT_DOMAIN}, server@${SERVER_DOMAIN})"
# Register through the same tool production uses, not an inline ioctl call.
if ! ros2 run ros2agnocast_discovery_agent register_domain_bridge --config "$cfg"; then
    echo "Rule registration failed (fresh kmod? a covered endpoint already exists?)." >&2
    rm -f "$cfg"; exit 1
fi
rm -f "$cfg"

# make_launch <exec> <node name> <service name>; echoes the generated launch file path. The
# packaged launches hardcode one service name, and the rename path needs two.
make_launch() {
    local f; f="$(mktemp --suffix=.launch.xml)"
    cat > "$f" <<XML
<launch>
  <node pkg="agnocast_sample_application" exec="$1" name="$2" output="screen">
    <remap from="/sum_int_array" to="$3" />
    <env name="LD_PRELOAD" value="libagnocast_heaphook.so:\$(env LD_PRELOAD '')" />
  </node>
</launch>
XML
    echo "$f"
}

# start_client <launch> <domain> <service> <label> <log>; leaves the pid in CLIENT_PID. Assigned
# rather than echoed: a command substitution would background the job in an unwaitable subshell.
start_client() {
    local launch="$1" domain="$2" svc="$3" label="$4" log="$5"
    echo ">>> client (${label}): calling ${svc} from domain ${domain} (~${CLIENT_SECS}s)"
    timeout -s INT -k 5 "$CLIENT_SECS" \
        env AGNOCAST_BRIDGE_MODE=off ROS_DOMAIN_ID="$domain" \
        ros2 launch "$launch" > "$log" 2>&1 &
    CLIENT_PID=$!
}

# check_client <log> <label>; 0 if both responses arrived with the sums this client asked for.
check_client() {
    local log="$1" label="$2" ret=0
    local got1 got2 bad_entry
    got1=$(grep -cE "Result1: ${SUM1}\$" "$log" || true)
    got2=$(grep -cE "Result2: ${SUM2}\$" "$log" || true)
    bad_entry=$(grep -c "bad entry id" "$log" || true)
    echo "  ${label}: Result1=${SUM1}x${got1} Result2=${SUM2}x${got2} bad_entry_id=${bad_entry}"
    if [ "$got1" -lt 1 ] || [ "$got2" -lt 1 ]; then
        echo "  FAIL (${label}): expected Result1: ${SUM1} and Result2: ${SUM2}." >&2
        ret=1
    fi
    if [ "$bad_entry" -ne 0 ]; then
        echo "  FAIL (${label}): a response could not be matched to its request." >&2
        ret=1
    fi
    [ "$ret" -ne 0 ] && sed 's/^/    /' "$log" >&2
    return "$ret"
}

# run_exchange <label> <client service> <server service>; 0 if every client got its own answers.
run_exchange() {
    local label="$1" client_service="$2" server_service="$3" bad=0
    local server_launch client_launch twin_launch serverlog
    local cross_log twin_log again_log cross_pid server_pid served1 served2

    server_launch="$(make_launch server server_node "$server_service")"
    client_launch="$(make_launch client client_node "$client_service")"
    # The twin shares the client's node name but calls the server in its own domain. Under a
    # rename that name is outside the bridged response prefix, so it only checks coexistence.
    twin_launch="$(make_launch client client_node "$server_service")"

    serverlog="$(mktemp)"
    echo ">>> [${label}] server: ${server_service}@${SERVER_DOMAIN} (~${SERVER_SECS}s)"
    timeout -s INT -k 5 "$SERVER_SECS" \
        env AGNOCAST_BRIDGE_MODE=off ROS_DOMAIN_ID="$SERVER_DOMAIN" \
        ros2 launch "$server_launch" > "$serverlog" 2>&1 &
    server_pid=$!
    sleep 5

    cross_log="$(mktemp)"; twin_log="$(mktemp)"; again_log="$(mktemp)"

    # Same node name, one client in each bridged domain, both alive at once.
    start_client "$client_launch" "$CLIENT_DOMAIN" "$client_service" "${label} cross-domain" \
        "$cross_log"
    cross_pid="$CLIENT_PID"
    start_client "$twin_launch" "$SERVER_DOMAIN" "$server_service" "${label} same-domain twin" \
        "$twin_log"
    wait "$cross_pid" "$CLIENT_PID" 2>/dev/null || true
    check_client "$cross_log" "${label} cross-domain" || bad=$((bad + 1))
    check_client "$twin_log" "${label} same-domain twin" || bad=$((bad + 1))

    start_client "$client_launch" "$CLIENT_DOMAIN" "$client_service" "${label} cross-domain again" \
        "$again_log"
    wait "$CLIENT_PID" 2>/dev/null || true
    check_client "$again_log" "${label} cross-domain again" || bad=$((bad + 1))

    wait "$server_pid" 2>/dev/null || true
    sleep 1

    served1=$(grep -c "Sending back response: \[${SUM1}\]" "$serverlog" || true)
    served2=$(grep -c "Sending back response: \[${SUM2}\]" "$serverlog" || true)
    echo "  [${label}] server: [${SUM1}]x${served1} [${SUM2}]x${served2} (expected ${CLIENT_RUNS} each)"
    if [ "$served1" -lt "$CLIENT_RUNS" ] || [ "$served2" -lt "$CLIENT_RUNS" ]; then
        echo "  FAIL (${label}): server served ${served1}/${served2} of the ${CLIENT_RUNS} of each kind." >&2
        sed 's/^/    /' "$serverlog" >&2
        bad=$((bad + 1))
    fi

    rm -f "$serverlog" "$cross_log" "$twin_log" "$again_log" \
        "$server_launch" "$client_launch" "$twin_launch"

    if [ "$bad" -ne 0 ]; then
        echo "FAIL (${label}): ${bad} check(s) failed." >&2
        return 1
    fi
    echo "PASS (${label}): ${server_service}@${SERVER_DOMAIN} served clients calling ${client_service}."
    return 0
}

failed=0
run_exchange plain "$SERVICE" "$SERVICE" || failed=$((failed + 1))
run_exchange rename "$RENAMED_FROM" "$RENAMED_TO" || failed=$((failed + 1))

if [ "$failed" -ne 0 ]; then
    echo "FAIL: ${failed} of 2 exchange(s) failed." >&2
    exit 1
fi
echo "PASS: the plain and the renamed service both bridged domains ${CLIENT_DOMAIN} and ${SERVER_DOMAIN}."
