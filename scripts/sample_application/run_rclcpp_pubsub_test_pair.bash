#!/bin/bash

set -eo pipefail

source install/setup.bash

set -u

TOPIC_COUNT="${1:-5}"
DEFAULT_PAIRS_PER_TYPE="${2:-50}"
RCLCPP_PAIRS="${3:-${DEFAULT_PAIRS_PER_TYPE}}"
MIXED_A2R_PAIRS="${4:-${DEFAULT_PAIRS_PER_TYPE}}"
MIXED_R2A_PAIRS="${5:-${DEFAULT_PAIRS_PER_TYPE}}"
AGNOCAST_PAIRS="${6:-${DEFAULT_PAIRS_PER_TYPE}}"
CROSS_PAIRS="${7:-${DEFAULT_PAIRS_PER_TYPE}}"

PIDS=()

start_node() {
  local launch_file="$1"
  local node_name="$2"
  local pub_prefix="$3"
  local sub_prefix="$4"

  ros2 launch agnocast_sample_application "${launch_file}" \
    node_name:="${node_name}" \
    topic_count:="${TOPIC_COUNT}" \
    pub_topic_prefix:="${pub_prefix}" \
    sub_topic_prefix:="${sub_prefix}" &
  PIDS+=("$!")
}

start_pair() {
  local pair_type="$1"
  local index="$2"
  local launch_file_a="$3"
  local launch_file_b="$4"
  local node_a_prefix="$5"
  local node_b_prefix="$6"

  local topic_prefix_ab="/pair_${pair_type}_${index}_a_to_b"
  local topic_prefix_ba="/pair_${pair_type}_${index}_b_to_a"

  local node_a_name="${node_a_prefix}_${index}_a"
  local node_b_name="${node_b_prefix}_${index}_b"

  start_node "${launch_file_a}" "${node_a_name}" "${topic_prefix_ab}" "${topic_prefix_ba}"
  start_node "${launch_file_b}" "${node_b_name}" "${topic_prefix_ba}" "${topic_prefix_ab}"
}

for ((i = 0; i < RCLCPP_PAIRS; ++i)); do
  start_pair \
    "rclcpp" "${i}" \
    "rclcpp_pubsub_test.launch.xml" "rclcpp_pubsub_test.launch.xml" \
    "rclcpp_pubsub_test" "rclcpp_pubsub_test"
done

for ((i = 0; i < MIXED_A2R_PAIRS; ++i)); do
  start_pair \
    "mixed_a2r" "${i}" \
    "agnocast_publisher_only_pubsub_test.launch.xml" "agnocast_publisher_only_pubsub_test.launch.xml" \
    "agnocast_publisher_only_pubsub_test" "agnocast_publisher_only_pubsub_test"
done

for ((i = 0; i < MIXED_R2A_PAIRS; ++i)); do
  start_pair \
    "mixed_r2a" "${i}" \
    "agnocast_subscriber_only_pubsub_test.launch.xml" "agnocast_subscriber_only_pubsub_test.launch.xml" \
    "agnocast_subscriber_only_pubsub_test" "agnocast_subscriber_only_pubsub_test"
done

for ((i = 0; i < AGNOCAST_PAIRS; ++i)); do
  start_pair \
    "agnocast" "${i}" \
    "agnocast_pubsub_test.launch.xml" "agnocast_pubsub_test.launch.xml" \
    "agnocast_pubsub_test" "agnocast_pubsub_test"
done

for ((i = 0; i < CROSS_PAIRS; ++i)); do
  start_pair \
    "cross" "${i}" \
    "rclcpp_pubsub_test.launch.xml" "agnocast_pubsub_test.launch.xml" \
    "rclcpp_pubsub_test_cross" "agnocast_pubsub_test_cross"
done

cleanup() {
  if [[ ${#PIDS[@]} -gt 0 ]]; then
    kill "${PIDS[@]}" 2>/dev/null || true
    wait "${PIDS[@]}" 2>/dev/null || true
  fi
}

trap cleanup INT TERM EXIT

TOTAL_PAIRS=$((RCLCPP_PAIRS + MIXED_A2R_PAIRS + MIXED_R2A_PAIRS + AGNOCAST_PAIRS + CROSS_PAIRS))
echo "Started rclcpp pairs: ${RCLCPP_PAIRS}"
echo "Started mixed_a2r pairs: ${MIXED_A2R_PAIRS}"
echo "Started mixed_r2a pairs: ${MIXED_R2A_PAIRS}"
echo "Started agnocast pairs: ${AGNOCAST_PAIRS}"
echo "Started cross pairs: ${CROSS_PAIRS}"
echo "Total pairs: ${TOTAL_PAIRS}, total nodes: $((TOTAL_PAIRS * 2)), topic_count=${TOPIC_COUNT}"

wait "${PIDS[@]}"
