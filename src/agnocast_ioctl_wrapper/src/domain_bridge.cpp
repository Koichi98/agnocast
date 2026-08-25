#include "agnocast_ioctl.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>

extern "C" {

// Register a domain bridge rule with the kmod so it relays the topic from
// (topic_name_from, from_domain) to (topic_name_to, to_domain) within this
// process's IPC namespace. The two names may differ (rename) or be equal (plain
// bridge). Returns 0 on success, -1 on failure.
int add_agnocast_domain_bridge_rule(
  const char * topic_name_from, const char * topic_name_to, uint32_t from_domain,
  uint32_t to_domain)
{
  // Exported C symbol: guard the pointers so a null from a caller returns an
  // error instead of segfaulting in strlen() below.
  if (topic_name_from == nullptr || topic_name_to == nullptr) {
    errno = EINVAL;
    return -1;
  }

  int fd = open("/dev/agnocast", O_RDONLY);
  if (fd < 0) {
    if (errno == ENOENT) {
      fprintf(stderr, "%s", AGNOCAST_DEVICE_NOT_FOUND_MSG);
    } else {
      perror("Failed to open /dev/agnocast");
    }
    return -1;
  }

  ioctl_add_domain_bridge_args args = {};
  args.topic_name_from = {topic_name_from, strlen(topic_name_from)};
  args.topic_name_to = {topic_name_to, strlen(topic_name_to)};
  args.from_domain = from_domain;
  args.to_domain = to_domain;

  if (ioctl(fd, AGNOCAST_ADD_DOMAIN_BRIDGE_CMD, &args) < 0) {
    perror("AGNOCAST_ADD_DOMAIN_BRIDGE_CMD failed");
    close(fd);
    return -1;
  }

  close(fd);
  return 0;
}

// Remove the domain bridge rule covering the cell (topic_name, domain_id) in this
// process's IPC namespace. The cell may be either side of the pair, named by that
// side's own topic name, and the whole rule (both directions) goes at once.
// Returns 0 on success, -1 on failure; errno is ENOENT when no rule covers the
// cell and EBUSY when an endpoint has still joined either side.
int remove_agnocast_domain_bridge_rule(const char * topic_name, uint32_t domain_id)
{
  if (topic_name == nullptr) {
    errno = EINVAL;
    return -1;
  }

  int fd = open("/dev/agnocast", O_RDONLY);
  if (fd < 0) {
    if (errno == ENOENT) {
      fprintf(stderr, "%s", AGNOCAST_DEVICE_NOT_FOUND_MSG);
    } else {
      perror("Failed to open /dev/agnocast");
    }
    return -1;
  }

  ioctl_remove_domain_bridge_args args = {};
  args.topic_name = {topic_name, strlen(topic_name)};
  args.domain_id = domain_id;

  if (ioctl(fd, AGNOCAST_REMOVE_DOMAIN_BRIDGE_CMD, &args) < 0) {
    // Preserved across close(), which may otherwise overwrite errno, so the
    // caller can distinguish ENOENT from EBUSY.
    const int saved_errno = errno;
    perror("AGNOCAST_REMOVE_DOMAIN_BRIDGE_CMD failed");
    close(fd);
    errno = saved_errno;
    return -1;
  }

  close(fd);
  return 0;
}
}
