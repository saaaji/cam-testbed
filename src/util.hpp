#pragma once

#include <fcntl.h>

#define UNIMPLEMENTED() do { \
    fprintf(stderr, "UNIMPLEMENTED: %s:%d %s()\n", __FILE__, __LINE__, __func__); \
    abort(); \
} while (0)

inline bool is_fd_valid(int fd) {
  if (fd < 0) return false;

  int ret = fcntl(fd, F_GETFD);
  return ret >= 0;
}