#ifndef UTIL_HPP
#define UTIL_HPP

#include <fcntl.h>

inline bool is_fd_valid(int fd) {
  if (fd < 0) return false;

  int ret = fcntl(fd, F_GETFD);
  return ret >= 0;
}

#endif