#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <cstddef>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <filesystem>
#include <sys/mman.h>
#include <string.h>

#include "cam.hpp"
#include "util.hpp"

int VideoCapV4L::free_resources() {
  if (has_buf_lock) {
    WARN_V4L2_BACKEND("requeuing buffer");
    try_v4l2_ioctl(VIDIOC_QBUF, &locked_buf);
    has_buf_lock = false;
  }

  if (has_stream) {
    WARN_V4L2_BACKEND("ending stream");
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    try_v4l2_ioctl(VIDIOC_STREAMOFF, &type);
    has_stream = false;
  }

  if (has_mmap) {
    WARN_V4L2_BACKEND("deallocating mmap");

    for (int i = 0; i < buf_count; i++) {
      int ret = munmap(mapped_offsets[i], mapped_size[i]);
      if (ret == -1) {
        WARN_V4L2_BACKEND("munmap failed (%d): %s", errno, strerror(errno));
      }
    }

    has_mmap = false;
  }

  if (is_fd_valid(fd)) {
    WARN_V4L2_BACKEND("closing video device");
    close(fd);
  }

  return 0;
}

int VideoCapV4L::try_v4l2_ioctl(int op, void *request) {
  int ret, should_retry;

  do {
    ret = ioctl(fd, op, request);

    // reattempt system call if interrupted or blocked
    should_retry = ret == -1 && (errno == EINTR || errno == EAGAIN);
  } while (should_retry);
  
  if (ret == -1 && errno != EINVAL) {
    WARN_V4L2_BACKEND("request failed (%d): %s", errno, strerror(errno));
  }

  return ret;
}

int VideoCapV4L::request_format(int width, int height, const char *pixel_format) {
  if (strlen(pixel_format) != 4) {
    WARN_V4L2_BACKEND("invalid pixel format: %s", pixel_format);
    return -1;
  }

  requested_format.pixel_format = v4l2_fourcc(pixel_format[0], pixel_format[1], pixel_format[2], pixel_format[3]);
  requested_format.width = width;
  requested_format.height = height;

  return 0;
}

std::tuple<int, int, std::string> VideoCapV4L::check_format() {
  char str[5];
  
  str[0] = (requested_format.actual_pixel_format << 0) & 0xff;
  str[1] = (requested_format.actual_pixel_format << 8) & 0xff;
  str[2] = (requested_format.actual_pixel_format << 16) & 0xff;
  str[3] = (requested_format.actual_pixel_format << 24) & 0xff;
  str[4] = '\0';
  

  return {
    requested_format.actual_width,
    requested_format.actual_height,
    str
  };
}

int VideoCapV4L::open(const char *dev_path) {
  free_resources();

  // try to open device
  fd = ::open(dev_path, O_RDWR | O_NONBLOCK);
  
  if (fd == -1) {
    WARN_V4L2_BACKEND("failed to open '%s'", dev_path);
    return -1;
  }

  // configure pixel format
  v4l2_format fmt_out = {0};
  fmt_out.type = V4L2_CAP_VIDEO_CAPTURE;

  if (try_v4l2_ioctl(VIDIOC_G_FMT, &fmt_out) == -1) {
    WARN_V4L2_BACKEND("could not get current format");
    return -1;
  }

  WARN_V4L2_BACKEND("Requested format: %dx%d, %u", requested_format.width, requested_format.height, requested_format.pixel_format);
  WARN_V4L2_BACKEND("Supported Image Formats");
#ifdef ENABLE_LOGGING_V4L2_BACKEND
  for (int i = 0, ret = 0; ret != -1; i++) {
    v4l2_fmtdesc desc = {0};  
    desc.index = i;
    desc.type = V4L2_CAP_VIDEO_CAPTURE;

    ret = try_v4l2_ioctl(VIDIOC_ENUM_FMT, &desc);
    if (ret != -1) {
      char str[5];

      str[0] = (desc.pixelformat >> 0) & 0xff;
      str[1] = (desc.pixelformat >> 8) & 0xff;
      str[2] = (desc.pixelformat >> 16) & 0xff;
      str[3] = (desc.pixelformat >> 24) & 0xff;
      str[4] = '\0';

      printf("\t|__ Enumerated Format: %s (%s)\n", desc.description, str);

      if (desc.flags & V4L2_FMT_FLAG_COMPRESSED) {
        printf("\t\t|__ V4L2_FMT_FLAG_COMPRESSED\n");
      }

      printf("\t\t|__ Frame Sizes\n");
      for (int j = 0, ret = 0; ret != -1; j++) {
        v4l2_frmsizeenum frmsize = {0};
        frmsize.index = j;
        frmsize.pixel_format = desc.pixelformat;

        ret = try_v4l2_ioctl(VIDIOC_ENUM_FRAMESIZES, &frmsize);
        if (ret != -1) {
          switch (frmsize.type) {
            case v4l2_frmsizetypes::V4L2_FRMSIZE_TYPE_CONTINUOUS:
              printf("\t\t\t|__ V4L2_FRMSIZE_TYPE_CONTINUOUS");
              break;
            case v4l2_frmsizetypes::V4L2_FRMSIZE_TYPE_DISCRETE:
              printf("\t\t\t|__ [%dx%d]", frmsize.discrete.width, frmsize.discrete.height);
              break;
            case v4l2_frmsizetypes::V4L2_FRMSIZE_TYPE_STEPWISE:
              printf("\t\t\t|__ [%dx%d] -> [%dx%d], d[%dx%d]", 
                frmsize.stepwise.min_width, 
                frmsize.stepwise.min_height,
                frmsize.stepwise.max_width, 
                frmsize.stepwise.max_height,
                frmsize.stepwise.step_width, 
                frmsize.stepwise.step_height);
              break;
          }

          if (desc.pixelformat == requested_format.pixel_format) {
            if (frmsize.type == v4l2_frmsizetypes::V4L2_FRMSIZE_TYPE_DISCRETE) {
              if (frmsize.discrete.width == requested_format.width && frmsize.discrete.height == requested_format.height) {
                printf(" (Requested)\n");
                continue;
              }
            }
          } else if (desc.pixelformat == fmt_out.fmt.pix.pixelformat) {
            if (frmsize.type == v4l2_frmsizetypes::V4L2_FRMSIZE_TYPE_DISCRETE) {
              if (frmsize.discrete.width == fmt_out.fmt.pix.width && frmsize.discrete.height == fmt_out.fmt.pix.height) {
                printf(" (Current)\n");
                continue;
              }
            }
          }
          printf("\n");
        }
      }
    }
  }
#endif

  if (requested_format.pixel_format > 0) {
    v4l2_format fmt_in = {0};
    fmt_in.type = V4L2_CAP_VIDEO_CAPTURE;
    fmt_in.fmt.pix.width = requested_format.width;
    fmt_in.fmt.pix.height = requested_format.height;
    fmt_in.fmt.pix.pixelformat = requested_format.pixel_format;
    if (try_v4l2_ioctl(VIDIOC_S_FMT, &fmt_in) == -1) {
      WARN_V4L2_BACKEND("could not set format");
      return -1;
    }

    if (fmt_in.fmt.pix.width != requested_format.width || fmt_in.fmt.pix.height != requested_format.height) {
      WARN_V4L2_BACKEND("frame size was adjusted: [%dx%d] -> [%dx%d]", 
        requested_format.width,
        requested_format.height,
        fmt_in.fmt.pix.width, 
        fmt_in.fmt.pix.height);
    }

    requested_format.actual_width = fmt_in.fmt.pix.width;
    requested_format.actual_height = fmt_in.fmt.pix.height;
    requested_format.actual_pixel_format = fmt_in.fmt.pix.pixelformat;
  }

  v4l2_requestbuffers buf_req = {0};

  // request memory mapped buffers
  buf_req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf_req.memory = V4L2_MEMORY_MMAP;
  buf_req.count = buf_count;

  if (try_v4l2_ioctl(VIDIOC_REQBUFS, &buf_req) == -1 || buf_req.count < buf_count) {
    WARN_V4L2_BACKEND("could not request buffers: (%d/%d received)", buf_req.count, buf_count);
    WARN_V4L2_BACKEND("downgrading to %d buffers", buf_req.count);

    buf_count = buf_req.count;

    // not sure if this should fail or not
    // return -1;
  }

  // map buffers
  for (int i = 0; i < buf_count; i++) {
    v4l2_buffer info = {0};

    // retrieve buffer info
    info.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    info.memory = V4L2_MEMORY_MMAP;
    info.index = i;

    if (try_v4l2_ioctl(VIDIOC_QUERYBUF, &info) == -1) {
      WARN_V4L2_BACKEND("could not query buffer@%d", i);
      return -1;
    }

    mapped_size[i] = info.length;
    mapped_offsets[i] = mmap(
      NULL,
      mapped_size[i],
      PROT_READ | PROT_WRITE,
      MAP_SHARED,
      fd,
      info.m.offset // offset for device
    );

    if (mapped_offsets[i] == MAP_FAILED) {
      WARN_V4L2_BACKEND("could not map region for buffer@%d", i);
      return -1;
    }
  }

  has_mmap = true;

  // start camera stream
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (try_v4l2_ioctl(VIDIOC_STREAMON, &type) == -1) {
    WARN_V4L2_BACKEND("could not turn stream on");
    return -1;
  }

  has_stream = true;

  // queue all buffers
  for (int i = 0; i < buf_count; i++) {
    v4l2_buffer info = {0};

    // queue buffer
    info.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    info.memory = V4L2_MEMORY_MMAP;
    info.index = i;

    if (try_v4l2_ioctl(VIDIOC_QBUF, &info) == -1) {
      WARN_V4L2_BACKEND("could not queue buffer@%d", i);
      return -1;
    }
  }

  return 0;
}

int VideoCapV4L::release() {
    if (has_buf_lock) {
      if (try_v4l2_ioctl(VIDIOC_QBUF, &locked_buf) == -1) {
        WARN_V4L2_BACKEND("could not queue (unlock) buffer@%d", locked_buf.index);
        return -1;
      }
      has_buf_lock = false;
    } else {
      WARN_V4L2_BACKEND("attempted unlock more than once");
    }

    return 0;
}

int VideoCapV4L::read(void *&data, size_t &size) {
  if (has_buf_lock) {
    WARN_V4L2_BACKEND("forgot to unlock buffer");
  }

  v4l2_buffer info = {0};
  
  // attempt to dequeue buffer
  info.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  info.memory = V4L2_MEMORY_MMAP;

  if (try_v4l2_ioctl(VIDIOC_DQBUF, &info) == -1) {
    WARN_V4L2_BACKEND("could not dequeue buffer");
    if (errno == ENODEV) {
      WARN_V4L2_BACKEND("device was lost, capture soiled (must destroy and reopen)");
    }
    return -errno;
  }

  // try to dequeue as many buffers as possible
  int ret = 1;
  while (ret) {
    // poll handle
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    
    timeval timeout = {
      .tv_sec = 0,
      .tv_usec = 0
    };

    ret = select(fd + 1, &fds, NULL, NULL, &timeout);
    if (ret == 1) {
      // if next buffer available (queued), dequeue it
      int q = try_v4l2_ioctl(VIDIOC_QBUF, &info);
      int d = try_v4l2_ioctl(VIDIOC_DQBUF, &info);

      WARN_V4L2_BACKEND("attempting enq/deq");
      if (q == -1 || d == -1) {
        WARN_V4L2_BACKEND("sequential enq/deq failed");
        return -1;
      }
    } else if (ret == -1) {
      WARN_V4L2_BACKEND("could not poll device");
      return -1;
    }
  }

  // return data
  data = mapped_offsets[info.index];
  size = info.bytesused;

  locked_buf = info;
  has_buf_lock = true;

  return 0;
}