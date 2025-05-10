

#include "discovery.hpp"
#include "cam.hpp"

std::optional<std::filesystem::path> probe_dev(const char* serial) {
  auto v4l_links = std::filesystem::path("/dev/v4l/by-id/");
  auto iter = std::filesystem::directory_iterator(v4l_links);

  for (const auto &entry : iter) {
    // looking for symbolic links
    if (entry.is_symlink()) {
      auto link = entry.path();
      auto to_dev = std::filesystem::read_symlink(link);
      
      // match the serial number of the camera
      WARN_V4L2_BACKEND("trying match: device @%s with serial #%s", link.c_str(), serial);
      if (link.string().find(serial) != std::string::npos) {
        auto dev_path = v4l_links / to_dev;

        // query the device capabilities (looking for video capture)
        // some cameras have additional devices that are not video capture (e.g. metadata)
        struct v4l2_capability cap;
        int fd = open(dev_path.c_str(), O_RDONLY | O_NONBLOCK);

        if (fd < 0) {
          continue;
        }

        int ret = ioctl(fd, VIDIOC_QUERYCAP, &cap);
        close(fd);

        if (ret != -1) {
          // if we found the video capture device, return the file descriptor
          if (cap.device_caps & V4L2_CAP_VIDEO_CAPTURE) {
            WARN_V4L2_BACKEND("successful match: device @%s with serial #%s", dev_path.c_str(), serial);
            return { dev_path };
          }
        } else {
          WARN_V4L2_BACKEND("couldn't query device (%d): %s", errno, strerror(errno));
          continue;
        }
      }
    }
  }

  return std::nullopt;
}

std::optional<std::filesystem::path> recover_device(const char *match_serial) {
  // acquire udev handle
  struct udev *udev = udev_new();
  if (!udev) {
    WARN_V4L2_BACKEND("can't create udev instance");
    return std::nullopt;
  }

  // listen for kernel events
  struct udev_monitor *mon = udev_monitor_new_from_netlink(udev, "udev");
  udev_monitor_filter_add_match_subsystem_devtype(mon, "video4linux", NULL);
  udev_monitor_enable_receiving(mon);

  // acquire file descriptor for monitor
  int fd = udev_monitor_get_fd(mon);

  while (true) {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    int ret = select(fd+1, &fds, NULL, NULL, NULL);
    if (ret > 0 && FD_ISSET(fd, &fds)) {
      struct udev_device *vid_driver = udev_monitor_receive_device(mon);

      if (vid_driver) {
        const char *action = udev_device_get_action(vid_driver);
        const char *dev_path_c_str = udev_device_get_devnode(vid_driver);

        udev_device *usb_driver = udev_device_get_parent_with_subsystem_devtype(vid_driver, "usb", "usb_device");
        const char *serial = udev_device_get_sysattr_value(usb_driver, "serial");
        
        if (strcmp(action, "add") == 0 && strcmp(serial, match_serial) == 0) {
          struct v4l2_capability cap;
          int fd = open(dev_path_c_str, O_RDONLY | O_NONBLOCK);

          if (fd < 0) {
            udev_device_unref(vid_driver);
            close(fd);
            continue;
          }

          int ret = ioctl(fd, VIDIOC_QUERYCAP, &cap);
          close(fd);

          if (ret != -1) {
            // if we found the video capture device, return the file descriptor
            if (cap.device_caps & V4L2_CAP_VIDEO_CAPTURE) {
              auto dev_path = std::filesystem::path(dev_path_c_str);

              WARN_V4L2_BACKEND("recovered device: %s", dev_path_c_str);
              udev_device_unref(vid_driver);
              udev_unref(udev);

              return { dev_path };
            }
          }
        }

        udev_device_unref(vid_driver);
      }
    }
  }

  udev_unref(udev);
  return std::nullopt;
}