#include "cam.hpp"
#include "util.hpp"

#include <libudev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <optional>
#include <chrono>

const int WIDTH = 1280;
const int HEIGHT = 720;
const char *PIXEL_FORMAT = "YUYV";

/// @brief probe /dev for video drivers
/// @return option containing path
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

/// @brief poll udev until camera has reconnected
/// @return boolean indicating whether connection was recovered
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

int main(int argc, char **argv) {
  if (argc < 2) exit(EXIT_FAILURE);

  // look for device on startup (Arducam serial# is BACK)
  VideoCapV4L cap(5);
  auto dev_path = probe_dev(argv[1]);

  // if device was found during initial probe, open the video capture
  bool has_device = false;
  if (dev_path.has_value()) {
    cap.request_format(WIDTH, HEIGHT, PIXEL_FORMAT);
    if (cap.open(dev_path.value().c_str()) >= 0) {
      has_device = true;
    }
  }

  while (true) {
    auto start = std::chrono::system_clock::now();

    // recovery sequence
    if (!has_device) {
      std::optional<std::filesystem::path> dev_path;

      // try to recover the device path upon reconnection
      do {
        dev_path = recover_device(argv[1]);
      } while (!dev_path.has_value());

      // open the capture again, with the new device mount
      cap.request_format(WIDTH, HEIGHT, PIXEL_FORMAT);
      if (cap.open(dev_path.value().c_str()) >= 0) {
        has_device = true;
      }

      // keep looking if something failed (should alter)
      if (!has_device) continue;
    }
  
    // try to read a frame
    void *data = nullptr;
    size_t size;

    int ret = cap.read(data, size);

    // if device was lost begin recovery
    if (ret == -ENODEV) {
      cv::destroyAllWindows();
      cap.free_resources();
      has_device = false;
    }

    // if we have data, do something useful with it
    if (ret == 0 && data) {
      cv::Mat raw_frame(720, 1280, CV_8UC2, data);
      cv::Mat frame;
      
      cv::cvtColor(raw_frame, frame, cv::COLOR_YUV2BGR_YUYV);
      cap.release();

      auto end = std::chrono::system_clock::now();
      auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
      auto fps = "fps: " + std::to_string(1000.0 / diff.count());

      cv::putText(frame, fps, {0, HEIGHT - 8}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 0}, 2);
      cv::imshow("cam_testbed", frame);
      cv::waitKey(1);
    }
  }

  return 0;
}