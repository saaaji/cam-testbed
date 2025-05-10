#include <filesystem>
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <chrono>

#include "cam.hpp"
#include "util.hpp"
#include "discovery.hpp"

#define OPTION_FAKE_CAPTURE 'f'
#define OPTION_SERIAL_CAPTURE 's'
#define OPTIONS "f:s:"

const int WIDTH = 1280;
const int HEIGHT = 720;
const char *PIXEL_FORMAT = "YUYV";

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

  int width, height;
  std::tie(width, height, std::ignore) = cap.check_format();

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
      cv::Mat raw_frame(height, width, CV_8UC2, data);
      cv::Mat frame;
      
      cv::cvtColor(raw_frame, frame, cv::COLOR_YUV2BGR_YUYV);
      cap.release();

      auto end = std::chrono::system_clock::now();
      auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
      auto fps = "fps: " + std::to_string(1000.0 / diff.count());

      cv::putText(frame, fps, {0, height - 8}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 0}, 2);
      cv::imshow("cam_testbed", frame);
      cv::waitKey(1);
    }
  }
 
  return 0;
}