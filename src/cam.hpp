#pragma once

#include <linux/videodev2.h>
#include <cstddef>
#include <cstdint>

#define BUF_COUNT_V4L2_BACKEND 5
#define ENABLE_LOGGING_V4L2_BACKEND

#ifdef ENABLE_LOGGING_V4L2_BACKEND
  #define WARN_V4L2_BACKEND(...) {\
    printf("[VideoCapV4L %d] ", __LINE__);\
    printf(__VA_ARGS__);\
    printf("\n");\
  }
#else
  #define WARN_V4L2_BACKEND(...) ((void)0)
#endif

/// @brief class for real-time video capture via Video4Linux
class VideoCapV4L {
  public:
    /// @brief constructor for video capture
    /// @param buf_count number of frame buffers to request from V4L
    VideoCapV4L(int buf_count): buf_count(buf_count) {}

    /// @brief default destructor (call destroy to free resources)
    ~VideoCapV4L() = default;
    
    /// @brief open video device, allocate frame buffers, and begin streaming
    /// @param dev_path 
    /// @return 0 on success and -1 on error (enable logging for detailed error information)
    int open(const char *dev_path);

    /// @brief release currently held frame buffer (enqueue it again)
    /// @return 0 on success and -1 on error (enable logging for detailed error information)
    int release();

    /// @brief read raw data from next available frame
    /// @param data empty pointer to frame data (in memory mapped region)
    /// @param size size of the frame
    /// @return 0 on success and -1 on error (enable logging for detailed error information)
    int read(void *&data, size_t &size);

    /// @brief free all resources associated with the current capture
    /// @return 0 on success or -1 on error (enable logging for detailed error information) 
    int free_resources();

    /// @brief set desired image format before initialization
    /// @param width width of image
    /// @param height height of image
    /// @param pixel_format format of image (fourcc)
    /// @return 0 on success and -1 on error (enable logging for detailed error information)
    int request_format(int width, int height, const char *pixel_format);

    /// @brief get format after opening camera
    /// @param width integer to output the width of the capture
    /// @param height integer to output the height of the capture
    /// @param pixel_format string to output the pixel format
    std::tuple<int, int, std::string> check_format();

    /// @brief get the video device file descriptor
    /// @return file descriptor of video device
    int get_fd() { return fd; }

  private:
    /// @brief file descriptor for video device (/dev/video<n>)
    int fd = -1;

    /// @brief requested buffer count
    int buf_count;
    
    /// @brief currently held frame buffer
    v4l2_buffer locked_buf = {0};
    
    /// @brief pointers to memory mapped regions for each frame buffer
    void *mapped_offsets[BUF_COUNT_V4L2_BACKEND];

    /// @brief sizes of memory mapped regions for each frame buffer
    size_t mapped_size[BUF_COUNT_V4L2_BACKEND];
    
    /// @brief boolean indicating whether a frame buffer is currently being held
    bool has_buf_lock = false;

    /// @brief boolean indicating whether frame buffers have been memory mapped
    bool has_mmap = false;

    /// @brief boolean indicating whether stream was started
    bool has_stream = false;

    struct {
      int width = -1;
      int height = -1;
      uint32_t pixel_format = 0;
      int actual_width = -1;
      int actual_height = -1;
      uint32_t actual_pixel_format = 0;
    } requested_format;

    /// @brief wrapper for ioctl to Video4Linux, with error checking
    /// @param op system call ID
    /// @param request request to pass to system call
    /// @return 0 on success and -1 on error (enable logging for detailed error information)
    int try_v4l2_ioctl(int op, void *request);
};