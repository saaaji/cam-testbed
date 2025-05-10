#include <libudev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>
#include <filesystem>
#include <optional>
#include <fcntl.h>

/// @brief probe /dev for video drivers
/// @return option containing path
std::optional<std::filesystem::path> probe_dev(const char* serial);

/// @brief poll udev until camera has reconnected
/// @return boolean indicating whether connection was recovered
std::optional<std::filesystem::path> recover_device(const char *match_serial);