#pragma once

#include "depthai/depthai.hpp"
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace dai_vi {
enum SyncType { SOFTWARE, CAMERA, BOARD };

struct CameraConfig;

class SensorWrapper {
public:
  SensorWrapper(const std::optional<std::string> &device_id = std::nullopt,
                dai::LogLevel dai_log_level = dai::LogLevel::WARN);
  SensorWrapper(dai::LogLevel dai_log_level)
      : SensorWrapper(std::nullopt, dai_log_level) {};
  SensorWrapper(const SensorWrapper &) = delete;
  SensorWrapper &operator=(const SensorWrapper &) = delete;
  ~SensorWrapper();

  // Pipeline construction
  bool addCamera(
      const std::string &name, dai::CameraBoardSocket socket,
      std::optional<std::pair<uint32_t, uint32_t>> resolution = std::nullopt,
      std::optional<float> hz = std::nullopt,
      std::optional<std::chrono::microseconds> exposure = std::nullopt,
      std::optional<uint32_t> iso = std::nullopt,
      bool color = true, bool encode = false);
  bool addCamera(
      dai::CameraBoardSocket socket,
      std::optional<std::pair<uint32_t, uint32_t>> resolution = std::nullopt,
      std::optional<float> hz = std::nullopt,
      std::optional<std::chrono::microseconds> exposure = std::nullopt,
      std::optional<uint32_t> iso = std::nullopt,
      bool color = true, bool encode = false);
  bool addIMU(std::vector<dai::IMUSensor> sensors, uint32_t hz);
  bool addIMU(uint32_t hz);
  bool buildPipeline();

  // Pipeline control
  void start();
  void stop();

  // Pipeline
  std::unique_ptr<dai::Pipeline> pipeline;

  std::unordered_map<std::string, std::shared_ptr<dai::node::Camera>> node_cam;
  std::unordered_map<std::string, std::shared_ptr<dai::node::VideoEncoder>> node_enc;
  std::shared_ptr<dai::node::Sync> node_sync;
  std::shared_ptr<dai::node::Script> node_fsync;
  std::unordered_map<std::string, std::shared_ptr<dai::MessageQueue>> queue_cam;

  std::shared_ptr<dai::node::IMU> node_imu;
  std::shared_ptr<dai::MessageQueue> queue_imu;

  std::function<void(std::shared_ptr<dai::ImgFrame>, const std::string &)> fn_proc_cam;
  std::function<void(const dai::IMUPacket &)> fn_proc_imu;

  // Configuration
  std::unordered_set<std::string> sync_cams;
  SyncType sync_type = SyncType::BOARD;
  dai::ProcessorType sync_proc = dai::ProcessorType::LEON_MSS;

private:
  void proc_synced(std::shared_ptr<dai::MessageGroup> msgpack);
  void proc_cam(std::shared_ptr<dai::ImgFrame> img, const std::string &name);
  void proc_imu(std::shared_ptr<dai::IMUData> msg);

  std::optional<int> board_revision;
  std::unordered_map<std::string, CameraConfig> cams;
  std::vector<dai::IMUSensor> imu_sensors;
  uint32_t imu_hz = 0;
  std::chrono::duration<double> imu_interval;
  std::chrono::duration<double> sync_interval;
  std::unordered_map<std::string, std::chrono::duration<double>> cam_interval;

#ifdef CHECK_MSGDROP
  static constexpr const double interval_threshold = 1.5;

  std::chrono::nanoseconds imu_interval_limit;
  std::chrono::nanoseconds sync_interval_limit;
  std::unordered_map<std::string, std::chrono::nanoseconds> cam_interval_limit;
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_cam_tp;
#endif
};
} // namespace dai_vi
