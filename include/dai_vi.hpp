#pragma once

#include "depthai/depthai.hpp"
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <optional>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace dai_vi {
enum SyncType { SOFTWARE, CAMERA, BOARD, EXTERNAL };

struct CameraConfig;
const int32_t JPEG_LOSSLESS = -1;

using CamCallback = std::function<void(std::shared_ptr<dai::ImgFrame>, const std::string &)>;
using IMUCallback = std::function<void(const dai::IMUPacket &)>;

class SensorWrapper {
public:
  SensorWrapper(const std::optional<std::string> &device_id = std::nullopt);
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
      bool color = true, std::optional<int32_t> encode = std::nullopt);
  bool addCamera(
      dai::CameraBoardSocket socket,
      std::optional<std::pair<uint32_t, uint32_t>> resolution = std::nullopt,
      std::optional<float> hz = std::nullopt,
      std::optional<std::chrono::microseconds> exposure = std::nullopt,
      std::optional<uint32_t> iso = std::nullopt,
      bool color = true, std::optional<int32_t> encode = std::nullopt);
  bool addIMU(std::vector<dai::IMUSensor> sensors, uint32_t hz);
  bool addIMU(uint32_t hz);
  void resetCamCallback(CamCallback callback = nullptr);
  void resetIMUCallback(IMUCallback callback = nullptr);
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
  std::shared_ptr<dai::MessageQueue> queue_sync;

  std::shared_ptr<dai::node::IMU> node_imu;
  std::shared_ptr<dai::MessageQueue> queue_imu;

  // Configuration
  std::unordered_set<std::string> sync_cams;
  SyncType sync_type = SyncType::SOFTWARE;
  dai::ProcessorType sync_proc = dai::ProcessorType::LEON_MSS;
  bool sync_host = false;
  bool sync_stamps = false;

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
  std::shared_mutex mtx_proc_cam;
  std::shared_mutex mtx_proc_imu;
  CamCallback fn_proc_cam;
  IMUCallback fn_proc_imu;
  std::vector<std::future<void>> sync_tasks;

#ifdef CHECK_MSGDROP
  static constexpr const double interval_threshold = 1.5;

  std::chrono::nanoseconds imu_interval_limit;
  std::chrono::nanoseconds sync_interval_limit;
  std::unordered_map<std::string, std::chrono::nanoseconds> cam_interval_limit;
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_cam_tp;
#endif
};
} // namespace dai_vi
