#include "dai_vi.hpp"
#include "device_scripts.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <ratio>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "depthai/depthai.hpp"
#include "spdlog/cfg/env.h"
#include "spdlog/spdlog.h"

namespace dai_vi {

struct CameraConfig {
  dai::CameraBoardSocket socket;
  std::optional<std::pair<uint32_t, uint32_t>> resolution;
  std::optional<float> hz;
  std::optional<std::chrono::microseconds> exposure;
  std::optional<uint32_t> iso;
  bool color;
  std::optional<int32_t> encode;
};

static dai::LogLevel log_level_from_env() {
  const char *env = std::getenv("DEPTHAI_DEVICE_LEVEL");
  if (!env) return dai::LogLevel::WARN;
  std::string upper(env);
  std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);
  if (upper == "TRACE")    return dai::LogLevel::TRACE;
  if (upper == "DEBUG")    return dai::LogLevel::DEBUG;
  if (upper == "INFO")     return dai::LogLevel::INFO;
  if (upper == "WARN")     return dai::LogLevel::WARN;
  if (upper == "ERR")      return dai::LogLevel::ERR;
  if (upper == "CRITICAL") return dai::LogLevel::CRITICAL;
  if (upper == "OFF")      return dai::LogLevel::OFF;
  spdlog::warn("Unknown DEPTHAI_DEVICE_LEVEL value '{}', defaulting to WARN", env);
  return dai::LogLevel::WARN;
}

static std::optional<int> detect_board_revision(const std::unique_ptr<dai::Device> &dev) {
  try {
    // Parse board revision to determine GPIO pinout
    const auto dev_data = dev->readCalibration2().getEepromData();
    spdlog::debug("Product name: {}, Board name: {}, Board revision: {}", dev_data.productName, dev_data.boardName, dev_data.boardRev);
    if (dev_data.productName != "OAK-FFC-4P" || dev_data.boardName != "DD2090") {
      throw std::runtime_error("Unsupported product/board");
    }
    if (dev_data.boardRev.length() < 2 || dev_data.boardRev[0] != 'R') {
      throw std::runtime_error("Failed to parse revision number");
    }
    const auto board_revision = std::stoi(dev_data.boardRev.substr(1, 2));
    spdlog::debug("Parsed board revision: {}", board_revision);

    return board_revision;
  } catch (const std::runtime_error &e) {
    spdlog::warn("Failed to detect board revision: {}", e.what());
    return std::nullopt;
  }
}

SensorWrapper::SensorWrapper(const std::optional<std::string> &device_id) {
  spdlog::cfg::load_env_levels();
  spdlog::trace("Constructor: START");
  const auto device_log_level = log_level_from_env();

  // Detect board and revision
  std::string dev_id;
  std::unique_ptr<dai::Device> dev;
  if (device_id) {
    dev = std::make_unique<dai::Device>(*device_id, dai::UsbSpeed::HIGH);
  } else {
    dev = std::make_unique<dai::Device>(dai::UsbSpeed::HIGH);
  }
  dev_id = dev->getDeviceId();
  board_revision = detect_board_revision(dev);
  spdlog::info("Connected to device with id: {}", dev_id);
  dev.reset();

  // Wait for device to come up again
  bool dev_rdy = false;
  dai::DeviceInfo dev_info;
  for (uint8_t i = 0; !dev_rdy && i < 10; ++i) {
    std::tie(dev_rdy, dev_info) = dai::Device::getDeviceById(dev_id);
  }
  dev_info.deviceId = dev_id;

  // Configure device and pipeline
  dai::DeviceBase::Config dev_cfg;
  dev_cfg.logLevel = device_log_level;
  dev_cfg.outputLogLevel = device_log_level;
  if (board_revision && *board_revision < 7) {
    // Connect FSIN_2LANE and FSIN_4LANE on older revisions
    const auto fsin_mode_select = *board_revision < 6 ? 6 : 38;
    dev_cfg.board.gpio[fsin_mode_select] = dai::BoardConfig::GPIO(dai::BoardConfig::GPIO::OUTPUT, dai::BoardConfig::GPIO::Level::HIGH);
  }
  pipeline = std::make_unique<dai::Pipeline>(std::make_shared<dai::Device>(dev_cfg, dev_info));
  pipeline->setXLinkChunkSize(0);

  spdlog::trace("Constructor: END");
}

SensorWrapper::~SensorWrapper() {
  spdlog::trace("Destructor: START");
  stop();
  spdlog::trace("Destructor: END");
}

bool SensorWrapper::addCamera(
    const std::string &name, dai::CameraBoardSocket socket,
    std::optional<std::pair<uint32_t, uint32_t>> resolution,
    std::optional<float> hz, std::optional<std::chrono::microseconds> exposure,
    std::optional<uint32_t> iso, bool color, std::optional<int32_t> encode) {
  spdlog::trace("addCamera: START");
  const bool is_new = cams.find(name) == cams.end();
  if (is_new) {
    cams[name] = {socket, resolution, hz, exposure, iso, color, encode};
  }
  spdlog::trace("addCamera: END");
  return is_new;
}
bool SensorWrapper::addCamera(
    dai::CameraBoardSocket socket,
    std::optional<std::pair<uint32_t, uint32_t>> resolution,
    std::optional<float> hz, std::optional<std::chrono::microseconds> exposure,
    std::optional<uint32_t> iso, bool color, std::optional<int32_t> encode) {
  return addCamera(dai::toString(socket), socket, resolution, hz, exposure, iso,
                   color, encode);
}

bool SensorWrapper::addIMU(std::vector<dai::IMUSensor> sensors, uint32_t hz) {
  spdlog::trace("addIMU: START");
  const bool is_new = imu_sensors.empty();
  if (is_new) {
    imu_sensors = std::move(sensors);
    imu_hz = hz;
  }
  spdlog::trace("addIMU: END");
  return is_new;
}
bool SensorWrapper::addIMU(uint32_t hz) {
  //TODO maybe add ROTATION_VECTOR and replace ACCELEROMETER_RAW with LINEAR_ACCELERATION
  return addIMU({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, hz);
}

void SensorWrapper::resetCamCallback(CamCallback callback) {
  std::unique_lock<std::shared_mutex> lock(mtx_proc_cam);
  fn_proc_cam = std::move(callback);
}

void SensorWrapper::resetIMUCallback(IMUCallback callback) {
  std::unique_lock<std::shared_mutex> lock(mtx_proc_imu);
  fn_proc_imu = std::move(callback);
}

bool SensorWrapper::buildPipeline() {
  spdlog::trace("buildPipeline: START");

  if (!imu_sensors.empty()) {
    node_imu = pipeline->create<dai::node::IMU>();
    node_imu->enableIMUSensor(imu_sensors, imu_hz);
    node_imu->setBatchReportThreshold(1);
    node_imu->setMaxBatchReports(imu_hz);

    queue_imu = node_imu->out.createOutputQueue(0, false);
    queue_imu->addCallback([this](std::shared_ptr<dai::ADatatype> data) {
      proc_imu(std::dynamic_pointer_cast<dai::IMUData>(data));
    });

    if (!fn_proc_imu)
      spdlog::warn("IMU is enabled without callback to process the data!");

    imu_interval = std::chrono::duration<double>(1.0 / imu_hz);
#ifdef CHECK_MSGDROP
    imu_interval_limit = std::chrono::duration_cast<std::chrono::nanoseconds>(
        imu_interval * interval_threshold);
#endif
  }

  if (!cams.empty()) {
    if (!fn_proc_cam)
      spdlog::warn("Cameras are enabled without callback to process the data!");

    if (!sync_cams.empty()) {
      node_sync = pipeline->create<dai::node::Sync>();
    }
    auto sync_hz = 0.0f;

    for (auto &[name, conf] : cams) {
      const auto node = pipeline->create<dai::node::Camera>()->build(conf.socket);
      node_cam[name] = node;
      const auto output_type = conf.color    ? dai::ImgFrame::Type::NV12
                               : conf.encode ? dai::ImgFrame::Type::YUV400p
                                             : dai::ImgFrame::Type::GRAY8;
      dai::Node::Output *node_output;
      if (conf.resolution.has_value()) {
        node_output = node->requestOutput(conf.resolution.value(), output_type,
                                  dai::ImgResizeMode::CROP, conf.hz, false);
      } else {
        node_output = node->requestFullResolutionOutput(output_type, conf.hz, true);
      }
      if (conf.iso.has_value() && !conf.exposure.has_value()) {
        spdlog::error("[{}] ISO can only be used together with manual exposure time", name);
        return false;
      }
      if (conf.exposure.has_value()) {
        node->initialControl.setManualExposure(conf.exposure.value(), conf.iso.value_or(100));
      }
      node->initialControl.setAntiBandingMode(dai::CameraControl::AntiBandingMode::MAINS_50_HZ);

      const auto cam_hz = node->getMaxRequestedFps();
      cam_interval[name] = std::chrono::duration<double>(1.0 / cam_hz);
#ifdef CHECK_MSGDROP
      cam_interval_limit[name] = std::chrono::duration_cast<std::chrono::nanoseconds>(cam_interval[name] * interval_threshold);
#endif

      if (conf.encode) {
        auto enc = pipeline->create<dai::node::VideoEncoder>();
        node_enc[name] = enc;
        enc->setDefaultProfilePreset(cam_hz, dai::VideoEncoderProperties::Profile::MJPEG);
        const auto &lossy = conf.encode.value();
        if (lossy >= 0) {
          enc->setQuality(lossy);
        } else {
          enc->setLossless(true);
        }
        node_output->link(enc->input);

        node_output = enc->getOutputRef("bitstream");
      }

      if (sync_cams.find(name) != sync_cams.end()) {
        if (sync_hz <= 0.0f) {
          sync_hz = cam_hz;
        } else if (sync_hz != cam_hz) {
          spdlog::error("All synced cameras must have the same Hz! Camera {} has {}Hz while {}Hz was expected", name, cam_hz, sync_hz);
          return false;
        }
        if (sync_type != SyncType::SOFTWARE) {
          node->initialControl.setFrameSyncMode(dai::CameraControl::FrameSyncMode::INPUT);
        }
        node_output->link(node_sync->inputs[name]);
      } else {
        queue_cam[name] = node_output->createOutputQueue(0, false);
        queue_cam[name]->addCallback(
            [this, name](std::shared_ptr<dai::ADatatype> data) {
              proc_cam(std::dynamic_pointer_cast<dai::ImgFrame>(data), name);
            });
      }

#ifdef CHECK_MSGDROP
      last_cam_tp[name] = std::chrono::steady_clock::time_point::max();
#endif
    }

    if (!sync_cams.empty()) {
      sync_interval = std::chrono::duration<double>(1.0 / sync_hz);
#ifdef CHECK_MSGDROP
      sync_interval_limit = std::chrono::duration_cast<std::chrono::nanoseconds>(sync_interval * interval_threshold);
#endif
      node_sync->setSyncThreshold(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::duration<double>((1.0 - 1.0 / sync_cams.size()) /
                                            sync_hz)));

      queue_sync = node_sync->out.createOutputQueue(0, false);
      queue_sync->addCallback([this](std::shared_ptr<dai::ADatatype> data) {
        proc_synced(std::dynamic_pointer_cast<dai::MessageGroup>(data));
      });

      if (sync_type == SyncType::BOARD) {
        if (board_revision) {
          // Trigger node for external FSYNC signal (required for AR0234)
          node_fsync = pipeline->create<dai::node::Script>();
          node_fsync->setProcessor(sync_proc);
          std::string script;
          if (sync_proc == dai::ProcessorType::LEON_MSS) {
            script = fmt::format(FSYNC_LOOP_PY_SCRIPT, *board_revision, sync_interval.count());
          } else if (sync_proc == dai::ProcessorType::LEON_CSS) {
            script = fmt::format(FSYNC_THREADING_PY_SCRIPT, *board_revision, sync_interval.count());
          } else {
            spdlog::error("Unsupported processor for FSYNC script");
            return false;
          }
          node_fsync->setScript(script);
          spdlog::debug("FSYNC script:\n{}", script);
        } else {
          spdlog::error("FSYNC can not be generated without known board revision");
          return false;
        }
      } else if (sync_type == SyncType::CAMERA) {
        auto &[name, node] = *node_cam.begin();
        node->initialControl.setFrameSyncMode(dai::CameraControl::FrameSyncMode::OUTPUT);
        spdlog::debug("Camera \"{}\" set to output FSYNC signal", name);
      }
    }
  }

  spdlog::trace("buildPipeline: END");
  return true;
}

void SensorWrapper::start() {
  spdlog::trace("start: START");
  pipeline->start();
  spdlog::trace("start: END");
}

void SensorWrapper::stop() {
  spdlog::trace("stop: START");
  pipeline->stop();
  spdlog::trace("stop: END");
}

void SensorWrapper::proc_synced(std::shared_ptr<dai::MessageGroup> msgpack) {
  // spdlog::trace("proc_synced: START");

#ifdef CHECK_MSGDROP
  static auto last_tp = msgpack->getTimestampDevice();
  const auto tp = msgpack->getTimestampDevice();
  const auto time_diff = tp - last_tp;
  last_tp = tp;
  if (time_diff > sync_interval_limit) {
    const auto time_delay = time_diff - sync_interval;
    spdlog::warn(
        "{} Synced Messages Dropped!! (by {:.3f}ms)",
        std::round(time_delay / sync_interval),
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_delay).count());
  }
#ifdef TRACE_MSGS
  else {
    spdlog::trace(
        "Synced frames received with delay: {:.3f} ms in interval: {:.3f} us",
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_diff).count(),
        std::chrono::duration_cast<std::chrono::duration<double, std::micro>>(
          std::chrono::nanoseconds(msgpack->getIntervalNs())).count());
  }
#endif
#endif

  std::queue<std::future<void>> futures;
  for (const auto &[name, msg] : *msgpack) {
    futures.push(std::async(std::launch::async, [this, msg, name]() {
      proc_cam(std::dynamic_pointer_cast<dai::ImgFrame>(msg), name);
    }));
  }
  while (!futures.empty()) {
    futures.front().wait();
    futures.pop();
  }

  // spdlog::trace("proc_synced: END");
}

void SensorWrapper::proc_cam(std::shared_ptr<dai::ImgFrame> msg, const std::string &name) {
  // spdlog::trace("proc_cam: START");
#ifdef CHECK_MSGDROP
  if (std::find(sync_cams.begin(), sync_cams.end(), name) == sync_cams.end()) {
    const auto tp = msg->getTimestampDevice();
    const auto time_diff = tp - last_cam_tp[name];
    last_cam_tp[name] = tp;
    const auto interval = cam_interval[name];
    if (time_diff > cam_interval_limit[name]) {
      const auto time_delay = time_diff - interval;
      spdlog::warn(
          "[{}] {} Frames Dropped!! (by {:.3f}ms)", name,
          std::round(time_delay / interval),
          std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_diff - interval).count());
    }
#ifdef TRACE_MSGS
    else {
      spdlog::trace(
          "[{}] Frame received with delay: {:.3f} ms", name,
          std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_diff).count());
    }
#endif
  }
#endif

  {
    std::shared_lock<std::shared_mutex> lock(mtx_proc_cam);
    if (fn_proc_cam) {
      fn_proc_cam(msg, name);
    }
  }

  // spdlog::trace("proc_cam: END");
}

void SensorWrapper::proc_imu(std::shared_ptr<dai::IMUData> msg) {
  // spdlog::trace("proc_imu: START");
#ifdef CHECK_MSGDROP
  static auto last_tp = msg->packets.front().gyroscope.getTimestampDevice();
#endif

  // Old implementation for normal IMU Data
  for (const auto &pkt : msg->packets) {
    {
      std::shared_lock<std::shared_mutex> lock(mtx_proc_imu);
      if (fn_proc_imu) {
        fn_proc_imu(pkt);
      }
    }

#ifdef CHECK_MSGDROP
    const auto tp = pkt.gyroscope.getTimestampDevice();
    const auto time_diff = tp - last_tp;
    last_tp = tp;
    if (time_diff > imu_interval_limit) {
      const auto time_delay = time_diff - imu_interval;
      spdlog::warn(
          "{} IMU Messages Dropped!! (by {:.3f}ms)",
          std::round(time_delay / imu_interval),
          std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_diff - imu_interval).count());
    }
#ifdef TRACE_MSGS
    else {
      spdlog::trace(
          "IMU packets received with delay: {:.3f} ms",
          std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_diff).count());
    }
#endif
#endif
  }
  // spdlog::trace("proc_imu: END");
}

} // namespace dai_vi
