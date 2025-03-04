#include "dai_vi.hpp"
#include "spdlog/spdlog.h"
#include "spdlog/cfg/env.h"

namespace dai_vi
{
  SensorWrapper::SensorWrapper()
  {
#ifndef NDEBUG
    spdlog::set_level(spdlog::level::debug);
#endif
    spdlog::cfg::load_env_levels();
    spdlog::trace("Constructor: START");
    // Sync both camera lanes
    devconf.board.gpio[6] = dai::BoardConfig::GPIO(dai::BoardConfig::GPIO::OUTPUT, dai::BoardConfig::GPIO::Level::HIGH);
#ifndef NDEBUG
    devconf.logLevel = dai::LogLevel::DEBUG;
    devconf.outputLogLevel = dai::LogLevel::DEBUG;
#endif
    spdlog::trace("Constructor: END");
  }

  SensorWrapper::~SensorWrapper()
  {
    spdlog::trace("Destructor: START");
    stop();
    spdlog::trace("Destructor: END");
  }

  std::shared_ptr<dai::node::MonoCamera> SensorWrapper::createCamera(const std::string &name, dai::CameraBoardSocket socket)
  {
    spdlog::trace("createCamera: START");
    std::shared_ptr<dai::node::MonoCamera> cam;
    if (node_cam.find(name) == node_cam.end())
    {
      cam = pipeline.create<dai::node::MonoCamera>();
      cam->setBoardSocket(socket);
      node_cam[name] = cam;
    }
    spdlog::trace("createCamera: END");
    return cam;
  }

  std::shared_ptr<dai::node::IMU> SensorWrapper::createIMU(uint16_t hz)
  {
    spdlog::trace("createIMU: START");
    node_imu = pipeline.create<dai::node::IMU>();
    imu_hz = hz;
    spdlog::trace("createIMU: END");
    return node_imu;
  }

  bool SensorWrapper::buildPipeline()
  {
    spdlog::trace("buildPipeline: START");
#ifdef CHECK_MSGDROP
    cam_pd = std::chrono::duration<double>(1. / cam_hz);
    imu_pd = std::chrono::duration<double>(1. / imu_hz);

    cam_pd_limit = std::chrono::duration_cast<std::chrono::nanoseconds>(cam_pd * pd_thresh);
    imu_pd_limit = std::chrono::duration_cast<std::chrono::nanoseconds>(imu_pd * pd_thresh);

#ifdef CHECK_MSGDROP_DETAILED
    cam_pi = cam_hz * pi_thresh;
    imu_pi = imu_hz * pi_thresh;
#endif
#endif

    if (node_imu)
    {
      node_imu->enableIMUSensor({dai::IMUSensor::GYROSCOPE_RAW, dai::IMUSensor::ACCELEROMETER_RAW}, imu_hz);
      node_imu->setBatchReportThreshold(cam_hz / 4);
      node_imu->setMaxBatchReports(cam_hz);
      node_link_imu = pipeline.create<dai::node::XLinkOut>();
      node_link_imu->setStreamName("imu");
      node_imu->out.link(node_link_imu->input);
    }

    if (!node_cam.empty())
    {
      if (cam_hz == 0)
      {
        spdlog::error("Camera Hz is unset");
        return false;
      }

      if (start_skip < 0)
      {
        // Skip the first second to let ISPs 3A adjust
        start_skip = cam_hz;
      }

      node_sync = pipeline.create<dai::node::Sync>();
      node_sync->setSyncThreshold(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.5 / cam_hz)));

      node_link_cam = pipeline.create<dai::node::XLinkOut>();
      node_link_cam->setStreamName("cam");
      node_sync->out.link(node_link_cam->input);

      for (auto &[name, node] : node_cam)
      {
        node->setFps(cam_hz);
        node->setResolution(dai::MonoCameraProperties::SensorResolution::THE_800_P);
        node->initialControl.setFrameSyncMode(dai::CameraControl::FrameSyncMode::INPUT);
        node->initialControl.setAntiBandingMode(dai::CameraControl::AntiBandingMode::MAINS_50_HZ);

        auto crop = pipeline.create<dai::node::ImageManip>();
        node_crop[name] = crop;
        crop->initialConfig.setCropRect(0.1, 0.0, 0.9, 1.0);
        node->out.link(crop->inputImage);

        if (encode)
        {
          auto enc = pipeline.create<dai::node::VideoEncoder>();
          node_enc[name] = enc;
          enc->setDefaultProfilePreset(node->getFps(), dai::VideoEncoderProperties::Profile::MJPEG);
          enc->setLossless(true);
          crop->out.link(enc->input);

          enc->out.link(node_sync->inputs[name]);
        }
        else
        {
          crop->out.link(node_sync->inputs[name]);
        }
      }
      node_cam.begin()->second->initialControl.setFrameSyncMode(dai::CameraControl::FrameSyncMode::OUTPUT);
    }
    spdlog::trace("buildPipeline: END");
    return true;
  }

  bool SensorWrapper::createDevice()
  {
    spdlog::trace("createDevice: START");
    device = std::make_unique<dai::Device>(devconf, dai::UsbSpeed::SUPER_PLUS);
    if (device->getConnectedIMU() != "BMI270")
    {
      spdlog::error("Only IMU of type BMI270");
      return false;
    }
    spdlog::trace("createDevice: END");
    return true;
  }

  bool SensorWrapper::start()
  {
    spdlog::trace("start: START");
    bool valid = false;
    device->startPipeline(pipeline);
    if (!node_cam.empty())
    {
      queue_cam = device->getOutputQueue("cam", cam_hz, false);
      queue_cam->addCallback([this](std::shared_ptr<dai::ADatatype> data)
                             { proc_synced(std::dynamic_pointer_cast<dai::MessageGroup>(data)); });
      if (!fn_proc_synced)
        spdlog::warn("Starting cameras without function to process the data!");
      valid = true;
    }
    if (node_imu)
    {
      queue_imu = device->getOutputQueue("imu", imu_hz, false);
      queue_imu->addCallback([this](std::shared_ptr<dai::ADatatype> data)
                             { proc_imu(std::dynamic_pointer_cast<dai::IMUData>(data)); });
      if (!fn_proc_imu)
        spdlog::warn("Starting imu without function to process the data!");
      valid = true;
    }
    spdlog::trace("start: END");
    return valid;
  }

  void SensorWrapper::stop()
  {
    spdlog::trace("stop: START");
    device.reset();
    spdlog::trace("stop: END");
  }

  void SensorWrapper::proc_synced(std::shared_ptr<dai::MessageGroup> msgpack)
  {
    if (start_skip > 0)
    {
      spdlog::trace("proc_synced: START SKIP");
      start_skip--;
      return;
    }
    else if (start_skip == 0)
    {
      start_skip--;
      spdlog::info("Image processing started");
    }

    spdlog::trace("proc_synced: START");
#ifdef CHECK_MSGDROP
#ifdef CHECK_MSGDROP_DETAILED
    static uint64_t counter = 0;
#endif
    static auto last_tp = msgpack->getTimestamp();
    const auto &tp = msgpack->getTimestamp();
    const auto &time_diff = tp - last_tp;
    last_tp = tp;
    if (time_diff > cam_pd_limit)
    {
      spdlog::warn("Messages Dropped!! (by {}ms)", std::chrono::duration_cast<std::chrono::duration<double>>(time_diff - cam_pd).count() * 1e3);
#ifdef CHECK_MSGDROP_DETAILED
      counter = 0;
    }
    else
    {
      ++counter;
      if (counter == cam_pi)
      {
        spdlog::debug("Got {} msgs without problems", cam_pi);
        counter = 0;
      }
#endif
      spdlog::trace("All good (with {}ms)", std::chrono::duration_cast<std::chrono::duration<double>>(time_diff).count() * 1e3);
    }
#endif

#ifndef NDEBUG
    spdlog::trace("sync diff: {}ms", msgpack->getIntervalNs() * 1e-6);
#endif

    if (fn_proc_synced)
      fn_proc_synced(msgpack);

    spdlog::trace("proc_synced: END");
  }

  void SensorWrapper::proc_imu(std::shared_ptr<dai::IMUData> msg)
  {
    spdlog::trace("proc_imu: START");
#ifdef CHECK_MSGDROP
    static auto last_tp = msg->packets.front().gyroscope.getTimestamp();
#ifdef CHECK_MSGDROP_DETAILED
    static uint64_t counter = 0;
#endif
#endif

    // Old implementation for normal IMU Data
    for (const auto &pkt : msg->packets)
    {
      if (fn_proc_imu)
        fn_proc_imu(pkt);

#ifdef CHECK_MSGDROP
      const auto &tp = pkt.gyroscope.getTimestamp();
      const auto &time_diff = tp - last_tp;
      last_tp = tp;
      if (time_diff > imu_pd_limit)
      {
        spdlog::warn("IMU Messages Dropped!! (by {})", std::chrono::duration_cast<std::chrono::duration<double>>(time_diff - imu_pd).count());
#ifdef CHECK_MSGDROP_DETAILED
        counter = 0;
      }
      else
      {
        ++counter;
        if (counter == imu_pi)
        {
          spdlog::debug("IMU got {} msgs without problems", imu_pi);
          counter = 0;
        }
#endif
        spdlog::trace("All good (with {})", std::chrono::duration_cast<std::chrono::duration<double>>(time_diff).count());
      }
#endif
    }
    spdlog::trace("proc_imu: END");
  }

}
