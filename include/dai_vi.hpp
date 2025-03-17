#pragma once

#include "depthai/depthai.hpp"

namespace dai_vi
{

  class SensorWrapper
  {
  public:
    SensorWrapper();
    ~SensorWrapper();

    std::shared_ptr<dai::node::IMU> createIMU(uint16_t hz);
    std::shared_ptr<dai::node::MonoCamera> createCamera(const std::string &name, dai::CameraBoardSocket socket, uint16_t hz = 0);
    bool buildPipeline();
    bool createDevice();

    bool start();
    void stop();

    dai::Pipeline pipeline;
    dai::DeviceBase::Config devconf;
    std::unique_ptr<dai::Device> device;

    std::map<std::string, std::shared_ptr<dai::node::MonoCamera>> node_cam;
    std::map<std::string, std::shared_ptr<dai::node::ImageManip>> node_crop;
    std::map<std::string, std::shared_ptr<dai::node::VideoEncoder>> node_enc;
    std::shared_ptr<dai::node::Sync> node_sync;
    std::shared_ptr<dai::node::XLinkOut> node_link_cam;
    std::shared_ptr<dai::DataOutputQueue> queue_cam;

    std::shared_ptr<dai::node::IMU> node_imu;
    std::shared_ptr<dai::node::XLinkOut> node_link_imu;
    std::shared_ptr<dai::DataOutputQueue> queue_imu;

    std::function<void(std::shared_ptr<dai::MessageGroup> msgpack)> fn_proc_synced;
    std::function<void(const dai::IMUPacket &)> fn_proc_imu;

    bool encode = false;
    uint16_t cam_hz = 0;
    uint16_t imu_hz = 0;
    int32_t start_skip = -1;

  private:
    void proc_synced(std::shared_ptr<dai::MessageGroup> msgpack);
    void proc_imu(std::shared_ptr<dai::IMUData> msg);

#ifdef CHECK_MSGDROP
    static constexpr const double pd_thresh = 1.5;

    std::chrono::duration<double> cam_pd;
    std::chrono::duration<double> imu_pd;

    std::chrono::nanoseconds cam_pd_limit;
    std::chrono::nanoseconds imu_pd_limit;

#ifdef CHECK_MSGDROP_DETAILED
    static const uint64_t pi_thresh = 10;

    uint64_t cam_pi;
    uint64_t imu_pi;
#endif
#endif
  };
}
