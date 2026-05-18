#include "argparse/argparse.hpp"
#include "depthai/depthai.hpp"
#include "spdlog/spdlog.h"

int main(int argc, char **argv) {
  argparse::ArgumentParser prog("update_imu");
  prog.add_argument("--device").help("Luxonis device ID or name");
  try {
    prog.parse_args(argc, argv);
  } catch (const std::exception &e) {
    spdlog::error(e.what());
    std::cerr << prog << std::endl;
    return 1;
  }

  std::unique_ptr<dai::Device> dev;
  if (const auto device = prog.present("--device")) {
    dev = std::make_unique<dai::Device>(*device, dai::UsbSpeed::HIGH);
  } else {
    dev = std::make_unique<dai::Device>(dai::UsbSpeed::HIGH);
  }
  spdlog::info("Connected to device with id: {}", dev->getDeviceId());

  spdlog::info("Starting IMU firmware update...");
  auto ret = dev->startIMUFirmwareUpdate();
  if (!ret) {
    spdlog::error("Failed to start IMU firmware update");
    return 1;
  }
  float status = 0.0;
  while (!ret) {
    std::tie(ret, status) = dev->getIMUFirmwareUpdateStatus();
    spdlog::info("IMU firmware update progress: {:.2f}%", status);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  spdlog::info("IMU firmware update completed with result: {}", ret);
}
