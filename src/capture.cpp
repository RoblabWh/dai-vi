#include "spdlog/spdlog.h"
#include "spdlog/cfg/env.h"
#include "argparse/argparse.hpp"
#include <signal.h>
#include <future>

#include "dai_vi.hpp"
#include "dai_io.hpp"

std::promise<void> exit_barrier;
std::unique_ptr<dai_vi::SensorWrapper> sensor;

int main(int argc, char **argv)
{
  spdlog::cfg::load_env_levels();

  // Parse Arguments
  argparse::ArgumentParser prog("capture");
  prog.add_argument("output")
      .help("Path to output directory");
  prog.add_argument("-f", "--force")
      .help("Force overwrite output directory (CAUTION!: directory gets deleted recursively)")
      .flag();
  prog.add_argument("--imu-hz")
      .help("Rate to capture IMU data in Hz")
      .default_value<uint16_t>(200)
      .scan<'u', uint16_t>();
  prog.add_argument("--cam-hz")
      .help("Rate to capture images in Hz")
      .default_value<uint16_t>(20)
      .scan<'u', uint16_t>();
  prog.add_argument("--exposure")
      .help("Manual exposure time to use in us or 0 for auto exposure")
      .default_value<uint32_t>(0)
      .scan<'u', uint32_t>();

  try
  {
    prog.parse_args(argc, argv);
  }
  catch (const std::exception &e)
  {
    spdlog::error(e.what());
    std::cout << prog << std::endl;
    return 1;
  }

  uint16_t cam_hz = prog.get<uint16_t>("--cam-hz");
  uint16_t imu_hz = prog.get<uint16_t>("--imu-hz");
  uint32_t exposure = prog.get<uint32_t>("--exposure");

  // Setup DepthAi Pipeline
  sensor = std::make_unique<dai_vi::SensorWrapper>();

  if (imu_hz > 0)
    sensor->createIMU(imu_hz);
  if (cam_hz > 0)
  {
    for (uint8_t i = 0; i < 4; ++i)
    {
      auto cam = sensor->createCamera("cam" + std::to_string(i), static_cast<dai::CameraBoardSocket>(i));
      if (exposure > 0)
        cam->initialControl.setManualExposure(std::chrono::microseconds(exposure), 100);
    }
    sensor->cam_hz = cam_hz;
    sensor->encode = true;
    if (exposure > 0)
      sensor->start_skip = 1;
  }

  if (!sensor->buildPipeline())
  {
    spdlog::error("Failed to build pipeline!");
    return 1;
  }
  if (!sensor->createDevice())
  {
    spdlog::error("Failed to create device!");
    return 1;
  }

  sensor->fn_proc_synced = write_jpeg_with_exposure;
  sensor->fn_proc_imu = imu_write_csv;

  // Setup Output Files
  if (!setup_output_folder(sensor, prog.get<std::string>("output"), prog.get<bool>("--force")))
  {
    spdlog::error("Failed to create output folder!");
    return 1;
  }

  // Start DepthAi Pipeline
  if (!sensor->start())
  {
    spdlog::error("Failed to start sensor!");
    return 1;
  }

  signal(SIGINT, [](int signum)
         {
    (void) signum;
    spdlog::info("signal: SIGINT");
    sensor->stop();
    close_output_files();
    exit_barrier.set_value(); });

  // Let it run
  exit_barrier.get_future().wait();
}
