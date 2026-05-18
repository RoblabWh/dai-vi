#include "dai_io.hpp"
#include "dai_vi.hpp"

#include "argparse/argparse.hpp"
#include "spdlog/cfg/env.h"
#include "spdlog/spdlog.h"
#include <atomic>
#include <future>
#include <signal.h>

std::promise<void> exit_barrier;
std::unique_ptr<dai_vi::SensorWrapper> sensor;

int main(int argc, char **argv) {
  spdlog::cfg::load_env_levels();

  // Parse Arguments
  argparse::ArgumentParser prog("capture");
  prog.add_argument("output").help("Path to output directory");
  prog.add_argument("-f", "--force")
      .help("Force overwrite output directory (CAUTION!: directory gets "
            "deleted recursively)")
      .flag();
  prog.add_argument("--dry-run")
      .help("Run without writing any files (for testing)")
      .flag();
  prog.add_argument("--device").help("Luxonis device ID or name");
  prog.add_argument("--imu-hz")
      .help("Rate to capture IMU data in Hz")
      .default_value<uint32_t>(200)
      .scan<'u', uint32_t>();
  prog.add_argument("--cam-hz")
      .help("Rate to capture images in Hz")
      .default_value<float>(20)
      .scan<'g', float>();
  prog.add_argument("--exposure")
      .help("Manual exposure time to use in us")
      .scan<'u', uint32_t>();
  prog.add_argument("--start-skip")
      .help("Number of initial frames to skip for stable exposure")
      .scan<'u', uint32_t>();

  try {
    prog.parse_args(argc, argv);
  } catch (const std::exception &e) {
    spdlog::error(e.what());
    std::cerr << prog << std::endl;
    return 1;
  }

  const auto output = prog.get<std::string>("--output");
  const auto force = prog.get<bool>("--force");
  const auto dry_run = prog.get<bool>("--dry-run");
  const auto device = prog.present<std::string>("--device");
  const auto imu_hz = prog.get<uint32_t>("--imu-hz");
  const auto cam_hz = prog.get<float>("--cam-hz");
  const auto exposure = prog.present<uint32_t>("--exposure");
  const auto start_skip = prog.present<uint32_t>("--start-skip");

  // Setup Pipeline
  sensor = std::make_unique<dai_vi::SensorWrapper>(device);

  if (imu_hz > 0) {
    sensor->addIMU(imu_hz);
  }
  if (cam_hz > 0) {
    for (uint8_t i = 0; i < 4; ++i) {
      const auto name = "cam" + std::to_string(i);
      sensor->addCamera(
          name, static_cast<dai::CameraBoardSocket>(i), std::nullopt, cam_hz,
          exposure ? std::optional<std::chrono::microseconds>(exposure.value())
                   : std::nullopt,
          std::nullopt, false, false);
      sensor->sync_cams.insert(name);
    }
  }

  if (start_skip.has_value() || exposure.has_value()) {
    std::atomic_uint64_t skip_imgs = start_skip.value_or(20);
    std::atomic_uint64_t skip_pkgs = std::ceil(imu_hz / cam_hz * skip_imgs);
    int skip_imgs_cb = sensor->queue_cam["sync"]->addCallback([&]() {
      if (skip_imgs.fetch_sub(1) == 1) {
        sensor->queue_cam["sync"]->removeCallback(skip_imgs_cb);
        sensor->fn_proc_cam = write_jpeg_with_exposure;
      }
    });
    int skip_pkgs_cb = sensor->queue_imu->addCallback([&]() {
      if (skip_pkgs.fetch_sub(1) == 1) {
        sensor->queue_imu->removeCallback(skip_pkgs_cb);
        sensor->fn_proc_imu = write_imu_csv;
      }
    });
  } else {
    sensor->fn_proc_cam = write_jpeg_with_exposure;
    sensor->fn_proc_imu = write_imu_csv;
  }

  if (!sensor->buildPipeline()) {
    spdlog::error("Failed to build pipeline!");
    return 1;
  }

  // Setup Output Files
  if (dry_run) {
    spdlog::warn("Running in dry-run mode, no files will be written!");
    sensor->fn_proc_cam = nullptr;
    sensor->fn_proc_imu = nullptr;
  } else {
    spdlog::info("Output will be written to: {}", output);
    if (!setup_output_folder(sensor, output, force)) {
      spdlog::error("Failed to create output folder!");
      return 1;
    }
  }

  // Run Pipeline
  signal(SIGINT, [](int signum) {
    (void)signum;
    spdlog::info("signal: SIGINT");
    sensor->stop();
    close_output_files();
    exit_barrier.set_value();
  });
  sensor->start();
  exit_barrier.get_future().wait();
}
