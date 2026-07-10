#include "dai_io.hpp"
#include "dai_vi.hpp"

#include "argparse/argparse.hpp"
#include "spdlog/cfg/env.h"
#include "spdlog/spdlog.h"
#include <atomic>
#include <future>
#include <signal.h>

struct SkipState {
  std::atomic_uint64_t count;
  int callback_id;
  std::promise<void> done;
  std::future<void> barrier;
};

std::promise<void> exit_barrier;

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
  prog.add_argument("--width")
      .help("Width of captured images")
      .scan<'u', uint32_t>();
  prog.add_argument("--height")
      .help("Height of captured images")
      .scan<'u', uint32_t>();
  prog.add_argument("--exposure")
      .help("Manual exposure time to use in us")
      .scan<'u', uint32_t>();
  prog.add_argument("--start-skip")
      .help("Number of initial frames to skip for stable exposure")
      .scan<'u', uint32_t>();
  prog.add_argument("--sync-type")
      .help("Sync type: software (default), camera (HW, camera generates "
            "FSYNC), board (HW, FFC board generates FSYNC via GPIO)")
      .default_value<std::string>("software")
      .choices("software", "camera", "board");

  try {
    prog.parse_args(argc, argv);
  } catch (const std::exception &e) {
    spdlog::error(e.what());
    std::cerr << prog << std::endl;
    return 1;
  }

  const auto output = prog.get<std::string>("output");
  const auto force = prog.get<bool>("--force");
  const auto dry_run = prog.get<bool>("--dry-run");
  const auto device = prog.present<std::string>("--device");
  const auto imu_hz = prog.get<uint32_t>("--imu-hz");
  const auto cam_hz = prog.get<float>("--cam-hz");
  const auto width = prog.present<uint32_t>("--width");
  const auto height = prog.present<uint32_t>("--height");
  const auto exposure = prog.present<uint32_t>("--exposure");
  const auto start_skip = prog.present<uint32_t>("--start-skip");
  const auto sync_type = prog.get<std::string>("--sync-type");

  std::optional<std::pair<uint32_t, uint32_t>> resolution;
  if (width.has_value() && height.has_value()) {
    resolution = {width.value(), height.value()};
  } else if (width.has_value() || height.has_value()) {
    spdlog::error("Both width and height must be specified to set resolution!");
    return 1;
  }

  // Setup Pipeline
  auto sensor = std::make_unique<dai_vi::SensorWrapper>(device);

  if (imu_hz > 0) {
    sensor->addIMU(imu_hz);
  }
  if (cam_hz > 0) {
    for (uint8_t i = 0; i < 4; ++i) {
      const auto name = "cam" + std::to_string(i);
      sensor->addCamera(
          name, static_cast<dai::CameraBoardSocket>(i), resolution, cam_hz,
          exposure ? std::optional<std::chrono::microseconds>(exposure.value())
                   : std::nullopt,
          std::nullopt, false, dai_vi::JPEG_LOSSLESS);
      sensor->sync_cams.insert(name);
    }
    sensor->sync_stamps = true;
    sensor->sync_type = sync_type == "board"    ? dai_vi::SyncType::BOARD
                        : sync_type == "camera" ? dai_vi::SyncType::CAMERA
                                                : dai_vi::SyncType::SOFTWARE;
    sensor->resetCamCallback(write_jpeg_with_exposure);
    sensor->resetIMUCallback(write_imu_csv);
  }

  if (!sensor->buildPipeline()) {
    spdlog::error("Failed to build pipeline!");
    return 1;
  }

  SkipState skip_imgs;
  SkipState skip_pkgs;

  // Setup Output Files
  if (dry_run) {
    spdlog::warn("Running in dry-run mode, no files will be written!");
    sensor->resetCamCallback();
    sensor->resetIMUCallback();
  } else {
    spdlog::info("Output will be written to: {}", output);
    if (!setup_output_folder(sensor, output, force)) {
      spdlog::error("Failed to create output folder!");
      return 1;
    }

    // Handle initial frame skipping
    if (start_skip.has_value() || exposure.has_value()) {
      sensor->resetCamCallback();
      sensor->resetIMUCallback();

      if (cam_hz > 0) {
        skip_imgs.count = start_skip.value_or(20);
        spdlog::debug("Skipping initial {} images", skip_imgs.count.load());

        skip_imgs.callback_id = sensor->queue_sync->addCallback([&]() {
          if (auto val = skip_imgs.count.load()) {
            spdlog::trace("Skipping image, remaining: {}", val);
          }
          if (skip_imgs.count.fetch_sub(1) == 1) {
            sensor->resetCamCallback(write_jpeg_with_exposure);
            skip_imgs.done.set_value();
          }
        });

        skip_imgs.barrier = std::async(std::launch::async, [&]() {
          skip_imgs.done.get_future().wait();
          sensor->queue_sync->removeCallback(skip_imgs.callback_id);
          spdlog::debug("Finished skipping initial images");
        });

        if (imu_hz > 0) {
          skip_pkgs.count = std::ceil(imu_hz / cam_hz * skip_imgs.count);
          spdlog::debug("Skipping initial {} IMU packages",
                        skip_pkgs.count.load());

          skip_pkgs.callback_id = sensor->queue_imu->addCallback([&]() {
            if (auto val = skip_pkgs.count.load()) {
              spdlog::trace("Skipping IMU package, remaining: {}", val);
            }
            if (skip_pkgs.count.fetch_sub(1) == 1) {
              sensor->resetIMUCallback(write_imu_csv);
              skip_pkgs.done.set_value();
            }
          });

          skip_pkgs.barrier = std::async(std::launch::async, [&]() {
            skip_pkgs.done.get_future().wait();
            sensor->queue_imu->removeCallback(skip_pkgs.callback_id);
            spdlog::debug("Finished skipping initial IMU packages");
          });
        }
      }
    }
  }

  // Run Pipeline
  signal(SIGINT, [](int signum) {
    (void)signum;
    spdlog::info("signal: SIGINT");
    exit_barrier.set_value();
    signal(SIGINT, SIG_DFL);
  });
  sensor->start();
  exit_barrier.get_future().wait();
  if (skip_imgs.barrier.valid()) {
    skip_imgs.barrier.wait();
  }
  if (skip_pkgs.barrier.valid()) {
    skip_pkgs.barrier.wait();
  }

  // Cleanup
  sensor->stop();
  close_output_files();
}
