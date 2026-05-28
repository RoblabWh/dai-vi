#include "dai_io.hpp"
#include "dai_vi.hpp"

#include "argparse/argparse.hpp"
#include "spdlog/cfg/env.h"
#include "spdlog/spdlog.h"
#include <signal.h>

const uint32_t INTERNAL_DELAY = 8;
std::atomic_bool cancel = false;

int main(int argc, char **argv) {
  spdlog::cfg::load_env_levels();

  // Parse Arguments
  argparse::ArgumentParser prog("capture_response");
  prog.add_argument("output").help("Path to output directory");
  prog.add_argument("-f", "--force")
      .help("Force overwrite output directory (CAUTION!: directory gets "
            "deleted recursively)")
      .flag();
  prog.add_argument("--dry-run")
      .help("Run without writing any files (for testing)")
      .flag();
  prog.add_argument("--device").help("Luxonis device ID or name");
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
  prog.add_argument("--start")
      .help("Start of exposure range in microseconds (us)")
      .scan<'u', uint32_t>();
  prog.add_argument("--stop")
      .help("End of exposure range in microseconds (us)")
      .scan<'u', uint32_t>();
  prog.add_argument("--step")
      .help("Step size of exposure in range in microseconds (us)")
      .default_value<uint32_t>(10)
      .scan<'u', uint32_t>();
  prog.add_argument("--samples")
      .help("Number of samples to capture per exposure")
      .default_value<uint32_t>(2)
      .scan<'u', uint32_t>();
  prog.add_argument("--sync-type")
      .help("Sync type: software (default), camera (HW, camera generates "
            "FSYNC), board (HW, FFC board generates FSYNC via GPIO)")
      .default_value(std::string("software"))
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
  const auto cam_hz = prog.get<float>("--cam-hz");
  const auto width = prog.present<uint32_t>("--width");
  const auto height = prog.present<uint32_t>("--height");
  const auto samples = prog.get<uint32_t>("--samples");
  const auto exposure_step = prog.get<uint32_t>("--step");
  const auto exposure_start =
      prog.present<uint32_t>("--start").value_or(exposure_step);
  const auto exposure_stop =
      prog.present<uint32_t>("--stop").value_or(1e6 / cam_hz);
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

  for (uint8_t i = 0; i < 4; ++i) {
    const auto name = "cam" + std::to_string(i);
    sensor->addCamera(name, static_cast<dai::CameraBoardSocket>(i), resolution,
                      cam_hz, std::chrono::microseconds(exposure_start),
                      std::nullopt, false, dai_vi::JPEG_LOSSLESS);
    sensor->sync_cams.insert(name);
  }
  sensor->sync_type = sync_type == "board"    ? dai_vi::SyncType::BOARD
                      : sync_type == "camera" ? dai_vi::SyncType::CAMERA
                                              : dai_vi::SyncType::SOFTWARE;
  sensor->resetCamCallback([](auto msg, const auto &name) -> void {
    (void)msg;
    (void)name;
  });

  if (!sensor->buildPipeline()) {
    spdlog::error("Failed to build pipeline!");
    return 1;
  }

  std::unordered_map<std::string, std::shared_ptr<dai::InputQueue>> queue_ctrl;
  for (auto &[name, node] : sensor->node_cam) {
    node->initialControl.setMisc("manual-exposure-handling", "fast");
    queue_ctrl[name] = node->inputControl.createInputQueue(1, true);
  }
  sensor->queue_sync->setMaxSize(1);
  sensor->queue_sync->setBlocking(true);

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
  }

  // Start Pipeline
  signal(SIGINT, [](int signum) {
    (void)signum;
    spdlog::info("signal: SIGINT");
    cancel = true;
    signal(SIGINT, SIG_DFL);
  });
  sensor->start();

  // Handle exposure changes
  uint32_t start_skip = INTERNAL_DELAY;
  for (uint32_t exposure = exposure_start; exposure <= exposure_stop && !cancel;
       exposure += exposure_step) {
    auto control = std::make_shared<dai::CameraControl>();
    control->setManualExposure(exposure, 100);
    for (auto &[name, queue_in] : queue_ctrl) {
      queue_ctrl[name]->send(control);
    }

    for (uint32_t sample = 0; sample < samples && !cancel; ++sample) {
      if (start_skip > 0) {
        --start_skip;
      }
      if (start_skip == 1 && !dry_run) {
        sensor->resetCamCallback(write_jpeg_with_exposure);
      }
      sensor->queue_sync->get();
    }

    spdlog::info("exposure: {}us/{}us - {:3.2f}%", exposure, exposure_stop,
                 exposure * 100.0 / exposure_stop);
  }
  for (uint32_t i = 0; i < INTERNAL_DELAY && !cancel; ++i) {
    sensor->queue_sync->get();
  }

  // Cleanup
  sensor->stop();
  close_output_files();
}
