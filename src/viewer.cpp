#include "dai_vi.hpp"

#include "argparse/argparse.hpp"
#include "opencv2/highgui.hpp"
#include "spdlog/cfg/env.h"
#include "spdlog/spdlog.h"
#include <future>
#include <signal.h>

std::promise<void> exit_barrier;

int main(int argc, char **argv) {
  spdlog::cfg::load_env_levels();

  // Parse Arguments
  argparse::ArgumentParser prog("viewer");
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
  prog.add_argument("--exposure")
      .help("Manual exposure time to use in us")
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

  const auto device = prog.present<std::string>("--device");
  const auto cam_hz = prog.get<float>("--cam-hz");
  const auto width = prog.present<uint32_t>("--width");
  const auto height = prog.present<uint32_t>("--height");
  const auto exposure = prog.present<uint32_t>("--exposure");
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
    sensor->addCamera(
        name, static_cast<dai::CameraBoardSocket>(i), resolution, cam_hz,
        exposure ? std::optional<std::chrono::microseconds>(exposure.value())
                 : std::nullopt,
        std::nullopt, false, std::nullopt);
    sensor->sync_cams.insert(name);
  }
  sensor->sync_type = sync_type == "board"    ? dai_vi::SyncType::BOARD
                      : sync_type == "camera" ? dai_vi::SyncType::CAMERA
                                              : dai_vi::SyncType::SOFTWARE;
  sensor->resetCamCallback([](const auto img, const auto &name) {
    cv::imshow(name, img->getCvFrame());
    cv::waitKey(1);
  });

  if (!sensor->buildPipeline()) {
    spdlog::error("Failed to build pipeline!");
    return 1;
  }

  // Setup Output Windows
  for (auto &[name, output] : sensor->node_cam) {
    cv::namedWindow(name, cv::WINDOW_NORMAL | cv::WINDOW_GUI_EXPANDED);
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
  sensor->stop();
}
