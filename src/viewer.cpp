#include "spdlog/spdlog.h"
#include "spdlog/cfg/env.h"
#include "argparse/argparse.hpp"
#include <signal.h>
#include <future>
#include "opencv2/highgui.hpp"

#include "dai_vi.hpp"

std::promise<void> exit_barrier;
std::unique_ptr<dai_vi::SensorWrapper> sensor;

void show_images(std::shared_ptr<dai::MessageGroup> msgpack)
{
  for (const auto &[name, msg] : *msgpack)
  {
    const auto &camname = name;
    const auto &img = std::dynamic_pointer_cast<dai::ImgFrame>(msg);
    std::thread([=]()
                { cv::imshow(camname, img->getCvFrame()); })
        .detach();

    cv::waitKey(1);
  }
}

int main(int argc, char **argv)
{
  spdlog::cfg::load_env_levels();

  // Parse Arguments
  argparse::ArgumentParser prog("viewer");
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
  uint32_t exposure = prog.get<uint32_t>("--exposure");

  // Setup DepthAi Pipeline
  sensor = std::make_unique<dai_vi::SensorWrapper>();

  for (uint8_t i = 0; i < 4; ++i)
  {
    auto cam = sensor->createCamera("cam" + std::to_string(i), static_cast<dai::CameraBoardSocket>(i));
    if (exposure > 0)
      cam->initialControl.setManualExposure(std::chrono::microseconds(exposure), 100);
  }
  sensor->cam_hz = cam_hz;
  sensor->encode = false;
  if (exposure > 0)
    sensor->start_skip = 0;

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

  sensor->fn_proc_synced = show_images;

  // Setup Output Windows
  for (auto &[name, output] : sensor->node_cam)
  {
    cv::namedWindow(name, cv::WINDOW_NORMAL | cv::WINDOW_GUI_EXPANDED);
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
    exit_barrier.set_value(); });

  // Let it run
  exit_barrier.get_future().wait();
}
