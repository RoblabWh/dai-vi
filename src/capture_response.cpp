#include "spdlog/spdlog.h"
#include "spdlog/cfg/env.h"
#include "argparse/argparse.hpp"
#include <signal.h>

#include "dai_vi.hpp"
#include "dai_io.hpp"

bool cancel = false;
std::unique_ptr<dai_vi::SensorWrapper> sensor;

int main(int argc, char **argv)
{
  spdlog::cfg::load_env_levels();

  // Parse Arguments
  argparse::ArgumentParser prog("capture_response");
  prog.add_argument("output")
      .help("Path to output directory");
  prog.add_argument("-f", "--force")
      .help("Force overwrite output directory (CAUTION!: directory gets deleted recursively)")
      .flag();
  prog.add_argument("--cam-hz")
      .help("Rate to capture images in Hz")
      .default_value<uint16_t>(20)
      .scan<'u', uint16_t>();
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
      .default_value<uint32_t>(1)
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
  uint32_t samples = prog.get<uint32_t>("--samples");
  uint32_t exposure_step = prog.get<uint32_t>("--step");
  uint32_t exposure_start = prog.is_used("--start") ? prog.get<uint32_t>("--start") : exposure_step;
  uint32_t exposure_stop = prog.is_used("--stop") ? prog.get<uint32_t>("--stop") : 1.0 / cam_hz * 1e6;

  // Setup DepthAi Pipeline
  sensor = std::make_unique<dai_vi::SensorWrapper>();

  auto xlink_in = sensor->pipeline.create<dai::node::XLinkIn>();
  xlink_in->setStreamName("control");

  for (uint8_t i = 0; i < 4; ++i)
  {
    auto cam = sensor->createCamera("cam" + std::to_string(i), static_cast<dai::CameraBoardSocket>(i));
    cam->initialControl.setManualExposure(exposure_start, 100);
    cam->initialControl.setMisc("manual-exposure-handling", "fast");
    xlink_in->out.link(cam->inputControl);
  }
  sensor->cam_hz = cam_hz;
  sensor->encode = true;
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
  sensor->queue_cam->setMaxSize(1);
  sensor->queue_cam->setBlocking(true);
  auto queue_in = sensor->device->getInputQueue("control", 1, true);

  signal(SIGINT, [](int signum)
         {
    (void) signum;
    spdlog::info("signal: SIGINT");
    cancel = true; });

  // Let it run
  for (uint32_t exposure = exposure_start; exposure <= exposure_stop && !cancel; exposure += exposure_step)
  {
    spdlog::info("exposure: {}us", exposure, 100);

    dai::CameraControl control;
    control.setManualExposure(exposure, 100);
    queue_in->send(control);

    for (uint32_t sample = 0; sample < samples && !cancel; ++sample)
    {
      write_jpeg_with_exposure(std::dynamic_pointer_cast<dai::MessageGroup>(sensor->queue_cam->get()));
    }
  }
  sensor->stop();
  close_output_files();
}
