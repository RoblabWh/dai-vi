#pragma once

#include "depthai/depthai.hpp"
#include "spdlog/spdlog.h"

std::ofstream imu_file;
std::map<std::string, std::ofstream> cam_meta_files;
std::filesystem::path img_folder;

void write_jpeg_with_exposure(std::shared_ptr<dai::MessageGroup> msgpack)
{
  const auto &timestamp = msgpack->getTimestamp().time_since_epoch().count();
  const auto &filename = std::to_string(timestamp) + ".jpg";
  for (const auto &[name, msg] : *msgpack)
  {
    const auto &camname = name;
    const auto &img = std::dynamic_pointer_cast<dai::EncodedFrame>(msg);
    const auto &exposure = std::chrono::duration_cast<std::chrono::nanoseconds>(img->getExposureTime()).count();
    std::thread([=]()
                {
                  std::ofstream file(img_folder / camname / filename, std::ios::binary);
                  const auto &vec = img->getData();
#ifndef DRYRUN
                  file.write(reinterpret_cast<const char *>(vec.data()), vec.size());
#endif
                })
        .detach();

    cam_meta_files[name] << timestamp << ',' << exposure << ',' << filename << '\n';
  }
}

void imu_write_csv(const dai::IMUPacket &pkt)
{
  const auto &acce = pkt.acceleroMeter;
  const auto &gyro = pkt.gyroscope;
#ifndef DRYRUN
  imu_file << gyro.getTimestamp().time_since_epoch().count() << ','
           << gyro.x << ',' << gyro.y << ',' << gyro.z << ','
           << acce.x << ',' << acce.y << ',' << acce.z << '\n';
#endif
}

bool setup_output_folder(const std::unique_ptr<dai_vi::SensorWrapper> &sensor, const std::string &path, bool force = false)
{
#ifndef DRYRUN
  std::filesystem::path out_folder(path);
  if (!std::filesystem::is_directory(out_folder.parent_path()))
  {
    spdlog::error("Directory \"{}\" does not exist.", out_folder.parent_path().string());
    return false;
  }
  if (std::filesystem::exists(out_folder))
  {
    if (force)
    {
      std::filesystem::remove_all(out_folder);
    }
    else
    {
      spdlog::error("Directory \"{}\" exists and -f is not specified.", out_folder.string());
      return false;
    }
  }
  if (!std::filesystem::create_directory(out_folder))
  {
    spdlog::error("Unable to create output directory \"{}\"", out_folder.string());
    return false;
  }
  img_folder = out_folder / "cams";
  for (auto &[name, output] : sensor->node_cam)
  {
    const auto &path = img_folder / name;
    if (!std::filesystem::create_directories(path))
    {
      spdlog::error("Unable to create directory \"{}\"", path.string());
      return false;
    }
    auto [iter, _] = cam_meta_files.emplace(name, img_folder / (name + ".csv"));
    iter->second << "#timestamp [ns],exposuretime [ns],filename [str]\n";
  }

  if (sensor->node_imu)
  {
    imu_file.open(out_folder / "imu.csv");
    imu_file << "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n"
             << std::fixed << std::setprecision(19);
  }
#endif
  return true;
}

void close_output_files()
{
  if (imu_file)
  {
    imu_file.flush();
    imu_file.close();
  }
  for (auto &[name, output] : cam_meta_files)
  {
    output.flush();
    output.close();
  }
}
