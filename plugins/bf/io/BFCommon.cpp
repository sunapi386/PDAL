//
// Created by jsun on 2/15/19.
//

#include "BFCommon.hpp"

PointCloud getLidarPoints(bf::Datum &datum) {
  size_t point_num = datum.size / sizeof(LidarPoint);
  const LidarPoint *data_ptr = static_cast<LidarPoint *>(datum.data);
  return std::vector<LidarPoint>(data_ptr, data_ptr + point_num);
}

std::string TimespecToString(const timespec &timestamp) {
  std::stringstream sstream;
  sstream << std::setfill('0') << std::setw(12) << timestamp.tv_sec << ".";
  sstream << std::setfill('0') << std::setw(9) << timestamp.tv_nsec;
  return sstream.str();
}

timespec DoubleToTimespec(const double double_time) {
  timespec timespec_time;
  timespec_time.tv_sec = static_cast<time_t>(double_time);
  timespec_time.tv_nsec = static_cast<long>(
      (double_time - static_cast<double>(timespec_time.tv_sec)) * 1e9);
  return timespec_time;
}
//void printLidarPC(PointCloud &pc) {
//  uint count = 0;
//  std::cout << "idx, x, y, z, intensity, timestamp, laser_id, lidar_angle\n";
//  for (LidarPoint p : pc) {
//
//    std::cout << count << ", " << p.x << ", " << p.y << ", " << p.z << ", "
//              << unsigned(p.intensity) << ", "
//              << TimespecToString(DoubleToTimespec(p.timestamp)) << ", "
//              << unsigned(p.laser_id) << ", " << unsigned(p.lidar_angle)
//              << "\n";
//    ++count;
//  }
//}

void writePCTextFile(PointCloud &pc, std::string &name) {
  // todo: save the points into PDAL acceptable format
  std::ofstream outfile(name);
  outfile << "idx, x, y, z, intensity, timestamp, laser_id, lidar_angle\n";
  for (uint i = 0; i < pc.size(); ++i) {
    LidarPoint &p = pc[i];
    outfile << i << ", " << p.x << ", " << p.y << ", " << p.z << ", "
            << unsigned(p.intensity) << ", "
            << TimespecToString(DoubleToTimespec(p.timestamp)) << ", "
            << unsigned(p.laser_id) << ", " << unsigned(p.lidar_angle) << "\n";
  }
  outfile.close();
}
