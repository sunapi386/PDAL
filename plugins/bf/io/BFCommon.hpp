//
// Created by jsun on 2/15/19.
//

#pragma once
#include <bf.hpp>
#include <fstream>
#include <sstream>
#include <string>

struct BFArgs {
  std::string fileRtk;
  std::string fileLidar;
  std::string fileTransf;

  //    std::string url;
  //    std::string resource;
  //    std::string sbounds;
  //    std::size_t depthBegin = 0;
  //    std::size_t depthEnd = 0;
  //    std::string tilePath;
  //    Json::Value filter;
  //    Json::Value dims;
  //    Json::Value schema;
  //    double buffer = 0;
};

struct LidarPoint {
  float x = 0;              // metres
  float y = 0;              // from
  float z = 0;              // lidar origin (0,0,0)
  uint8_t intensity = 0;    // 0-255
  double timestamp = 0;     // always 0
  uint8_t laser_id = 0;     // which beam id (0-127)
  uint16_t lidar_angle = 0; // always 49780 (unused)
};

typedef std::vector<LidarPoint> PointCloud;
PointCloud getLidarPoints(bf::Datum &datum);
std::string TimespecToString(const timespec &timestamp);
double TimespecToDouble(const timespec& timestamp);
timespec DoubleToTimespec(const double double_time);
//void printLidarPC(PointCloud &pc);
void writePCTextFile(PointCloud &pc, std::string &name);
