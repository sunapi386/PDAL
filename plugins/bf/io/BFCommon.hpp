//
// Created by jsun on 2/15/19.
//

#pragma once
#include <bf.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <vendor/eigen/Eigen/Dense>
#include <json/value.h>
#include <json/json.h>
#include <vehicle_state.pb.h>

struct BFArgs
{
    std::string fileRtk;
    std::string fileLidar;
    std::string fileAffine;
    bool dumpFrames;
};

template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args)
{
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

struct LidarPoint
{
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
std::string TimespecToString(const timespec &timestamp, bool useDash = false);
double TimespecToDouble(const timespec& timestamp);
timespec DoubleToTimespec(const double double_time);
//void printLidarPC(PointCloud &pc);
void writePCTextFile(PointCloud &pc, std::string &name);
Eigen::Affine3d readAffineFromJson(Json::Value &root);
BFArgs readArgsFromJson(Json::Value &root);
bool jsonValueFromFile(std::string &filename, Json::Value &root);
