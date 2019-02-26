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
    bool dumpFrames = false;
    int nFramesSkip;
    int nFramesRead;
    double mDistanceJump;
};

template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args)
{
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

/*!
 * Copied from BF messages
 */
struct LidarPoint
{
    float x = 0;              // metres
    float y = 0;              // from
    float z = 0;              // lidar origin (0,0,0)
    uint8_t intensity = 0;    // 0-255
    double timestamp = 0;     // always 0, interpolated
    uint8_t laser_id = 0;     // which beam id (0-127)
    uint16_t lidar_angle = 0; // always 49780 (unused)
};


typedef std::vector<LidarPoint> PointCloud;

struct TimePlace
{
    msg::RTKMessage rtkMessage;
    timespec time;
    TimePlace()
    {
        rtkMessage = {};
        time = {};
    }
    TimePlace(msg::RTKMessage &rtk, timespec &t)
    {
        rtkMessage = rtk;
        time = t;
    }
};

struct TimePlaceSegment
{
    TimePlace start, finish;
    TimePlaceSegment(TimePlace &start_, TimePlace &finish_)
    {
        start = std::move(start_);
        finish = std::move(finish_);
    }
};


PointCloud getLidarPoints(bf::Datum &datum);

std::string TimespecToString(const timespec &timestamp, bool useDash = false);
double TimespecToDouble(const timespec& timestamp);
timespec DoubleToTimespec(const double double_time);
//void printLidarPC(PointCloud &pc);

/*!
 * Dump the point cloud into CSV text
 * @param pc
 * @param name
 */
void writePCTextFile(PointCloud &pc, std::string &name);


Eigen::Affine3d readAffineFromJson(Json::Value &root);
BFArgs readArgsFromJson(Json::Value &root);
bool jsonValueFromFile(std::string &filename, Json::Value &root);
std::string rtkToString(msg::RTKMessage &rtkMessage);
double distanceMeters(msg::RTKMessage &m1, msg::RTKMessage &m2);
std::string preciseDoubleStr(double d, uint precision);

/*!
 * A lidar scan is lidar coordinates has center 0,0,0 so we can compute
 * what the rotation angle theta is.
 * This need is because we do not record the angle which the scan occurs
 * @param lidarPoint
 * @return 2D x-y radians to rotate
 */
double radiansFromCoord(LidarPoint &lidarPoint);
