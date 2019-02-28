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

// also edit: readArgsFromJson
struct BFArgs
{
    std::string fileRtk;
    std::string fileLidar;
    std::string fileAffine;
    bool dumpFrames = false;
    int nFramesSkip;
    int nFramesRead;
    double mDistanceJump;
    bool mCompensate = true;
    int nPointsReadLimit = -1; // ignore if negative
    double mLidarDistanceReturnFilter = -1; // ignore if negative
};

template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args)
{
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

/*!
 * Similar as BF messages BFLidarPointSerialized but different in using doubles
 */
struct LidarPoint
{
    double x = 0;
    double y = 0;
    double z = 0;
    uint8_t intensity = 0;
    double timestamp = 0;
    uint8_t laser_id = 0;
    double lidar_angle = 0; // since we're not using this I can calculate it
};


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
    TimePlaceSegment() {}
    TimePlaceSegment(TimePlace &start_, TimePlace &finish_)
    {
        start = std::move(start_);
        finish = std::move(finish_);
    }
};


struct PointCloud
{
    std::vector<LidarPoint> points;
    TimePlaceSegment timePlaceSegment;
};

typedef std::vector<LidarPoint> LidarPointVector;
typedef PointCloud& PointCloudRef;
typedef LidarPoint& LidarPointRef;

std::string TimespecToString(const timespec &timestamp, bool useDash = false);
double TimespecToDouble(const timespec& timestamp);
timespec DoubleToTimespec(double double_time);
//void printLidarPC(PointCloudRef pc);

/*!
 * Dump the point cloud into CSV text
 * @param pc
 * @param name
 */
void savePCToCSV(PointCloudRef pc, std::string &name);


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
double radiansFromCoord(double y, double x);
double rad2deg(double rad);
double deg2rad(double deg);
