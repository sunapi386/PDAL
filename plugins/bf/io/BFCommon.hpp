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
#include <bf_base_interpolator.h>

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
void savePCToCSV(PointCloudRef pc);


Eigen::Affine3d readAffineFromJson(Json::Value &root);
BFArgs readArgsFromJson(Json::Value &root);
bool jsonValueFromFile(std::string &filename, Json::Value &root);
std::string rtkToString(msg::RTKMessage &rtkMessage);
double distanceEarth(double lat1d, double lon1d, double lat2d, double lon2d);
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

class LidarTimestampInterpolator
{
public:
    void UpdateTimestamp(const double scan_period, const timespec &start_time, std::vector<LidarPoint> *cloud);

private:
    double GetRelativeTime(double theta);

    double scan_period_;
    double start_theta_;
    double end_theta_;
    bool half_passed_;
};

class Pose3D
{
public:
    Pose3D()
        : translation_(Eigen::Vector3d::Zero()),
          quaternion_(Eigen::Quaterniond::Identity()) {}

    Pose3D(const Eigen::Vector3d &translation, const Eigen::Quaterniond &quaternion)
        : translation_(translation),
          quaternion_(quaternion) {}

    Pose3D(double x, double y, double z, const Eigen::Quaterniond &quaternion)
        : translation_(Eigen::Vector3d(x, y, z)),
          quaternion_(quaternion) {}

    Pose3D(const msg::Pose3DMessage &pose3d)
        : translation_(Eigen::Vector3d(pose3d.x(),
                                       pose3d.y(),
                                       pose3d.z())),
          quaternion_(Eigen::Quaterniond(pose3d.qw(),
                                         pose3d.qx(),
                                         pose3d.qy(),
                                         pose3d.qz())) {}

    Pose3D(double x, double y, double z, double qw, double qx, double qy, double qz)
        : translation_(Eigen::Vector3d(x, y, z)),
          quaternion_(Eigen::Quaterniond(qw, qx, qy, qz)) {}

    Pose3D(const Eigen::Affine3d& affine3d)
        : translation_(affine3d.translation()),
          quaternion_(affine3d.rotation()) {}

    Pose3D(const Pose3D& pose)
        : translation_(pose.GetTranslation()),
          quaternion_(pose.GetQuaternion()) {}

    /*
     *  Get msg::Pose3DMessage
    */
    msg::Pose3DMessage GetPose3DMessage() const;
    /*
     *  Get Transformation Matrix
    */
    Eigen::Matrix4d GetTransformation() const;

    /*
     *  Get Translation Matrix
    */
    Eigen::Vector3d GetTranslation() const;

    /*
     *  Get Rotation Matrix
    */
    Eigen::Matrix3d GetRotation() const;

    /*
     *  Get Inverse Rotation Matrix
    */
    Eigen::Matrix3d GetRotationInv() const;

    /*
     *  Get Quaternion
    */
    Eigen::Quaterniond GetQuaternion() const;

    /*
    *  Get Affine3d
    */
    Eigen::Affine3d GetAffine3D() const;

private:
    Eigen::Vector3d translation_;
    Eigen::Quaterniond quaternion_;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


class LidarMotionCompensator
{
public:
    void CompensateCloudToStart(const std::vector<LidarPoint> &cloud_in, const Eigen::Affine3d &start_pose,
                                const timespec &start_time, const Eigen::Affine3d &end_pose, const timespec &end_time,
                                std::vector<LidarPoint> *cloud_out) const;

    void CompensateCloudToEnd(const std::vector<LidarPoint> &cloud_in, const Eigen::Affine3d &start_pose,
                              const timespec &start_time, const Eigen::Affine3d &end_pose, const timespec &end_time,
                              std::vector<LidarPoint> *cloud_out) const;

    void CompensateCloud(const std::vector<LidarPoint> &cloud_in, const Eigen::Affine3d &start_pose,
                         const timespec &start_time, const Eigen::Affine3d &end_pose, const timespec &end_time,
                         const timespec &to_time, std::vector<LidarPoint> *cloud_out) const;

    void CompensateCloud(const std::vector<LidarPoint> &cloud_in,
                         const std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> &poses,
                         const std::vector<timespec> &timestamps, const timespec &to_time,
                         std::vector<LidarPoint> *cloud_out) const;

private:
    void CompensatePoint(const LidarPoint &pi, const Eigen::Affine3d &from_pose_eigen,
                         const Eigen::Affine3d &to_pose_eigen, LidarPoint *po) const;

    msg::Pose3DMessage EigenToProto(const Eigen::Affine3d &pose_eigen) const;

    Eigen::Affine3d ProtoToEigen(const msg::Pose3DMessage &pose_proto) const;
};

class Pose3DInterpolator : public bf::BaseInterpolator<msg::Pose3DMessage>
{
public:
    Pose3DInterpolator(size_t capacity) : bf::BaseInterpolator<msg::Pose3DMessage>(capacity, "Pose3D") {}
    Pose3DInterpolator() : bf::BaseInterpolator<msg::Pose3DMessage>("Pose3D") {}

    /*
    *   Return pose3D class with a timestamp
    */
    bool GetTimedPose3D(const timespec &input_time, Pose3D *result_data);

protected:
    /*
        Given begin and end timespec/Pose3D. Interpolate Pose3D.
     */
    virtual msg::Pose3DMessage InterpolateDataByTimestamp(const timespec &input_time,
            const timespec &begin_time,
            const msg::Pose3DMessage &begin_data,
            const timespec &end_time,
            const msg::Pose3DMessage &end_data) const;

    /*
        Given begin and end timespec/Pose3D. Extrapolate Pose3D.
     */
    virtual msg::Pose3DMessage ExtrapolateDataByTimestamp(const timespec &input_time,
            const timespec &begin_time,
            const msg::Pose3DMessage &begin_data,
            const timespec &end_time,
            const msg::Pose3DMessage &end_data) const;
};
