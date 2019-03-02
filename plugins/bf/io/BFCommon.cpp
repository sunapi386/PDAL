//
// Created by jsun on 2/15/19.
//

#include "BFCommon.hpp"

std::string rtkToString(msg::RTKMessage &rtkMessage)
{
    std::stringstream sstream;
    int precision = std::numeric_limits<double>::max_digits10;
    sstream << std::setprecision(precision);
    sstream << "latitude:" << rtkMessage.latitude() << " ";
    sstream << "longitude:" << rtkMessage.longitude() << " ";
    sstream << "altitude:" << rtkMessage.altitude() << " ";
    sstream << "heading:" << rtkMessage.heading() << " ";
    sstream << "roll:" << rtkMessage.roll() << " ";
    sstream << "pitch:" << rtkMessage.pitch();
    return sstream.str();
}

std::string TimespecToString(const timespec &timestamp, bool useDash)
{
    std::stringstream sstream;
    sstream << std::setfill('0') << std::setw(12) << timestamp.tv_sec;
    sstream << (useDash ? "-" : ".");
    sstream << std::setfill('0') << std::setw(9) << timestamp.tv_nsec;
    return sstream.str();
}

double TimespecToDouble(const timespec& timestamp)
{
    return timestamp.tv_sec + double(timestamp.tv_nsec) / 1e9;
}

timespec DoubleToTimespec(const double double_time)
{
    timespec timespec_time;
    timespec_time.tv_sec = static_cast<time_t>(double_time);
    timespec_time.tv_nsec = static_cast<long>(
                                (double_time - static_cast<double>(timespec_time.tv_sec)) * 1e9);
    return timespec_time;
}

void savePCToCSV(PointCloudRef pc)
{
    std::string timespecString = TimespecToString(pc.timePlaceSegment.start.time, true);
    std::string rtkString = rtkToString(pc.timePlaceSegment.start.rtkMessage);
    std::string name = timespecString + "_" + rtkString;
    std::ofstream outfile(name + ".csv");
    outfile << "idx, x, y, z, intensity, timestamp, laser_id, lidar_angle\n";

    for (uint i = 0; i < pc.points.size(); ++i)
    {
        LidarPointRef p = pc.points[i];
        outfile << i << ", " << p.x << ", " << p.y << ", " << p.z << ", "
                << unsigned(p.intensity) << ", " << TimespecToString(DoubleToTimespec(p.timestamp)) << ", "
                << unsigned(p.laser_id) << ", " << p.lidar_angle << "\n";
    }
    outfile.close();
}

BFArgs readArgsFromJson(Json::Value &root)
{
    BFArgs args;
    args.fileRtk = root.get("rtk", "").asString();
    args.fileLidar = root.get("lidar", "").asString();
    args.fileAffine = root.get("affine", "").asString();
    args.dumpFrames = root.get("dumpFrames", false).asBool();
    args.nFramesSkip = root.get("nFramesSkip", false).asInt();
    args.nFramesRead = root.get("nFramesRead", false).asInt();
    args.mDistanceJump = root.get("mDistanceJump", false).asDouble();
    args.mCompensate = root.get("mCompensate", false).asBool();
    args.nPointsReadLimit = root.get("nPointsReadLimit", -1).asInt();
    args.mLidarDistanceReturnFilter = root.get("mLidarDistanceReturnFilter", -1.0).asDouble();
    return args;
}

Eigen::Affine3d readAffineFromJson(Json::Value &root)
{
    Eigen::Affine3d affine3d;
    // todo: remove hard coded affine
//    Json::Value &value = root["translation"];
//    affine3d.translation() = value;
//    affine3d.linear() =  root["linear"];
    affine3d.translation() = Eigen::Vector3d(0.101814, 0.329364, 2.62183);
    affine3d.linear() << 0.999814, 0.0088356, 0.0171308,
                    -0.00869718, 0.999929, -0.00813756,
                    -0.0172015, 0.00798706, 0.99982;
    return affine3d;
}
bool jsonValueFromFile(std::string &filename, Json::Value &root)
{
    Json::Reader jsonReader;
    std::ifstream inFile(filename);
    bool parseSuccess = jsonReader.parse(inFile, root);
    inFile.close();
    return parseSuccess;
}

#include <cmath>
#include <unordered_map>

#define earthRadiusKm 6371.0

// This function converts decimal degrees to radians
double deg2rad(double deg)
{
    return (deg * M_PI / 180);
}

double constrainAngle(double x)
{
    x = fmod(x,360);
    if (x < 0)
        x += 360;
    return x;
}

//  This function converts radians to decimal degrees
double rad2deg(double rad)
{
    return constrainAngle(rad * 180 / M_PI);
}

/**
 * Returns the distance between two points on the Earth.
 * Direct translation from http://en.wikipedia.org/wiki/Haversine_formula
 * @param lat1d Latitude of the first point in degrees
 * @param lon1d Longitude of the first point in degrees
 * @param lat2d Latitude of the second point in degrees
 * @param lon2d Longitude of the second point in degrees
 * @return The distance between the two points in kilometers
 */
double distanceEarth(double lat1d, double lon1d, double lat2d, double lon2d)
{
    double lat1r, lon1r, lat2r, lon2r, u, v;
    lat1r = deg2rad(lat1d);
    lon1r = deg2rad(lon1d);
    lat2r = deg2rad(lat2d);
    lon2r = deg2rad(lon2d);
    u = sin((lat2r - lat1r)/2);
    v = sin((lon2r - lon1r)/2);
    return 2.0 * earthRadiusKm * asin(sqrt(u * u + cos(lat1r) * cos(lat2r) * v * v));
}

double distanceMeters(msg::RTKMessage &m1, msg::RTKMessage &m2)
{
    double lat1 = m1.latitude();
    double lon1 = m1.longitude();
    double lat2 = m2.latitude();
    double lon2 = m2.longitude();
    double distanceInKilometers = distanceEarth(lat1, lon1, lat2, lon2);
    double meters = distanceInKilometers * 1000;
    return meters;
}

std::string preciseDoubleStr(double d, uint precision)
{
    std::stringstream stream;
    stream << std::fixed << std::setprecision(precision) << d;
    return stream.str();
}

double radiansFromCoord(double y, double x)
{
    double theta = -std::atan2(y, x);
    return theta;
}

void LidarTimestampInterpolator::UpdateTimestamp(const double scan_period, const timespec &start_time, std::vector<LidarPoint> *laser_cloud_in)
{
    size_t cloud_size = laser_cloud_in->size();
    if (cloud_size == 0) return;
    scan_period_ = scan_period;

    // determine scan start and end orientations
    start_theta_ = -std::atan2((*laser_cloud_in)[0].y, (*laser_cloud_in)[0].x);
    end_theta_ = -std::atan2((*laser_cloud_in)[cloud_size - 1].y, (*laser_cloud_in)[cloud_size - 1].x) + 2 * M_PI;
    if (end_theta_ - start_theta_ > 3 * M_PI)
    {
        end_theta_ -= 2 * M_PI;
    }
    else if (end_theta_ - start_theta_ < M_PI)
    {
        end_theta_ += 2 * M_PI;
    }
    half_passed_ = false;

    for (size_t i = 0; i < cloud_size; i++)
    {
        double theta = -std::atan2((*laser_cloud_in)[i].y, (*laser_cloud_in)[i].x);
        (*laser_cloud_in)[i].timestamp = start_time.tv_sec + start_time.tv_nsec / 1e9 + GetRelativeTime(theta);
    }
}

double LidarTimestampInterpolator::GetRelativeTime(double theta)
{
    if (!half_passed_)
    {
        if (theta < start_theta_ - M_PI / 2)
        {
            theta += 2 * M_PI;
        }
        else if (theta > start_theta_ + M_PI * 3 / 2)
        {
            theta -= 2 * M_PI;
        }
        if (theta - start_theta_ > M_PI)
        {
            half_passed_ = true;
        }
    }
    else
    {
        theta += 2 * M_PI;
        if (theta < end_theta_ - M_PI * 3 / 2)
        {
            theta += 2 * M_PI;
        }
        else if (theta > end_theta_ + M_PI / 2)
        {
            theta -= 2 * M_PI;
        }
    }
    return scan_period_ * (theta - start_theta_) / (end_theta_ - start_theta_);
}


void LidarMotionCompensator::CompensateCloudToStart(const std::vector<LidarPoint>& cloud_in,
        const Eigen::Affine3d& start_pose, const timespec& start_time,
        const Eigen::Affine3d& end_pose, const timespec& end_time,
        std::vector<LidarPoint>* cloud_out) const
{
    CompensateCloud(cloud_in, start_pose, start_time, end_pose, end_time, start_time, cloud_out);
}

void LidarMotionCompensator::CompensateCloudToEnd(const std::vector<LidarPoint>& cloud_in,
        const Eigen::Affine3d& start_pose, const timespec& start_time,
        const Eigen::Affine3d& end_pose, const timespec& end_time,
        std::vector<LidarPoint>* cloud_out) const
{
    CompensateCloud(cloud_in, start_pose, start_time, end_pose, end_time, end_time, cloud_out);
}

void LidarMotionCompensator::CompensateCloud(const std::vector<LidarPoint>& cloud_in,
        const Eigen::Affine3d& start_pose, const timespec& start_time,
        const Eigen::Affine3d& end_pose, const timespec& end_time,
        const timespec& to_time, std::vector<LidarPoint>* cloud_out) const
{
    std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> poses({start_pose, end_pose});
    std::vector<timespec> timestamps({start_time, end_time});
    CompensateCloud(cloud_in, poses, timestamps, to_time, cloud_out);
}

struct TimespecHash
{
    size_t operator()(const timespec &timestamp) const
    {
        return std::hash<unsigned long long>()(
                   static_cast<unsigned long long>(timestamp.tv_sec) * 1000000000ull +
                   static_cast<unsigned long long>(timestamp.tv_nsec));
    }
};

struct TimespecEqual
{
    bool operator()(const timespec& lhs, const timespec& rhs) const
    {
        return (lhs.tv_sec == rhs.tv_sec) && (lhs.tv_nsec == rhs.tv_nsec);
    }
};

void LidarMotionCompensator::CompensateCloud(
    const std::vector<LidarPoint>& cloud_in,
    const std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>>& poses,
    const std::vector<timespec>& timestamps, const timespec& to_time, std::vector<LidarPoint>* cloud_out) const
{
    if (poses.size() != timestamps.size())
    {
//        ER("The number of poses should equal the number of time stamps. Please check input.");
        return;
    }
    if (poses.size() < 2)
    {
//        ER("Need 2 or more poses to interpolate. Please check input.");
        return;
    }
    cloud_out->resize(cloud_in.size());

    Pose3DInterpolator pose_interpolator;
    for (size_t i = 0; i < poses.size(); i++)
    {
        pose_interpolator.InsertNewData(timestamps[i], EigenToProto(poses[i]));
    }

    msg::Pose3DMessage from_pose_proto, to_pose_proto;
    pose_interpolator.GetTimedData(to_time, &to_pose_proto);
    Eigen::Affine3d to_pose_eigen = ProtoToEigen(to_pose_proto);

    // stores pose for the same timestamp
    std::unordered_map<timespec, Eigen::Affine3d, TimespecHash, TimespecEqual,
        Eigen::aligned_allocator<std::pair<timespec, Eigen::Affine3d>>> pose_map;
    pose_map[to_time] = to_pose_eigen;

    for (size_t i = 0; i < cloud_in.size(); i++)
    {
        timespec from_time = DoubleToTimespec(cloud_in[i].timestamp);
        if (pose_map.find(from_time) == pose_map.end())
        {
            pose_interpolator.GetTimedData(from_time, &from_pose_proto);
            Eigen::Affine3d from_pose_eigen = ProtoToEigen(from_pose_proto);
            pose_map[from_time] = from_pose_eigen;
        }
        CompensatePoint(cloud_in[i], pose_map[from_time], to_pose_eigen, &((*cloud_out)[i]));
    }
}

void LidarMotionCompensator::CompensatePoint(const LidarPoint& pi, const Eigen::Affine3d& from_pose_eigen,
        const Eigen::Affine3d& to_pose_eigen, LidarPoint* po) const
{
    Eigen::Vector3d point_in(pi.x, pi.y, pi.z);
    Eigen::Vector3d point_out = to_pose_eigen.inverse() * from_pose_eigen * point_in;

    *po = pi;
    po->x = point_out[0];
    po->y = point_out[1];
    po->z = point_out[2];
}

msg::Pose3DMessage LidarMotionCompensator::EigenToProto(const Eigen::Affine3d& pose_eigen) const
{
    msg::Pose3DMessage pose_proto;
    Eigen::Quaternion<double> quat(pose_eigen.rotation());
    pose_proto.set_x(pose_eigen.translation()[0]);
    pose_proto.set_y(pose_eigen.translation()[1]);
    pose_proto.set_z(pose_eigen.translation()[2]);
    pose_proto.set_qw(quat.w());
    pose_proto.set_qx(quat.x());
    pose_proto.set_qy(quat.y());
    pose_proto.set_qz(quat.z());
    return pose_proto;
}

Eigen::Affine3d LidarMotionCompensator::ProtoToEigen(const msg::Pose3DMessage& pose_proto) const
{
    Eigen::Affine3d pose_eigen;
    pose_eigen.translation() = Eigen::Vector3d(pose_proto.x(), pose_proto.y(), pose_proto.z());
    Eigen::Quaternion<double> quat(pose_proto.qw(), pose_proto.qx(), pose_proto.qy(), pose_proto.qz());
    pose_eigen.linear() = quat.matrix();
    return pose_eigen;
}

bool Pose3DInterpolator::GetTimedPose3D(const timespec &input_time, Pose3D *result_data)
{
    msg::Pose3DMessage interpolted_pose3d_msg;
    if (GetTimedData(input_time, &interpolted_pose3d_msg))
    {
        *result_data = Pose3D(interpolted_pose3d_msg);
        return true;
    }
    else
    {
        return false;
    }
}

msg::Pose3DMessage Pose3DInterpolator::ExtrapolateDataByTimestamp(const timespec &input_time,
        const timespec &begin_time,
        const msg::Pose3DMessage &begin_data,
        const timespec &end_time,
        const msg::Pose3DMessage &end_data) const
{
    msg::Pose3DMessage result;

    if (SignedTimestampDiff(input_time, begin_time) > 0)
    {
        const double weight = SignedTimestampDiff(input_time, begin_time) /
                              SignedTimestampDiff(begin_time, end_time);

        if (begin_data.has_x() && end_data.has_x())
        {
            result.set_x(ExtrapolatorAheadByWeight(begin_data.x(), end_data.x(), weight));
        }

        if (begin_data.has_y() && end_data.has_y())
        {
            result.set_y(ExtrapolatorAheadByWeight(begin_data.y(), end_data.y(), weight));
        }

        if (begin_data.has_z() && end_data.has_z())
        {
            result.set_z(ExtrapolatorAheadByWeight(begin_data.z(), end_data.z(), weight));
        }

        if (begin_data.has_qw() && end_data.has_qw() &&
                begin_data.has_qx() && end_data.has_qx() &&
                begin_data.has_qy() && end_data.has_qy() &&
                begin_data.has_qz() && end_data.has_qz())
        {
            Eigen::Quaterniond begin_quaternion(begin_data.qw(), begin_data.qx(), begin_data.qy(), begin_data.qz());
            Eigen::Quaterniond end_quaternion(end_data.qw(), end_data.qx(), end_data.qy(), end_data.qz());

            // Step1: Calculate begin_data and end_data quaternion space angle difference.
            Eigen::AngleAxisd rot(end_quaternion.inverse() * begin_quaternion);
            double angle_interpolated = rot.angle();
            while (angle_interpolated > M_PI)
            {
                angle_interpolated -= 2 * M_PI;
            }

            // Step2: Extrapolate for the quaternion space angle difference between end_data and result_data.
            angle_interpolated = std::fmod(angle_interpolated * (1 + weight), 2 * M_PI);
            Eigen::Quaterniond rot_ext(Eigen::AngleAxisd(angle_interpolated, rot.axis()));
            Eigen::Quaterniond result_quaternion = end_quaternion * rot_ext;

            result.set_qw(result_quaternion.w());
            result.set_qx(result_quaternion.x());
            result.set_qy(result_quaternion.y());
            result.set_qz(result_quaternion.z());
        }
    }
    else
    {
        const double weight = SignedTimestampDiff(end_time, input_time) /
                              SignedTimestampDiff(begin_time, end_time);
        if (begin_data.has_x() && end_data.has_x())
        {
            result.set_x(ExtrapolatorBehindByWeight(begin_data.x(), end_data.x(), weight));
        }

        if (begin_data.has_y() && end_data.has_y())
        {
            result.set_y(ExtrapolatorBehindByWeight(begin_data.y(), end_data.y(), weight));
        }

        if (begin_data.has_z() && end_data.has_z())
        {
            result.set_z(ExtrapolatorBehindByWeight(begin_data.z(), end_data.z(), weight));
        }

        if (begin_data.has_qw() && end_data.has_qw() &&
                begin_data.has_qx() && end_data.has_qx() &&
                begin_data.has_qy() && end_data.has_qy() &&
                begin_data.has_qz() && end_data.has_qz())
        {
            Eigen::Quaterniond begin_quaternion(begin_data.qw(), begin_data.qx(), begin_data.qy(), begin_data.qz());
            Eigen::Quaterniond end_quaternion(end_data.qw(), end_data.qx(), end_data.qy(), end_data.qz());

            // Step1: Calculate begin_data and end_data quaternion space angle difference.
            Eigen::AngleAxisd rot(end_quaternion * begin_quaternion.inverse());
            double angle_interpolated = rot.angle();
            while (angle_interpolated > M_PI)
            {
                angle_interpolated -= 2 * M_PI;
            }
            // Step2: Extrapolate for the quaternion space angle difference between begin_data and result_data.
            angle_interpolated = std::fmod(angle_interpolated * (1 + weight), 2 * M_PI);
            Eigen::Quaterniond rot_ext(Eigen::AngleAxisd(angle_interpolated, rot.axis()));
            Eigen::Quaterniond result_quaternion = rot_ext * begin_quaternion;

            result.set_qw(result_quaternion.w());
            result.set_qx(result_quaternion.x());
            result.set_qy(result_quaternion.y());
            result.set_qz(result_quaternion.z());
        }
    }

    return result;
}

msg::Pose3DMessage Pose3DInterpolator::InterpolateDataByTimestamp(const timespec &input_time,
        const timespec &begin_time,
        const msg::Pose3DMessage &begin_data,
        const timespec &end_time,
        const msg::Pose3DMessage &end_data) const
{
    msg::Pose3DMessage result;

    const double weight = SignedTimestampDiff(begin_time, input_time) /
                          SignedTimestampDiff(begin_time, end_time);

    if (begin_data.has_x() && end_data.has_x())
    {
        result.set_x(InterpolatorByWeight(begin_data.x(), end_data.x(), weight));
    }

    if (begin_data.has_y() && end_data.has_y())
    {
        result.set_y(InterpolatorByWeight(begin_data.y(), end_data.y(), weight));
    }

    if (begin_data.has_z() && end_data.has_z())
    {
        result.set_z(InterpolatorByWeight(begin_data.z(), end_data.z(), weight));
    }
    if (begin_data.has_qw() && end_data.has_qw() &&
            begin_data.has_qx() && end_data.has_qx() &&
            begin_data.has_qy() && end_data.has_qy() &&
            begin_data.has_qz() && end_data.has_qz())
    {
        /*
        * Step1: Transform row/pitch/yaw(in radians) to quaternions
        */
        Eigen::Quaterniond begin_quaternion(begin_data.qw(), begin_data.qx(), begin_data.qy(), begin_data.qz());
        Eigen::Quaterniond end_quaternion(end_data.qw(), end_data.qx(), end_data.qy(), end_data.qz());

        /*
         * Step2: Do the slerp(spherical linear interpolation)
         */
        Eigen::Quaterniond result_quaternion = begin_quaternion.slerp(weight, end_quaternion);

        result.set_qw(result_quaternion.w());
        result.set_qx(result_quaternion.x());
        result.set_qy(result_quaternion.y());
        result.set_qz(result_quaternion.z());
    }
    return result;
}


Eigen::Matrix4d Pose3D::GetTransformation() const
{
    Eigen::Matrix4d transform;
    transform.setIdentity();
    transform.block<3, 3>(0, 0) = quaternion_.toRotationMatrix();
    transform.block<3, 1>(0, 3) = translation_;
    return transform;
}

msg::Pose3DMessage Pose3D::GetPose3DMessage() const
{
    msg::Pose3DMessage pose_msg;
    pose_msg.set_x(translation_(0));
    pose_msg.set_y(translation_(1));
    pose_msg.set_z(translation_(2));
    pose_msg.set_qw(quaternion_.w());
    pose_msg.set_qx(quaternion_.x());
    pose_msg.set_qy(quaternion_.y());
    pose_msg.set_qz(quaternion_.z());
    return pose_msg;
}

Eigen::Vector3d Pose3D::GetTranslation() const
{
    return translation_;
}

Eigen::Matrix3d Pose3D::GetRotation() const
{
    return quaternion_.toRotationMatrix();
}

Eigen::Matrix3d Pose3D::GetRotationInv() const
{
    return GetRotation().transpose().eval();
}

Eigen::Quaterniond Pose3D::GetQuaternion() const
{
    return quaternion_;
}

Eigen::Affine3d Pose3D::GetAffine3D() const
{
    Eigen::Affine3d affine3d = Eigen::Affine3d::Identity();
    affine3d.linear() = quaternion_.toRotationMatrix();
    affine3d.translation() = translation_;
    return affine3d;
}
