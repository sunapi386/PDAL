//
// Created by jsun on 2/15/19.
//

#include "BFCommon.hpp"

std::string rtkToString(msg::RTKMessage &rtkMessage)
{
    std::stringstream sstream;
    int precision = std::numeric_limits<double>::max_digits10;
    sstream << std::setprecision(precision);
    sstream << rtkMessage.longitude();
    sstream << "-";
    sstream << rtkMessage.latitude();
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
    return static_cast<double>(timestamp.tv_sec) +
           static_cast<double>(timestamp.tv_nsec) / 1e9;
}

timespec DoubleToTimespec(const double double_time)
{
    timespec timespec_time;
    timespec_time.tv_sec = static_cast<time_t>(double_time);
    timespec_time.tv_nsec = static_cast<long>(
                                (double_time - static_cast<double>(timespec_time.tv_sec)) * 1e9);
    return timespec_time;
}
//void printLidarPC(PointCloudRef pc) {
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

void savePCToCSV(PointCloudRef pc, std::string &name)
{
    // todo: save the points into PDAL acceptable format
    std::ofstream outfile(name + ".csv");
    outfile << "idx, x, y, z, intensity, timestamp, laser_id, lidar_angle\n";

    for (uint i = 0; i < pc.points.size(); ++i)
    {
        LidarPointRef p = pc.points[i];
        outfile << std::setfill('0') << std::setw(10) << std::fixed << std::setprecision(8);
        outfile << i << ", " << p.x << ", " << p.y << ", " << p.z << ", ";
        outfile << std::setfill('0') << std::setw(7) << std::fixed << std::setprecision(8)
                << unsigned(p.intensity) << ", "
                << TimespecToString(DoubleToTimespec(p.timestamp)) << ", ";
        outfile << std::setfill('0') << std::setw(6) << std::fixed << std::setprecision(5)
                << unsigned(p.laser_id) << ", " << p.lidar_angle << "\n";
    }
    outfile.close();
}

BFArgs readArgsFromJson(Json::Value &root)
{

    BFArgs args;
    // use values from json file if not specified with arguments
    args.fileRtk = root.get("rtk", "").asString();
    args.fileLidar = root.get("lidar", "").asString();
    args.fileAffine = root.get("affine", "").asString();
    args.dumpFrames = root.get("dumpFrames", false).asBool();
    args.nFramesSkip = root.get("nFramesSkip", false).asInt();
    args.nFramesRead = root.get("nFramesRead", false).asInt();
    args.mDistanceJump = root.get("mDistanceJump", false).asDouble();
    args.mCompensate = root.get("mCompensate", false).asBool();
    args.nPointsReadLimit = root.get("nPointsReadLimit", -1).asInt();
    return args;
}

Eigen::Affine3d readAffineFromJson(Json::Value &root)
{
    Eigen::Affine3d affine3d;
//    Json::Value &value = root["translation"];
//    affine3d.translation() = value;
//    affine3d.linear() =  root["linear"];
    affine3d.translation() =
        Eigen::Vector3d(0.101814, 0.329364, 2.62183);
    affine3d.linear() << 0.999814, 0.0088356, 0.0171308, -0.00869718,
                    0.999929, -0.00813756, -0.0172015, 0.00798706, 0.99982;
    return affine3d;
}
bool jsonValueFromFile(std::string &filename, Json::Value &root)
{
    // we were given a json file
    Json::Reader jsonReader;
    std::ifstream inFile(filename);
    bool parseSuccess = jsonReader.parse(inFile, root);
    inFile.close();
    return parseSuccess;
}

#include <cmath>
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
