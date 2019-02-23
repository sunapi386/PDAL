//
// Created by jsun on 2/15/19.
//

#include "BFCommon.hpp"

PointCloud getLidarPoints(bf::Datum &datum)
{
    size_t point_num = datum.size / sizeof(LidarPoint);
    const LidarPoint *data_ptr = static_cast<LidarPoint *>(datum.data);
    return std::vector<LidarPoint>(data_ptr, data_ptr + point_num);
}

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

void writePCTextFile(PointCloud &pc, std::string &name)
{
    // todo: save the points into PDAL acceptable format
    std::ofstream outfile(name);
    outfile << "idx, x, y, z, intensity, timestamp, laser_id, lidar_angle\n";
    for (uint i = 0; i < pc.size(); ++i)
    {
        LidarPoint &p = pc[i];
        outfile << i << ", " << p.x << ", " << p.y << ", " << p.z << ", "
                << unsigned(p.intensity) << ", "
                << TimespecToString(DoubleToTimespec(p.timestamp)) << ", "
                << unsigned(p.laser_id) << ", " << unsigned(p.lidar_angle) << "\n";
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
