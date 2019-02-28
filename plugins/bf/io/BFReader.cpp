#include "BFReader.hpp"
#include "BFCommon.hpp"
#include <memory>
#include <pdal/util/ProgramArgs.hpp>
#include <pdal/util/FileUtils.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <json/json.h>

namespace pdal
{
static PluginInfo const s_info
{
    "readers.bf",
    "BF Reader (input file should be set to the JSON input)",
    "http://path/to/documentation"
};

CREATE_SHARED_STAGE(BFReader, s_info)

std::string BFReader::getName() const
{
    return s_info.name;
}

void BFReader::addArgs(ProgramArgs& args)
{
    // also change BFCommon readArgsFromJson()
    args.add("rtk", "BF file, RTK", m_args.fileRtk);
    args.add("lidar", "BF file, lidar", m_args.fileLidar);
    args.add("affine", "Text file, spatial transformation affine", m_args.fileAffine);
    args.add("dumpFrames", "Bool flag to dump lidar csv", m_args.dumpFrames);
    args.add("nFramesSkip", "Number of BF frames to skip", m_args.nFramesSkip);
    args.add("nFramesRead", "Number of BF frames to read", m_args.nFramesRead);
    args.add("nPointsReadLimit", "Debug: Limit the number of points in a frames to read", m_args.nPointsReadLimit);
    args.add("mDistanceJump", "Distance to wait until car moves", m_args.mDistanceJump);
    args.add("motionCompensate", "Perform motion compensation", m_args.mCompensate);
}

void BFReader::initialize()
{
    log()->get(LogLevel::Info) << "BFReader m_filename: " << m_filename << "\n";
    log()->get(LogLevel::Debug) << std::setprecision(precision);
    // load arguments from json file when provided, but manual args takes precedence
    if (!m_filename.empty())
    {
        Json::Value parsedJson;
        bool success = jsonValueFromFile(m_filename, parsedJson);
        if (!success)
        {
            throwError("Failed to parse JSON input file " + m_filename);
        }
        m_args = readArgsFromJson(parsedJson);
        checkForValidInputFiles();
        log()->get(LogLevel::Debug) << "Opening:\n-rtk=" << m_args.fileRtk << "\n-lidar="
                                    << m_args.fileLidar << "\n-affine=" << m_args.fileAffine << "\n";
        log()->get(LogLevel::Debug) << "-dumpFrames=" << m_args.dumpFrames << "\n";
        log()->get(LogLevel::Debug) << "-nFramesSkip=" << m_args.nFramesSkip << "\n";
        log()->get(LogLevel::Debug) << "-nFramesRead=" << m_args.nFramesRead << "\n";
        log()->get(LogLevel::Debug) << "-nPointsReadLimit=" << m_args.nPointsReadLimit << "\n";

        // handle rtk file
        bf::DatumParser rtkDatumParser(m_args.fileRtk);
        uint count = insertRtkDatumsIntoInterpolator(rtkDatumParser);
        log()->get(LogLevel::Debug) << "Inserted " << count << " rtk datums to interpolate\n";

        // handle lidar file
        m_datumParserLidar = make_unique<bf::DatumParser>(m_args.fileLidar);

        // handle affine file
        Json::Value affineJsonValue;
        success = jsonValueFromFile(m_args.fileAffine,affineJsonValue);
        if (!success)
        {
            throwError("Failed to parse JSON affine file " + m_args.fileAffine);
        }
        m_affine = readAffineFromJson(affineJsonValue);

    }
    else
    {
        throwError("Did not get an input filename");
    }
}

void BFReader::
done(PointTableRef)
{
//    log()->get(LogLevel::Info) << "Finished reading BF\n";
}

// corresponds to bf::msg::LidarPoint
void BFReader::addDimensions(PointLayoutPtr layout)
{
    using namespace Dimension;
    layout->registerDim(Id::X); // lat,lng
    layout->registerDim(Id::Y); // x,y,z are in meters
    layout->registerDim(Id::Z);
    layout->registerDim(Id::Intensity);
    layout->registerDim(Id::InternalTime); // timestamp
//    layout->registerDim(Id::GpsTime);      // from rtk
    m_LaserId = layout->registerOrAssignDim("LaserId", Type::Unsigned8);
// laser_id (e.g. 40 beam lidar has id 0-40)
    m_LidarAngle = layout->registerOrAssignDim("LidarAngle", Type::Double);
// lidar_angle (e.g. a full lidar rotation)
    layout->registerDim(Id::PointSourceId); // which frame id this point is from
}

void BFReader::ready(PointTableRef)
{
//    SpatialReference ref("EPSG:3857+5703");
    SpatialReference ref("EPSG:26910");
//    SpatialReference ref("EPSG:26910+5705");
//    SpatialReference ref("EPSG:3857");
    setSpatialReference(ref);
}


point_count_t BFReader::read(PointViewPtr view, point_count_t nPtsToRead)
{
    // Note that we don’t read more points than requested
    // 18446744073709551615 is 2^64-1
    log()->get(LogLevel::Info) << "Requested " << nPtsToRead << " points to be read\n";

    PointLayoutPtr layout = view->layout();

    // Determine the ID of the next point in the point view
    PointId nextId = view->size();

    bf::DatumParser datumParserLidar(m_args.fileLidar);
    bf::Datum datumLidar{};

    point_count_t nPtsRead = 0;
    int nBFDatumsRead = 0;
    int skip = m_args.nFramesSkip;

    auto time_point = std::chrono::system_clock::now();

    // lidar datum and rtk datum are in separate BF files
    // when we read a lidar datum, also read in a rtk datum
    // this of course assumes the capture frequency are same
    // i.e. both captured at 10Hz.
    msg::RTKMessage lastUsedRtkMsg;
    log()->get(LogLevel::Info) << "Motion compensation is " << (m_args.mCompensate ? "Disabled" : "Enabled") << "\n";

    while (nPtsRead < nPtsToRead &&
            nBFDatumsRead < m_args.nFramesRead &&
            datumParserLidar.GetDatum(datumLidar))
    {
        if (--skip > 0)
        {
            // log()->get(LogLevel::Debug) << "Skip Datum (" << skip << " to skip)\n";
            free(datumLidar.data);
            continue;
        }
//        auto pointCloud = fakeLidarPoint(datumLidar);
        auto pointCloud = getLidarPoints(datumLidar);

        free(datumLidar.data);
        // timestamp is not recorded in the lidar readings and the lidar_angle isn't used
        // approx. 200k points in a datum for 128 beam lidar
        auto segment = pointCloud.timePlaceSegment;

        double distTravelledDuringScan = distanceMeters(segment.start.rtkMessage, segment.finish.rtkMessage);
        log()->get(LogLevel::Debug) << "RTK travelled "
                                    << preciseDoubleStr(distTravelledDuringScan, 5) << "m during scan\n";

        // check and skips datums if they're too close to each other
        // make sure we have moved a certain distance first
        double distSinceLastRtkMsgUsed = distanceMeters(segment.start.rtkMessage, lastUsedRtkMsg);
        if (distSinceLastRtkMsgUsed < m_args.mDistanceJump)
        {
            log()->get(LogLevel::Debug) << "Datum skipped, distance moved: "
                                        << preciseDoubleStr(distSinceLastRtkMsgUsed, 5)
                                        << "m less than " << m_args.mDistanceJump << "m\n";
            continue;
        }
        lastUsedRtkMsg = segment.start.rtkMessage;
        log()->get(LogLevel::Info) << "Datum " << nBFDatumsRead << " "
                                   << preciseDoubleStr(unsigned(datumLidar.size)/ 1000.0, 5) << " KBytes\n";

        /* 0. The point cloud does not store capture time for each point, so we have to interpolate.
         * 1. The vehicle may be moving while we captured a scan, so we first must do motion compensation
         * 2. The lidar points were captured in lidar reference (center is lidar),
         *    we transform the coordinates into RTK frame, so measurements are from the RTK.
         *    This uses the affine json.
         * 3. We must then transform the RTK coordinates into a standardized global reference frame,
         *    so in our case it is UTM. We may change this to lat-lng later on.
         *
         */
        if (m_args.mCompensate)
        {
            mutatePC_doMotionCompensation(pointCloud);
        }
        mutatePC_referenceFromLidarToRTK(pointCloud);
        mutatePC_referenceFromRtkToUTM(pointCloud);

        if (m_args.dumpFrames)
        {
            std::string timespecString = TimespecToString(segment.start.time, true);
            std::string rtkString = rtkToString(segment.start.rtkMessage);
            std::string name = timespecString + "_" + rtkString;
            savePCToCSV(pointCloud, name);
        }


        // the values we read and put them into the PointView object
        for (LidarPointRef pt : pointCloud.points)
        {
            view->setField(Dimension::Id::X, nextId, pt.x);
            view->setField(Dimension::Id::Y, nextId, pt.y);
            view->setField(Dimension::Id::Z, nextId, pt.z);
            view->setField(Dimension::Id::Intensity, nextId, pt.intensity);
            view->setField(Dimension::Id::InternalTime, nextId, pt.timestamp);
//            view->setField(Dimension::Id::GpsTime, nextId, pt..);
            view->setField(m_LaserId, nextId, pt.laser_id);
            view->setField(m_LidarAngle, nextId, pt.lidar_angle);
            view->setField(Dimension::Id::PointSourceId, nextId, nBFDatumsRead);

            /*processOne(point);*/ // todo: add streaming

            nPtsRead++;
            nextId++;

        }
        nBFDatumsRead++;

        if (m_cb)
        {
            m_cb(*view, nextId);
        }
    }

    auto time_point_2 = std::chrono::system_clock::now();
    log()->get(LogLevel::Info) << "nFramesSkip: " << m_args.nFramesSkip << "\n";
    log()->get(LogLevel::Info) << "nBFDatumsRead: " << nBFDatumsRead << "\n";
    log()->get(LogLevel::Info) << "nPtsRead: " << nPtsRead << "\n";
    auto delta = std::chrono::duration_cast<std::chrono::milliseconds>(time_point_2 - time_point);
    log()->get(LogLevel::Info) << delta.count() << "ms" << "\n";

    return nPtsRead;
}

TimePlaceSegment BFReader::interpolateTimePlaceSegment(const bf::Datum &datumLidar) const
{
    msg::RTKMessage interpolatedScanStartRtkMsg;
    msg::RTKMessage interpolatedScanFinishRtkMsg;
    timespec timespecStart= DoubleToTimespec(TimespecToDouble(datumLidar.time) - 0.1);// 0.1s prior
    timespec timespecFinish = datumLidar.time;
    m_rtkInterpolator.GetTimedData(timespecStart, &interpolatedScanStartRtkMsg);
    m_rtkInterpolator.GetTimedData(timespecFinish, &interpolatedScanFinishRtkMsg);
//    TimePlaceSegment timePlaceSegment
//    {
//        .start = TimePlace{.time=timespecBegin, .rtkMessage=interpolatedScanStartRtkMsg},
//        .finish = TimePlace{.time=timespecBegin, .rtkMessage=interpolatedScanStartRtkMsg}
//    }; //  sorry, unimplemented: non-trivial designated initializers not supported
    TimePlace start = TimePlace(interpolatedScanStartRtkMsg, timespecStart);
    TimePlace finish = TimePlace(interpolatedScanFinishRtkMsg, timespecFinish);
    return TimePlaceSegment(start, finish);
}


void BFReader::checkForValidInputFiles()
{
    std::__cxx11::string missingFiles;
    if (!Utils::fileExists(m_args.fileRtk))
    {
        missingFiles += "\n-rtk=" + m_args.fileRtk + " ";
    }
    if (!Utils::fileExists(m_args.fileLidar))
    {
        missingFiles += "\n-lidar=" + m_args.fileLidar + " ";
    }
    if (!Utils::fileExists(m_args.fileAffine))
    {
        missingFiles += "\n-affine=" + m_args.fileAffine + " ";
    }
    if (!missingFiles.empty())
    {
        throwError("The specified files in " + m_filename + " were not found:" + missingFiles);
    }

}


/*

bool BFReader::processOne(PointRef &point)
{
    // todo: add streaming option
    using namespace Dimension;
    point.setField(Id::X, pt.x);
    point.setField(Id::Y, pt.y);
    point.setField(Id::Z, pt.z);
    point.setField(Id::Intensity, pt.intensity);
    point.setField(Id::InternalTime, TimespecToDouble(timespec));
    point.setField(Id::GpsTime, TimespecToDouble(timespec));
    point.setField(m_LaserId, pt.laser_id);
    point.setField(m_LidarAngle, pt.lidar_angle);
    point.setField(Id::PointSourceId, nFramesRead);
    return true;
}
*/



uint BFReader::insertRtkDatumsIntoInterpolator(bf::DatumParser &parser)
{
    uint count = 0;
    bf::Datum datum {};
    while (parser.GetDatum(datum))
    {
        msg::RTKMessage rtkMessage;
        timespec &timespec = datum.time;

        bool success = rtkMessage.ParseFromArray(datum.data, datum.size);
        free(datum.data);

        if (!success)
        {
            log()->get(LogLevel::Warning) << "Failed to ParseFromArray at " << TimespecToString(timespec) << "\n";
            continue;
        }

        m_rtkInterpolator.InsertNewData(timespec, rtkMessage, false);
        count++;
    }

    return count;
}

void BFReader::mutatePC_referenceFromLidarToRTK(PointCloudRef pcIn)
{
    // for every point, apply a transform from the affine matrix
    for (LidarPointRef pt : pcIn.points)
    {
        affineSinglePoint(pt, m_affine);
    }
}

void BFReader::affineSinglePoint(LidarPointRef pt, const Eigen::Affine3d &affine)
{
    Eigen::Vector3d pt3d(pt.x, pt.y, pt.z);
    pt3d = affine * pt3d;
    pt.x = pt3d[0];
    pt.y = pt3d[1];
    pt.z = pt3d[2];
}

Eigen::Affine3d BFReader::createAffineFromRtkMessage(msg::RTKMessage &rtkMessage)
{
    int utmZone;
    bool utmNorth;
    double utmX, utmY;
    double utmZ = rtkMessage.altitude();

    // todo: look into handling altitude;
    // todo: for performance simplify the lidar-rtk and rtk-utm transform into a single affine

    // with the x,y do transform of the lidarPt (use original row, pitch, yaw, altitude)
    // create affine from x,y,z,r,p,y (with this 6 parameters) we can construct affine transform for lidarPt to UTM
    GeographicLib::UTMUPS::Forward(rtkMessage.latitude(), rtkMessage.longitude(), utmZone, utmNorth, utmX, utmY);


    if (m_rtkFirstUtmX == 0 && m_rtkFirstUtmY == 0)
    {
        log()->get(LogLevel::Debug) << "m_rtkFirstFrameUtmX=" << utmX << "\n";
        log()->get(LogLevel::Debug) << "m_rtkFirstFrameUtmY=" << utmY << "\n";
        m_rtkFirstUtmX = utmX;
        m_rtkFirstUtmY = utmY;
    }

    // any units in meters, we'll have to project into UTM. Because we can't work with lat/lng in meters
    // think of UTM as a tool to treat global coordinates (radial) as euclidean space (flat)

    Eigen::Affine3d affine3d = Eigen::Affine3d::Identity();
//    affine3d.translation() = Eigen::Vector3d(rtkMsg.latitude(), rtkMsg.longitude(), utmZ);
//    affine3d.translation() = Eigen::Vector3d(utmX, utmY, utmZ);
    affine3d.translation() = Eigen::Vector3d(utmX - m_rtkFirstUtmX, utmY - m_rtkFirstUtmY, utmZ);

    double roll = deg2rad(rtkMessage.roll());
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    double pitch = deg2rad(rtkMessage.pitch());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    double yaw = M_PI / 2 - deg2rad(rtkMessage.heading());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaternion<double> quaternion =  yawAngle * rollAngle * pitchAngle;
    affine3d.linear() = quaternion.matrix();

    log()->get(LogLevel::Debug) << "utmZone=" << utmZone << ", utmNorth=" << utmNorth << "\n";
    log()->get(LogLevel::Debug) << "utmX=" << utmX << ", utmY=" << utmY << ", utmZ=" << utmZ << "\n";
    log()->get(LogLevel::Debug) << "translation=" << affine3d.translation() << "\n";
    log()->get(LogLevel::Debug) << "roll=" << roll << "\n";
    log()->get(LogLevel::Debug) << "pitch=" << pitch << "\n";
    log()->get(LogLevel::Debug) << "yaw=" << yaw << "\n";
    log()->get(LogLevel::Debug) << "affine=\n" << affine3d.matrix() << "\n";
    return affine3d;
}

void BFReader::mutatePC_referenceFromRtkToUTM(PointCloudRef cloud)
{
    msg::RTKMessage &rtkMsgStart = cloud.timePlaceSegment.start.rtkMessage;
    msg::RTKMessage &rtkMsgFinish = cloud.timePlaceSegment.finish.rtkMessage;
    log()->get(LogLevel::Debug) << "mutatePC_referenceFromRtkToUTM.rtkMsg=" << rtkMsgStart.longitude() << ", " << rtkMsgStart.latitude() << ", " << rtkMsgStart.altitude() << " H:" << rtkMsgStart.heading() << "\n";

//    const Eigen::Affine3d &affineFromRtkMessageStart = createAffineFromRtkMessage(rtkMsgStart);
    const Eigen::Affine3d &affineFromRtkMessageFinish = createAffineFromRtkMessage(rtkMsgFinish);

    for (LidarPointRef pt : cloud.points)
    {
        affineSinglePoint(pt, affineFromRtkMessageFinish);
    }
}

void BFReader::mutatePC_doMotionCompensation(PointCloudRef cloud)
{
    // todo: fix efficiency?
    for (LidarPointRef point : cloud.points)
    {
        compensatePoint(point);
    }
}

/*!
 * The reason for having a point-level compensation is we may want to do streaming for large file sizes
 * @param point
 */
void BFReader::compensatePoint(LidarPointRef point)
{
    msg::RTKMessage interpolatedLocation;
    m_rtkInterpolator.GetTimedData(DoubleToTimespec(point.timestamp), &interpolatedLocation);

    Eigen::Affine3d affine3d = Eigen::Affine3d::Identity();
    affine3d.translation() = Eigen::Vector3d(interpolatedLocation.longitude(),
                             interpolatedLocation.latitude(),
                             interpolatedLocation.altitude());

    Eigen::AngleAxisd rollAngle(deg2rad(interpolatedLocation.roll()), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(deg2rad(interpolatedLocation.pitch()), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(M_PI / 2 - deg2rad(interpolatedLocation.heading()), Eigen::Vector3d::UnitZ());
    Eigen::Quaternion<double> quaternion =  yawAngle * rollAngle * pitchAngle;
    affine3d.linear() = quaternion.matrix();

    affineSinglePoint(point, affine3d);
}


/*!
 * Copied from BF messages
 */
struct BFLidarPointSerialized
{
    float x = 0;
    float y = 0;
    float z = 0;
    uint8_t intensity = 0;
    double timestamp = 0;
    uint8_t laser_id = 0;
    uint16_t lidar_angle = 0;
};


PointCloud BFReader::fakeLidarPoint(bf::Datum &datum)
{
    PointCloud pointCloud = getLidarPoints(datum);
    LidarPoint &point = pointCloud.points.front();

    double latitude = pointCloud.timePlaceSegment.start.rtkMessage.latitude();
    double longitude = pointCloud.timePlaceSegment.start.rtkMessage.longitude();
    double altitude = pointCloud.timePlaceSegment.start.rtkMessage.altitude();

    LidarPoint pt = point;

    pt.x = 0;
    pt.y = 0;
    pt.z = 0;
    pointCloud.points.clear();
    pointCloud.points.emplace_back(pt);


//{
//    LidarPoint xLidarPoint = point, yLidarPoint = point, zLidarPoint = point, lidarPoint = point;
//
//    lidarPoint.x = 0;
//    lidarPoint.y = 0;
//    lidarPoint.z = 0;
//
//    xLidarPoint.x = 1;
//    xLidarPoint.y = 0;
//    xLidarPoint.z = 0;
//
//    yLidarPoint.x = 0;
//    yLidarPoint.y = 1;
//    yLidarPoint.z = 0;
//
//    zLidarPoint.x = 0;
//    zLidarPoint.y = 0;
//    zLidarPoint.z = 1;
//
//    pointCloud.points.clear();
//    pointCloud.points.emplace_back(lidarPoint);
//    pointCloud.points.emplace_back(xLidarPoint);
//    pointCloud.points.emplace_back(yLidarPoint);
//    pointCloud.points.emplace_back(zLidarPoint);
//}


    return pointCloud;
}


PointCloud BFReader::getLidarPoints(bf::Datum &datum)
{
    PointCloud pointCloud;
    pointCloud.timePlaceSegment = interpolateTimePlaceSegment(datum);

    // add interpolated time to each point read
    double startTime = TimespecToDouble(pointCloud.timePlaceSegment.start.time);
    double finishTime = TimespecToDouble(pointCloud.timePlaceSegment.finish.time);
    double travelTimeSeconds = finishTime - startTime;
    double radiansPerSecond = 2 * M_PI / travelTimeSeconds;

    size_t point_num = datum.size / sizeof(BFLidarPointSerialized);
    const BFLidarPointSerialized *data_ptr = static_cast<BFLidarPointSerialized *>(datum.data);
    auto readPoints = std::vector<BFLidarPointSerialized>(data_ptr, data_ptr + point_num);
    auto outPoints = std::vector<LidarPoint>();
    outPoints.reserve(readPoints.size());
    double y = readPoints.front().y;
    double x = readPoints.front().x;
    double startTheta = radiansFromCoord(y, x);

    for (size_t i = 0; i < readPoints.size(); i++)
    {
        if (m_args.nPointsReadLimit > 0 && i == unsigned(m_args.nPointsReadLimit))
        {
            // stop reading points when we've got enough (nPointsReadLimit)
            break;
        }
        BFLidarPointSerialized &readPoint = readPoints[i];
//        if (i % 1 != 0)
//        {
//            continue; // todo: remove me for artifically skinnying down the data
//        }
        // converts to a Lidar Point which uses doubles
        LidarPoint writePoint;
        writePoint.x = readPoint.x;
        writePoint.y = readPoint.y;
        writePoint.z = readPoint.z;
        writePoint.intensity = readPoint.intensity;
        double radiansFromStart = radiansFromCoord(readPoint.y, readPoint.x) - startTheta;
        writePoint.lidar_angle = rad2deg(radiansFromStart);

        double secondsToRotateToTheta = writePoint.lidar_angle / radiansPerSecond;
        writePoint.timestamp = TimespecToDouble(datum.time) + secondsToRotateToTheta;
        writePoint.laser_id = readPoint.laser_id;
        outPoints.emplace_back(writePoint);
    }

    pointCloud.points = std::move(outPoints);

    return pointCloud;
}


} //namespace pdal
