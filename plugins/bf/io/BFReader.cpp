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
    args.add("mLidarDistanceReturnFilter", "Filter out lidar points that are this meters away", m_args.mLidarDistanceReturnFilter);
}

void BFReader::initialize()
{
    log()->get(LogLevel::Info) << "BFReader m_filename: " << m_filename << "\n";
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
        log()->get(LogLevel::Info) << "\n-rtk=" << m_args.fileRtk << "\n";
        log()->get(LogLevel::Info) << "-lidar=" << m_args.fileLidar << "\n";
        log()->get(LogLevel::Info) << "-affine=" << m_args.fileAffine << "\n";
        log()->get(LogLevel::Info) << "-dumpFrames=" << m_args.dumpFrames << "\n";
        log()->get(LogLevel::Info) << "-nFramesSkip=" << m_args.nFramesSkip << "\n";
        log()->get(LogLevel::Info) << "-nFramesRead=" << m_args.nFramesRead << "\n";
        log()->get(LogLevel::Info) << "-nPointsReadLimit=" << m_args.nPointsReadLimit << "\n";
        log()->get(LogLevel::Info) << "-mCompensate=" << (m_args.mCompensate ? "true" : "false") << "\n";

        bf::DatumParser rtkDatumParser(m_args.fileRtk);
        uint count = insertRtkDatumsIntoInterpolator(rtkDatumParser);
        log()->get(LogLevel::Debug) << "Read " << count << " rtk datums to interpolator\n";

        m_datumParserLidar = make_unique<bf::DatumParser>(m_args.fileLidar);

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
    m_LaserId = layout->registerOrAssignDim("LaserId", Type::Unsigned8);
    m_LidarAngle = layout->registerOrAssignDim("LidarAngle", Type::Double);
    layout->registerDim(Id::PointSourceId);
}

void BFReader::ready(PointTableRef)
{
//    SpatialReference ref("EPSG:3857+5703");
//    SpatialReference ref("EPSG:26910");
//    SpatialReference ref("EPSG:26910+5705");
//    SpatialReference ref("EPSG:3857");
//    setSpatialReference(ref);
}


point_count_t BFReader::read(PointViewPtr view, point_count_t nPtsToRead)
{
    // don’t read more points than requested (18446744073709551615 == 2^64-1)
    log()->get(LogLevel::Debug) << "Requested " << nPtsToRead << " points to be read\n";

    // Determine the ID of the next point in the point view
    PointId nextId = view->size();

    bf::DatumParser datumParserLidar(m_args.fileLidar);
    bf::Datum datumLidar{};

    point_count_t nPtsRead = 0;
    int nBFDatumsRead = 0;
    int skip = m_args.nFramesSkip;

    auto time_point = std::chrono::system_clock::now();

    msg::RTKMessage lastUsedRtkMsg;

    while (nPtsRead < nPtsToRead &&
            nBFDatumsRead < m_args.nFramesRead &&
            datumParserLidar.GetDatum(datumLidar))
    {
        if (--skip > 0)
        {
            log()->get(LogLevel::Debug) << skip << " datums remaining to skip\n";
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
            log()->get(LogLevel::Debug) << "Datum skipped, distance=" << preciseDoubleStr(distSinceLastRtkMsgUsed, 5)
                                        << "m less than " << m_args.mDistanceJump << "m\n";
            continue;
        }
        lastUsedRtkMsg = segment.finish.rtkMessage;
        log()->get(LogLevel::Debug) << "Datum " << nBFDatumsRead << " " << preciseDoubleStr(unsigned(datumLidar.size)/ 1000.0, 3) << " KBytes\n";

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
            savePCToCSV(pointCloud);
        }

        for (LidarPointRef pt : pointCloud.points)
        {
            /*processOne(point);*/ // todo: add streaming
            view->setField(Dimension::Id::X, nextId, pt.x);
            view->setField(Dimension::Id::Y, nextId, pt.y);
            view->setField(Dimension::Id::Z, nextId, pt.z);
            view->setField(Dimension::Id::Intensity, nextId, pt.intensity);
            view->setField(Dimension::Id::InternalTime, nextId, pt.timestamp);
            view->setField(m_LaserId, nextId, pt.laser_id);
            view->setField(m_LidarAngle, nextId, pt.lidar_angle);
            view->setField(Dimension::Id::PointSourceId, nextId, nBFDatumsRead);
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
    timespec timespecStart = datumLidar.time;
    timespec timespecFinish= DoubleToTimespec(TimespecToDouble(datumLidar.time) + 0.1);
    m_rtkInterpolator.GetTimedData(timespecStart, &interpolatedScanStartRtkMsg);
    m_rtkInterpolator.GetTimedData(timespecFinish, &interpolatedScanFinishRtkMsg);
    log()->get(LogLevel::Debug) << TimespecToDouble(timespecStart) << " interpolatedScanFinishRtkMsg=" << rtkToString(interpolatedScanStartRtkMsg) << "\n";
    log()->get(LogLevel::Debug) << TimespecToDouble(timespecFinish) << " interpolatedScanFinishRtkMsg=" << rtkToString(interpolatedScanFinishRtkMsg) << "\n";

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

        bool success = rtkMessage.ParseFromArray(datum.data, static_cast<int>(datum.size));
        free(datum.data);

        if (!success)
        {
            log()->get(LogLevel::Warning) << "Failed parsing rtk datum " << TimespecToString(timespec) << "(skipping)\n";
            continue;
        }

        // the interpolator expects headings to be radians so do conversion here
        rtkMessage.set_heading(deg2rad(rtkMessage.heading()));
        m_rtkInterpolator.InsertNewData(timespec, rtkMessage, false);

        if (count == 0)
        {
            int utmZone;
            bool utmNorth;
            double utmMetersEasting, utmMetersNorthing;
            double utmZ = rtkMessage.altitude();
            GeographicLib::UTMUPS::Forward(rtkMessage.latitude(), rtkMessage.longitude(), utmZone, utmNorth, utmMetersEasting, utmMetersNorthing);
            log()->get(LogLevel::Debug) << "m_rtkFirstFrameUtmX=" << utmMetersEasting << "\n";
            log()->get(LogLevel::Debug) << "m_rtkFirstFrameUtmY=" << utmMetersNorthing << "\n";
            m_rtkFirstUtmEasting = utmMetersEasting;
            m_rtkFirstUtmNorthing = utmMetersNorthing;
        }
        count++;
    }




    return count;
}

void BFReader::mutatePC_referenceFromLidarToRTK(PointCloudRef pcIn)
{
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
    double utmMetersEasting, utmMetersNorthing;
    double utmZ = rtkMessage.altitude();
    GeographicLib::UTMUPS::Forward(rtkMessage.latitude(), rtkMessage.longitude(), utmZone, utmNorth, utmMetersEasting, utmMetersNorthing);

    Eigen::Affine3d affine3d = Eigen::Affine3d::Identity();
//    affine3d.translation() = Eigen::Vector3d(rtkMessage.latitude(), rtkMessage.longitude(), utmZ);
//    double metersX = distanceEarth(rtkMessage.latitude(), m_rtkFirst.longitude(), m_rtkFirst.latitude(), m_rtkFirst.longitude()) * 1000;
//    double metersY = distanceEarth(m_rtkFirst.latitude(), rtkMessage.longitude(), m_rtkFirst.latitude(), m_rtkFirst.longitude()) * 1000;

//    affine3d.translation() = Eigen::Vector3d(metersX, metersY, utmZ);
//    affine3d.translation() = Eigen::Vector3d(utmMetersEasting, utmMetersNorthing, utmZ);
//    affine3d.translation() = Eigen::Vector3d(utmMetersNorthing, utmMetersEasting, utmZ);
    double deltaX = utmMetersEasting - m_rtkFirstUtmEasting;
    double deltaY = utmMetersNorthing - m_rtkFirstUtmNorthing;
    affine3d.translation() = Eigen::Vector3d(deltaX, deltaY, utmZ);

    double roll = deg2rad(rtkMessage.roll())- deg2rad(m_rtkFirst.roll());
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    double pitch = deg2rad(rtkMessage.pitch())- deg2rad(m_rtkFirst.pitch());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
//    double heading =  M_PI / 2 - deg2rad(rtkMessage.heading());
    double heading =  M_PI / 2 - rtkMessage.heading(); // already in radians
    Eigen::AngleAxisd yawAngle(heading, Eigen::Vector3d::UnitZ());
    Eigen::Quaternion<double> quaternion =  yawAngle * pitchAngle * rollAngle;
    affine3d.linear() = quaternion.matrix();
    /*{
        log()->get(LogLevel::Debug) << "lat=" << rtkMessage.latitude() << ", lng=" << rtkMessage.longitude() << "\n";
//        log()->get(LogLevel::Debug) << "metersX=" << metersX << ", metersY=" << metersY << "\n";
        log()->get(LogLevel::Debug) << "utmZone=" << utmZone << ", utmNorth=" << utmNorth << "\n";
        log()->get(LogLevel::Debug) << "utmX=" << utmMetersEasting << ", utmY=" << utmMetersNorthing << ", utmZ=" << utmZ << "\n";
        log()->get(LogLevel::Debug) << "translation=" << affine3d.translation() << "\n";
        log()->get(LogLevel::Debug) << "roll=" << roll  << "(" << rtkMessage.roll() << "deg)\n";
        log()->get(LogLevel::Debug) << "pitch=" << pitch  << "(" << rtkMessage.pitch() << "deg)\n";
        log()->get(LogLevel::Debug) << "yaw=" << heading  << "(" << rtkMessage.heading() << "deg)\n";
        log()->get(LogLevel::Debug) << "affine=\n" << affine3d.matrix() << "\n";
    }*/
    return affine3d;
}

void BFReader::mutatePC_referenceFromRtkToUTM(PointCloudRef cloud)
{
    msg::RTKMessage &rtkMsgStart = cloud.timePlaceSegment.start.rtkMessage;
    msg::RTKMessage &rtkMsgFinish = cloud.timePlaceSegment.finish.rtkMessage;
    log()->get(LogLevel::Debug) << "rtkMsgStart=" << rtkToString(rtkMsgStart) << "\n";
    log()->get(LogLevel::Debug) << "rtkMsgFinish=" << rtkToString(rtkMsgFinish) << "\n";

    const Eigen::Affine3d &affineFromRtkMessageFinish = createAffineFromRtkMessage(rtkMsgFinish);
    const Eigen::Affine3d &affineFromRtkMessageStart = createAffineFromRtkMessage(rtkMsgStart);

    /*{
        // doing special motion compensation doesn't seem to be the issue
        LidarTimestampInterpolator interpolator;
        double timespec = TimespecToDouble(cloud.timePlaceSegment.finish.time);
        interpolator.UpdateTimestamp(0.1, utility::bf::DoubleToTimespec(timespec - 0.1), &cloud.points);
        LidarMotionCompensator motion_compensator;
        motion_compensator.CompensateCloudToEnd(cloud.points, affineFromRtkMessageStart,
                                                utility::bf::DoubleToTimespec(timespec - 0.1), affineFromRtkMessageFinish,
                                                utility::bf::DoubleToTimespec(timespec), &cloud.points);
    }*/

    for (LidarPointRef pt : cloud.points)
    {
        // todo: fix this
//        affineSinglePoint(pt, affineFromRtkMessageStart);
        affineSinglePoint(pt, affineFromRtkMessageFinish);
    }
}

void BFReader::mutatePC_doMotionCompensation(PointCloudRef cloud)
{
    for (LidarPointRef point : cloud.points)
    {
        motionCompensatePoint(point);
    }
}

/*!
 * The reason for having a point-level compensation is we may want to do streaming for large file sizes
 * @param point
 */
void BFReader::motionCompensatePoint(LidarPointRef point)
{
    msg::RTKMessage interpolatedLocation;
    m_rtkInterpolator.GetTimedData(DoubleToTimespec(point.timestamp), &interpolatedLocation);
    Eigen::Affine3d affine3d = createAffineFromRtkMessage(interpolatedLocation);
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
//    lidarPoint.x = 0;
//    lidarPoint.y = 0;
//    lidarPoint.z = 0;
//    xLidarPoint.x = 1;
//    xLidarPoint.y = 0;
//    xLidarPoint.z = 0;
//    yLidarPoint.x = 0;
//    yLidarPoint.y = 1;
//    yLidarPoint.z = 0;
//    zLidarPoint.x = 0;
//    zLidarPoint.y = 0;
//    zLidarPoint.z = 1;
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
        bool enoughPointsRead = m_args.nPointsReadLimit > 0 && i == unsigned(m_args.nPointsReadLimit);
        if (enoughPointsRead)
        {
            break;
        }
        BFLidarPointSerialized &readPoint = readPoints[i];

        float distance = std::hypot(readPoint.x, readPoint.y);
        bool farPointDrop = m_args.mLidarDistanceReturnFilter >= 0 && distance > m_args.mLidarDistanceReturnFilter;
        if (farPointDrop)
        {
            continue;
        }

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
