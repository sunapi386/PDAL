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
    args.add("mDistanceJump", "Distance to wait until car moves", m_args.mDistanceJump);
}

void BFReader::initialize()
{
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
        log()->get(LogLevel::Debug) << "Opening:\n-rtk=" << m_args.fileRtk << "\n-lidar=" << m_args.fileLidar << "\n-affine=" << m_args.fileAffine << "\n";
        log()->get(LogLevel::Debug) << "-dumpFrames=" << m_args.dumpFrames << "\n";
        log()->get(LogLevel::Debug) << "-nFramesSkip=" << m_args.nFramesSkip << "\n";
        log()->get(LogLevel::Debug) << "-nFramesRead=" << m_args.nFramesRead << "\n";

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
//    layout->registerDim(Id::InternalTime); // timestamp
//    layout->registerDim(Id::GpsTime);      // from rtk
//    m_LaserId = layout->registerOrAssignDim("LaserId", Type::Unsigned8); // laser_id (e.g. 40 beam lidar has id 0-40)
//    m_LidarAngle = layout->registerOrAssignDim("LidarAngle", Type::Unsigned16); // lidar_angle (e.g. a full lidar rotation)
//    layout->registerDim(Id::PointSourceId); // which frame id this point is from
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

    log()->get(LogLevel::Info) << "BFReader m_filename: " << m_filename << std::endl;

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

    while (nPtsRead < nPtsToRead && nBFDatumsRead < m_args.nFramesRead && datumParserLidar.GetDatum(datumLidar))
    {
        if (--skip > 0)
        {
            // log()->get(LogLevel::Debug) << "Skip Datum (" << skip << " to skip)\n";
            free(datumLidar.data);
            continue;
        }
        auto pointCloud = getLidarPoints(datumLidar);
        free(datumLidar.data);
        // timestamp is not recorded in the lidar readings and the lidar_angle isn't used
        // approx. 200k points in a datum for 128 beam lidar


        // interpolate a location from the time
        timespec &timespec = datumLidar.time;
        msg::RTKMessage interpolatedRtkMessage;
        m_rtkInterpolator.GetTimedData(timespec, &interpolatedRtkMessage);

        // check and skips datums if they're too close to each other
        // make sure we have moved a certain distance first
        double distanceM = distanceMetres(interpolatedRtkMessage, lastUsedRtkMsg);
        if (distanceM < m_args.mDistanceJump)
        {
            // log()->get(LogLevel::Debug) << " (distanceM: " << std::setprecision(precision) << distanceM << ")\n";
//            log()->get(LogLevel::Debug) << "Skip " << rtkToString(interpolatedRtkMessage) << "\n";
            continue;
        }
        lastUsedRtkMsg = interpolatedRtkMessage;
        log()->get(LogLevel::Info) << "Datum " << nBFDatumsRead << " " << unsigned(datumLidar.size)/ 1000.0 << " KBytes\n";

        // not factoring in rotational angle movement during that 10 ms.
        mutatePCFromLidarToRTK(pointCloud);
        mutatePCFromRtkToUTM(pointCloud, interpolatedRtkMessage);

        if (m_args.dumpFrames)
        {
            std::string timespecString = TimespecToString(timespec, true);
            std::string rtkString = rtkToString(interpolatedRtkMessage);
            std::string name = "lidar_" + timespecString + "_" + rtkString + ".csv";
            writePCTextFile(pointCloud, name);
        }


        // the values we read and put them into the PointView object
        for (auto &pt : pointCloud)
        {
            view->setField(Dimension::Id::X, nextId, pt.x);
            view->setField(Dimension::Id::Y, nextId, pt.y);
            view->setField(Dimension::Id::Z, nextId, pt.z);
            view->setField(Dimension::Id::Intensity, nextId, pt.intensity);
//            view->setField(Dimension::Id::InternalTime, nextId, TimespecToDouble(timespec));
//            view->setField(Dimension::Id::GpsTime, nextId, TimespecToDouble(timespec));
//            view->setField(m_LaserId, nextId, pt.laser_id);
//            view->setField(m_LidarAngle, nextId, pt.lidar_angle);
//            view->setField(Dimension::Id::PointSourceId, nextId, nBFDatumsRead);

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

void BFReader::mutatePCFromLidarToRTK(PointCloud &pcIn)
{
    // for every point, apply a transform from the affine matrix
    for (auto &pt : pcIn)
    {
        affineSinglePoint(pt, m_affine);
    }
}

void BFReader::affineSinglePoint(LidarPoint &pt, Eigen::Affine3d &affine)
{
    Eigen::Vector3d pt3d(pt.x, pt.y, pt.z);
    pt3d = affine * pt3d;
    pt.x = pt3d[0];
    pt.y = pt3d[1];
    pt.z = pt3d[2];
}



void BFReader::mutatePCFromRtkToUTM(PointCloud &pc, msg::RTKMessage &rtkMsg)
{

    // convert to UTM, don't touch the altitude
    int utmZone;
    bool utmNorth;
    double utmX, utmY;
    double utmZ = rtkMsg.altitude();

    // todo: look into handling altitude;
    // todo: for performance simplify the lidar-rtk and rtk-utm transform into a single affine

    // with the x,y do transform of the lidarPt (use original row, pitch, yaw, altitude)
    // create affine from x,y,z,r,p,y (with this 6 parameters) we can construct affine transform for lidarPt to UTM
    GeographicLib::UTMUPS::Forward(rtkMsg.latitude(), rtkMsg.longitude(), utmZone, utmNorth, utmX, utmY);

//    affine3d = q;

    // any units in meters, we'll have to project into UTM. Because we can't work with lat/lng in meters
    // think of UTM as a tool to treat global coordinates (radial) as euclidean space (flat)

    if (m_rtkFirstUtmX == 0 && m_rtkFirstUtmY == 0)
    {
        log()->get(LogLevel::Debug) << std::setprecision(precision) << "m_rtkFirstFrameUtmX=" << utmX << "\n";
        log()->get(LogLevel::Debug) << "m_rtkFirstFrameUtmY=" << utmY << "\n";
        m_rtkFirstUtmX = utmX;
        m_rtkFirstUtmY = utmY;
    }

    Eigen::Affine3d affine3d = Eigen::Affine3d::Identity();
    affine3d.translation() = Eigen::Vector3d(utmX - m_rtkFirstUtmX, utmY - m_rtkFirstUtmY, utmZ);
    Eigen::AngleAxisd rollAngle(rtkMsg.roll() / 180.0 * M_PI, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(rtkMsg.pitch() / 180.0 * M_PI, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(M_PI / 2 - rtkMsg.heading()  / 180.0 * M_PI, Eigen::Vector3d::UnitZ());
    Eigen::Quaternion<double> quaternion =  yawAngle * rollAngle * pitchAngle;
    affine3d.linear() = quaternion.matrix();
    affine3d.linear() = quaternion.matrix();


    for (auto &pt : pc)
    {
        affineSinglePoint(pt, affine3d);
    }
}


} //namespace pdal