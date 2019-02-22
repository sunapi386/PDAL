#include "BFReader.hpp"
#include "BFCommon.hpp"
#include <memory>
#include <pdal/util/ProgramArgs.hpp>
#include <pdal/util/FileUtils.hpp>
#include <vendor/jsoncpp/dist/json/json.h>

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
    args.add("rtk", "BF file, RTK", m_args.fileRtk);
    args.add("lidar", "BF file, lidar", m_args.fileLidar);
    args.add("affine", "Text file, spatial transformation affine", m_args.fileAffine);
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
            throwError("Failed to parse JSON file " + m_filename);
        }
        m_args = readArgsFromJson(parsedJson);
        openInputFiles();
    }
    else
    {
        throwError("Did not get an input filename");
    }
}

void BFReader::
done(PointTableRef)
{
    log()->get(LogLevel::Info) << "Done\n";
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
    layout->registerDim(Id::GpsTime);      // from rtk
    m_LaserId = layout->registerOrAssignDim("LaserId", Type::Unsigned8); // laser_id (e.g. 40 beam lidar has id 0-40)
    m_LidarAngle = layout->registerOrAssignDim("LidarAngle", Type::Unsigned16); // lidar_angle (e.g. a full lidar rotation)
    layout->registerDim(Id::PointSourceId); // which frame id this point is from
}

void BFReader::ready(PointTableRef)
{
    SpatialReference ref("EPSG:3857");
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

    bf::DatumParser datumParser(m_args.fileLidar);
    bf::Datum datum{};

    point_count_t nPtsRead = 0;
    uint nBFFramesRead = 0;
    uint nFramesToRead = 1;

    auto time_point = std::chrono::system_clock::now();

    while (nPtsRead < nPtsToRead && nBFFramesRead < nFramesToRead && datumParser.GetDatum(datum))
    {
        auto pointCloud = getLidarPoints(datum);
        // note the timestamp is not recorded in the datum itself
        // and the lidar_angle is not used
        // there are about 200k points in a single scan (for 128 beam lidar)

        log()->get(LogLevel::Debug) << "Datum " << nBFFramesRead << " size: " << unsigned(datum.size) << " Bytes\n";
        free(datum.data);

        timespec &timespec = datum.time; // todo: interpolate on time for
//        std::string timespecToString = "lidar_" + TimespecToString(timespec) + ".csv";
//        writePCTextFile(pointCloud, timespecToString);

        // transform lidar frame to global reference
        //    auto pcInVehicleReference =
        //        transformReferenceFromLidarToVehicle(pointcloud);
        //    savePC(pcInVehicleReference);

        //    auto pcInGlobalFrame =
        //        transformReferenceFromVehicleToGlobalApproximate(pcInVehicleReference);
        //    savePC(pcInGlobalFrame);


        // the values we read and put them into the PointView object
        for (auto &pt : pointCloud)
        {
            view->setField(Dimension::Id::X, nextId, pt.x);
            view->setField(Dimension::Id::Y, nextId, pt.y);
            view->setField(Dimension::Id::Z, nextId, pt.z);
            view->setField(Dimension::Id::Intensity, nextId, pt.intensity);
            view->setField(Dimension::Id::InternalTime, nextId, TimespecToDouble(timespec));
            view->setField(Dimension::Id::GpsTime, nextId, TimespecToDouble(timespec));
            view->setField(m_LaserId, nextId, pt.laser_id);
            view->setField(m_LidarAngle, nextId, pt.lidar_angle);
            view->setField(Dimension::Id::PointSourceId, nextId, nBFFramesRead);

            /*processOne(point);*/ // todo: add streaming

            nPtsRead++;
            nextId++;

        }
        nBFFramesRead++;

        if (m_cb)
        {
            m_cb(*view, nextId);
        }
    }

    auto time_point_2 = std::chrono::system_clock::now();
    log()->get(LogLevel::Info) << nPtsRead << "\n";
    auto delta = std::chrono::duration_cast<std::chrono::milliseconds>(time_point_2 - time_point);
    log()->get(LogLevel::Info) << delta.count() << "ms" << "\n";

    return nPtsRead;
}




void BFReader::openInputFiles()
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
    log()->get(LogLevel::Info) << "Opening:\n-rtk=" << m_args.fileRtk << "\n-lidar=" << m_args.fileLidar << "\n-affine=" << m_args.fileAffine << "\n";
    m_datumParserLidar = make_unique<bf::DatumParser>(m_args.fileLidar);
    m_datumParserRtk = make_unique<bf::DatumParser>(m_args.fileRtk);
    Json::Value value;
    bool success = jsonValueFromFile(m_args.fileAffine,value);
    if (!success)
    {
        throwError("Failed to parse json file " + m_args.fileAffine);
    }
    m_affine = readAffineFromJson(value);
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


} //namespace pdal