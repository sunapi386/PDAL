#include "BFReader.hpp"
#include "BFCommon.hpp"
#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{
static PluginInfo const s_info
{
    "readers.bf",
    "BF Reader",
    "http://path/to/documentation"
};

CREATE_SHARED_STAGE(BFReader, s_info)

std::string BFReader::getName() const
{
    return s_info.name;
}

void BFReader::addArgs(ProgramArgs& args)
{
    args.add("rtk,r", "Path to BF RTK file", m_args.fileRtk);
    args.add("lidar,l", "Path to BF lidar file", m_args.fileLidar);
    args.add("trans,t", "Path to intrinsic/extrinsic lidar/cam transform file", m_args.fileTransf);
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

    std::string filename = "/autox/map/datasets/ABCD_test/vls128_lidar";
    bf::DatumParser datumParser(filename);
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


void BFReader::done(PointTableRef)
{
}


void BFReader::initialize()
{

}

} //namespace pdal