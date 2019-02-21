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
    layout->registerDim(Dimension::Id::X); // lat,lng
    layout->registerDim(Dimension::Id::Y); // x,y,z are in meters
    layout->registerDim(Dimension::Id::Z);
    layout->registerDim(Dimension::Id::Intensity);
    layout->registerDim(Dimension::Id::InternalTime); // timestamp
    layout->registerDim(Dimension::Id::GpsTime);      // from rtk
    layout->registerOrAssignDim("LaserId", Dimension::Type::Unsigned8); // laser_id (e.g. 40 beam lidar has id 0-40)
    layout->registerOrAssignDim("LidarAngle", Dimension::Type::Unsigned16); // lidar_angle (e.g. a full lidar rotation)
    layout->registerDim(Dimension::Id::PointSourceId); // which frame id this point is from
}

void BFReader::ready(PointTableRef)
{
    SpatialReference ref("EPSG:3857");
    setSpatialReference(ref);
}


point_count_t BFReader::read(PointViewPtr view, point_count_t nPtsToRead)
{
    PointLayoutPtr layout = view->layout();
    PointId nextId = view->size(); // ID of the point incremented in each iteration of the loop
    log()->get(LogLevel::Info) << "BFReader m_filename: " << m_filename << std::endl;

    std::string filename = "/autox/map/datasets/ABCD_test/vls128_lidar";
    bf::DatumParser datumParser(filename);
    bf::Datum datum{};

    point_count_t nPtsRead = 0;
    uint nFramesRead = 0;
    uint nFramesToRead = 1;

    auto time_point = std::chrono::system_clock::now();

    while (nPtsRead < nPtsToRead && nFramesRead++ < nFramesToRead && datumParser.GetDatum(datum))
    {
        auto pointCloud = getLidarPoints(datum);
        // note the timestamp is not recorded in the datum itself
        // and the lidar_angle is not used
        // there are about 200k points in a single scan (for 128 beam lidar)

        log()->get(LogLevel::Debug) << "Datum " << nFramesRead << " size: " << unsigned(datum.size) << " Bytes\n";
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
            view->setField(layout->findProprietaryDim("LaserId"), nextId, pt.laser_id);
            view->setField(layout->findProprietaryDim("LidarAngle"), nextId, pt.lidar_angle);
            view->setField(Dimension::Id::PointSourceId, nextId, nFramesRead);
            ++nPtsRead;
            ++nextId;
        }

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

void BFReader::done(PointTableRef)
{
}

void BFReader::initialize(BasePointTable &table)
{

//    if (m_args.fileRtk.empty())
//    {
//      throwError("Unable to open rtk file '" + m_args.fileRtk + "'.");
//    } else {
//        m_datumParserRtk = std::unique_ptr<bf::DatumParser>(new bf::DatumParser(m_args.fileRtk));
//    }


//    if (!m_args.fileLidar.empty())
//    {
//        m_datumParserLidar = bf::DatumParser(m_args.fileLidar);
//    }
//    if (!m_args.fileTransf.empty())
//    {
//        m_istreamTransf = Utils::openFile(m_filename, false);
//    if (!m_istreamTransf)
//      throwError("Unable to open text file '" + m_filename + "'.");
//    }

}

} //namespace pdal