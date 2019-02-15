#include "AutoXBFReader.hpp"
#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{
static StaticPluginInfo const s_info
{
    "readers.autoxbf",
    "AutoXBF Reader",
    "https://autox.ai",
    { "bf", "autoxbf" }
};

CREATE_SHARED_STAGE(AutoXBFReader, s_info)

std::string AutoXBFReader::getName() const
{
    return s_info.name;
}

void AutoXBFReader::addArgs(ProgramArgs& args)
{
    args.add("rtk,r", "Path to BF RTK file", m_filenameRtk);
    args.add("lidar,l", "Path to BF lidar file", m_filenameLidar);
    args.add("trans,t", "Path to intrinsic/extrinsic lidar/cam transform file", m_filenameTransf);
}

// corresponds to bf::msg::LidarPoint
void AutoXBFReader::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(Dimension::Id::X); // lat,lng
    layout->registerDim(Dimension::Id::Y);
    layout->registerDim(Dimension::Id::Z);
    layout->registerDim(Dimension::Id::Intensity);
    layout->registerDim(Dimension::Id::InternalTime); // timestamp
    layout->registerDim(Dimension::Id::GpsTime);      // from rtk
    layout->registerOrAssignDim("LaserId", Dimension::Type::Unsigned8); // laser_id (e.g. 40 beam lidar has id 0-40)
    layout->registerOrAssignDim("LidarAngle", Dimension::Type::Unsigned16); // lidar_angle (e.g. a full lidar rotation)
    // todo: need a frame reference id
//        layout->registerDim(Dimension::Id::PointSourceId); // which frame id this point is from
}

void AutoXBFReader::ready(PointTableRef)
{
    SpatialReference ref("EPSG:3857");
    setSpatialReference(ref);
}

template <typename T>
T convert(const StringList& s, const std::string& name, size_t fieldno)
{
    T output;
    bool bConverted = Utils::fromString(s[fieldno], output);
    if (!bConverted)
    {
        std::stringstream oss;
        oss << "Unable to convert " << name << ", " << s[fieldno] << ", to double";
        throw pdal_error(oss.str());
    }

    return output;
}


point_count_t AutoXBFReader::read(PointViewPtr view, point_count_t count)
{
    PointLayoutPtr layout = view->layout();
    PointId nextId = view->size(); // ID of the point incremented in each iteration of the loop
    point_count_t beginId = nextId;
    log()->get(LogLevel::Info) << "AutoXBFReader m_filename: " << m_filename << std::endl;

    size_t HEADERSIZE(1);
    size_t line_no(1);

    // each line thereafter is a single point
    for (std::string line; std::getline(*m_istream, line); line_no++)
    {
        log()->get(LogLevel::Debug4) << line_no << ": " << line << std::endl;
        if (line_no <= HEADERSIZE)
        {
            continue;
        }

        // AutoXBFReader format:  X::Y::Z::Data
        // Here we take the line we read in the for block header, split it, and
        StringList s = Utils::split2(line, ':');

        unsigned long u64(0);
        // make sure that we have the proper number of fields.
        if (s.size() != m_expected_no_fields)
        {
            std::stringstream oss;
            oss << "Unable to split proper number of fields.  Expected 4, got "
                << s.size();
            throw pdal_error(oss.str());
        }

        // the values we read and put them into the PointView object
        view->setField(Dimension::Id::X, nextId, convert<double>(s, "X", 0));
        view->setField(Dimension::Id::Y, nextId, convert<double>(s, "Y", 1));
        view->setField(Dimension::Id::Z, nextId, convert<double>(s, "Z", 2));
        view->setField(Dimension::Id::Intensity, nextId, convert<double>(s, "Intensity", 3));
        view->setField(Dimension::Id::InternalTime, nextId, convert<double>(s, "InternalTime", 4));
        view->setField(Dimension::Id::GpsTime, nextId, convert<double>(s, "GpsTime", 5));
        view->setField(layout->findProprietaryDim("LaserId"), nextId, convert<unsigned int>(s, "LaserId", 6));
        view->setField(layout->findProprietaryDim("LidarAngle"), nextId, convert<unsigned int>(s, "LidarAngle", 7));

        nextId++;
        if (m_cb)
            m_cb(*view, nextId);
    }
    m_numPts = nextId - beginId;

    return m_numPts;
}

void AutoXBFReader::done(PointTableRef)
{
    Utils::closeFile(m_istream);
}

void AutoXBFReader::initialize(BasePointTable &table)
{
    m_numPts = 0;
    m_istream = Utils::openFile(m_filename, false);
    if (!m_istream)
        throwError("Unable to open text file '" + m_filename + "'.");


}

} //namespace pdal