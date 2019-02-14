#include "AutoXBFReader.hpp"
#include <pdal/util/ProgramArgs.hpp>
#include "autox_datum_writer.h"

namespace pdal
{
    static PluginInfo const s_info
        {
            "readers.autoxbf",
            "AutoXBF Reader",
            "jsun@autox.ai"
        };

    CREATE_SHARED_STAGE(AutoXBFReader, s_info)

    std::string AutoXBFReader::getName() const { return s_info.name; }

    void AutoXBFReader::addArgs(ProgramArgs& args)
    {
        args.add("gps", "GPS RTK file", m_scale_z, 1.0);
    }

    // corresponds to bf::msg::LidarPoint
    void AutoXBFReader::addDimensions(PointLayoutPtr layout)
    {
/*
decimal
places   degrees          distance
-------  -------          --------
0        1                111  km
1        0.1              11.1 km
2        0.01             1.11 km
3        0.001            111  m
4        0.0001           11.1 m
5        0.00001          1.11 m
6        0.000001         11.1 cm
7        0.0000001        1.11 cm
8        0.00000001       1.11 mm
We would not need more than 8 digits accuracy with lat/lng.
Lidar x,y,z are in metres (the driver that captures the BF dump ensures this)
and the center 0, 0, 0 is the inside of the lidar.
Height from the vehicle is physically measured into txt file.
 */
        layout->registerDim(Dimension::Id::X); // lat,lng
        layout->registerDim(Dimension::Id::Y);
        layout->registerDim(Dimension::Id::Z);
        layout->registerDim(Dimension::Id::Intensity);
        layout->registerDim(Dimension::Id::InternalTime); // timestamp
        layout->registerDim(Dimension::Id::GpsTime);      // from rtk
        // for future work
//        layout->registerDim(Dimension::Id::PointId); // laser_id (e.g. 40 beam lidar has id 0-40)
//        layout->registerDim(Dimension::Id::ScanAngleRank); // lidar_angle (e.g. a full lidar rotation)
//        layout->registerDim(Dimension::Id::PointSourceId); // which frame id this point is from

        layout->registerOrAssignDim("LaserId", Dimension::Type::Unsigned8); // laser_id (e.g. 40 beam lidar has id 0-40)
        layout->registerOrAssignDim("LidarAngle", Dimension::Type::Unsigned16); // lidar_angle (e.g. a full lidar rotation)
        // todo: need a frame reference id
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
            oss << "Unable to convert " << name << ", " << s[fieldno] <<
                ", to double";
            throw pdal_error(oss.str());
        }

        return output;
    }


    point_count_t AutoXBFReader::read(PointViewPtr view, point_count_t count)
    {
        PointLayoutPtr layout = view->layout();
        PointId nextId = view->size(); // ID of the point incremented in each iteration of the loop
        PointId idx = m_index;
        point_count_t numRead = 0;

        m_stream.reset(new ILeStream(m_filename));

        // this skips lines, we don't have a header so we don't need this
        size_t HEADERSIZE(1);
        size_t skip_lines((std::max)(HEADERSIZE, (size_t)m_index));
        size_t line_no(1);

        // each line thereafter is a single point
        for (std::string line; std::getline(*m_stream->stream(), line); line_no++)
        {
            if (line_no <= skip_lines)
            {
                continue;
            }

            // AutoXBFReader format:  X::Y::Z::Data
            // Here we take the line we read in the for block header, split it, and
            StringList s = Utils::split2(line, ':');

            unsigned long u64(0);
            // make sure that we have the proper number of fields.
            if (s.size() != 4)
            {
                std::stringstream oss;
                oss << "Unable to split proper number of fields.  Expected 4, got "
                    << s.size();
                throw pdal_error(oss.str());
            }

            // the values we read and put them into the PointView object
            std::string name("X");
            view->setField(Dimension::Id::X, nextId, convert<double>(s, name, 0));

            name = "Y";
            view->setField(Dimension::Id::Y, nextId, convert<double>(s, name, 1));

            name = "Z";
            double z = convert<double>(s, name, 2) * m_scale_z;
            view->setField(Dimension::Id::Z, nextId, z);

            name = "MyData";
            view->setField(layout->findProprietaryDim(name),
                           nextId,
                           convert<unsigned int>(s, name, 3));

            nextId++;
            if (m_cb)
                m_cb(*view, nextId);
        }
        m_index = nextId;
        numRead = nextId;

        return numRead;
    }

    void AutoXBFReader::done(PointTableRef)
    {
        m_stream.reset();
    }

} //namespace pdal