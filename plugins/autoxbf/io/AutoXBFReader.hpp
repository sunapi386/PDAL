#pragma once

#include <pdal/PointView.hpp>
#include <pdal/Reader.hpp>
#include <pdal/util/IStream.hpp>

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

namespace pdal
{
    class AutoXBFReader : public Reader
    {
    public:
        AutoXBFReader() : Reader() {};
        std::string getName() const override;

    private:
        std::string m_rtk;
        std::string m_lidar;
        std::string m_rig;
        point_count_t m_numPts;
        uint8_t m_expected_no_fields = 8;

        std::istream* m_istream;
        void initialize(PointTableRef table) override;
        void addDimensions(PointLayoutPtr layout) override;
        void addArgs(ProgramArgs& args) override;
        void ready(PointTableRef table) override;
        point_count_t read(PointViewPtr view, point_count_t count) override;
        void done(PointTableRef table) override;
    };
}