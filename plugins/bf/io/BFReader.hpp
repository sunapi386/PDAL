#pragma once

#include <pdal/PointView.hpp>
#include <pdal/Reader.hpp>
#include <pdal/util/IStream.hpp>
#include <json/value.h>
#include <vendor/eigen/Eigen/Dense>
#include <bf_rtk_interpolator.h>
#include "bf_datum_parser.h"
#include "BFCommon.hpp"

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

class PDAL_DLL BFReader : public Reader/*, public Streamable*/
{
public:
    BFReader() : Reader() {};
    std::string getName() const override;

private:
    Dimension::Id m_LaserId, m_LidarAngle;
    BFArgs m_args;
    point_count_t m_nPtsAlreadyRead;

    // the lidar BF file is usually large, 10-100Gb
    std::unique_ptr<bf::DatumParser> m_datumParserLidar;

    // this rtk datum file is usually small, 1-100 Mbs so we can store it in memory
    bf::RTKInterpolator m_rtkInterpolator;

    // the affine is small
    Eigen::Affine3d m_affine;
    double m_rtkFirstUtmX = 0, m_rtkFirstUtmY = 0;
    int precision = std::numeric_limits<double>::max_digits10;


    void addArgs(ProgramArgs &args) override;

    void initialize() override; // open files
    void addDimensions(PointLayoutPtr layout) override;

    void ready(PointTableRef table) override; // reset input data to read first point

    point_count_t read(PointViewPtr view, point_count_t nPtsToRead) override;
    /*bool processOne(PointRef& point);*/

    void done(PointTableRef table) override; // close files or reset state initialized in ready()

    void checkForValidInputFiles();

    uint insertRtkDatumsIntoInterpolator(bf::DatumParser &parser);

    void mutatePC_referenceFromLidarToRTK(PointCloudRef inPc);
    void mutatePC_referenceFromRtkToUTM(PointCloudRef cloud);

    void affineSinglePoint(LidarPointRef point, Eigen::Affine3d &affine);

    /*!
     * We need to interpolate the travelled distance during that one lidar
     * scan sweep.
     * @param datumLidar
     * @return time and location for the start and end of the lidar scan.
     */
    TimePlaceSegment interpolateTimePlaceSegment(const bf::Datum &datumLidar) const;

    /*!
     * Motion compensation for the point cloud, given a start-finish time-place.
     * @param segment
     * @param cloud
     */
    void compensatePoint(LidarPointRef point);

    PointCloud getLidarPoints(bf::Datum &datum);

    void mutatePC_doMotionCompensation(PointCloud &cloud);

    void mutatePC_addInterpolatedTimeEachPointToPC(PointCloud &cloud);

    PointCloud getSingleLidarPoint(bf::Datum &datum);
};
}
