#pragma once

#include <pdal/Writer.hpp>
#include <pdal/StageFactory.hpp>

#include <vector>
#include <string>

pdal::Writer* createGeoWaveWriter();

namespace pdal
{

    class PDAL_DLL GeoWaveWriter : public Writer
    {
    public:
        std::string getName() const;

    private:
        virtual void addArgs(ProgramArgs& args);
        virtual void initialize();
        virtual void ready(PointTableRef table);
        virtual void write(const PointViewPtr view);

        int createJvm();

        std::string m_zookeeperUrl;
        std::string m_instanceName;
        std::string m_username;
        std::string m_password;
        std::string m_tableNamespace;
        std::string m_featureTypeName;
        std::string m_dataAdapter;
        bool m_useFeatCollDataAdapter;
        uint32_t m_pointsPerEntry;
        Dimension::IdList m_dims;
        std::vector<Dimension::Type> m_dimTypes;
    };

} // namespace pdal
