// Copyright (c) 2014-2018 Bauhaus-Universitaet Weimar
// This Software is distributed under the Modified BSD License, see license.txt.
//
// Virtual Reality and Visualization Research Group 
// Faculty of Media, Bauhaus-Universitaet Weimar
// http://www.uni-weimar.de/medien/vr

#include <stdexcept>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <chrono>
#include <lamure/types.h>
#include <e57/E57Foundation.h>
#include <e57/E57Simple.h>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <vector>
#include <algorithm>
#include <random>
#include <map>
#include <osg/Matrix>
#include <osg/Vec3>
#include <deque>
#include <lamure/pre/io/format_e57.h>

#if defined(__GNUC__) && !defined(__clang__)
#include <parallel/algorithm>
namespace alg = __gnu_parallel;
#else
namespace alg = std;
#endif

using namespace std;

bool intensityOnly;
bool readScannerPositions = false;

namespace lamure
{
namespace pre
{

void format_e57::getFilenames(const boost::filesystem::path &p, std::vector<std::string> &f)
{
    boost::filesystem::directory_iterator end_itr;
    for(boost::filesystem::directory_iterator itr(p); itr != end_itr; ++itr)
    {
        if(is_regular_file(itr->path()))
        {
            std::string current_file = itr->path().string();
            if(current_file.substr(current_file.size() - 4) == std::string(".e57"))
            {
                f.push_back(current_file);
                std::cout << current_file << std::endl;
            }
        }
    }
}

void format_e57::write(const std::string &filename, buffer_callback_function callback) { LOGGER_ERROR("Not implemented"); }


void format_e57::read(const std::string &file, surfel_callback_function callback)
{
    osg::Matrix m;
    m.makeIdentity();

    try
    {
        // --- E57 öffnen und Metadaten ausgeben ---
        e57::Reader reader(file.c_str());
        e57::E57Root root;
        reader.GetE57Root(root);

        std::cout << "=== E57 Metadata ===\n";
        std::cout << "File:             " << file << "\n";
        std::cout << "GUID:             " << root.guid << "\n";
        std::cout << "Version:          " << root.versionMajor << "." << root.versionMinor << "\n";
        int data3DCount = reader.GetData3DCount();
        std::cout << "Data3DCount:      " << data3DCount << "\n";

        // Container für erste/letzte 5 Surfels
        std::vector<surfel> firstSurfels;
        std::deque<surfel> lastSurfels;
        size_t surfelCount = 0;

        // Für jeden Scan
        for(int scanIndex = 0; scanIndex < data3DCount; ++scanIndex)
        {
            // Header lesen und Feld-Verfügbarkeit prüfen
            e57::Data3D header;
            reader.ReadData3D(scanIndex, header);

            bool hasCartesian = header.pointFields.cartesianXField;
            bool hasSpherical = header.pointFields.sphericalRangeField;
            bool hasIntensity = header.pointFields.intensityField;
            bool hasColorRed = header.pointFields.colorRedField;
            bool hasColorGreen = header.pointFields.colorGreenField;
            bool hasColorBlue = header.pointFields.colorBlueField;

            // Pose
            osg::Matrix trans, rot;
            trans.makeTranslate(header.pose.translation.x, header.pose.translation.y, header.pose.translation.z);
            rot.makeRotate(osg::Quat(header.pose.rotation.x, header.pose.rotation.y, header.pose.rotation.z, header.pose.rotation.w));
            m = rot * trans;

            std::cout << "[Scan " << scanIndex << " | Name: " << header.name << "]\n";
            std::cout << "Koordinaten:     " << (hasCartesian ? "Cartesian" : (hasSpherical ? "Spherical" : "keine")) << "\n";
            std::cout << "Intensity:       " << (hasIntensity ? "ja" : "nein") << "\n";
            std::cout << "Color:          R=" << (hasColorRed ? "ja" : "nein") << "  G=" << (hasColorGreen ? "ja" : "nein") << "  B=" << (hasColorBlue ? "ja" : "nein") << "\n";
            std::cout << "Translation:   (" << header.pose.translation.x << ", " << header.pose.translation.y << ", " << header.pose.translation.z << ")\n";
            std::cout << "Rotation (quat): (" << header.pose.rotation.x << ", " << header.pose.rotation.y << ", " << header.pose.rotation.z << ", " << header.pose.rotation.w << ")\n\n";
            std::cout << "====================\n\n";

            // Buffer-Größen ermitteln
            int64_t nRows, nCols, nPointsSize, nGroupsSize, nCountSize;
            bool bColumnIndex = false;
            reader.GetData3DSizes(scanIndex, nRows, nCols, nPointsSize, nGroupsSize, nCountSize, bColumnIndex);
            int64_t chunkSize = (nRows > 0 ? nRows : 1024);

            // Buffer anlegen
            std::vector<int8_t> isInvalid(chunkSize, 0);
            std::vector<double> xData, yData, zData;
            std::vector<double> rangeData, azData, elData, intData;
            std::vector<uint16_t> redData, greenData, blueData;
            double intOffset = 0.0, intRange = 1.0;
            int32_t redOffset = 0, redRange = 1;
            int32_t greenOffset = 0, greenRange = 1;
            int32_t blueOffset = 0, blueRange = 1;

            if(hasCartesian)
            {
                xData.resize(chunkSize);
                yData.resize(chunkSize);
                zData.resize(chunkSize);
            }
            if(hasSpherical)
            {
                rangeData.resize(chunkSize);
                azData.resize(chunkSize);
                elData.resize(chunkSize);
            }
            if(hasIntensity)
            {
                intData.resize(chunkSize);
                intOffset = header.intensityLimits.intensityMinimum;
                intRange = header.intensityLimits.intensityMaximum - intOffset;
            }
            if(hasColorRed || hasColorGreen || hasColorBlue)
            {
                redData.resize(chunkSize);
                greenData.resize(chunkSize);
                blueData.resize(chunkSize);
                redOffset = header.colorLimits.colorRedMinimum;
                redRange = header.colorLimits.colorRedMaximum - redOffset;
                greenOffset = header.colorLimits.colorGreenMinimum;
                greenRange = header.colorLimits.colorGreenMaximum - greenOffset;
                blueOffset = header.colorLimits.colorBlueMinimum;
                blueRange = header.colorLimits.colorBlueMaximum - blueOffset;
            }

            // Komprimierten Leser aufsetzen
            e57::CompressedVectorReader dataReader = reader.SetUpData3DPointsData(
                scanIndex, chunkSize, hasCartesian ? xData.data() : nullptr, hasCartesian ? yData.data() : nullptr, hasCartesian ? zData.data() : nullptr, isInvalid.data(),
                hasIntensity ? intData.data() : nullptr, nullptr, hasColorRed ? redData.data() : nullptr, hasColorGreen ? greenData.data() : nullptr, hasColorBlue ? blueData.data() : nullptr, nullptr,
                hasSpherical ? rangeData.data() : nullptr, hasSpherical ? azData.data() : nullptr, hasSpherical ? elData.data() : nullptr);

            // Punkte auslesen
            size_t sz = 0;
            while((sz = dataReader.read()) > 0)
            {
                for(size_t i = 0; i < sz; ++i)
                {
                    if(isInvalid[i] != 0)
                        continue;

                    // Position berechnen
                    osg::Vec3 p;
                    if(hasCartesian)
                    {
                        p.set(xData[i], yData[i], zData[i]);
                    }
                    else
                    {
                        p.set(rangeData[i] * cos(elData[i]) * cos(azData[i]), rangeData[i] * cos(elData[i]) * sin(azData[i]), rangeData[i] * sin(elData[i]));
                    }
                    p = p * m;

                    // Surfel zusammenbauen
                    lamure::vec3r pos(p[0], p[1], p[2]);
                    lamure::vec3b col(0, 0, 0);
                    double intensity_value = 0.0;

                    if(hasIntensity)
                    {
                        intensity_value = (intData[i] - intOffset) / intRange;
                    }
                    if(hasColorRed)
                        col.r = static_cast<uint8_t>(round((redData[i] - redOffset) * 255.0 / redRange));
                    if(hasColorGreen)
                        col.g = static_cast<uint8_t>(round((greenData[i] - greenOffset) * 255.0 / greenRange));
                    if(hasColorBlue)
                        col.b = static_cast<uint8_t>(round((blueData[i] - blueOffset) * 255.0 / blueRange));

                    // Hier Surfel-Konstruktor anpassen, falls dein Typ andere Parameter erwartet
                    surfel s(pos, col, intensity_value);
                    callback(s);

                    // Für erste/letzte 5 sammeln
                    if(surfelCount < 5)
                    {
                        firstSurfels.push_back(s);
                    }
                    lastSurfels.push_back(s);
                    if(lastSurfels.size() > 5)
                    {
                        lastSurfels.pop_front();
                    }
                    ++surfelCount;
                }
            }
            dataReader.close();
        }

        reader.Close();

        // Ausgabe der ersten/letzten 5 Surfels
        auto print_surfel = [&](const surfel &s)
        {
            std::cout << "Position(" << s.pos()[0] << ", " << s.pos()[1] << ", " << s.pos()[2] << ")";
            std::cout << " Color(" << int(s.color().r) << ", " << int(s.color().g) << ", " << int(s.color().b) << ")\n";
        };

        std::cout << "\n=== Erste 5 Surfels ===\n";
        for(auto const &s : firstSurfels)
            print_surfel(s);

        std::cout << "\n=== Letzte 5 Surfels ===\n";
        for(auto const &s : lastSurfels)
            print_surfel(s);
        std::cout << "\n";
    }
    catch(const std::exception &ex)
    {
        std::cerr << "Exception reading " << file << ": " << ex.what() << std::endl;
    }
    catch(...)
    {
        std::cerr << "Unknown exception reading " << file << std::endl;
    }
}


} // namespace pre

} // namespace lamure
