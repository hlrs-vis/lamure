#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <e57/E57Foundation.h>
#include <e57/E57Simple.h>
#include <fstream>
#include <iostream>
#include <lamure/pre/surfel.h>
#include <lamure/types.h>
#include <math.h>
#include <osg/Matrix>
#include <osg/Vec3>
#include <stdexcept>
#include <string>
#include <vector>


void getFilenames(const boost::filesystem::path &p, std::vector<std::string> &files)
{
    boost::filesystem::directory_iterator end;
    for(boost::filesystem::directory_iterator it(p); it != end; ++it)
    {
        if(boost::filesystem::is_regular_file(it->path()) && it->path().extension() == ".e57")
        {
            files.push_back(it->path().string());
            std::cout << "Found E57 file: " << it->path().string() << std::endl;
        }
    }
}


void readE57(const std::string &file, std::vector<lamure::pre::surfel> &vec)
{
    std::cout << "readE57: Opening file " << file << std::endl;
    osg::Matrix transform;
    transform.makeIdentity();

    try
    {
        e57::Reader reader(file.c_str());
        e57::E57Root root;
        reader.GetE57Root(root);

        int dataCount = reader.GetData3DCount();
        for(int idx = 0; idx < dataCount; ++idx)
        {
            e57::Data3D header;
            reader.ReadData3D(idx, header);
            std::cout << "  Processing scan " << (idx + 1) << "/" << dataCount << std::endl;

            // compute transformation
            osg::Matrix trans;
            trans.makeTranslate(header.pose.translation.x, header.pose.translation.y, header.pose.translation.z);
            osg::Matrix rot;
            rot.makeRotate(osg::Quat(header.pose.rotation.x, header.pose.rotation.y, header.pose.rotation.z, header.pose.rotation.w));
            transform = rot * trans;

            // get sizes
            int64_t nRows, nCols, nPointsSize, nGroups, nCounts;
            bool hasColumnIndex;
            reader.GetData3DSizes(idx, nRows, nCols, nPointsSize, nGroups, nCounts, hasColumnIndex);
            int64_t chunkSize = (nRows > 0 ? nRows : 1024);

            // reserve space to avoid reallocation
            if(nPointsSize > 0)
                vec.reserve(vec.size() + static_cast<size_t>(nPointsSize));

            // allocate buffers
            std::vector<int8_t> invalid(chunkSize, 0);
            std::vector<double> x(chunkSize), y(chunkSize), z(chunkSize);
            std::vector<double> range(chunkSize), az(chunkSize), el(chunkSize);
            bool haveXYZ = header.pointFields.cartesianXField;
            bool haveRange = header.pointFields.sphericalRangeField;

            bool haveIntensity = header.pointFields.intensityField;
            std::vector<double> intensity;
            double intOffset = 0.0, intRange = 1.0;
            if(haveIntensity)
            {
                intensity.resize(chunkSize);
                intOffset = header.intensityLimits.intensityMinimum;
                intRange = header.intensityLimits.intensityMaximum - intOffset;
            }

            bool haveColor = header.pointFields.colorRedField;
            std::vector<uint16_t> red, green, blue;
            int rOffset = 0, gOffset = 0, bOffset = 0;
            double rRange = 1.0, gRange = 1.0, bRange = 1.0;
            if(haveColor)
            {
                red.resize(chunkSize);
                green.resize(chunkSize);
                blue.resize(chunkSize);
                rOffset = header.colorLimits.colorRedMinimum;
                rRange = header.colorLimits.colorRedMaximum - rOffset;
                gOffset = header.colorLimits.colorGreenMinimum;
                gRange = header.colorLimits.colorGreenMaximum - gOffset;
                bOffset = header.colorLimits.colorBlueMinimum;
                bRange = header.colorLimits.colorBlueMaximum - bOffset;
            }

            // read data
            e57::CompressedVectorReader readerPoints =
                reader.SetUpData3DPointsData(idx, chunkSize, haveXYZ ? x.data() : nullptr, haveXYZ ? y.data() : nullptr, haveXYZ ? z.data() : nullptr, invalid.data(),
                                             haveIntensity ? intensity.data() : nullptr, nullptr, haveColor ? red.data() : nullptr, haveColor ? green.data() : nullptr,
                                             haveColor ? blue.data() : nullptr, nullptr, haveRange ? range.data() : nullptr, haveRange ? az.data() : nullptr, haveRange ? el.data() : nullptr);

            size_t beforeCount = vec.size();
            size_t count;
            while((count = readerPoints.read()) > 0)
            {
                for(size_t i = 0; i < count; ++i)
                {
                    if(invalid[i] != 0)
                        continue;

                    osg::Vec3 p;
                    if(haveXYZ)
                        p.set(x[i], y[i], z[i]);
                    else if(haveRange)
                        p.set(range[i] * cos(el[i]) * cos(az[i]), range[i] * cos(el[i]) * sin(az[i]), range[i] * sin(el[i]));
                    p = p * transform;

                    lamure::vec3r pos{p.x(), p.y(), p.z()};
                    lamure::vec3b color{0, 0, 0};
                    if(haveColor)
                    {
                        color.r = static_cast<uint8_t>(round((red[i] - rOffset) * 255.0 / rRange));
                        color.g = static_cast<uint8_t>(round((green[i] - gOffset) * 255.0 / gRange));
                        color.b = static_cast<uint8_t>(round((blue[i] - bOffset) * 255.0 / bRange));
                    }

                    vec.emplace_back(lamure::pre::surfel(pos, color));
                }
            }
            readerPoints.close();

            size_t afterCount = vec.size();
            std::cout << "    Added " << (afterCount - beforeCount) << " surfels from scan " << (idx + 1) << ", expected ~" << nPointsSize << std::endl;
        }

        std::cout << "Finished reading file: " << file << ", total surfels so far: " << vec.size() << std::endl;
        reader.Close();
    }
    catch(const std::exception &ex)
    {
        std::cerr << "Exception reading " << file << ": " << ex.what() << std::endl;
    }
}

void writeBinary(std::string filename, std::vector<lamure::pre::surfel> &vec)
{
    std::cout << "writeBinary: Writing " << vec.size() << " surfels (" << (vec.size() * (3 * sizeof(float) + 3 * sizeof(uint8_t))) << " bytes) to " << filename << std::endl;

    ofstream file(filename, ios::out | ios::ate | ios::binary);

    if(!file.is_open())
        throw std::runtime_error("Unable to open file: " + filename);

    for(int i = 0; i < vec.size(); i++)
    {
        file.write((char *)&(vec.at(i).pos().x), sizeof(float));
        file.write((char *)&(vec.at(i).pos().y), sizeof(float));
        file.write((char *)&(vec.at(i).pos().z), sizeof(float));

        file.write((char *)&(vec.at(i).color().r), sizeof(uint8_t));
        file.write((char *)&(vec.at(i).color().g), sizeof(uint8_t));
        file.write((char *)&(vec.at(i).color().b), sizeof(uint8_t));
    }

    file.close();
}

int main(int argc, char **argv)
{
    namespace fs = boost::filesystem;
    namespace po = boost::program_options;

    po::options_description visible("Usage: " + std::string(argv[0]) + " <input(.e57|dir)> <output(.bin|dir)>");
    visible.add_options()("help,h", "show help");

    po::options_description hidden;
    hidden.add_options()("input", po::value<std::string>()->required(), "input file or directory")("output", po::value<std::string>()->required(), "output file or directory");

    po::positional_options_description pod;
    pod.add("input", 1);
    pod.add("output", 1);

    po::variables_map vm;
    try
    {
        po::store(po::command_line_parser(argc, argv).options(visible.add(hidden)).positional(pod).run(), vm);
        if(vm.count("help"))
        {
            std::cout << visible << std::endl;
            return 0;
        }
        po::notify(vm);
    }
    catch(const po::error &e)
    {
        std::cerr << "Error: " << e.what() << std::endl << visible << std::endl;
        return 1;
    }

    fs::path inPath(vm["input"].as<std::string>());
    fs::path outPath(vm["output"].as<std::string>());

    bool inIsDir = fs::is_directory(inPath);
    bool outExists = fs::exists(outPath);
    bool outIsDir = outExists && fs::is_directory(outPath);
    bool outHasBinExt = (outPath.extension() == ".bin");

    std::vector<std::string> files;
    std::vector<lamure::pre::surfel> surfels;

    if(inIsDir)
    {
        // Input directory
        getFilenames(inPath, files);
        if(files.empty())
        {
            std::cerr << "Error: No .e57 files found in input directory." << std::endl;
            return 1;
        }

        if(outHasBinExt)
        {
            // Directory -> single .bin
            readE57(files[0], surfels);
            for(size_t i = 1; i < files.size(); ++i)
                readE57(files[i], surfels);

            writeBinary(outPath.string(), surfels);
        }
        else
        {
            // Directory -> directory
            if(!outExists)
                fs::create_directories(outPath);
            else if(!outIsDir)
            {
                std::cerr << "Error: Output path exists and is not a directory." << std::endl;
                return 1;
            }

            for(const auto &f : files)
            {
                surfels.clear();
                readE57(f, surfels);
                fs::path outFile = outPath / fs::path(f).filename();
                outFile.replace_extension(".bin");
                writeBinary(outFile.string(), surfels);
            }
        }
    }
    else if(fs::is_regular_file(inPath) && inPath.extension() == ".e57")
    {
        // Input single file
        if(outHasBinExt)
        {
            // File -> File
            readE57(inPath.string(), surfels);
            writeBinary(outPath.string(), surfels);
        }
        else
        {
            // File -> Directory
            if(!outExists)
                fs::create_directories(outPath);
            else if(!fs::is_directory(outPath))
            {
                std::cerr << "Error: Output path exists and is not a directory." << std::endl;
                return 1;
            }
            fs::path outFile = outPath / inPath.filename();
            outFile.replace_extension(".bin");

            readE57(inPath.string(), surfels);
            writeBinary(outFile.string(), surfels);
        }
    }
    else
    {
        std::cerr << "Error: Input must be a .e57 file or a directory containing .e57 files." << std::endl;
        return 1;
    }

    std::cout << "Conversion complete." << std::endl;
    return 0;
}