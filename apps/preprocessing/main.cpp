// Copyright (c) 2014-2018 Bauhaus-Universitaet Weimar
// This Software is distributed under the Modified BSD License, see license.txt.
//
// Virtual Reality and Visualization Research Group
// Faculty of Media, Bauhaus-Universitaet Weimar
// http://www.uni-weimar.de/medien/vr

#include <chrono>
#include <iostream>
#include <omp.h>
#include <stdexcept>
#include <vector>
#include <fstream>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <lamure/pre/builder.h>
#include <lamure/pre/io/converter.h>
#include <lamure/pre/io/format_bin.h>
#include <lamure/pre/io/format_bin_all.h>
#include <lamure/pre/io/format_bin_prov.h>
#include <lamure/pre/io/format_e57.h>
#include <lamure/pre/io/format_ply.h>
#include <lamure/pre/io/format_xyz.h>
#include <lamure/pre/io/format_xyz_all.h>
#include <lamure/pre/io/format_xyz_grey.h>
#include <lamure/pre/io/merger.h>

static std::string normalize_ext(const std::string &e)
{
    if(e.empty())
        return "";
    return (e.front() == '.') ? e : "." + e;
}

enum class Mode
{
    MERGE,            // >1 Input → Merge
    CONVERT_ONLY,     // -c → nur Konvertieren/Mergen
    BUILD_ONLY,       // genau 1 .bin/.bin_all + Dir → nur Bauen
    CONVERT_AND_BUILD // anderer Fall ohne -c → Konvertieren + Bauen
};

struct Config
{
    std::vector<boost::filesystem::path> inputs;
    boost::filesystem::path output;
    bool output_is_dir;
    bool convert_only;
    bool merge_only;
    std::string input_ext_hint;
    std::string output_ext_hint;
};


int main(int argc, const char *argv[])
{
    // Start timer and enable nested OpenMP
    auto start = std::chrono::steady_clock::now();
    omp_set_nested(1);
    namespace po = boost::program_options;
    namespace fs = boost::filesystem;

    // executable name for usage
    const std::string exec_name = (argc > 0) ? fs::basename(argv[0]) : "";
    const std::string details_msg = "\nFor details use -h or --help option.\n";
    po::variables_map vm;
    po::positional_options_description pod;
    po::options_description od_hidden("hidden");
    po::options_description od_cmd("cmd");
    po::options_description od("Usage: " + exec_name + " -i INPUT -o OUTPUT [OPTIONS]\n\n"
                               "  - Convert:  -c   -i INPUT | (-i INPUT_DIR + -x EXT) + -o OUTPUT | (-o OUTPUT_DIR + -y EXT)\n"
                               "  - Merge:    -m   -i INPUT | (-i INPUT_DIR + -x EXT) + -o OUTPUT | (-o OUTPUT_DIR + -y EXT)\n"
                               "  - Build:         -i INPUT | (-i INPUT_DIR + -x EXT) + -o OUTPUT_DIR  [OPTIONS]\n\n");
    try
    {
        od.add_options()
            ("help,h", "print this help message")
            ("convert,c", "nur konvertieren (Batch‐ oder Einzel‐Convert), ohne Build")
            ("merge,g", "nur mergen, ohne Build")
            ("input,i", po::value<std::vector<std::string>>()->composing()->required(), "input file(s) or directory")
            ("input-extension,x", po::value<std::string>(), "when input is a directory")
            ("output,o", po::value<std::string>()->required(), "output file or directory")
            ("output-extension,y", po::value<std::string>(), "when output is a directory and not Build-Mode")

            ("final-stage,s", po::value<int>()->default_value(5),
             "number of the stage to stop after.\n"
             "  1 - Conversion to intermediate binary format\n"
             "  2 - Normal and radius computation\n"
             "  3 - Downsweep (BVH creation, optional outlier removal)\n"
             "  4 - Upsweep (LOD generation)\n"
             "  5 - Final serialization to .bvh/.lod format.\n"
             "make sure the final stage number is equal or higher than the start stage, which is implicitly defined by INPUT")

            ("max-fanout", po::value<int>()->default_value(2),
            "maximum fan-out factor for tree properties computation. "
            "Actual factor value depends on total number of surfels and desired "
            "number of surfels per node (option -d), so it might be less than "
            "provided value")

            ("no-translate-to-origin", "do not translate surfels to origin. "
            "If this option is set, the surfels in the input file will not be "
            "translated to the root bounding box center.")

            ("desired,d", po::value<int>()->default_value(1024),
            "the application tries to achieve a number of surfels per node close to "
            "this value")

            ("recompute,r", "recompute surfel normals and radii, even if they data is present in the "
            "input data")

            ("outlier-ratio", po::value<float>()->default_value(0.0f),
            "the application will remove (<outlier-ratio> * total_num_surfels) points "
            " with the max avg-distance to it's k-nearest neighbors from the input point "
            "cloud before computing the LODs")

            ("num-outlier-neighbours", po::value<int>()->default_value(24), // davor: 24
            "defines the number of nearest neighbours that are searched for the outlier detection")

            ("neighbours", po::value<int>()->default_value(10), // davor: 40
            "Number of neighbours for the plane-fitting when computing normals")

            ("keep-interm,k", "prevents deletion of intermediate files")

            ("resample", "resample to replace huge surfels by collection of smaller one")

            ("memory-budget,m", po::value<float>()->default_value(8.0, "8.0"),
            "the total amount of physical memory allowed to be used by the "
            "application in gigabytes")

            ("buffer-size,b", po::value<int>()->default_value(150), "buffer size in megabytes")

            ("max-threads,t", po::value<int>()->default_value(8), "max. hardware threads")

            ("prov-file", po::value<std::string>()->default_value(""),
            "Optional ascii-file with provanance attribs per point. Extensions suppoterd: \n"
            "  .prov for single float value per line\n"
            "  .xyz_prov for xyzrgb +p per line")

            ("reduction-algo", po::value<std::string>()->default_value("const"),
            "Reduction strategy for the LOD construction. Possible values:\n"
            "  ndc - normal deviation clustering (ndc)\n"
            "  ndc_prov - output provenance information alongside ndc\n"
            "  const - ndc with constant radius\n"
            "  everysecond - take every fanout-factor's surfel\n"
            "  random - randomly select points with possible duplicates\n"
            "  entropy - take sufels with min entropy\n"
            "  particlesim - perform particle simulation\n"
            "  hierarchical - create clusters by binary splitting of the point cloud\n"
            "  kclustering - hash-based k-clustering\n"
            "  pair - use iterative point-pair contractions\n"
            "  spatiallyrandom - subdivide scene into equally sized cubes and choose random surfels from different cubes")

            ("normal-computation-algo", po::value<std::string>()->default_value("planefitting"),
            "Algorithm for computing surfel normal. Possible values:\n"
            "  planefitting ")

            ("radius-computation-algo", po::value<std::string>()->default_value("naturalneighbours"),
            "Algorithm for computing surfel radius. Possible values:\n"
            "  averagedistance \n"
            "  naturalneighbours")

            ("radius-multiplier", po::value<float>()->default_value(1.0, "1.0"), // davor: 0.7
            "the multiplier for the average distance to the neighbours"
            "during radius computation when using the averagedistance strategy")

            ("rep-radius-algo", po::value<std::string>()->default_value("amean"),
            "Algorithm for computing representative surfel radius for tree nodes. Possible values:\n"
            "  amean - arithmetic mean\n"
            "  gmean - geometric mean\n"
            "  hmean - harmonic mean");

        pod.add("input", -1);
        od_cmd.add(od);
        po::store(po::command_line_parser(argc, argv).options(od_cmd).positional(pod).run(), vm);
        if(vm.count("help"))
        {
            std::cout << od << std::endl;
            std::cout << "Build mode:\n"
                         "  The starting stage is determined by the INPUT file extension:\n"
                         "    Stage 0 (.xyz, .xyz_all, .ply, .e57): From raw point cloud.\n"
                         "    Stage 1 (.bin): Skips conversion. Starts with normal/radius computation.\n"
                         "    Stage 2 (.bin_all, .bin_all_wo_outlier): Skips conversion and normal/radius computation. Starts with downsweep.\n"
                         "    Stage 4 (.bvhd): Skips downsweep. Starts with upsweep.\n"
                         "    Stage 5 (.bvhu): Skips upsweep. Starts with final serialization.\n"
                         "  Intermediate files from previous runs must exist when starting from stages > 1 (e.g. using -k).\n"
                         "  Example: /lamure_preprocessing.exe -i file1.e57 file2.e57 -o /out --final-stage 5 --max-fanout 2 -d 1024 -r --outlier-ratio 0.00001 --num-outlier-neighbours 24 --neighbours 16 -m 12.0 -t 20 --reduction-algo ndc --normal-computation-algo planefitting --radius-computation-algo naturalneighbours --radius-multiplier 1.0 --rep-radius-algo amean\n\n"
                         "Conversion mode (-c option):\n"
                         "  Converts one or more input files to a target format without building a BVH.\n"
                         "  Supported input formats: .xyz, .xyz_all, .xyz_grey, .ply, .e57\n"
                         "  Supported output formats: .bin, .bin_all, .bin_prov\n"
                         "  Example: " + exec_name + " -c -i file.e57 -o file.bin\n\n"
                         "Merge mode (-g option):\n"
                         "  Merges multiple input files of the same type into a single output file (can convert also).\n"
                         "  Example (multiple files): " + exec_name + " -g -i file1.e57 file2.e57 -o merged.bin\n"
                         "  Example (directory): " + exec_name + " -g -i point_cloud_dir/ -x .e57 -o merged.bin\n\n";
            return EXIT_SUCCESS;
        }

        po::notify(vm);
    }
    catch(po::error &e)
    {
        std::cerr << "Error: " << e.what() << "\nFor details use -h or --help.\n";
        return EXIT_FAILURE;
    }


    // --- setup format factory ---
    size_t buffer_size = size_t(std::max(vm["buffer-size"].as<int>(), 20)) * 1024UL * 1024UL;
    lamure::pre::format_factory f;
    f[".xyz"] = &lamure::pre::create_format_instance<lamure::pre::format_xyz>;
    f[".xyz_all"] = &lamure::pre::create_format_instance<lamure::pre::format_xyz_all>;
    f[".xyz_grey"] = &lamure::pre::create_format_instance<lamure::pre::format_xyz_grey>;
    f[".ply"] = &lamure::pre::create_format_instance<lamure::pre::format_ply>;
    f[".bin"] = &lamure::pre::create_format_instance<lamure::pre::format_bin>;
    f[".bin_all"] = &lamure::pre::create_format_instance<lamure::pre::format_bin_all>;
    f[".bin_prov"] = &lamure::pre::create_format_instance<lamure::pre::format_bin_prov>;
    f[".e57"] = &lamure::pre::create_format_instance<lamure::pre::format_e57>;


    // --- helper lambdas ---
    auto convert_file = [&](const fs::path &in, const fs::path &out)
    {
        std::string in_ext = normalize_ext(in.extension().string());
        std::string out_ext = normalize_ext(out.extension().string());
        if(in_ext == out_ext)
        {
            throw std::runtime_error("Input and output formats must differ: " + in_ext);
        }
        if(f.find(in_ext) == f.end() || f.find(out_ext) == f.end())
        {
            throw std::runtime_error("Unknown format for conversion: " + in_ext + " → " + out_ext);
        }
        auto inp = f[in_ext]();
        auto outp = f[out_ext]();
        lamure::pre::converter conv(*inp, *outp, buffer_size);
        conv.convert(in.string(), out.string());
        delete inp;
        delete outp;
    };

    auto merge_files = [&](const std::vector<fs::path> &ins, const fs::path &out)
    {
        std::string in_ext = normalize_ext(ins.front().extension().string());
        std::string out_ext = normalize_ext(out.extension().string());
        if(f.find(in_ext) == f.end() || f.find(out_ext) == f.end())
        {
            throw std::runtime_error("Unknown format for merge.");
        }
        for(auto &p : ins)
        {
            if(normalize_ext(p.extension().string()) != in_ext)
            {
                throw std::runtime_error("All inputs for merge must share the same extension.");
            }
        }
        auto inp = f[in_ext]();
        auto outp = f[out_ext]();
        lamure::pre::merger mer(*inp, *outp, buffer_size);
        std::vector<std::string> names;
        for(auto &p : ins)
            names.push_back(p.string());
        mer.merge(names, out.string());
        delete inp;
        delete outp;
    };

    
    Config cfg;
    cfg.merge_only = vm.count("merge") > 0;
    cfg.convert_only = vm.count("convert") > 0;
    cfg.input_ext_hint = vm.count("input-extension") ? vm["input-extension"].as<std::string>() : "";
    cfg.output_ext_hint = vm.count("output-extension") ? vm["output-extension"].as<std::string>() : "";
    cfg.output = fs::absolute(vm["output"].as<std::string>());
    cfg.output_is_dir = fs::exists(cfg.output) && fs::is_directory(cfg.output) || cfg.output.extension().string().empty();

    // Inputs sammeln
    for(auto &s : vm["input"].as<std::vector<std::string>>())
        cfg.inputs.emplace_back(fs::absolute(s));

    // Falls ein Hint gesetzt, Verzeichnisse durchsuchen
    if(!cfg.input_ext_hint.empty())
    {
        std::vector<fs::path> expanded;
        for(auto &p : cfg.inputs)
        {
            if(fs::is_directory(p))
            {
                for(auto &e : fs::directory_iterator(p))
                    if(e.path().extension() == cfg.input_ext_hint)
                        expanded.push_back(e.path());
            }
            else
            {
                expanded.push_back(p);
            }
        }
        cfg.inputs.swap(expanded);
    }

    // Merge-Only
    if(cfg.merge_only && cfg.inputs.size() > 1)
    {
        std::cout << "[Merge] -> " << cfg.output << "\n";
        merge_files(cfg.inputs, cfg.output);
        return EXIT_SUCCESS;
    }

    // Convert-Only
    if(cfg.convert_only)
    {
        for(auto &in : cfg.inputs)
        {
            const auto in_ext = normalize_ext(in.extension().string());
            // Ziel-Extension bestimmen
            const auto target_ext = cfg.output_is_dir ? (!cfg.output_ext_hint.empty() ? cfg.output_ext_hint : ".bin") : normalize_ext(cfg.output.extension().string());

            const fs::path outp = cfg.output_is_dir ? (cfg.output / (in.stem().string() + target_ext)) : fs::path(cfg.output).replace_extension(target_ext);

            if(in_ext == target_ext)
            {
                std::cout << "[Skip]    " << in << " (bereits " << target_ext << ")\n";
            }
            else
            {
                std::cout << "[Convert] " << in << " -> " << outp << "\n";
                convert_file(in, outp);
            }
        }
        auto end = std::chrono::steady_clock::now();
        std::cout << "Total time [s]: " << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() << "\n";
        return EXIT_SUCCESS;
    }

    // Falls weder Merge noch Convert, nur Inputs ausgeben
    std::cout << "Input Files:\n";
    for(auto &i : cfg.inputs)
        std::cout << "  " << i << "\n";

    lamure::pre::builder::descriptor desc;
    desc.max_fan_factor = std::min(std::max(vm["max-fanout"].as<int>(), 2), 8);
    desc.surfels_per_node = vm["desired"].as<int>();
    desc.final_stage = vm["final-stage"].as<int>();
    desc.max_threads = vm["max-threads"].as<int>();
    desc.compute_normals_and_radii = vm.count("recompute");
    desc.keep_intermediate_files = vm.count("keep-interm");
    desc.resample = vm.count("resample");
    desc.memory_budget = std::max(vm["memory-budget"].as<float>(), 1.0f);
    desc.buffer_size = buffer_size;
    desc.number_of_neighbours = std::max(vm["neighbours"].as<int>(), 1);
    desc.translate_to_origin = !vm.count("no-translate-to-origin");
    desc.outlier_ratio = std::max(0.0f, vm["outlier-ratio"].as<float>());
    desc.number_of_outlier_neighbours = std::max(vm["num-outlier-neighbours"].as<int>(), 1);
    desc.radius_multiplier = vm["radius-multiplier"].as<float>();
    desc.prov_file = vm["prov-file"].as<std::string>();
    std::string ra = vm["reduction-algo"].as<std::string>();
    if(ra == "ndc")
        desc.reduction_algo = lamure::pre::reduction_algorithm::ndc;
    else if(ra == "ndc_prov")
        desc.reduction_algo = lamure::pre::reduction_algorithm::ndc_prov;
    else if(ra == "const")
        desc.reduction_algo = lamure::pre::reduction_algorithm::constant;
    else if(ra == "everysecond")
        desc.reduction_algo = lamure::pre::reduction_algorithm::every_second;
    else if(ra == "random")
        desc.reduction_algo = lamure::pre::reduction_algorithm::random;
    else if(ra == "entropy")
        desc.reduction_algo = lamure::pre::reduction_algorithm::entropy;
    else if(ra == "particlesim")
        desc.reduction_algo = lamure::pre::reduction_algorithm::particle_sim;
    else if(ra == "hierarchical")
        desc.reduction_algo = lamure::pre::reduction_algorithm::hierarchical_clustering;
    else if(ra == "kclustering")
        desc.reduction_algo = lamure::pre::reduction_algorithm::k_clustering;
    else if(ra == "pair")
        desc.reduction_algo = lamure::pre::reduction_algorithm::pair;
    else if(ra == "spatiallyrandom")
        desc.reduction_algo = lamure::pre::reduction_algorithm::spatially_subdivided_random;
    else
        throw std::runtime_error("Unknown reduction algorithm: " + ra);

    // normal computation
    std::string na = vm["normal-computation-algo"].as<std::string>();
    if(na == "planefitting")
        desc.normal_computation_algo = lamure::pre::normal_computation_algorithm::plane_fitting;
    else
        throw std::runtime_error("Unknown normal computation algorithm: " + na);

    // radius computation
    std::string ra2 = vm["radius-computation-algo"].as<std::string>();
    if(ra2 == "averagedistance")
        desc.radius_computation_algo = lamure::pre::radius_computation_algorithm::average_distance;
    else if(ra2 == "naturalneighbours")
        desc.radius_computation_algo = lamure::pre::radius_computation_algorithm::natural_neighbours;
    else
        throw std::runtime_error("Unknown radius computation algorithm: " + ra2);

    // representative radius
    std::string rra = vm["rep-radius-algo"].as<std::string>();
    if(rra == "amean")
        desc.rep_radius_algo = lamure::pre::rep_radius_algorithm::arithmetic_mean;
    else if(rra == "gmean")
        desc.rep_radius_algo = lamure::pre::rep_radius_algorithm::geometric_mean;
    else if(rra == "hmean")
        desc.rep_radius_algo = lamure::pre::rep_radius_algorithm::harmonic_mean;
    else
        throw std::runtime_error("Unknown representative radius algorithm: " + rra);

    
    for(auto &conv_file : cfg.inputs)
    {
        desc.input_file = fs::canonical(conv_file).string();
        fs::path filename = conv_file.filename();
        desc.working_directory = cfg.output_is_dir ? fs::canonical(cfg.output).string() : fs::canonical(conv_file.parent_path()).string();

        fs::path bvh_path = fs::path(desc.working_directory) / filename;
        bvh_path.replace_extension(".bvh");

        fs::path log_path = fs::path(desc.working_directory) / filename;
        log_path.replace_extension(".log");

        std::cout << bvh_path << std::endl;

        if(!fs::exists(desc.working_directory) || !fs::is_directory(desc.working_directory))
        {
            throw std::runtime_error("Arbeitsverzeichnis existiert nicht: " + desc.working_directory);
        }

        //if(boost::filesystem::exists(bvh_path))
        //{
        //    std::cout << "[Skip Build] " << conv_file << " (\"" << bvh_path.filename().string() << "\" existiert bereits)\n";
        //    continue;
        //}

        std::cout << log_path << std::endl;
        std::cout << "[Description]:\n" << desc << "\n";

        lamure::pre::builder builder(desc);
        bool ok = builder.construct();
        if(!ok) { throw std::runtime_error("Build failed für " + desc.input_file); }

        std::ofstream log_file(log_path.string());
        if(log_file)
        {
            log_file << "[Build Configuration]\n" << desc << std::endl;
            log_file.close();
            std::cout << "[Log] " << log_path << "\n";
        }
        else
        {
            std::cerr << "[Warnung] Build-Konfiguration konnte nicht geschrieben werden: " << log_path << "\n";
        }
    }


    // --- Laufzeit ausgeben und beenden ---
    auto end = std::chrono::steady_clock::now();
    std::cout << "Total time [s]: " << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() << std::endl;

    return EXIT_SUCCESS;
}
