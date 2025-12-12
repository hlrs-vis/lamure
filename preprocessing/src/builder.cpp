// Copyright (c) 2014-2018 Bauhaus-Universitaet Weimar
// This Software is distributed under the Modified BSD License, see license.txt.
//
// Virtual Reality and Visualization Research Group 
// Faculty of Media, Bauhaus-Universitaet Weimar
// http://www.uni-weimar.de/medien/vr

#include <lamure/pre/builder.h>

#include <lamure/utils.h>
#include <lamure/memory.h>
#include <lamure/pre/bvh.h>
#include <lamure/pre/io/format_abstract.h>
#include <lamure/pre/io/format_xyz.h>
#include <lamure/pre/io/format_xyz_all.h>
#include <lamure/pre/io/format_xyz_prov.h>
#include <lamure/pre/io/format_xyz_grey.h>
#include <lamure/pre/io/format_bin.h>
#include <lamure/pre/io/format_bin_all.h>
#include <lamure/pre/io/format_bin_prov.h>
#include <lamure/pre/io/format_ply.h>
#include <lamure/pre/io/format_e57.h>
#include <lamure/pre/io/converter.h>

#include <lamure/pre/normal_computation_plane_fitting.h>
#include <lamure/pre/radius_computation_average_distance.h>
#include <lamure/pre/radius_computation_natural_neighbours.h>
#include <lamure/pre/reduction_normal_deviation_clustering.h>
#include <lamure/pre/reduction_normal_deviation_clustering_provenance.h>
#include <lamure/pre/reduction_constant.h>
#include <lamure/pre/reduction_every_second.h>
#ifdef CMAKE_OPTION_ENABLE_ALTERNATIVE_STRATEGIES
#include <lamure/pre/reduction_random.h>
#include <lamure/pre/reduction_entropy.h>
#include <lamure/pre/reduction_particle_simulation.h>
#include <lamure/pre/reduction_hierarchical_clustering.h>
#include <lamure/pre/reduction_k_clustering.h>
#include <lamure/pre/reduction_spatially_subdivided_random.h>
#include <lamure/pre/reduction_pair_contraction.h>
#include <lamure/pre/reduction_hierarchical_clustering_mk5.h>
#endif
#include <cstdio>
#include <fstream>
#include <unordered_map>
#include <unordered_set>


#define CPU_TIMER auto_timer timer("CPU time: %ws wall, usr+sys = %ts CPU (%p%)\n")

namespace fs = boost::filesystem;

namespace lamure
{
namespace pre
{

builder::
builder(const descriptor &desc)
    : desc_(desc), memory_limit_(calculate_memory_limit()) {
    std::string stem = desc_.output_base_name.empty() 
                     ? fs::path(desc_.input_file).stem().string() 
                     : desc_.output_base_name;
    base_path_ = fs::path(desc_.working_directory) / stem;
}

builder::~builder()
{}

auto get_start_stage = [](const std::string &ext) -> uint16_t
{
    static const std::unordered_map<std::string, uint16_t> mapping = {
        {".xyz", 0}, 
        {".xyz_all", 0}, 
        {".xyz_bin", 0}, 
        {".ply", 0}, 
        {".e57", 0}, 
        {".bin", 1}, 
        {".bin_all", 2}, 
        {".bin_wo_outlier", 2}, 
        {".bin_all_wo_outlier", 2},
        {".bvhd", 4}, 
        {".bvhu", 5},
    };
    auto it = mapping.find(ext);
    if(it == mapping.end())
    {
        LOGGER_ERROR("Unknown input file format: " + ext);
        return UINT16_MAX;
    }
    return it->second;
};

reduction_strategy *builder::get_reduction_strategy(reduction_algorithm algo) const
{
    switch (algo) {
        case reduction_algorithm::ndc:
            return new reduction_normal_deviation_clustering();
        case reduction_algorithm::ndc_prov:
            return new reduction_normal_deviation_clustering_provenance(base_path_.string());
        case reduction_algorithm::constant:
            return new reduction_constant();
        case reduction_algorithm::every_second:
            return new reduction_every_second();
#ifdef CMAKE_OPTION_ENABLE_ALTERNATIVE_STRATEGIES
        case reduction_algorithm::random:
            return new reduction_random();
        case reduction_algorithm::entropy:
            return new reduction_entropy();
        case reduction_algorithm::particle_sim:
            return new reduction_particle_simulation();
        case reduction_algorithm::hierarchical_clustering:
            return new reduction_hierarchical_clustering();
        case reduction_algorithm::k_clustering:
            return new reduction_k_clustering(desc_.number_of_neighbours);
        case reduction_algorithm::spatially_subdivided_random:
            return new reduction_spatially_subdivided_random();
        case reduction_algorithm::pair:
            return new reduction_pair_contraction(desc_.number_of_neighbours);
        case reduction_algorithm::hierarchical_clustering_extended:
            return new reduction_hierarchical_clustering_mk5();
#endif
        default:
            LOGGER_ERROR("Non-implemented reduction algorithm");
            return nullptr;
    };
}

radius_computation_strategy *builder::get_radius_strategy(radius_computation_algorithm algo) const
{
    switch (algo) {
        case radius_computation_algorithm::average_distance:return new radius_computation_average_distance(desc_.number_of_neighbours, desc_.radius_multiplier);
        case radius_computation_algorithm::natural_neighbours:return new radius_computation_natural_neighbours(20, 10, 3);
        default:LOGGER_ERROR("Non-implemented radius computation algorithm");
            return nullptr;
    };
}

normal_computation_strategy *builder::get_normal_strategy(normal_computation_algorithm algo) const
{
    switch (algo) {
        case normal_computation_algorithm::plane_fitting:return new normal_computation_plane_fitting(desc_.number_of_neighbours);
        default:LOGGER_ERROR("Non-implemented normal computation algorithm");
            return nullptr;
    };
}

boost::filesystem::path builder::convert_to_binary(std::string const &input_filename, std::string const &input_type) const
{
    std::cout << std::endl;
    std::cout << "--------------------------------" << std::endl;
    std::cout << "convert input file" << std::endl;
    std::cout << "--------------------------------" << std::endl;

    LOGGER_TRACE("convert " << input_type << " file to a binary file");
    auto input_file = fs::canonical(fs::path(input_filename));
    auto binary_file = base_path_;

    // Formate dynamisch wählen
    std::shared_ptr<format_abstract> format_in;
    std::shared_ptr<format_abstract> format_out;

    if(input_type == ".e57")
    {
        binary_file += ".bin";
        format_in = std::make_shared<format_e57>();
        format_out = std::make_shared<format_bin>();
    }
    else if(input_type == ".xyz_prov")
    {
        binary_file += "bin_prov";
        // spezielle statische Konvertierung
        format_xyz_prov::convert(input_filename, binary_file.string(), true);
        return binary_file;
    }
    else if(input_type == ".xyz")
    {
        binary_file += ".bin";
        format_in = std::make_shared<format_xyz>();
        format_out = std::make_shared<format_bin>();
    }
    else if(input_type == ".xyz_all")
    {
        binary_file += ".bin_all";
        format_in = std::make_shared<format_xyz_all>();
        format_out = std::make_shared<format_bin_all>();
    }
    else if(input_type == ".ply")
    {
        binary_file += ".bin";
        format_in = std::make_shared<format_ply>();
        format_out = std::make_shared<format_bin>();
    }
    else
    {
        LOGGER_ERROR("Unable to convert input file: Unknown file format");
        return {};
    }

    // Konvertierung ausführen
    converter conv(*format_in, *format_out, desc_.buffer_size);
    conv.set_surfel_callback(
        [](surfel &s, bool &keep)
        {
            if(s.pos() == vec3r(0.0, 0.0, 0.0))
                keep = false;
        });

    CPU_TIMER;
    conv.convert(input_file.string(), binary_file.string());

    return binary_file;
}

boost::filesystem::path builder::downsweep(boost::filesystem::path input_file, uint16_t start_stage) const
{
    bool performed_outlier_removal = false;
    do {
        std::string status_suffix = "";
        if (true == performed_outlier_removal) {
            status_suffix = " (after outlier removal)";
        }

        std::cout << std::endl;
        std::cout << "--------------------------------" << std::endl;
        std::cout << "bvh properties" << status_suffix << std::endl;
        std::cout << "--------------------------------" << std::endl;

        lamure::pre::bvh bvh(memory_limit_, desc_.buffer_size, desc_.rep_radius_algo, desc_.max_threads);

        bvh.init_tree(input_file.string(),
                      desc_.max_fan_factor,
                      desc_.surfels_per_node,
                      base_path_);

        bvh.print_tree_properties();
        std::cout << std::endl;

        std::cout << "--------------------------------" << std::endl;
        std::cout << "downsweep" << status_suffix << std::endl;
        std::cout << "--------------------------------" << std::endl;
        LOGGER_TRACE("downsweep stage");

        CPU_TIMER;
        bvh.downsweep(desc_.translate_to_origin, input_file.string(), desc_.prov_file);

        auto bvhd_file = add_to_path(base_path_, ".bvhd");

        bvh.serialize_tree_to_file(bvhd_file.string(), true);

        if ((!desc_.keep_intermediate_files) && (start_stage < 1)) {
            // do not remove input file
            std::remove(input_file.string().c_str());
        }

        input_file = bvhd_file;

        // LOGGER_DEBUG("Used memory: " << GetProcessUsedMemory() / 1024 / 1024 << " MiB");

        if (3 <= start_stage) {
            break;
        }

        if (performed_outlier_removal) {
            break;
        }

        if (start_stage <= 2) {

            if (desc_.outlier_ratio != 0.0) {

                size_t num_outliers = desc_.outlier_ratio * (bvh.nodes().size() - bvh.first_leaf()) * bvh.max_surfels_per_node();
                size_t ten_percent_of_surfels = std::max(size_t(0.1 * (bvh.nodes().size() - bvh.first_leaf()) * bvh.max_surfels_per_node()), size_t(1));
                num_outliers = std::min(std::max(num_outliers, size_t(1)), ten_percent_of_surfels); // remove at least 1 surfel, for any given ratio != 0.0


                std::cout << std::endl;
                std::cout << "--------------------------------" << std::endl;
                std::cout << "outlier removal ( " << int(desc_.outlier_ratio * 100) << " percent = " << num_outliers << " surfels)" << std::endl;
                std::cout << "--------------------------------" << std::endl;
                LOGGER_TRACE("outlier removal stage");

                surfel_vector kept_surfels = bvh.remove_outliers_statistically(num_outliers, desc_.number_of_outlier_neighbours);

                format_bin_all format_out;

                std::unique_ptr<format_abstract> dummy_format_in{new format_xyz()};

                converter conv(*dummy_format_in, format_out, desc_.buffer_size);

                conv.set_surfel_callback([](surfel &s, bool &keep)
                                         { if (s.pos() == vec3r(0.0, 0.0, 0.0)) keep = false; });

                auto binary_outlier_removed_file = add_to_path(base_path_, ".bin_all_wo_outlier");

                conv.write_in_core_surfels_out(kept_surfels, binary_outlier_removed_file.string());

                bvh.reset_nodes();

                input_file = fs::canonical(binary_outlier_removed_file);

                performed_outlier_removal = true;
            }
            else {
                break;
            }

        }

    }
    while (true);

    return input_file;
}

boost::filesystem::path builder::upsweep(boost::filesystem::path input_file,
                                         uint16_t start_stage,
                                         reduction_strategy const *reduction_strategy,
                                         normal_computation_strategy const *normal_comp_strategy,
                                         radius_computation_strategy const *radius_comp_strategy) const
{
    std::cout << std::endl;
    std::cout << "--------------------------------" << std::endl;
    std::cout << "upsweep" << std::endl;
    std::cout << "--------------------------------" << std::endl;
    LOGGER_TRACE("upsweep stage");

    lamure::pre::bvh bvh(memory_limit_, desc_.buffer_size, desc_.rep_radius_algo, desc_.max_threads);

    if (!bvh.load_tree(input_file.string())) {
        return boost::filesystem::path{};
    }

    if (bvh.state() != bvh::state_type::after_downsweep) {
        LOGGER_ERROR("Wrong processing state!");
        return boost::filesystem::path{};
    }

    CPU_TIMER;
    // perform upsweep
    bvh.upsweep(*reduction_strategy, *normal_comp_strategy, *radius_comp_strategy, desc_.compute_normals_and_radii, desc_.resample);

    auto bvhu_file = add_to_path(base_path_, ".bvhu");
    bvh.serialize_tree_to_file(bvhu_file.string(), true);

    if ((!desc_.keep_intermediate_files) && (start_stage < 2)) {
        std::remove(input_file.string().c_str());
    }

    input_file = bvhu_file;
// LOGGER_DEBUG("Used memory: " << GetProcessUsedMemory() / 1024 / 1024 << " MiB");
    return input_file;
}

bool builder::resample_surfels(boost::filesystem::path const &input_file) const
{
    std::cout << std::endl;
    std::cout << "--------------------------------" << std::endl;
    std::cout << "resample" << std::endl;
    std::cout << "--------------------------------" << std::endl;
    LOGGER_TRACE("resample stage");

    lamure::pre::bvh bvh(memory_limit_, desc_.buffer_size, desc_.rep_radius_algo, desc_.max_threads);

    if (!bvh.load_tree(input_file.string())) {
        return false;
    }

    if (bvh.state() != bvh::state_type::after_downsweep) {
        LOGGER_ERROR("Wrong processing state!");
        return false;
    }

    CPU_TIMER;
    // perform resample
    bvh.resample();

    //write resampled leaf level out
    format_xyz format_out;
    std::unique_ptr<format_xyz> dummy_format_in{new format_xyz()};
    auto xyz_res_file = add_to_path(base_path_, "_res.xyz");
    converter conv(*dummy_format_in, format_out, desc_.buffer_size);
    surfel_vector resampled_ll = bvh.get_resampled_leaf_lv_surfels();
    conv.write_in_core_surfels_out(resampled_ll, xyz_res_file.string());

    std::remove(input_file.string().c_str());

// LOGGER_DEBUG("Used memory: " << GetProcessUsedMemory() / 1024 / 1024 << " MiB");
    return true;
}

bool builder::reserialize(boost::filesystem::path const &input_file, uint16_t start_stage) const
{
    std::cout << std::endl;
    std::cout << "--------------------------------" << std::endl;
    std::cout << "serialize to file" << std::endl;
    std::cout << "--------------------------------" << std::endl;

    lamure::pre::bvh bvh(memory_limit_, desc_.buffer_size, desc_.rep_radius_algo, desc_.max_threads);
    if (!bvh.load_tree(input_file.string())) {
        return false;
    }
    if (bvh.state() != bvh::state_type::after_upsweep) {
        LOGGER_ERROR("Wrong processing state!");
        return false;
    }

    CPU_TIMER;
    auto lod_file = add_to_path(base_path_, ".lod");
    auto prov_file = add_to_path(base_path_, ".lod_prov");
    auto kdn_file = add_to_path(base_path_, ".bvh");
    auto json_file = add_to_path(base_path_, ".json");

    if (bvh.nodes()[0].has_provenance()) {
      std::cout << "write paradata json description: " << json_file << std::endl;
      prov::write_json(json_file.string());
    }
    bvh.serialize_surfels_to_file(lod_file.string(), prov_file.string(), desc_.buffer_size);
    bvh.serialize_tree_to_file(kdn_file.string(), false);

    if ((!desc_.keep_intermediate_files) && (start_stage < 3)) {
        std::remove(input_file.string().c_str());
        bvh.reset_nodes();

    }
    //LOGGER_DEBUG("Used memory: " << GetProcessUsedMemory() / 1024 / 1024 << " MiB");
    return true;
}

size_t builder::calculate_memory_limit() const
{
    size_t memory_budget = desc_.memory_budget * 1024UL * 1024UL * 1024UL;
    LOGGER_INFO("Total physical memory: " << get_total_memory() / 1024.0 / 1024.0 / 1024.0 << " GiB");
    LOGGER_INFO("Memory budget: " << desc_.memory_budget << " GiB (use -m to choose memory budget in GiB)");
    if (get_total_memory() <= memory_budget) {
        LOGGER_ERROR("Not enough memory. Go buy more RAM");
        return false;
    }
    LOGGER_INFO("Precision for storing coordinates and radii: " << std::string((sizeof(real) == 8) ? "double" : "single"));
    return desc_.memory_budget;
}

bool builder::resample() {
    memory_limit_ = calculate_memory_limit();

    fs::path infile = fs::canonical(desc_.input_file);
    auto ext = infile.extension().string();

    // Stage0-Formate, die erst konvertiert werden müssen
    static const std::unordered_set<std::string> raw_formats = {".bin"};

    // Falls .bin → convertieren
    if(ext == ".bin")
    {
        infile = convert_to_binary(desc_.input_file, ext);
        if(infile.empty()) return false;
        ext = infile.extension().string();
    }

    uint16_t start_stage = get_start_stage(ext);
    // convert to binary file
    if (0 >= start_stage) {
        infile = convert_to_binary(desc_.input_file, ext);
        if(infile.empty()) return false;
    }

    // downsweep (create bvh)
    if (3 >= start_stage) {
        infile = downsweep(infile, start_stage);
        if(infile.empty()) return false;
    }

    // resample (create new xyz)
    bool resample_success = resample_surfels(infile);
    return resample_success;
}

bool builder::construct()
{
    memory_limit_ = calculate_memory_limit();

    fs::path bin_file = add_to_path(base_path_, ".bin");
    fs::path bin_all_file = add_to_path(base_path_, ".bin_all");
    fs::path infile = fs::canonical(desc_.input_file);
    auto ext = infile.extension().string();
    uint16_t final_stage = desc_.final_stage;

    uint16_t start_stage = get_start_stage(ext);
    if(start_stage == UINT16_MAX)
        return false;

    // Stage 1: Konvertierung zu .bin (inkl. prov)
    if((start_stage <= 1) && (final_stage >= 1) && (ext != ".bin"))
    {
        std::cout << "[Stage 1] Konvertiere " << infile << " -> .bin\n";
        bin_file = convert_to_binary(infile.string(), ext);
        if(bin_file.empty())
        {
            LOGGER_ERROR("Conversion to .bin failed");
            return false;
        }
        infile = bin_file;
        ext = infile.extension().string();

        // ---------- 1a) externe Provenance-Datei --------------------------------------------------
        if(!desc_.prov_file.empty())
        {
            const fs::path prov_src = fs::canonical(desc_.prov_file);
            const std::string prov_ext = prov_src.extension().string();

            // Zielpfad IM working_directory
            fs::path prov_target = add_to_path(base_path_, ".bin_prov");

            desc_.prov_file = convert_to_binary(prov_src.string(), prov_ext).string();
            if(desc_.prov_file.empty())
            {
                LOGGER_ERROR("Conversion of prov-file failed");
                return false;
            }
            fs::rename(desc_.prov_file, prov_target);
            desc_.prov_file = prov_target.string();
        }
        // ---------- 1b) Dummy-Provenance, falls ndc_prov OHNE Eingabe ------------------------------
        else if(desc_.reduction_algo == reduction_algorithm::ndc_prov)
        {
            // Anzahl Surfels ermitteln
            std::ifstream surfel_bin(infile.string(), std::ios::binary | std::ios::ate);
            const uint64_t num = surfel_bin.tellg() / sizeof(surfel);
            surfel_bin.close();

            // Dummy-Datei innerhalb des working_directory anlegen
            fs::path prov_dummy = add_to_path(base_path_, ".bin_prov");
            desc_.prov_file = prov_dummy.string();

            std::ofstream dummy(desc_.prov_file, std::ios::binary | std::ios::trunc);
            dummy.seekp(num * sizeof(prov) - 1);
            dummy.put(0); // Datei sofort auf Endgröße bringen
        }

        std::cout << "desc_.prov_file: " << desc_.prov_file << std::endl;
    }

    // Stage 2: Radius and Normalen
    if((start_stage <= 2) && (final_stage >= 2))
    {
        if(fs::exists(bin_file))
        {
            auto fmt_in = std::make_shared<format_bin>();
            auto fmt_out = std::make_shared<format_bin_all>();
            converter conv(*fmt_in, *fmt_out, desc_.buffer_size);
            conv.convert(bin_file.string(), bin_all_file.string());
            infile = bin_all_file;
            ext = infile.extension().string();
            desc_.compute_normals_and_radii = true;
        }
        else
        {
            LOGGER_ERROR("[Stage 2] failed: " << bin_file);
            return false;
        }
    }
    
    // Stage 3: Downsweep
    if((start_stage <= 3) && (final_stage >= 3))
    {
        std::cout << "[Stage 3] Downsweep " << infile << "\n";
        fs::path out_file = downsweep(infile, start_stage);
        if(out_file.empty())
        {
            LOGGER_ERROR("[Stage 3] failed: " << infile); // Eingabe loggen
            return false;
        }
        infile = std::move(out_file);
    }

    // Stage 4: Upsweep
    if((start_stage <= 4) && final_stage >= 4)
    {
        std::cout << "[Stage 4] Upsweep " << infile << "\n";
        auto *red = get_reduction_strategy(desc_.reduction_algo);
        auto *nor = get_normal_strategy(desc_.normal_computation_algo);
        auto *rad = get_radius_strategy(desc_.radius_computation_algo);

        infile = upsweep(infile, start_stage, red, nor, rad);
        delete red;
        delete nor;
        delete rad;

        if(infile.empty())
        {
            LOGGER_ERROR("[Stage 4] failed");
            return false;
        }
    }

    // Stage 5: Serialisierung
    if((start_stage <= 5) && (final_stage >= 5))
    {
        std::cout << "[Stage 5] Serialisiere " << infile << "\n";
        if(!reserialize(infile, start_stage))
        {
            LOGGER_ERROR("Serialize failed");
            return false;
        }
        if(start_stage < 2 && !bin_all_file.empty() && fs::exists(bin_all_file))
        {
            fs::remove(bin_all_file);
        }
    }

    return true;
}

} // namespace pre
} // namespace lamure
