// Copyright (c) 2014-2018 Bauhaus-Universitaet Weimar
// This Software is distributed under the Modified BSD License, see license.txt.
//
// Virtual Reality and Visualization Research Group 
// Faculty of Media, Bauhaus-Universitaet Weimar
// http://www.uni-weimar.de/medien/vr

#ifndef PRE_BUILDER_H_
#define PRE_BUILDER_H_

#include <string>
#include <ostream>
#include <lamure/pre/platform.h>
#include <lamure/pre/common.h>

#include <boost/filesystem.hpp>

#include <lamure/pre/logger.h>

namespace lamure
{
namespace pre
{

class reduction_strategy;
class radius_computation_strategy;
class normal_computation_strategy;

class PREPROCESSING_DLL builder
{
public:
    struct descriptor
    {
        std::string input_file;
        std::string working_directory;
        std::string prov_file;
        uint32_t max_fan_factor;
        size_t surfels_per_node;
        uint16_t final_stage;
        bool compute_normals_and_radii;
        bool keep_intermediate_files;
        bool resample;
        float memory_budget;
        uint16_t max_threads;
        float radius_multiplier;
        size_t buffer_size;
        uint16_t number_of_neighbours;
        bool translate_to_origin;
        uint16_t number_of_outlier_neighbours;
        float outlier_ratio;

        rep_radius_algorithm rep_radius_algo;
        reduction_algorithm reduction_algo;
        radius_computation_algorithm radius_computation_algo;
        normal_computation_algorithm normal_computation_algo;
    };


    explicit builder(const descriptor &desc);

    virtual             ~builder();
    builder(const builder &other) = delete;
    builder &operator=(const builder &other) = delete;

    bool construct();
    bool resample();

private:
    reduction_strategy *get_reduction_strategy(reduction_algorithm algo) const;
    radius_computation_strategy *get_radius_strategy(radius_computation_algorithm algo) const;
    normal_computation_strategy *get_normal_strategy(normal_computation_algorithm algo) const;
    boost::filesystem::path convert_to_binary(std::string const& input_filename, std::string const &input_type) const;
    boost::filesystem::path downsweep(boost::filesystem::path input_file, uint16_t start_stage) const;
    boost::filesystem::path upsweep(boost::filesystem::path input_file,
                                    uint16_t start_stage,
                                    reduction_strategy const *reduction_strategy,
                                    normal_computation_strategy const *normal_comp_strategy,
                                    radius_computation_strategy const *radius_comp_strategy) const;
    bool resample_surfels(boost::filesystem::path const &input_file) const;
    bool reserialize(boost::filesystem::path const &input_file, uint16_t start_stage) const;

    size_t calculate_memory_limit() const;

    descriptor desc_;
    size_t memory_limit_;
    boost::filesystem::path base_path_;
};


inline std::string to_string(reduction_algorithm algo)
{
    switch(algo)
    {
    case reduction_algorithm::ndc:
        return "ndc";
    case reduction_algorithm::ndc_prov:
        return "ndc_prov";
    case reduction_algorithm::constant:
        return "const";
    case reduction_algorithm::every_second:
        return "everysecond";
    case reduction_algorithm::random:
        return "random";
    case reduction_algorithm::entropy:
        return "entropy";
    case reduction_algorithm::particle_sim:
        return "particlesim";
    case reduction_algorithm::hierarchical_clustering:
        return "hierarchical";
    case reduction_algorithm::k_clustering:
        return "kclustering";
    case reduction_algorithm::pair:
        return "pair";
    case reduction_algorithm::spatially_subdivided_random:
        return "spatiallyrandom";
    default:
        return "unknown";
    }
}

inline std::string enum_to_string(rep_radius_algorithm algo)
{
    switch(algo)
    {
    case rep_radius_algorithm::arithmetic_mean:
        return "arithmetic_mean";
    case rep_radius_algorithm::geometric_mean:
        return "geometric_mean";
    case rep_radius_algorithm::harmonic_mean:
        return "harmonic_mean";
    }
    return "unknown";
}

inline std::string enum_to_string(reduction_algorithm algo)
{
    switch(algo)
    {
    case reduction_algorithm::ndc:
        return "ndc";
    case reduction_algorithm::ndc_prov:
        return "ndc_prov";
    case reduction_algorithm::constant:
        return "const";
    case reduction_algorithm::every_second:
        return "everysecond";
    case reduction_algorithm::random:
        return "random";
    case reduction_algorithm::entropy:
        return "entropy";
    case reduction_algorithm::particle_sim:
        return "particlesim";
    case reduction_algorithm::hierarchical_clustering:
        return "hierarchical";
    case reduction_algorithm::k_clustering:
        return "kclustering";
    case reduction_algorithm::pair:
        return "pair";
    case reduction_algorithm::spatially_subdivided_random:
        return "spatiallyrandom";
    }
    return "unknown";
}

inline std::string enum_to_string(radius_computation_algorithm algo)
{
    switch(algo)
    {
    case radius_computation_algorithm::average_distance:
        return "averagedistance";
    case radius_computation_algorithm::natural_neighbours:
        return "naturalneighbours";
    }
    return "unknown";
}

inline std::string enum_to_string(normal_computation_algorithm algo)
{
    if(algo == normal_computation_algorithm::plane_fitting)
    {
        return "planefitting";
    }
    return "unknown";
}

// ─── FREIER OPERATOR<< FÜR descriptor ────────────────────────────────────

inline std::ostream &operator<<(std::ostream &os, builder::descriptor const &d)
{
    os << "input_file:                   " << d.input_file << "\n"
       << "working_directory:            " << d.working_directory << "\n"
       << "prov_file:                    " << d.prov_file << "\n"
       << "max_fan_factor:               " << d.max_fan_factor << "\n"
       << "surfels_per_node:             " << d.surfels_per_node << "\n"
       << "final_stage:                  " << d.final_stage << "\n"
       << "compute_normals_and_radii:    " << (d.compute_normals_and_radii ? "true" : "false") << "\n"
       << "keep_intermediate_files:      " << (d.keep_intermediate_files ? "true" : "false") << "\n"
       << "resample:                     " << (d.resample ? "true" : "false") << "\n"
       << "memory_budget (GB):           " << d.memory_budget << "\n"
       << "radius_multiplier:            " << d.radius_multiplier << "\n"
       << "buffer_size (bytes):          " << d.buffer_size << "\n"
       << "max_threads:          " << d.max_threads << "\n"
       << "number_of_neighbours:         " << d.number_of_neighbours << "\n"
       << "translate_to_origin:          " << (d.translate_to_origin ? "true" : "false") << "\n"
       << "number_of_outlier_neighbours: " << d.number_of_outlier_neighbours << "\n"
       << "outlier_ratio:                " << d.outlier_ratio << "\n"
       << "rep_radius_algo:              " << enum_to_string(d.rep_radius_algo) << "\n"
       << "reduction_algo:               " << enum_to_string(d.reduction_algo) << "\n"
       << "radius_computation_algo:      " << enum_to_string(d.radius_computation_algo) << "\n"
       << "normal_computation_algo:      " << enum_to_string(d.normal_computation_algo) << "\n";
    return os;
}

} // namespace pre
} // namespace lamure

#endif // PRE_BUILDER_H_
