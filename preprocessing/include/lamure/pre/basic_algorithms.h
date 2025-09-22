// Copyright (c) 2014-2018 Bauhaus-Universitaet Weimar
// This Software is distributed under the Modified BSD License, see license.txt.
//
// Virtual Reality and Visualization Research Group 
// Faculty of Media, Bauhaus-Universitaet Weimar
// http://www.uni-weimar.de/medien/vr

#ifndef PRE_BASIC_ALGORITHMS_H_
#define PRE_BASIC_ALGORITHMS_H_

#include <lamure/pre/common.h>
#include <lamure/pre/surfel_disk_array.h>
#include <lamure/pre/surfel_mem_array.h>
#include <lamure/bounding_box.h>

namespace lamure
{
namespace pre
{

/**
* A group of basic algorithms, which are used during tree construction.
*/
class basic_algorithms
{
public:

    struct surfel_group_properties {
        real         max_radius_deviation;
        real         rep_radius;
        vec3r        centroid;
        bounding_box bbox;
    };

    template<class T>
    using splitted_array = std::vector<std::pair<T, bounding_box>>;

    basic_algorithms() = delete;

    static bounding_box compute_aabb(const surfel_mem_array &sa,
                                     const bool parallelize = true);

    static bounding_box compute_aabb(const surfel_disk_array &sa,
                                     const size_t buffer_size,
                                     const bool parallelize = true);

    static void translate_surfels(surfel_mem_array &sa,
                                  const vec3r &translation);

    static void translate_surfels(surfel_disk_array &sa,
                                  const vec3r &translation,
                                  const size_t buffer_size);

    static surfel_group_properties
    compute_properties(const surfel_mem_array &sa,
                       const rep_radius_algorithm rep_radius_algo,
                       bool use_radii_for_node_expansion = true);

    static void sort_and_split(surfel_mem_array &sa,
                               splitted_array<surfel_mem_array> &out,
                               const bounding_box &box,
                               const uint8_t split_axis,
                               const uint8_t fan_factor,
                               const bool parallelize = false);

    static void sort_and_split(surfel_disk_array &sa,
                               splitted_array<surfel_disk_array> &out,
                               const bounding_box &box,
                               const uint8_t split_axis,
                               const uint8_t fan_factor,
                               const size_t memory_limit);

private:

    template<class T>
    static void split_surfel_array(T &sa,
                                   splitted_array<T> &out,
                                   const bounding_box &box,
                                   const uint8_t split_axis,
                                   const uint8_t fan_factor);

};

}
} // namespace lamure

#endif // PRE_BASIC_ALGORITHMS_H_

