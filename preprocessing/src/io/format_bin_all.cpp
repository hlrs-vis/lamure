// Copyright (c) 2014-2018 Bauhaus-Universitaet Weimar
// This Software is distributed under the Modified BSD License, see license.txt.
//
// Virtual Reality and Visualization Research Group 
// Faculty of Media, Bauhaus-Universitaet Weimar
// http://www.uni-weimar.de/medien/vr

#include <lamure/pre/io/format_bin_all.h>
#include <lamure/pre/node_serializer.h>
#include <lamure/pre/surfel.h>
#include <stdexcept>

namespace lamure
{
namespace pre
{

void format_bin_all::read(const std::string &filename, surfel_callback_function callback)
{
    shared_surfel_file input_file_disk_access = std::make_shared<surfel_file>();
    surfel_disk_array input;

    input_file_disk_access->open(filename);
    size_t input_size = input_file_disk_access->get_size();
    input = surfel_disk_array(input_file_disk_access, 0, input_size);

    for(size_t k = 0; k < input_size; ++k)
    {
        surfel surf = input.read_surfel(k);

        vec3r pos = surf.pos();
        vec3b color = surf.color();
        real radius = surf.radius();
        vec3f normal = surf.normal();

        callback(surfel(vec3r(pos.x, pos.y, pos.z), vec3b(color.r, color.g, color.b), radius, vec3f(normal.x, normal.y, normal.z)));
    }

    input_file_disk_access->close();
    LOGGER_INFO("Total number of surfels: " << input.length());
}


void format_bin_all::write(const std::string &filename, buffer_callback_function callback)
{
    surfel_file file;
    surfel_vector buffer;
    size_t count = 0;

    file.open(filename, true);
    while(true)
    {
        bool ret = callback(buffer);
        if(!ret)
            break;

        file.append(&buffer);
        count += buffer.size();
    }
    file.close();

    LOGGER_TRACE("Output surfels: " << count);
}

}
}

