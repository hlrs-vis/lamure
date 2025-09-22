// Copyright (c) 2014-2018 Bauhaus-Universitaet Weimar
// This Software is distributed under the Modified BSD License, see license.txt.
//
// Virtual Reality and Visualization Research Group 
// Faculty of Media, Bauhaus-Universitaet Weimar
// http://www.uni-weimar.de/medien/vr

#ifndef PRE_FORMAT_E57_H_
#define PRE_FORMAT_E57_H_
#include <boost/filesystem.hpp>
#include <lamure/pre/platform.h>
#include <lamure/pre/io/format_abstract.h>

namespace lamure
{
namespace pre
{

class PREPROCESSING_DLL format_e57: public format_abstract
{
public:
    explicit format_e57()
        : format_abstract()
    {
        has_normals_ = false;
        has_radii_ = false;
        has_color_ = true;
    }

protected:
    virtual void read(const std::string &filename, surfel_callback_function callback) override;
    virtual void write(const std::string &filename, buffer_callback_function callback) override;
    void getFilenames(const boost::filesystem::path &p, std::vector<std::string> &f);

};

} // namespace pre
} // namespace lamure

#endif // PRE_FORMAT_E57_H_

