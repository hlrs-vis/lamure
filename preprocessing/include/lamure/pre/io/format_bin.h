// Copyright (c) 2014-2018 Bauhaus-Universitaet Weimar
// This Software is distributed under the Modified BSD License, see license.txt.
//
// Virtual Reality and Visualization Research Group 
// Faculty of Media, Bauhaus-Universitaet Weimar
// http://www.uni-weimar.de/medien/vr

#ifndef PRE_FORMAT_BIN_H_
#define PRE_FORMAT_BIN_H_

#include <lamure/pre/platform.h>
#include <lamure/pre/io/file.h>
#include <lamure/pre/io/format_abstract.h>

namespace lamure
{
namespace pre
{

class PREPROCESSING_DLL format_bin: public format_abstract
{
public:
    explicit format_bin(): format_abstract()
    {
        has_normals_ = false;
        has_radii_ = false;
        has_color_ = true;
    }

protected:
    virtual void read(const std::string &filename, surfel_callback_function callback);
    virtual void write(const std::string &filename, buffer_callback_function callback);

};

} // namespace pre
} // namespace lamure

#endif // PRE_FORMAT_BIN_H_
