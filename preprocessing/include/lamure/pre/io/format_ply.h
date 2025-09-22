// Copyright (c) 2014-2018 Bauhaus-Universitaet Weimar
// This Software is distributed under the Modified BSD License, see license.txt.
//
// Virtual Reality and Visualization Research Group 
// Faculty of Media, Bauhaus-Universitaet Weimar
// http://www.uni-weimar.de/medien/vr

#ifndef PRE_FORMAT_PLY_H_
#define PRE_FORMAT_PLY_H_

#include <functional>

#include <lamure/pre/platform.h>
#include <lamure/pre/io/format_abstract.h>

namespace lamure
{
namespace pre
{

class PREPROCESSING_DLL format_ply: public format_abstract
{
public:
    explicit format_ply()
        : format_abstract()
    {
        has_normals_ = false;
        has_radii_ = false;
        has_color_ = true;
    }


protected:
    virtual void read(const std::string &filename, surfel_callback_function callback) override;
    virtual void write(const std::string &filename, buffer_callback_function callback) override;

private:
    surfel current_surfel_;

    template<typename ScalarType>
    std::function<void(ScalarType)> scalar_callback(const std::string &element_name,
                                                    const std::string &property_name);
};

template<>
std::function<void(float)> format_ply::
scalar_callback(const std::string &element_name, const std::string &property_name);

template<>
std::function<void(uint8_t)> format_ply::
scalar_callback(const std::string &element_name, const std::string &property_name);

} // namespace pre
} // namespace lamure

#endif // PRE_FORMAT_PLY_H_

