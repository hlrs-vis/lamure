
#ifndef PRE_MERGER_H_
#define PRE_MERGER_H_

#include <mutex>
#include <condition_variable>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <lamure/pre/platform.h>
#include <lamure/pre/io/format_abstract.h>

#include <lamure/pre/logger.h>

namespace lamure
{
namespace pre
{

class PREPROCESSING_DLL merger
{
  public:
    typedef std::function<void(surfel &, bool &)> surfel_modifier_function;

    std::vector<std::string> files;

    explicit merger(format_abstract &in_format,
                    format_abstract &out_format,
                    size_t buffer_size) // buffer_size - in bytes
        : in_format_(in_format),
        out_format_(out_format),
        translation_(vec3r(0.0)),
        override_radius_(false),
        override_color_(false),
        scale_factor_(1.0),
        new_radius_(0.0),
        discarded_(0)
    {
        surfels_in_buffer_ = buffer_size / sizeof(surfel);
    }

    virtual ~merger() {}

    void merge(std::vector<std::string> &input_filenames, const std::string &output_filename);
    
    void getFilenames(const boost::filesystem::path &p, const std::string &input_ending,  std::vector<std::string> &filenames);

    void write_in_core_surfels_out(const surfel_vector &, const std::string &output_filename);

    const size_t surfels_in_buffer() const 
    { 
        return surfels_in_buffer_;
    }

    void override_radius(const real radius)
    {
        override_radius_ = true;
        new_radius_ = radius;
    }

    void override_color(const vec3b &color)
    {
        override_color_ = true;
        new_color_ = color;
    }

    void set_scale_factor(const real factor) 
    { scale_factor_ = factor; }

    void set_translation(const vec3r &translation) 
    { translation_ = translation; }

    void set_surfel_callback(const surfel_modifier_function &callback) 
    { surfel_callback_ = callback; }


    private:
        bool flush_ready_ = false;
        bool flush_done_ = false;

        std::mutex mtx_;
        std::condition_variable cv_;

        void append_surfel(const surfel &surfel);
        void flush_buffer();
        const bool is_degenerate(const surfel &s) const;

        std::vector<std::string> file_names;
        format_abstract &in_format_;
        format_abstract &out_format_;

        vec3r translation_;
        size_t surfels_in_buffer_;
        bool override_radius_;
        bool override_color_;
        real scale_factor_;
        surfel_modifier_function surfel_callback_;

        surfel_vector buffer_;

        real new_radius_;
        vec3b new_color_;
        size_t discarded_;

        
};
} // namespace pre
} // namespace lamure

#endif // PRE_MERGER_H_