#include <sstream>
#include <iostream>
#include <lamure/pre/io/merger.h>
#include <thread>
#include <cmath>

namespace lamure
{
namespace pre
{
void merger::getFilenames(const boost::filesystem::path &p, const std::string &input_ending, std::vector<std::string> &filenames)
{
    boost::filesystem::directory_iterator end_itr;
    for(boost::filesystem::directory_iterator itr(p); itr != end_itr; ++itr)
    {
        if(is_regular_file(itr->path()))
        {
            std::string current_file = itr->path().string();
            if(current_file.substr(current_file.size() - 4) == std::string(input_ending))
            {
                filenames.push_back(current_file);
            }
        }
    }
    std::cout << "Number of files found: " << filenames.size() << std::endl;
}

void merger::
merge(std::vector<std::string> &input_filenames, 
      const std::string &output_filename)
{
    discarded_ = 0;
    flush_ready_ = false;
    flush_done_ = false;

    auto buf_callback = [&](surfel_vector &surfels)
    {
        std::unique_lock<std::mutex> lk(mtx_);
        cv_.wait(lk, [this] { return flush_ready_; });

        surfels = buffer_;
        flush_ready_ = false;
        bool has_data = !buffer_.empty();

        // notify main thread
        flush_done_ = true;
        lk.unlock();
        cv_.notify_one();
        return has_data;
    };

    // output thread
    std::thread tr([&] { out_format_.write(output_filename, buf_callback); });

    // read input
    for(int itr = 0; itr != input_filenames.size(); ++itr)
    {
        std::cout << "reading file: " << input_filenames.at(itr) << std::endl;
        in_format_.read(input_filenames.at(itr), [&](const surfel &s) { this->append_surfel(s); });
    }

    flush_buffer();
    {
        std::lock_guard<std::mutex> lk(mtx_);
        flush_ready_ = true;
    }
    cv_.notify_one();
    tr.join();

    if(discarded_ > 0)
    {
        LOGGER_WARN("Discarded degenerate surfels: " << discarded_);
    }
}

void merger::write_in_core_surfels_out(const surfel_vector &surf_vec, const std::string &output_filename)
{
    discarded_ = 0;
    flush_ready_ = false;
    flush_done_ = false;

    std::string extended_output_filename = output_filename;

    auto buf_callback = [&](surfel_vector &surfels)
    {
        std::unique_lock<std::mutex> lk(mtx_);
        cv_.wait(lk, [this] { return flush_ready_; });

        surfels = buffer_;
        flush_ready_ = false;
        bool has_data = !buffer_.empty();

        // notify main thread
        flush_done_ = true;
        lk.unlock();
        cv_.notify_one();
        return has_data;
    };

    // output thread
    std::thread tr([&] { out_format_.write(extended_output_filename, buf_callback); });

    // read input
    for(auto const &surf : surf_vec)
    {
        this->append_surfel(surfel(surf.pos(), surf.color()));
    

    flush_buffer();
    {
        std::lock_guard<std::mutex> lk(mtx_);
        flush_ready_ = true;
    }
    cv_.notify_one();
    tr.join();

    if(discarded_ > 0)
    {
        LOGGER_WARN("Discarded degenerate surfels: " << discarded_);
    }
    }
}

void merger::append_surfel(const surfel &surf)
{
    if(is_degenerate(surf))
    {
        ++discarded_;
        return;
    }

    surfel s(surf);
    bool keep = true;

    if(surfel_callback_)
        surfel_callback_(s, keep);

    if(keep)
    {
        if(scale_factor_ != 1.0)
        {
            s.pos() *= scale_factor_;
            s.radius() *= scale_factor_;
        }

        if(translation_ != vec3r(0.0))
        {
            s.pos() += translation_;
        }

        if(override_radius_)
            s.radius() = new_radius_;

        if(override_color_)
            s.color() = new_color_;

        // LOGGER_DEBUG("Pos: " << s.pos());

        buffer_.push_back(s);

        if(buffer_.size() > surfels_in_buffer_)
            flush_buffer();
    }
}

void merger::flush_buffer()
{
    if(!buffer_.empty())
    {
        //LOGGER_INFO("Flush buffer to disk. buffer size: " << buffer_.size() << "\n");
        {
            std::lock_guard<std::mutex> lk(mtx_);
            flush_ready_ = true;
        }
        cv_.notify_one();
        {
            std::unique_lock<std::mutex> lk(mtx_);
            // get back result
            cv_.wait(lk, [this] { return flush_done_; });
            flush_done_ = false;
            buffer_.clear();
        }
    }
}

const bool merger::is_degenerate(const surfel &s) const { 
    return !std::isfinite(s.pos().x) || !std::isfinite(s.pos().y) || !std::isfinite(s.pos().z); 
}

} // namespace pre
} // namespace lamure