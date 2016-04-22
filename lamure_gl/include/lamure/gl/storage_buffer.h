// Copyright (c) 2014 Bauhaus-Universitaet Weimar
// This Software is distributed under the Modified BSD License, see license.txt.
//
// Virtual Reality and Visualization Research Group 
// Faculty of Media, Bauhaus-Universitaet Weimar
// http://www.uni-weimar.de/medien/vr

#ifndef LAMURE_GL_STORAGE_BUFFER_H_
#define LAMURE_GL_STORAGE_BUFFER_H_

#include <lamure/gl/buffer.h>
#include <lamure/platform_gl.h>

namespace lamure {
namespace gl {

class storage_buffer_t : public buffer_t {
public:
  storage_buffer_t(uint32_t num_elements, uint32_t element_size);
  ~storage_buffer_t();

  void* map(GLuint map_mode);
  void unmap();

  void enable(uint32_t slot);
  void disable();

private:

};

} // namespace gl
} // namespace lamure

#endif
