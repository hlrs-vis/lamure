#ifndef LAMURE_PVS_GRID_H
#define LAMURE_PVS_GRID_H

#include <string>

#include "lamure/pvs/view_cell.h"
#include "lamure/types.h"

namespace lamure
{
namespace pvs
{

class grid
{
public:
	virtual ~grid() {}

	virtual size_t get_cell_count() const = 0;

	virtual const view_cell* get_cell_at_index(const size_t& index) const = 0;
	virtual const view_cell* get_cell_at_position(const scm::math::vec3d& position) const = 0;

	virtual void set_cell_visibility(const size_t& cell_index, const model_t& model_id, const node_t& node_id, const bool& visibility) = 0;

	virtual void save_grid_to_file(const std::string& file_path, const std::vector<node_t>& ids) const = 0;
	virtual void save_visibility_to_file(const std::string& file_path, const std::vector<node_t>& ids) const = 0;

	virtual bool load_grid_from_file(const std::string& file_path) = 0;
	virtual bool load_visibility_from_file(const std::string& file_path) = 0;

	virtual model_t get_num_models() const = 0;
	virtual node_t get_num_nodes(const model_t& model_id) const = 0;
};

}
}

#endif
