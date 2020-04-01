#pragma once

#include "point_cloud_tool.h"

#include "lib_begin.h"

class CGV_API generate_tool : public point_cloud_tool
{
public:
	typedef cgv::math::fvec<int, 2> index_range;
	enum GenerateMode {
		GM_DOME,
		GM_END,
		GM_START = GM_DOME
	};
private:
	GenerateMode generate_mode;

protected:
	bool auto_generate;
	Idx samples_per_row;
	Idx nr_rows;
	bool one_component_per_row;
	bool append;
	vec3 extent;
	float R;
	void generate_points();
public:
	generate_tool(point_cloud_viewer_ptr pcv_ptr);
	std::string get_icon_file_name() const { return "res://generate96.png"; }
	void on_set(void* member_ptr);
	void draw(cgv::render::context& ctx);
	bool self_reflect(cgv::reflect::reflection_handler& srh);
	void create_gui();
};

#include <cgv/config/lib_end.h>
