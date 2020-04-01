#pragma once

#include "point_cloud_tool.h"

#include "lib_begin.h"

enum IndexImageType
{
	IIT_X,
	IIT_Y,
	IIT_Z,
	IIT_NORMAL,
	IIT_COLOR,
	IIT_POINT_INDEX,
	IIT_NEIGHBOR_COUNT
};

class CGV_API index_image_inspector : public point_cloud_tool
{
protected:
	bool show_image;
	unsigned image_component_index;
	IndexImageType image_type;
	unsigned image_scale;
	cgv::math::fvec<Cnt, 2> component_range;
public:
	index_image_inspector(point_cloud_viewer_ptr pcv_ptr);
	std::string get_icon_file_name() const { return "res://image96.png"; }
	void on_point_cloud_change_callback(PointCloudChangeEvent pcc_event);
	bool self_reflect(cgv::reflect::reflection_handler& srh);
	void draw(cgv::render::context& ctx);
	void create_gui();
};

#include <cgv/config/lib_end.h>