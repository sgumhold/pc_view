#pragma once

#include "point_cloud_tool.h"

#include "lib_begin.h"

class CGV_API clip_tool : public point_cloud_tool
{
protected:
	Box clip_box;
	bool show_clipping;
	Rgba clip_box_color;
	void clip_points();
	void reset_clip_box();
public:
	clip_tool(point_cloud_viewer_ptr pcv_ptr);
	void on_point_cloud_change_callback(PointCloudChangeEvent pcc_event);
	bool self_reflect(cgv::reflect::reflection_handler& srh);
	void draw(cgv::render::context& ctx);
	void create_gui();
};

#include <cgv/config/lib_end.h>