#pragma once

#include "point_cloud_tool.h"

#include "lib_begin.h"

class CGV_API transform_tool : public point_cloud_tool
{
public:
	float scale;
	vec3 translate;
	vec3 axis;
	float angle;
	float translation_scale;
	void transform();
	mat4 get_transformation() const;
public:
	transform_tool(point_cloud_viewer_ptr pcv_ptr);
	std::string get_icon_file_name() const { return "res://transform96.png"; }
	bool self_reflect(cgv::reflect::reflection_handler& srh);
	void on_set(void* member_ptr);
	void reset();
	void draw(cgv::render::context& ctx);
	void finish_draw(cgv::render::context& ctx);
	void create_gui();
};

#include <cgv/config/lib_end.h>
