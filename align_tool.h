#pragma once

#include "point_cloud_tool.h"

#include "lib_begin.h"

class CGV_API align_tool : public point_cloud_tool
{
public:
	Idx source_selection;
	Idx target_selection;
	void align();
public:
	align_tool(point_cloud_viewer_ptr pcv_ptr);
	std::string get_icon_file_name() const { return "res://align96.png"; }
	bool self_reflect(cgv::reflect::reflection_handler& srh);
	void center();
	void create_gui();
};

#include <cgv/config/lib_end.h>
