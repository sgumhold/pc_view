#pragma once

#include <cgv/math/fvec.h>
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
	bool self_reflect(cgv::reflect::reflection_handler& srh);
	void create_gui();
};

#include <cgv/config/lib_end.h>
