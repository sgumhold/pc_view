#pragma once

#include "point_cloud_tool.h"

#include "lib_begin.h"

class CGV_API image_based_normal_estimator : public point_cloud_tool
{
protected:
	void compute_normals_from_index_image();
public:
	image_based_normal_estimator(point_cloud_viewer_ptr pcv_ptr);
	void create_gui();
};

#include <cgv/config/lib_end.h>