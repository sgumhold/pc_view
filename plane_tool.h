#pragma once

#include <cgv/media/color_scale.h>
#include <cgv_gl/arrow_renderer.h>
#include <cgv_gl/sphere_renderer.h>
#include "point_cloud_tool.h"

#include "lib_begin.h"

class CGV_API plane_tool : public point_cloud_tool
{
protected:
	std::vector<cgv::vec3> plane_centers;
	std::vector<cgv::vec3> plane_normals;
	std::vector<cgv::rgba> plane_colors;
	RGB default_point_color = RGB(0.5f,0.5f,0.5f);
	cgv::media::ColorScale color_scale = cgv::media::CS_HUE_LUMINANCE;
	bool show_planes = true;
	bool colorize_points  =true;
	unsigned min_nr_points = 50;
	unsigned max_nr_planes = 1000;
	float inlier_distance = 0.00001f;
	float inlier_percentage = 0.1f;
	bool early_termination = true;
	float probability_threshold = 0.01f;
	cgv::render::sphere_render_style srs;
	cgv::render::arrow_render_style ars;
	void compute_planes();
	void ensure_colors();
	void init_colors();
	void reset_planes();
public:
	plane_tool(point_cloud_viewer_ptr pcv_ptr);
	void on_point_cloud_change_callback(PointCloudChangeEvent pcc_event);
	bool self_reflect(cgv::reflect::reflection_handler& srh);
	bool init(cgv::render::context& ctx);
	void clear(cgv::render::context& ctx);
	void draw(cgv::render::context& ctx);
	void create_gui();
};

#include <cgv/config/lib_end.h>