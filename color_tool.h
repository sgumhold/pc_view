#pragma once

#include "point_cloud_tool.h"

#include "lib_begin.h"

class CGV_API color_tool : public point_cloud_tool
{
public:
	typedef cgv::math::fvec<int, 2> index_range;
	enum ColoringMode {
		CM_NBR_GRAPH,
		CM_WEIGHTS,
		CM_BILATERAL_WEIGHTS,
		CM_END,
		CM_START = CM_NBR_GRAPH
	};
private:
	ColoringMode coloring_mode;

protected:
	Idx select_index;
	Clr default_color;
	Clr highlight_color;
	Clr highlight2_color;
	void normalize_weights(std::vector<Crd>& weights) const;
	void config_gui();
	void compute_colors();
public:
	color_tool(point_cloud_viewer_ptr pcv_ptr);
	std::string get_icon_file_name() const { return "res://color96.png"; }
	void on_point_cloud_change_callback(PointCloudChangeEvent pcc_event);
	void on_activation_change_callback(bool gets_active);
	void on_set(void* member_ptr);
	void draw(cgv::render::context& ctx);
	bool self_reflect(cgv::reflect::reflection_handler& srh);
	bool handle(cgv::gui::event& e);
	void create_gui();
};

#include <cgv/config/lib_end.h>
