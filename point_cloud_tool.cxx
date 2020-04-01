#include "point_cloud_tool.h"
#include <map>

void*& point_cloud_tool::get_variable_pointer(const std::string& variable_name)
{
	static std::map<std::string, void*> pointer_map;
	if (pointer_map.find(variable_name) == pointer_map.end())
		pointer_map[variable_name] = 0;
	return pointer_map[variable_name];
}

point_cloud& point_cloud_tool::ref_pc() const
{
	return viewer_ptr->ref_point_cloud();
}

neighbor_graph& point_cloud_tool::ref_ng() const
{
	return viewer_ptr->ref_neighbor_graph();
}

normal_estimator& point_cloud_tool::ref_ne() const
{
	return viewer_ptr->ref_normal_estimator();
}

std::string point_cloud_tool::get_icon_file_name() const
{
	return std::string();
}


cgv::render::view* point_cloud_tool::ref_view_ptr() const
{
	return viewer_ptr->view_ptr;
}

std::vector<point_cloud_tool::RGBA>& point_cloud_tool::ref_point_selection_colors() const
{
	return viewer_ptr->point_selection_colors;
}


std::vector<cgv::type::uint8_type>& point_cloud_tool::ref_point_selection() const
{
	return viewer_ptr->point_selection;
}

std::vector<cgv::type::uint8_type>& point_cloud_tool::ref_component_selection() const
{
	return viewer_ptr->component_selection;
}

point_cloud_tool::point_cloud_tool(point_cloud_viewer_ptr pcv_ptr, const std::string& _name) : cgv::base::node(_name)
{
	viewer_ptr = pcv_ptr;
}

void point_cloud_tool::on_set(void* member_ptr)
{
	update_member(member_ptr);
	post_redraw();
}

std::string point_cloud_tool::get_type_name() const
{
	return "point_cloud_tool";
}

bool point_cloud_tool::am_i_active() const
{
	return viewer_ptr->am_i_active(const_cast<point_cloud_tool*>(this));
}

bool point_cloud_tool::handle(cgv::gui::event& e)
{
	return false;
}

void point_cloud_tool::stream_help(std::ostream& os)
{

}

void point_cloud_tool::on_point_cloud_change_callback(PointCloudChangeEvent pcc_event)
{

}

void point_cloud_tool::on_activation_change_callback(bool tool_gets_active)
{

}
