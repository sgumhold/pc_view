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

std::vector<cgv::type::uint8_type>& point_cloud_tool::ref_point_selection() const
{
	return viewer_ptr->point_selection;
}

std::vector<cgv::type::uint8_type>& point_cloud_tool::ref_component_selection() const
{
	return viewer_ptr->component_selection;
}

point_cloud_tool::Clr& point_cloud_tool::ref_selection_color(unsigned i) const
{
	return viewer_ptr->point_selection_colors[i];
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

bool point_cloud_tool::handle(cgv::gui::event& e)
{
	return false;
}

void point_cloud_tool::stream_help(std::ostream& os)
{

}

void point_cloud_tool::on_point_cloud_change_callback()
{

}

void point_cloud_tool::on_nr_points_change_callback()
{

}

void point_cloud_tool::on_nr_components_change_callback()
{

}
