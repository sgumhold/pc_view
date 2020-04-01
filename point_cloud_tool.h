#pragma once

#include <cgv/base/node.h>
#include <cgv/data/ref_ptr.h>
#include <cgv/render/drawable.h>
#include <libs/point_cloud/point_cloud.h>
#include <cgv/gui/event_handler.h>
#include <cgv/gui/provider.h>
#include "point_cloud_viewer.h"

#include "lib_begin.h"

class CGV_API point_cloud_tool : public cgv::base::node, public cgv::render::drawable, public cgv::gui::event_handler, public cgv::gui::provider, public point_cloud_types
{
private:
	static void*& get_variable_pointer(const std::string& variable_name);
protected:
	point_cloud_viewer_ptr viewer_ptr;
	point_cloud& ref_pc() const;
	neighbor_graph& ref_ng() const;
	normal_estimator& ref_ne() const;
	cgv::render::view* ref_view_ptr() const;
	bool get_picked_point(int x, int y, unsigned& index) { return viewer_ptr ? viewer_ptr->get_picked_point(x,y,index) : false; }
	std::vector<RGBA>& ref_point_selection_colors() const;
	std::vector<cgv::type::uint8_type>& ref_point_selection() const;
	std::vector<cgv::type::uint8_type>& ref_component_selection() const;
	template <typename T>
	T& ref_variable(const std::string& variable_name, const T& initial_value) {
		void*& var_ptr = get_variable_pointer(variable_name);
		if (var_ptr == 0)
			var_ptr = new T(initial_value);
		return *(reinterpret_cast<T*>(var_ptr));
	}
	bool am_i_active() const;

public:
	point_cloud_tool(point_cloud_viewer_ptr pcv_ptr, const std::string& _name);
	std::string get_type_name() const;
	void on_set(void* member_ptr);
	bool handle(cgv::gui::event& e);
	void stream_help(std::ostream& os);
	virtual std::string get_icon_file_name() const;
	virtual void on_point_cloud_change_callback(PointCloudChangeEvent pcc_event);
	virtual void on_activation_change_callback(bool tool_gets_active);
};

typedef cgv::data::ref_ptr<point_cloud_tool> point_cloud_tool_ptr;

#include <cgv/config/lib_end.h>