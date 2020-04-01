#pragma once

#include "point_cloud_tool.h"
#include <cgv/math/fvec.h>

#include "lib_begin.h"

class CGV_API selection_tool : public point_cloud_tool
{
public:
	typedef cgv::math::fvec<int, 2> index_range;
	enum SelectMode {
		SM_SINGLE,
		SM_RANGE,
		SM_BOX
	};
	enum SelectAction {
		SA_SET,
		SA_ADD,
		SA_DEL
	};
private:
	SelectType last_type;
	SelectMode last_mode;
	Idx last_select_index;
	index_range last_select_range;
protected:
	bool auto_perform_action;
	bool auto_select_mode;
	std::vector<int> show_selection;
	SelectType type;
	SelectMode mode;
	SelectAction action;
	cgv::type::uint8_type current_selection;
	Idx select_index;
	index_range select_range;
	Box select_box;
	bool show_box;
	Rgba box_color;
	std::vector<cgv::type::uint8_type>& ref_selection() const;
	void perform_action();
	void create_components();
	void reset_selection_box();
	void apply_action(Idx i, bool part_of_selection);
	void config_gui();
public:
	selection_tool(point_cloud_viewer_ptr pcv_ptr);
	std::string get_icon_file_name() const { return "res://select96.png"; }
	void on_point_cloud_change_callback(PointCloudChangeEvent pcc_event);
	void on_set(void* member_ptr);
	bool self_reflect(cgv::reflect::reflection_handler& srh);
	void stream_help(std::ostream& os);
	bool handle(cgv::gui::event& e);
	void draw(cgv::render::context& ctx);
	void create_gui();
};

#include <cgv/config/lib_end.h>
