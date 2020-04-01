#include "selection_tool.h"
#include <cgv/gui/dialog.h>
#include <cgv/gui/key_event.h>
#include <cgv/gui/mouse_event.h>
#include <cgv_gl/gl/gl.h>
#include <cgv_reflect_types/media/axis_aligned_box.h>
#include <cgv_reflect_types/media/color.h>

selection_tool::selection_tool(point_cloud_viewer_ptr pcv_ptr) : point_cloud_tool(pcv_ptr,"select"), box_color(0, 1, 0, 1)
{
	auto_perform_action = false;
	auto_select_mode = true;
	type = ST_POINT;
	mode = SM_SINGLE;
	action = SA_SET;
	current_selection = 1;
	select_index = 0;
	select_range = index_range(0,1);
	show_box = true;
	show_selection.resize(ref_point_selection_colors().size());
	for (unsigned i=0; i<ref_point_selection_colors().size(); ++i)
		(bool&)(show_selection[i]) = ref_point_selection_colors()[i][3] > 0.0f ? true : false;
}

void selection_tool::reset_selection_box()
{
	select_box = Box(Pnt(0, 0, 0), Pnt(1, 1, 1));
	for (unsigned c = 0; c < 3; ++c) {
		update_member(&select_box.ref_min_pnt()(c));
		update_member(&select_box.ref_max_pnt()(c));
	}
}

std::vector<cgv::type::uint8_type>& selection_tool::ref_selection() const
{
	if (type == ST_POINT)
		return ref_point_selection();
	else
		return ref_component_selection();
}

void selection_tool::apply_action(Idx i, bool part_of_selection)
{
	if (part_of_selection) {
		switch (action) {
		case SA_SET:
		case SA_ADD:
			ref_selection()[i]= current_selection;
			break;
		case SA_DEL:
			ref_selection()[i] = 0;
		}
	}
	else {
		switch (action) {
		case SA_SET:
			if (ref_selection()[i] == current_selection)
				ref_selection()[i] = 0;
			break;
		}
	}
}

void selection_tool::perform_action()
{
	Idx i;
	Box box;
	// check for incremental update
	if (last_mode == mode && last_type == type && mode != SM_BOX) {
		switch (mode) {
		case SM_SINGLE:
			apply_action(last_select_index, false);
			apply_action(select_index, true);
			last_select_index = select_index;
			break;
		case SM_RANGE:
			for (i = last_select_range[0]; i <= last_select_range[1]; ++i)
				apply_action(i, false);
			for (i = select_range[0]; i <= select_range[1]; ++i)
				apply_action(i, true);
			last_select_range = select_range;
			break;
		}
	}
	else {
		Cnt nr = Cnt(type == ST_POINT ? ref_pc().get_nr_points() : ref_pc().get_nr_components());
		switch (mode) {
		case SM_SINGLE:
			for (i = 0; i < (Idx)nr; ++i)
				apply_action(i, i == select_index);
			last_select_index = select_index;
			break;
		case SM_RANGE:
			for (i = 0; i < select_range[0]; ++i)
				apply_action(i, false);
			for (; i <= select_range[1]; ++i)
				apply_action(i, true);
			for (; i < (Idx)nr; ++i)
				apply_action(i, false);
			last_select_range = select_range;
			break;
		case SM_BOX :
			box = Box(ref_pc().box().get_min_pnt() + ref_pc().box().get_extent() * select_box.get_min_pnt(), ref_pc().box().get_min_pnt() + ref_pc().box().get_extent() * select_box.get_max_pnt());
			if (type == ST_POINT) {
				for (i = 0; i < (Idx)ref_pc().get_nr_points(); ++i) 
					apply_action(i, box.inside(ref_pc().transformed_pnt(i)));
			}
			else {
				std::vector<bool> selected(ref_pc().get_nr_components(), false);
				for (i = 0; i < (Idx)ref_pc().get_nr_points(); ++i)
					if (box.inside(ref_pc().transformed_pnt(i)))
						selected[ref_pc().component_index(i)] = true;
				for (i = 0; i < (Idx)ref_pc().get_nr_components(); ++i)
					apply_action(i, selected[i]);
			}
			break;
		}
	}
	viewer_ptr->on_selection_change_callback(type);
	post_redraw();
}

bool selection_tool::handle(cgv::gui::event& e)
{
	if (e.get_kind() == cgv::gui::EID_KEY) {
		cgv::gui::key_event& ke = static_cast<cgv::gui::key_event&>(e);
		if (ke.get_action() == cgv::gui::KA_PRESS || ke.get_action() == cgv::gui::KA_REPEAT) {
			switch (ke.get_key()) {
			case 'C':
				if (ke.get_modifiers() == 0) {
					type = ST_COMPONENT;
					on_set(&type);
					return true;
				}
				break;
			case 'P':
				if (ke.get_modifiers() == 0) {
					type = ST_POINT;
					on_set(&type);
					return true;
				}
				break;
			case '0':
			case '1':
			case '2':
			case '3':
				if (ke.get_modifiers() == 0) {
					current_selection = ke.get_key() - '0';
					on_set(&current_selection);
					return true;
				}
				if (ke.get_modifiers() == cgv::gui::EM_SHIFT) {
					show_selection[ke.get_key() - '0'] = !show_selection[ke.get_key() - '0'];
					on_set(&show_selection[ke.get_key() - '0']);
					return true;
				}
			}
		}
		return false;
	}

	if (e.get_kind() != cgv::gui::EID_MOUSE)
		return false;
	cgv::gui::mouse_event& me = (cgv::gui::mouse_event&) e;
	if (me.get_action() == cgv::gui::MA_MOVE) {
		if (me.get_modifiers() != cgv::gui::EM_SHIFT)
			return false;
		unsigned picked_index;
		if (get_picked_point(me.get_x(), me.get_y(), picked_index)) {
			if (ref_pc().has_normals()) {
				Pnt eye = ref_view_ptr()->get_eye();
				if (dot(eye - ref_pc().pnt(picked_index), ref_pc().nml(picked_index)) < 0)
					return false;
			}
			ref_point_selection()[picked_index] = current_selection;
			viewer_ptr->on_selection_change_callback(type);
		}
		return true;
	}
	return false;
}

void selection_tool::stream_help(std::ostream& os)
{
	os << "selection tool: 1-3 set index, S-0-3 toggle show" << std::endl;
}

bool selection_tool::self_reflect(cgv::reflect::reflection_handler& srh)
{
	return
		srh.reflect_member("mode", (int&)mode) &&
		srh.reflect_member("type", (int&)type) &&
		srh.reflect_member("action", (int&)action) &&
		srh.reflect_member("current_selection", current_selection) &&
		srh.reflect_member("auto_perform_action", auto_perform_action) &&
		srh.reflect_member("auto_select_mode", auto_select_mode) &&
		srh.reflect_member("select_index", select_index) &&
		srh.reflect_member("select_range", select_range) &&
		srh.reflect_member("show_box", show_box) &&
		srh.reflect_member("box_color", box_color) &&
		srh.reflect_member("select_box", select_box);
}

void selection_tool::draw(cgv::render::context& ctx)
{
	if (!show_box || !select_box.is_valid())
		return;
	viewer_ptr->draw_box(ctx, Box(ref_pc().box().get_min_pnt() + ref_pc().box().get_extent() * select_box.get_min_pnt(), ref_pc().box().get_min_pnt() + ref_pc().box().get_extent() * select_box.get_max_pnt()), box_color);
}

void selection_tool::on_point_cloud_change_callback(PointCloudChangeEvent pcc_event)
{
	if ((pcc_event & PCC_POINTS_MASK) != 0)
		reset_selection_box();
}

void selection_tool::on_set(void* member_ptr)
{
	if (ref_pc().has_components() && type == ST_COMPONENT) {
		for (unsigned si = 0; si < ref_point_selection_colors().size(); ++si) {
			if (member_ptr >= &ref_point_selection_colors()[si] && member_ptr < &ref_point_selection_colors()[si] + 1) {
				bool new_show = ref_point_selection_colors()[si][3] > 0.0f ? true : false;
				if (new_show != (bool&)(show_selection[si])) {
					(bool&)(show_selection[si]) = new_show;
					update_member(&show_selection[si]);
				}
				viewer_ptr->on_selection_change_callback(type);
				break;
			}
			if (member_ptr == &show_selection[si]) {
				if (show_selection[si]) {
					if (ref_point_selection_colors()[si][3] == float_to_color_component(0.0)) {
						ref_point_selection_colors()[si][3] = float_to_color_component(1.0);
						viewer_ptr->on_selection_change_callback(type);
						update_member(&ref_point_selection_colors()[si][3]);
					}
				}
				else {
					if (ref_point_selection_colors()[si][3] > float_to_color_component(0.0)) {
						ref_point_selection_colors()[si][3] = float_to_color_component(0.0);
						viewer_ptr->on_selection_change_callback(type);
						update_member(&ref_point_selection_colors()[si][3]);
					}
				}
				break;
			}
		}
	}
	if (member_ptr == &type) {
		if (!ref_pc().has_components() && type == ST_COMPONENT) {
			type = ST_POINT;
			cgv::gui::message("point cloud has no components that could be selected");
		}
		config_gui();
		update_member(&type);
		return;
	}
	if (member_ptr == &select_index) {
		if (auto_select_mode && mode != SM_SINGLE) {
			mode = SM_SINGLE;
			on_set(&mode);
		}
		if (auto_perform_action) {
			perform_action();
		}
	}
	if (member_ptr == &select_range[0] || member_ptr == &select_range[1]) {
		if (auto_select_mode && mode != SM_RANGE) {
			mode = SM_RANGE;
			on_set(&mode);
		}
		if (auto_perform_action) {
			perform_action();
		}
	}
	if (member_ptr >= &select_box && member_ptr < &select_box+1) {
		if (auto_select_mode && mode != SM_BOX) {
			mode = SM_BOX;
			on_set(&mode);
		}
		if (auto_perform_action) {
			perform_action();
		}
		post_redraw();
	}
	update_member(member_ptr);
}

void selection_tool::create_components()
{
	point_cloud& pc = ref_pc();
	std::vector<Cnt> cnts(4, 0);
	for (auto pi : ref_point_selection())
		++cnts[pi];
	Cnt nr = 0;
	for (auto c : cnts)
		if (c > 0)
			++nr;
	if (nr == 1) {
		cgv::gui::message("no points selected for component creation");
		return;
	}
	// compute permutation

	// create components and accumulate counts
	pc.create_components();
	pc.create_component_colors();
	pc.create_component_tranformations();
	Cnt i;
	std::vector<Cnt> cis(4, 0);
	for (i = 0; i < 4; ++i) {
		if (cnts[i] > 0) {
			cis[i] = (i > 0 ? pc.add_component() : 0);
			pc.component_point_range(cis[i]).index_of_first_point = (i == 0 ? 0 : cnts[i - 1]);
			pc.component_point_range(cis[i]).nr_points = cnts[i];
			pc.component_color(cis[i]) = ref_point_selection_colors()[i];
		}
		if (i > 0)
			cnts[i] += cnts[i - 1];
	}
	// compute per selection first points index
	std::vector<Cnt> fsts(4, 0);
	for (i = 1; i < 4; ++i)
		fsts[i] = cnts[i - 1];
	
	// compute component inidices and permutation
	std::vector<Idx> perm(pc.get_nr_points());
	for (i = 0; i < perm.size(); ++i) {
		Idx si = ref_point_selection()[i];
		Cnt new_i = fsts[si]++;
		pc.component_index(new_i) = cis[si];
		perm[i] = new_i;
	}
	pc.permute(perm, false);
}

void selection_tool::config_gui()
{
	if (find_control(select_index)) {
		Cnt max_value = Cnt((type == ST_POINT ? ref_pc().get_nr_points() : ref_pc().get_nr_components()) - 1);
		find_control(select_index)->set<Cnt>("max", max_value);
		find_control(select_range[0])->set<Cnt>("max", max_value);
		find_control(select_range[1])->set<Cnt>("max", max_value);
	}
}
void selection_tool::create_gui()
{
	for (unsigned i = 0; i < ref_point_selection_colors().size(); ++i) {
		add_member_control(this, std::string("C") + cgv::utils::to_string(i), ref_point_selection_colors()[i], "", "w=150", " ");
		add_member_control(this, "show", (bool&)show_selection[i], "toggle", "w=50");
	}

	add_member_control(this, "type", type, "dropdown", "enums='point,component'");
	add_member_control(this, "action", action, "dropdown", "enums='set,add,del'");
	connect_copy(add_button("select")->click, cgv::signal::rebind(this, &selection_tool::perform_action));
	add_member_control(this, "auto_select", auto_perform_action, "toggle");
	add_member_control(this, "auto_mode", auto_select_mode, "toggle");
	add_member_control(this, "mode", mode, "dropdown", "enums='single,range,box'");
	add_member_control(this, "current_selection", current_selection, "value_slider", "min=1;max=3");
	add_member_control(this, "index", select_index, "value_slider", "min=0;ticks=true");
	add_gui("range", select_range, "ascending", "components='<>';min_size=1;options='min=0;ticks=true'");
	add_member_control(this, "show", show_box, "toggle");
	add_gui("select_box", select_box, "", "order_by_coords=true;min_size=0.1;main_label='first';align_col=' ';align_row='%Y-=6\n%Y+=6';align_end='\n';gui_type='slider';options='min=0;max=1;w=60;ticks=true;step=0.001'");
	add_member_control(this, "color", box_color);
	connect_copy(add_button("create components")->click, cgv::signal::rebind(this, &selection_tool::create_components));
	config_gui();
}
