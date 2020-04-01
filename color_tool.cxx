#include "color_tool.h"
#include <cgv/gui/mouse_event.h>
#include <cgv_gl/gl/gl.h>
#include <cgv_reflect_types/media/color.h>

color_tool::color_tool(point_cloud_viewer_ptr pcv_ptr) : 
	point_cloud_tool(pcv_ptr,"color"), 
	default_color(float_to_color_component(0.3f), float_to_color_component(0.6f), float_to_color_component(0.6f)),
	highlight_color(float_to_color_component(1), float_to_color_component(0), float_to_color_component(0)),
	highlight2_color(float_to_color_component(0), float_to_color_component(0), float_to_color_component(1))
{
	coloring_mode = CM_NBR_GRAPH;
	select_index = 0;
}

bool color_tool::self_reflect(cgv::reflect::reflection_handler& srh)
{
	return
		srh.reflect_member("coloring_mode", (int&)coloring_mode) &&
		srh.reflect_member("select_index", select_index);
}

void color_tool::normalize_weights(std::vector<Crd>& weights) const
{
	Crd sum = 0;
	for (auto w : weights)
		sum += w;
	Crd scale = weights.size() / sum;
	for (auto& wr : weights)
		wr *= scale;
}

void color_tool::compute_colors()
{
	size_t n = ref_pc().get_nr_points();
	for (unsigned i = 0; i < n; ++i) {
		ref_pc().clr(i) = (i == select_index) ? highlight_color : default_color;
	}
	if (select_index >= 0 && ref_ng().size() == ref_pc().get_nr_points()) {
		std::vector<graph_location::Idx>& N = ref_ng()[select_index];
		std::vector<Crd> weights;
		weights.resize(N.size() + 1, 0.0f);
		switch (coloring_mode) {
		case CM_NBR_GRAPH:
			break;
		case CM_WEIGHTS:
			ref_ne().compute_weights(select_index, weights);
			normalize_weights(weights);
			break;
		case CM_BILATERAL_WEIGHTS:
			ref_ne().compute_bilateral_weights(select_index, weights);
			normalize_weights(weights);
			break;
		}
		for (unsigned n = 0; n < N.size(); ++n) {
			ref_pc().clr(N[n]) = (1-weights[n+1])*highlight2_color + weights[n+1]*highlight_color;
		}
	}
}

void color_tool::on_point_cloud_change_callback(PointCloudChangeEvent pcc_event)
{
	if (!am_i_active())
		return;
	
	if ( 
		((pcc_event & PCC_POINTS_MASK) != 0) ||
		((pcc_event & PCC_NORMALS_MASK) != 0) ||
		((pcc_event & PCC_WEIGHTS) != 0) ||
		((pcc_event & PCC_NEIGHBORGRAPH_MASK) != 0)
		) {
		if (!ref_pc().has_colors()) {
			ref_pc().create_colors();
		}
		compute_colors();
		viewer_ptr->on_point_cloud_change_callback(PCC_COLORS);
		post_redraw();
	}
}

void color_tool::on_activation_change_callback(bool gets_active)
{
	if (gets_active) {
		if (!ref_pc().has_colors()) {
			ref_pc().create_colors();
			compute_colors();
			viewer_ptr->on_point_cloud_change_callback(PCC_COLORS);
			post_redraw();
		}
	}
}

void color_tool::on_set(void* member_ptr)
{
	if (member_ptr == &select_index || member_ptr == &coloring_mode) {
		compute_colors();
	}
	post_redraw();
	update_member(member_ptr);
}

void color_tool::config_gui()
{
	if (find_control(select_index)) {
		Cnt max_value = ref_pc().get_nr_points() - 1;
		find_control(select_index)->set<Cnt>("max", max_value);
	}
}

void color_tool::create_gui()
{
	add_member_control(this, "coloring_mode", coloring_mode, "dropdown", "enums='nbr_graph,weights,bilateral'");
	add_member_control(this, "index", select_index, "value_slider", "min=0;ticks=true");
	add_member_control(this, "default_color", default_color);
	add_member_control(this, "highlight_color", highlight_color);
	add_member_control(this, "highlight2_color", highlight2_color);
	config_gui();
}

bool color_tool::handle(cgv::gui::event& e)
{
	if (e.get_kind() == cgv::gui::EID_KEY) {
		cgv::gui::key_event& ke = static_cast<cgv::gui::key_event&>(e);
		if (ke.get_action() == cgv::gui::KA_PRESS || ke.get_action() == cgv::gui::KA_REPEAT) {
			switch (ke.get_key()) {
			case 'C' : 
				if (ke.get_modifiers() == 0) {
					if (++(int&)coloring_mode == CM_END)
						coloring_mode = CM_START;
					on_set(&coloring_mode);
					return true;
				}
				if (ke.get_modifiers() == cgv::gui::EM_SHIFT) {
					if (coloring_mode == CM_START)
						coloring_mode = CM_END;
					--(int&)coloring_mode;
					on_set(&coloring_mode);
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
		if (me.get_modifiers() != cgv::gui::EM_ALT)
			return false;
		unsigned picked_index;
		if (get_picked_point(me.get_x(), me.get_y(), picked_index)) {
			select_index = picked_index;
			on_set(&select_index);
		}
		return false;
	}
	return false;
}

void color_tool::draw(cgv::render::context& ctx)
{

}


