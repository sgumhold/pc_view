#include "align_tool.h"
#include <cgv/gui/dialog.h>
#include <libs/point_cloud/ann_tree.h>
#include <cgv_gl/gl/gl.h>
#include <cgv_reflect_types/media/axis_aligned_box.h>
#include <cgv_reflect_types/media/color.h>
#include <cgv/utils/file.h>
#include <cgv/utils/dir.h>
#include <cgv/utils/advanced_scan.h>
#include <fstream>

align_tool::align_tool(point_cloud_viewer_ptr pcv_ptr) : point_cloud_tool(pcv_ptr,"align")
{
	source_selection = 1;
	target_selection = 0;
}

void align_tool::center()
{
	ref_pc().translate(-ref_pc().box().get_center());
	post_redraw();
}

void align_tool::align()
{
	point_cloud& pc = ref_pc();
	if (!pc.has_components()) {
		cgv::gui::message("no components available for alignment");
		return;
	}
	std::vector<Idx> source_components;
	std::vector<Idx> target_components;
	for (Cnt ci = 0; ci < pc.get_nr_components(); ++ci) {
		if (ref_component_selection()[ci] == source_selection)
			source_components.push_back(ci);
		else if (ref_component_selection()[ci] == target_selection)
			target_components.push_back(ci);
	}
	if (source_components.empty()) {
		cgv::gui::message("no source components selected for alignment");
		return;
	}
	if (target_components.empty()) {
		cgv::gui::message("no target components selected for alignment");
		return;
	}
	ann_tree KD;
	KD.build(pc, target_components);
	// correspondence search
	for (auto ci : source_components) {
		Idx pi_end = Idx(pc.component_point_range(ci).index_of_first_point + pc.component_point_range(ci).nr_points);
		std::vector<const Pnt*> knn;
		for (Idx pi = Idx(pc.component_point_range(ci).index_of_first_point); pi < pi_end; ++pi) {
			KD.find_closest_points(pc.transformed_pnt(pi), 1, knn);
			// filter by normals
		}
	}
}

bool align_tool::self_reflect(cgv::reflect::reflection_handler& srh)
{
	return
		srh.reflect_member("source_selection", source_selection) &&
		srh.reflect_member("target_selection", target_selection);
}

void align_tool::create_gui()
{
	connect_copy(add_button("center")->click, cgv::signal::rebind(this, &align_tool::center));
	add_member_control(this, "source_selection", source_selection, "value_slider", "min=0;max=3");
	add_member_control(this, "target_selection", target_selection, "value_slider", "min=0;max=3");
	connect_copy(add_button("align")->click, cgv::signal::rebind(this, &align_tool::align));
}
