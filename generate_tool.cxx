#include "generate_tool.h"
#include <cgv/gui/mouse_event.h>
#include <cgv_gl/gl/gl.h>
#include <cgv_reflect_types/media/color.h>

generate_tool::generate_tool(point_cloud_viewer_ptr pcv_ptr) :
	point_cloud_tool(pcv_ptr, "generate"), extent(1, 1, 1)
{
	auto_generate = true;
	generate_mode = GM_DOME;
	samples_per_row = 200;
	nr_rows = 400;
	R = 1.0f;
	one_component_per_row = true;
	append = false;
}

bool generate_tool::self_reflect(cgv::reflect::reflection_handler& srh)
{
	return
		srh.reflect_member("auto_generate", auto_generate) &&
		srh.reflect_member("generate_mode", (int&)generate_mode) &&
		srh.reflect_member("samples_per_row", samples_per_row) &&
		srh.reflect_member("R", R) &&
		srh.reflect_member("nr_rows", nr_rows) &&
		srh.reflect_member("one_component_per_row", one_component_per_row) &&
		srh.reflect_member("append", append);
}

void generate_tool::generate_points()
{
	point_cloud& pc = ref_pc();
	PointCloudChangeEvent pcc_event = PCC_NEW_POINT_CLOUD;
	if (!append) 
		pc.clear();
	if (!pc.has_normals()) {
		pc.create_normals();
		pcc_event = PointCloudChangeEvent(pcc_event | PCC_NORMALS_CREATE);
	}
	if (one_component_per_row && !pc.has_components()) {
		pc.create_components();
		pcc_event = PointCloudChangeEvent(pcc_event | PCC_COMPONENTS_CREATE);
	}
	for (Idx li = 0; li < nr_rows; ++li) {
		if (one_component_per_row)
			pc.add_component();
		float y = (float)li / (nr_rows - 1) - 0.5f;
		for (Idx ci = 0; ci < samples_per_row; ++ci) {
			float x = (float)ci / (samples_per_row - 1) - 0.5f;
			vec3 p, n;
			float r2 = x * x + y * y;
			if (r2 > 0.25f) {
				p = vec3(x, y, 0);
				n = vec3(0, 0, 1);
			}
			else {
				float g = sqrt(R*R - r2);
				float f = g - sqrt(R*R - 0.25f);
				p = vec3(x, y, f);
				n = vec3(x / g, y / g, 1);
			}
			Idx pi = Idx(pc.add_point(extent*p));
			pc.nml(pi) = normalize(n/extent);
		}
	}
	viewer_ptr->on_point_cloud_change_callback(pcc_event);
}

void generate_tool::on_set(void* member_ptr)
{
	if (auto_generate && 
		(member_ptr == &generate_mode) ||
		(member_ptr == &samples_per_row) ||
		(member_ptr == &nr_rows) ||
		(member_ptr == &R) ||
		(member_ptr == &one_component_per_row) ||
		(member_ptr >= &extent && member_ptr >= &extent+1)) {
		generate_points();
	}
	post_redraw();
	update_member(member_ptr);
}

void generate_tool::create_gui()
{
	
	add_member_control(this, "auto_generate", auto_generate, "toggle");
	add_member_control(this, "generate_mode", generate_mode, "dropdown", "enums='dome'");
	add_member_control(this, "samples_per_row", samples_per_row, "value_slider", "min=1;max=1000;log=true;ticks=true");
	add_member_control(this, "nr_rows", nr_rows, "value_slider", "min=1;max=1000;log=true;ticks=true");
	add_member_control(this, "one_component_per_row", one_component_per_row, "check");
	add_member_control(this, "append", append, "check");
	add_member_control(this, "R", R, "value_slider", "min=0.1;max=10;log=true;ticks=true");
	add_gui("extent", extent, "vector", "options='min=0.1;max=10;log=true;ticks=true'");
}

void generate_tool::draw(cgv::render::context& ctx)
{

}


