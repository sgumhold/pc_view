#include "transform_tool.h"
#include <cgv/gui/dialog.h>
#include <libs/point_cloud/ann_tree.h>
#include <cgv/render/clipped_view.h>
#include <cgv_gl/gl/gl.h>
#include <cgv_reflect_types/media/axis_aligned_box.h>
#include <cgv_reflect_types/media/color.h>
#include <cgv/math/ftransform.h>
#include <cgv/utils/file.h>
#include <cgv/utils/dir.h>
#include <cgv/utils/advanced_scan.h>
#include <fstream>

transform_tool::transform_tool(point_cloud_viewer_ptr pcv_ptr) : point_cloud_tool(pcv_ptr,"transform")
{
	translation_scale = 1.0f;
	reset();
}

void transform_tool::reset()
{
	scale = 1.0f;
	translate = vec3(0.0f);
	axis = vec3(0.0f, 1.0f, 0.0f);;
	angle = 0.0f;
	update_all_members();
	post_redraw();
}

transform_tool::mat4 transform_tool::get_transformation() const
{
	return
		cgv::math::translate4<float>(translate) *
		cgv::math::rotate4<float>(angle, axis) *
		cgv::math::scale4<float>(scale);
}

void transform_tool::transform()
{
	point_cloud& pc = ref_pc();
	pc.transform(get_transformation());
	viewer_ptr->on_point_cloud_change_callback(PointCloudChangeEvent(PCC_COMPONENTS_RESIZE|PCC_NORMALS|PCC_POINTS));
	reset();
	post_redraw();
}

void transform_tool::draw(cgv::render::context& ctx)
{
	ctx.push_modelview_matrix();
	ctx.mul_modelview_matrix(get_transformation());
}
void transform_tool::finish_draw(cgv::render::context& ctx)
{
	ctx.pop_modelview_matrix();
}

bool transform_tool::self_reflect(cgv::reflect::reflection_handler& srh)
{
	return
		srh.reflect_member("scale", scale) &&
		srh.reflect_member("angle", angle) &&
		srh.reflect_member("axis", axis) &&
		srh.reflect_member("translate", translate);
}

void transform_tool::on_set(void* member_ptr)
{
	if (member_ptr == &translation_scale) {
		for (unsigned ci = 0; ci < 3; ++ci) {
			auto cp = find_control(translate[ci]);
			if (cp) {
				cp->set("min", -translation_scale);
				cp->set("max",  translation_scale);
			}
		}
	}
	cgv::render::clipped_view* clipped_view_ptr = dynamic_cast<cgv::render::clipped_view*>(ref_view_ptr());
	if (clipped_view_ptr) {
		box3 B = ref_pc().box();
		box3 Btrans;
		mat4 T = get_transformation();
		for (unsigned ci = 0; ci < 8; ++ci) {
			vec4 hP = T * vec4(B.get_corner(ci), 1.0f);
			Btrans.add_point(vec3(3, &hP[0]));
		}
		Btrans.add_point(vec3(0.0f));
		clipped_view_ptr->set_scene_extent(Btrans);
	}

	update_member(member_ptr);
	post_redraw();
}

void transform_tool::create_gui()
{
	connect_copy(add_button("reset")->click, cgv::signal::rebind(this, &transform_tool::reset));
	add_member_control(this, "scale", scale, "value_slider", "min=0.01;max=100;ticks=true;log=true");
	add_member_control(this, "angle", angle, "value_slider", "min=-180;max=180;ticks=true");
	add_gui("axis", axis, "direction", "options='min=-1;max=1;ticks=true'");
	add_member_control(this, "translation_scale", translation_scale, "value_slider", "min=0.01;max=100;ticks=true;log=true");
	add_gui("translate", translate, "vector", "options='min=-10;max=10;ticks=true'");
	on_set(&translation_scale);
	connect_copy(add_button("transform")->click, cgv::signal::rebind(this, &transform_tool::transform));
}
