#include "clip_tool.h"
#include <cgv_gl/gl/gl.h>
#include <cgv_reflect_types/media/axis_aligned_box.h>
#include <cgv_reflect_types/media/color.h>

clip_tool::clip_tool(point_cloud_viewer_ptr pcv_ptr) : point_cloud_tool(pcv_ptr,"clip"), clip_box_color(0, 1, 0, 1)
{
	show_clipping = true;
}

void clip_tool::reset_clip_box()
{
	clip_box = Box(Pnt(0, 0, 0), Pnt(1, 1, 1));
	for (unsigned c = 0; c < 3; ++c) {
		update_member(&clip_box.ref_min_pnt()(c));
		update_member(&clip_box.ref_max_pnt()(c));
	}
}

void clip_tool::clip_points()
{
	ref_pc().clip(Box(ref_pc().box().get_min_pnt() + ref_pc().box().get_extent() * clip_box.get_min_pnt(), ref_pc().box().get_min_pnt() + ref_pc().box().get_extent() * clip_box.get_max_pnt()));
	reset_clip_box();
	viewer_ptr->on_nr_points_change_callback();
}

bool clip_tool::self_reflect(cgv::reflect::reflection_handler& srh)
{
	return
		srh.reflect_member("show_clipping", show_clipping) &&
		srh.reflect_member("clip_box_color", clip_box_color) &&
		srh.reflect_member("clip_box", clip_box);
}

void clip_tool::draw(cgv::render::context& ctx)
{
	if (!show_clipping || !clip_box.is_valid())
		return;
	viewer_ptr->draw_box(ctx, Box(ref_pc().box().get_min_pnt() + ref_pc().box().get_extent() * clip_box.get_min_pnt(), ref_pc().box().get_min_pnt() + ref_pc().box().get_extent() * clip_box.get_max_pnt()), clip_box_color);
}

void clip_tool::on_point_cloud_change_callback()
{
	reset_clip_box();
}

void clip_tool::create_gui()
{
	add_member_control(this, "show", show_clipping, "toggle");
	connect_copy(add_button("clip to box")->click, cgv::signal::rebind(this, &clip_tool::clip_points));
	add_gui("clip_box", clip_box, "", "order_by_coords=true;min_size=0.1;main_label='first';align_col=' ';align_row='%Y-=6\n%Y+=6';align_end='\n';gui_type='slider';options='min=0;max=1;w=60;ticks=true;step=0.001'");
	add_gui("color", clip_box_color);
}
