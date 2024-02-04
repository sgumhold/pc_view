#include "plane_tool.h"
#include <cgv_gl/gl/gl.h>
#include <cgv_reflect_types/media/axis_aligned_box.h>
#include <cgv_reflect_types/media/color.h>
#include <random>

plane_tool::plane_tool(point_cloud_viewer_ptr pcv_ptr) : point_cloud_tool(pcv_ptr,"plane")
{
	srs.radius = 0.03f;
	ars.head_length_relative_to_radius = 2.8f;
	ars.head_length_relative_to_length = 1.0f;
	ars.head_radius_scale = 2.5f;
	ars.radius_relative_to_length = 0.01f;
	if (colorize_points)
		init_colors();
}
void plane_tool::ensure_colors()
{
	auto& pc = ref_pc();
	if (!pc.has_colors()) {
		pc.create_colors();
		viewer_ptr->on_point_cloud_change_callback(PCC_COLORS_CREATE);
	}
}
void plane_tool::compute_planes()
{
	reset_planes();
	auto& pc = ref_pc();
	std::default_random_engine RE;
	float eps = inlier_distance*pc.box().get_extent().length();
	std::vector<unsigned> I(pc.get_nr_points(), 0);
	for (unsigned i = 0; i < pc.get_nr_points(); ++i)
		I[i] = i;
	std::vector<unsigned> end_idx;
	unsigned nr_used_points = 0;
	while (end_idx.size() < max_nr_planes) {
		std::uniform_int_distribution<int> D(nr_used_points, pc.get_nr_points() - 1);
		size_t iteration_count = size_t(ceil(log(probability_threshold) / log(1.0 - inlier_percentage * inlier_percentage * inlier_percentage)));
		unsigned nr_remaining_points = pc.get_nr_points() - nr_used_points;
		size_t min_nr_target_points = std::max(unsigned(nr_remaining_points * inlier_percentage), min_nr_points);
		std::cout << "plane " << plane_centers.size() + 1 << ": iteration_count = " << iteration_count << " on " << nr_remaining_points << " points, min_nr_points = " << min_nr_target_points;
		std::cout.flush();
		Pnt p0_max, nml_max;
		size_t max_nr_inliers = 0;
		// find plane with largest number of inliers
		for (size_t i = 0; i < iteration_count; ++i) {
			// randomly choose three points to construct plane
			Pnt p0, p1, p2, nml;
			float len;
			do {
				int i0 = D(RE), i1, i2;
				do {
					i1 = D(RE);
				} while (i1 == i0);
				do {
					i2 = D(RE);
				} while (i2 == i0 || i2 == i1);
				p0 = pc.pnt(I[i0]);
				p1 = pc.pnt(I[i1]);
				p2 = pc.pnt(I[i2]);

				nml = cross(p1 - p0, p2 - p0);
				len = nml.length();
			} while (len < eps);
			// count number of inliers
			nml /= len;
			p0 = 0.33333333333f * (p0 + p1 + p2);
			size_t nr_inliers = 0;
			for (unsigned pi = nr_used_points; pi < pc.get_nr_points(); ++pi)
				if (fabs(dot(pc.pnt(I[pi]) - p0, nml)) < eps)
					++nr_inliers;
			if (nr_inliers > max_nr_inliers) {
				max_nr_inliers = nr_inliers;
				nml_max = nml;
				p0_max = p0;
				if (early_termination && max_nr_inliers >= min_nr_target_points) {
					std::cout << ", early termination = " << i;
					break;
				}
			}
		}
		//std::cout << "max inliers = " << max_nr_inliers << " vs " << pc.get_nr_points() * inlier_percentage << std::endl;
		//std::cout << "nml_max = " << nml_max << std::endl;
		//std::cout << "p0_max = " << p0_max << std::endl;
		std::cout << ", max_nr_inliers = " << max_nr_inliers << std::endl;
		if (max_nr_inliers >= min_nr_target_points) {
			for (unsigned pi = nr_used_points; pi < pc.get_nr_points(); ++pi) {
				if (fabs(dot(pc.pnt(I[pi]) - p0_max, nml_max)) < eps) {
					if (pi > nr_used_points)
						std::swap(I[pi], I[nr_used_points]);
					++nr_used_points;
				}
			}
			end_idx.push_back(nr_used_points);
			plane_centers.push_back(p0_max);
			plane_normals.push_back(nml_max);
		}
		else
			break;
	}
	if (plane_centers.empty())
		return;
	// first create plane colors
	double cs_step = 1.0/double(plane_centers.size());
	for (size_t pli = 0; pli < plane_centers.size(); ++pli)
		plane_colors.push_back(cgv::media::color_scale(cs_step * pli, this->color_scale));
	// next colorize points
	if (colorize_points) {
		for (size_t pi = 0; pi < plane_centers.size(); ++pi) {
			unsigned i = 0;
			if (pi > 0)
				i = end_idx[pi - 1];
			for (; i < end_idx[pi]; ++i)
				pc.clr(I[i]) = plane_colors[pi];
		}
		viewer_ptr->on_point_cloud_change_callback(PCC_COLORS);
	}
	post_redraw();
}
void plane_tool::reset_planes()
{
	plane_centers.clear();
	plane_normals.clear();
	plane_colors.clear();
	if (colorize_points)
		init_colors();
	post_redraw();
}
void plane_tool::init_colors()
{
	if (colorize_points) {
		ensure_colors();
		auto& pc = ref_pc();
		for (size_t pi = 0; pi < pc.get_nr_points(); ++pi)
			pc.clr(pi) = default_point_color;
	}
}

bool plane_tool::self_reflect(cgv::reflect::reflection_handler& srh)
{
	return
		srh.reflect_member("show_planes", show_planes);
}

bool plane_tool::init(cgv::render::context& ctx)
{
	cgv::render::ref_sphere_renderer(ctx, 1);
	cgv::render::ref_arrow_renderer(ctx, 1);
	return true;
}
void plane_tool::clear(cgv::render::context& ctx)
{
	cgv::render::ref_sphere_renderer(ctx, -1);
	cgv::render::ref_arrow_renderer(ctx, -1);
}
void plane_tool::draw(cgv::render::context& ctx)
{
	if (plane_centers.empty())
		return;
	auto& sr = cgv::render::ref_sphere_renderer(ctx);
	sr.set_render_style(srs);
	sr.set_position_array(ctx, plane_centers);
	sr.set_color_array(ctx, plane_colors);
	sr.render(ctx, 0, plane_centers.size());
	auto& ar = cgv::render::ref_arrow_renderer(ctx);
	ar.set_render_style(ars);
	ar.set_position_array(ctx, plane_centers);
	ar.set_direction_array(ctx, plane_normals);
	ar.set_color_array(ctx, plane_colors);
	ar.render(ctx, 0, plane_centers.size());

	//if (!show_clipping || !clip_box.is_valid())
	//	return;
	//viewer_ptr->draw_box(ctx, Box(ref_pc().box().get_min_pnt() + ref_pc().box().get_extent() * clip_box.get_min_pnt(), ref_pc().box().get_min_pnt() + ref_pc().box().get_extent() * clip_box.get_max_pnt()), clip_box_color);
}

void plane_tool::on_point_cloud_change_callback(PointCloudChangeEvent pcc_event)
{
	if ( (pcc_event&PCC_POINTS_MASK) != 0)
		if (colorize_points)
			init_colors();
}

void plane_tool::create_gui()
{
	add_member_control(this, "Show Planes", show_planes, "toggle");
	add_member_control(this, "Colorize Points", colorize_points, "toggle");
	add_member_control(this, "Max Nr Planes", max_nr_planes, "value_slider", "min=1;max=1000;log=true;ticks=true");
	add_member_control(this, "Min Nr Points", min_nr_points, "value_slider", "min=1;max=1000;log=true;ticks=true");
	add_member_control(this, "Inlier Distance", inlier_distance, "value_slider", "min=.0000001;max=1;step=0.000000001;log=true;ticks=true");
	add_member_control(this, "Early Termination", early_termination, "toggle");
	add_member_control(this, "Inlier Percentage", inlier_percentage, "value_slider", "min=0.001;max=1;step=0.0000001;log=true;ticks=true");
	add_member_control(this, "Probability Threshold", probability_threshold, "value_slider", "min=0.001;max=1;step=0.0000001;log=true;ticks=true");
	connect_copy(add_button("Find Planes")->click, cgv::signal::rebind(this, &plane_tool::compute_planes));
	connect_copy(add_button("Reset Planes")->click, cgv::signal::rebind(this, &plane_tool::reset_planes));
	add_member_control(this, "Default Point Color", default_point_color);
	if (begin_tree_node("Sphere Style", srs)) {
		align("\a");
		add_gui("Style", srs);
		align("\b");
		end_tree_node(srs);
	}
	if (begin_tree_node("Arrow Style", ars)) {
		align("\a");
		add_gui("Style", ars);
		align("\b");
		end_tree_node(ars);
	}
}
