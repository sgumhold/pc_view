#include "index_image_inspector.h"
#include <cgv_gl/gl/gl.h>


index_image_inspector::index_image_inspector(point_cloud_viewer_ptr pcv_ptr) : point_cloud_tool(pcv_ptr, "image_inspector")
{
	show_image = true;
	image_component_index = 0;
	image_type = IIT_NORMAL;
	image_scale = 1;
	component_range[0] = 0; component_range[1] = 10;
}

bool index_image_inspector::self_reflect(cgv::reflect::reflection_handler& srh)
{
	return
		srh.reflect_member("show_image", show_image) &&
		srh.reflect_member("image_scale", image_scale) &&
		srh.reflect_member("image_component_index", image_component_index);
}

void index_image_inspector::draw(cgv::render::context& ctx)
{
	if (!show_image || !ref_pc().has_pixel_coordinates())
		return;

	int ci = -1;
	if (ref_pc().has_components()) {
		ci = image_component_index;
		if (ci >= int(ref_pc().get_nr_components()))
			ci = int(ref_pc().get_nr_components()) - 1;
	}

	index_image img;
	ref_pc().compute_index_image(img, 1, ci);
	std::vector<cgv::media::color<cgv::type::uint8_type, cgv::media::RGB, cgv::media::OPACITY> > clrs;
	unsigned w = image_scale*img.get_width(), h = image_scale*img.get_height();
	std::vector<size_t> Ni;
	cgv::utils::statistics dist_stats;
	ref_pc().compute_image_neighbor_distance_statistic(img, dist_stats, ci);
	float distance_threshold = float(dist_stats.get_min()) * ref_variable("relative_distance_threshold", 5.0f);
	clrs.resize(w*h);
	for (size_t j = 0; j < h; ++j) {
		for (size_t i = 0; i < w; ++i) {
			PixCrd pixcrd = PixCrd(i / image_scale, j / image_scale) + img.get_pixel_range().get_min_pnt();
			Idx pi = img(pixcrd);
			if (pi == -1)
				clrs[j*w + i] = cgv::media::color<cgv::type::uint8_type, cgv::media::RGB, cgv::media::OPACITY>(byte_to_color_component(255), 0, 0, byte_to_color_component(255));
			else {
				switch (image_type) {
				case IIT_X:
				case IIT_Y:
				case IIT_Z: {
					ClrComp v = float_to_color_component((ref_pc().pnt(pi)(int(image_type)) - ref_pc().box(ci).get_min_pnt()(int(image_type))) / ref_pc().box(ci).get_extent()(int(image_type)));
					clrs[j*w + i] = Rgba(v, v, v, 1);
				}
							break;
				case IIT_NORMAL: {
					Dir d = 0.5f*ref_pc().nml(pi) + 0.5f;
					clrs[j*w + i] = Rgba(float_to_color_component(d(0)), float_to_color_component(d(1)), float_to_color_component(d(2)), float_to_color_component(1.0f));
				}
								 break;
				case IIT_COLOR:
					clrs[j*w + i] = ref_pc().clr(pi);
					break;
				case IIT_POINT_INDEX: {
					float v = float(pi - ref_pc().component_point_range(ci).index_of_first_point) / ref_pc().component_point_range(ci).nr_points;
					clrs[j*w + i] = Rgba(v, v, v, 1);
				}
									  break;
				case IIT_NEIGHBOR_COUNT: {
					int cnt = ref_pc().collect_valid_image_neighbors(pi, img, Ni, distance_threshold);
					ClrComp v = float_to_color_component(float(cnt) / 8);
					clrs[j*w + i] = Rgba(v, v, v, 1);
				}
										 break;

				}
			}
		}
	}
	ctx.push_pixel_coords();
	glRasterPos2i(0, h);
	glDrawPixels(w, h, GL_RGBA, GL_UNSIGNED_BYTE, &clrs[0][0]);
	ctx.pop_pixel_coords();
}

void index_image_inspector::on_nr_components_change_callback()
{
	if (find_control(image_component_index))
		find_control(image_component_index)->set("max", ref_pc().has_components() ? ref_pc().get_nr_components() - 1 : 0);
}

void index_image_inspector::create_gui()
{
	add_member_control(this, "show", show_image, "toggle");
	add_member_control(this, "image_component_index", image_component_index, "value_slider", "min=0;ticks=true");
	find_control(image_component_index)->set("max", ref_pc().has_components() ? ref_pc().get_nr_components() - 1 : 0);
	add_member_control(this, "image_type", image_type, "dropdown", "enums='x,y,z,normal,color,point index,neighbor count");
	add_member_control(this, "image_scale", image_scale, "value_slider", "min=1;max=4;ticks=true");
	add_member_control(this, "relative_distance_threshold", ref_variable("relative_distance_threshold", 5.0f), "value_slider", "min=0;max=20;ticks=true");
	add_gui("component_range", component_range, "ascending", "main_label='component_range';components='lu';long_label=true;min_size=2;options='min=0;max=20;ticks=true'");
	add_gui("component_range", (cgv::media::axis_aligned_box<Cnt,1>&) component_range, "", "main_label='component_range';components='lu';long_label=true;min_size=2;options='min=0;max=20;ticks=true'");
}
