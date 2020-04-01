#include "image_based_normal_estimator.h"

void image_based_normal_estimator::compute_normals_from_index_image()
{
	if (!ref_pc().has_pixel_coordinates())
		return;
	if (!ref_pc().has_normals())
		ref_pc().create_normals();

	Idx cnt = Idx(ref_pc().has_components() ? ref_pc().get_nr_components() : 1);
	for (Idx i = 0; i < Idx(ref_pc().get_nr_components()); ++i) {
		index_image img;
		int ci = ref_pc().has_components() ? i : -1;
		ref_pc().compute_index_image(img, 1, ci);
		cgv::utils::statistics dist_stats;
		ref_pc().compute_image_neighbor_distance_statistic(img, dist_stats, ci);
		int nr_isolated, nr_iterations, nr_left_over;
		ref_pc().estimate_normals(img, float(dist_stats.get_min()) * ref_variable("relative_distance_threshold", 5.0f), ci, &nr_isolated, &nr_iterations, &nr_left_over);
	}
	post_redraw();
}
image_based_normal_estimator::image_based_normal_estimator(point_cloud_viewer_ptr pcv_ptr) : point_cloud_tool(pcv_ptr, "image_normal_estimator")
{

}
void image_based_normal_estimator::create_gui()
{
	add_member_control(this, "relative_distance_threshold", ref_variable("relative_distance_threshold", 5.0f), "value_slider", "min=0;max=20;ticks=true");
	cgv::signal::connect_copy(add_button("compute from index image")->click, cgv::signal::rebind(this, &image_based_normal_estimator::compute_normals_from_index_image));
}
