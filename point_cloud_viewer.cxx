#include "point_cloud_viewer.h"
#include <algorithm>
#include <libs/point_cloud/ann_tree.h>
#include <cgv/base/find_action.h>
#include <cgv/signal/rebind.h>
#include <cgv/base/import.h>
#include <cgv/utils/file.h>
#include <cgv/render/clipped_view.h>
#include <cgv/gui/dialog.h>
#include <cgv/gui/trigger.h>
#include <cgv/gui/mouse_event.h>
#include <cgv/gui/file_dialog.h>
#include <cgv/reflect/reflect_extern.h>
#include <cgv_gl/gl/gl.h>
#include <cgv/utils/tokenizer.h>
#include <libs/cg_gamepad/gamepad_server.h>

#include "align_tool.h"
#include "clip_tool.h"
#include "color_tool.h"
#include "generate_tool.h"
#include "selection_tool.h"
#include "image_based_normal_estimator.h"
#include "index_image_inspector.h"

#define FILE_SAVE_TITLE "Save Point Cloud"
#define FILE_OPEN_TITLE "Open Point Cloud"
#define FILE_APPEND_TITLE "Append Point Cloud"
#define FILE_OPEN_FILTER "Point Clouds (apc,bpc,pct,txt):*.apc;*.bpc;*.pct;*.txt|Mesh Files (obj,ply):*.obj;*.ply|All Files:*.*"
#define TRANSFORMATION_FILE_OPEN_TITLE "Open Transformations"
#define TRANSFORMATION_FILE_OPEN_FILTER "Alignment files (txt,aln):*.txt;*.aln;*.som|All Files:*.*"

static bool as_matrix = false;

enum FileBase
{
	FB_NORMAL_ESTIMATION,
	FB_LEGO,
	FB_HAHN,
	FB_LAST
};

static FileBase file_base = FB_NORMAL_ESTIMATION;

std::string fill_with_zeros(int v, unsigned cnt)
{
	std::string res = cgv::utils::to_string(v);
	return std::string("000000000000000000000000000").substr(0,cnt - res.size())+res;
}
std::string get_file_name(FileBase file_base, int file_idx)
{
	static const char* names[] = {
		 "bunny_34835.bpc"
		,"camel_homog_28926.bpc"
		,"camel_uniform_12000.bpc"
		,"car_uniform_8001.bpc"
		,"car_uniform_noisy_8001.bpc"
		,"creases_uniform_30000.bpc"
		,"cube.bpc"
		,"cylindric_shape_uniform_15281.bpc"
		,"feature_test_uniform_8000.bpc"
		,"frauenkirche_193601.bpc"
		,"gear_plate_homog_57844.bpc"
		,"gear_plate_uniform_40000.bpc"
		,"goldenerreiter.bpc"
		,"hand_noisy_28788.bpc"
		,"hand_uniform_28788.bpc"
		,"klein_sampled_homog_50400.bpc"
		,"moebius_homog_5460.bpc"
		,"moebius_uniform_10000.bpc"
		,"star_uniform_30310.bpc"
		,"star_uniform_noisy_30310.bpc"
	};
	switch (file_base) {
	case FB_NORMAL_ESTIMATION :
		return std::string("S:/data/points/bpc/")+names[file_idx];	
	case FB_LEGO :
		return std::string("S:/data/points/cgv/lego/lego/lego")+fill_with_zeros(file_idx, 5)+"_scan.ply";
	case FB_HAHN :
		return std::string("S:/data/points/cgv/scan_porzellan_hahn_05_03_2014/porzellan_hahn")+fill_with_zeros(file_idx, 5)+"_scan.ply";
	default:
		return "";
	}
}

namespace cgv {
	namespace reflect {
reflection_traits<gl_point_cloud_drawable,RTK_SELF_REFLECT,false> get_reflection_traits(const gl_point_cloud_drawable&) 
{
	return reflection_traits<gl_point_cloud_drawable,RTK_SELF_REFLECT,false>();
}
	}
}

void point_cloud_viewer::add_tool(point_cloud_tool_ptr t, const cgv::gui::shortcut& sc)
{
	tools.push_back(t);
	tool_shortcuts.push_back(sc);
	std::string fn = t->get_icon_file_name();
	cgv::media::image::image* I = 0;
	bool success = false;
	if (!fn.empty()) {
		cgv::media::image::image J;
		if (J.read(fn)) {
			J.hflip();
			I = new cgv::media::image::image();
			I->downsample(2, 2, J);
			success = true;
		}
	}
	if (I && !success) {
		delete I;
		I = 0;
	}
	tool_icons.push_back(I);
	if (get_context())
		get_context()->configure_new_child(t);
}

point_cloud_viewer::point_cloud_viewer() : ne(pc, ng)
{
	target_max_extent = 1.0f;

	accelerate_picking = true;
	tree_ds_out_of_date = true;
	tree_ds = 0;

	set_name("point_cloud_viewer");
	do_append = false;
	last_modifier_press = 0;

	relative_distance_threshold = 5.0f;
	show_nmls = false;
	interact_point_step = 1;
	show_point_count = 0;
	show_point_start = 0;
	interact_delay = 0.15;

	interact_state = IS_INTERMEDIATE_FRAME;

	cgv::signal::connect(interact_trigger.shoot, this, &point_cloud_viewer::interact_callback);
	interact_trigger.schedule_recuring(interact_delay);

	show_neighbor_graph = false;
	k = 30;
	do_symmetrize = false;
	reorient_normals = true;

	use_component_transformations = false;
	color_mode_overwrite = CMO_NONE;

	surfel_style.use_group_transformation = false;

	point_selection_colors.push_back(RGBA(0.5f, 0.5f, 0.5f, 1.0f));
	point_selection_colors.push_back(RGBA(1.0f, 1.0f, 0.5f, 1.0f));
	point_selection_colors.push_back(RGBA(0.0f, 0.0f, 1.0f, 1.0f));
	point_selection_colors.push_back(RGBA(1.0f, 0.0f, 0.0f, 1.0f));

	last_tool_index = -1;
	selected_tool = -1;
	add_tool(new align_tool(this), cgv::gui::shortcut(int('A'), cgv::gui::EM_CTRL));
	add_tool(new selection_tool(this), cgv::gui::shortcut(int('S'), cgv::gui::EM_CTRL));
	add_tool(new clip_tool(this), cgv::gui::shortcut(int('X'), cgv::gui::EM_CTRL));
	add_tool(new generate_tool(this), cgv::gui::shortcut(int('G'), cgv::gui::EM_CTRL));
	add_tool(new color_tool(this), cgv::gui::shortcut(int('C'), cgv::gui::EM_CTRL));
	add_tool(new image_based_normal_estimator(this), cgv::gui::shortcut(int('N'), cgv::gui::EM_CTRL));
	add_tool(new index_image_inspector(this), cgv::gui::shortcut(int('I'), cgv::gui::EM_CTRL));

	surfel_style.point_size = 4.0f;
	surfel_style.halo_color_strength = 0.0f;
	surfel_style.percentual_halo_width = 25.0f;
	surfel_style.blend_points = true;
	surfel_style.blend_width_in_pixel = 1.0f;
	sort_points = true;

	cgv::gui::connect_gamepad_server();
}

void point_cloud_viewer::draw_edge_color(unsigned int vi, unsigned int j, bool is_symm, bool is_start) const
{
	if (is_symm)
		glColor3f(0.5f, 0.5f, 0.5f);
	else {
		if (is_start)
			glColor3f(1, 0.5f, 0.5f);
		else
			glColor3f(1, 1, 0.5f);
	}
}

void point_cloud_viewer::draw_graph(cgv::render::context& ctx)
{
	if (!show_neighbor_graph)
		return;

	glDisable(GL_LIGHTING);
	glColor3f(0.5f, 0.5f, 0.5f);
	glLineWidth(1.0f);
	glBegin(GL_LINES);
	for (unsigned int vi = 0; vi<ng.size(); ++vi) {
		const std::vector<Idx> &Ni = ng[vi];
		for (unsigned int j = 0; j<Ni.size(); ++j) {
			unsigned int vj = Ni[j];
			// check for symmetric case and only draw once
			if (ng.is_directed_edge(vj, vi)) {
				if (vi < vj) {
					draw_edge_color(vi, j, true, true);
					glArrayElement(vi);
					draw_edge_color(vi, j, true, false);
					glArrayElement(vj);
				}
			}
			else {
				draw_edge_color(vi, j, false, true);
				glArrayElement(vi);
				draw_edge_color(vi, j, false, false);
				glArrayElement(vj);
			}
		}
	}
	glEnd();
	glEnable(GL_LIGHTING);
}

void point_cloud_viewer::interact_callback(double t, double dt)
{

	if (interact_state == IS_FULL_FRAME || interact_state == IS_DRAW_FULL_FRAME)
		return;

	if (interact_state == IS_INTERMEDIATE_FRAME)
		interact_state = IS_WAIT_INTERACTION_TO_STOP;
	else {
		interact_state = IS_DRAW_FULL_FRAME;
		post_redraw();
	}

}

bool point_cloud_viewer::init(cgv::render::context& ctx)
{
	if (!gl_point_cloud_drawable::init(ctx))
		return false;

	get_root()->set("bg_index", 4);

	for (auto t:tools)
		ctx.configure_new_child(t);

	return true;
}
void point_cloud_viewer::init_frame(cgv::render::context& ctx)
{
	static bool my_tab_selected = false;
	if (!my_tab_selected) {
		my_tab_selected = true;
		cgv::gui::gui_group_ptr gg = ((provider*)this)->get_parent_group();
		if (gg) {
			cgv::gui::gui_group_ptr tab_group = gg->get_parent()->cast<cgv::gui::gui_group>();
			if (tab_group) {
				cgv::base::base_ptr c = gg;
				tab_group->select_child(c, true);
			}
		}
	}
	gl_point_cloud_drawable::init_frame(ctx);
}

void point_cloud_viewer::clear()
{
	ng.clear();
}
void point_cloud_viewer::ensure_tree_ds()
{
	if (tree_ds_out_of_date) {
		if (tree_ds)
			delete tree_ds;
		tree_ds = new ann_tree;
		tree_ds->build(pc);
		tree_ds_out_of_date = false;
	}
}

void point_cloud_viewer::build_neighbor_graph()
{
	clear();
	ensure_tree_ds();
	cgv::utils::statistics he_stats;
	ng.build(pc.get_nr_points(), k, *tree_ds, &he_stats);
	if (do_symmetrize)
		ng.symmetrize();
	on_point_cloud_change_callback(PCC_NEIGHBORGRAPH_CREATE);

	std::cout << "half edge statistics " << he_stats << std::endl;
	std::cout << "v " << pc.get_nr_points()
		<< ", he = " << ng.nr_half_edges
		<< " ==> " << (float)ng.nr_half_edges / ((unsigned)(pc.get_nr_points())) << " half edges per vertex" << std::endl;
}

void point_cloud_viewer::toggle_normal_orientations()
{
	if (!pc.has_normals())
		return;
	for (Idx i = 0; i < Idx(pc.get_nr_points()); ++i)
		pc.nml(i) = -pc.nml(i);
	on_point_cloud_change_callback(PCC_NORMALS);
	post_redraw();
}

void point_cloud_viewer::compute_normals()
{
	if (ng.empty())
		build_neighbor_graph();
	ne.compute_weighted_normals(reorient_normals && pc.has_normals());
	on_point_cloud_change_callback(PCC_NORMALS);
	post_redraw();
}

void point_cloud_viewer::recompute_normals()
{
	if (ng.empty())
		build_neighbor_graph();
	if (!pc.has_normals())
		compute_normals();
	//	ne.compute_bilateral_weighted_normals(reorient_normals);
	ne.compute_plane_bilateral_weighted_normals(reorient_normals);
	on_point_cloud_change_callback(PCC_NORMALS);
	post_redraw();
}

void point_cloud_viewer::orient_normals()
{
	if (ng.empty())
		build_neighbor_graph();
	if (!pc.has_normals())
		compute_normals();
	ne.orient_normals();
	on_point_cloud_change_callback(PCC_NORMALS);
	post_redraw();
}

/// return the point under the mouse pointer in world coordinates
bool point_cloud_viewer::get_picked_point(int x, int y, unsigned& index)
{
	cgv::math::fvec<double, 3> world_location;
	if (!get_world_location(x, y, *view_ptr, world_location))
		return false;
	//  unproject to world coordinates with smaller (closer to eye) z-value one	
	Pnt p_pick_world = world_location;

	// find closest point
	int i_closest = -1;
	if (accelerate_picking) {
		ensure_tree_ds();
		i_closest = tree_ds->find_closest(p_pick_world);
	}
	else {
		int n = (int)pc.get_nr_points();
		float smallest_sqr_dist = 0;
		for (int i = 0; i < n; ++i) {
			if (i_closest == -1) {
				i_closest = i;
				smallest_sqr_dist = (pc.pnt(i) - p_pick_world).sqr_length();
			}
			else {
				float new_sqr_dist = (pc.pnt(i) - p_pick_world).sqr_length();
				if (new_sqr_dist < smallest_sqr_dist) {
					i_closest = i;
					smallest_sqr_dist = new_sqr_dist;
				}
			}
		}
	}
	if (i_closest == -1)
		return false;
	index = i_closest;
	return true;
}

void point_cloud_viewer::orient_normals_to_view_point()
{
	if (ensure_view_pointer()) {
		if (ng.empty())
			build_neighbor_graph();
		Pnt view_point = view_ptr->get_eye();
		ne.orient_normals(view_point);
		on_point_cloud_change_callback(PCC_NORMALS);
		post_redraw();
	}
}

void point_cloud_viewer::draw_gui(cgv::render::context& ctx)
{
	ctx.push_pixel_coords();
	unsigned h = ctx.get_height();
	unsigned x = 2;
	for (unsigned ti = 0; ti < tools.size(); ++ti) {
		if (tool_icons[ti]) {
			glRasterPos2i(x, ti == selected_tool ? h-12 : h - 2);
			glDrawPixels(tool_icons[ti]->get_width(), tool_icons[ti]->get_height(), GL_RGB, GL_UNSIGNED_BYTE, tool_icons[ti]->get_ptr<char>());
			x += tool_icons[ti]->get_width() + 2;
		}
	}
	ctx.pop_pixel_coords();
}

void point_cloud_viewer::draw(cgv::render::context& ctx)
{
	draw_gui(ctx);

	if (pc.get_nr_points() == 0)
		return;

	glVertexPointer(3, GL_FLOAT, 0, &(pc.pnt(0).x()));
	glEnableClientState(GL_VERTEX_ARRAY);
	draw_graph(ctx);
	glDisableClientState(GL_VERTEX_ARRAY);


	if (interact_state != IS_DRAW_FULL_FRAME)
		std::swap(show_point_step, interact_point_step);

	gl_point_cloud_drawable::draw(ctx);

	if (interact_state != IS_DRAW_FULL_FRAME) {
		std::swap(show_point_step, interact_point_step);
		interact_state = IS_INTERMEDIATE_FRAME;
	}
	else
		interact_state = IS_FULL_FRAME;

	if (selected_tool != -1)
		tools[selected_tool]->draw(ctx);
}

bool point_cloud_viewer::save(const std::string& fn)
{
	if (!write(fn)) {
		cgv::gui::message(last_error);
		return false;
	}
	return true;
}
bool point_cloud_viewer::open(const std::string& fn)
{
	std::string file_path = fn;
	if (!read(file_path, &data_path)) {
		cgv::gui::message(last_error);
		return false;
	}
	file_name = file_path;
	update_member(&file_name);
	on_point_cloud_change_callback(PCC_NEW_POINT_CLOUD);
	return true;
}
void point_cloud_viewer::on_tool_change_callback(unsigned prev_tool_index)
{
	if (prev_tool_index != -1)
		tools[prev_tool_index]->on_activation_change_callback(false);
	if (selected_tool != -1)
		tools[selected_tool]->on_activation_change_callback(true);

	post_recreate_gui();
}

void point_cloud_viewer::on_point_cloud_change_callback(PointCloudChangeEvent pcc_event)
{
	if (use_component_transformations != pc.has_component_transformations()) {
		use_component_transformations = pc.has_component_transformations();
		on_set(&use_component_transformations);
	}
	if (use_component_colors != pc.has_component_colors()) {
		use_component_colors = pc.has_component_colors();
		on_set(&use_component_colors);
	}
	if ((pcc_event & PCC_COMPONENT_TRANSFORMATION_CHANGE) != 0) {
		pc.box_out_of_date = true;
		for (Idx ci = 0; ci < (Idx)pc.get_nr_components(); ++ci) {
			for (unsigned i = 0; i < 3; ++i) {
				update_member(&pc.component_translation(ci)(i));
				update_member(&pc.component_rotation(ci)(i));
			}
			update_member(&pc.component_rotation(ci)(3));
		}
	}
	if (!view_ptr)
		view_ptr = find_view_as_node();
	cgv::render::clipped_view* clipped_view_ptr = dynamic_cast<cgv::render::clipped_view*>(view_ptr);
	if (clipped_view_ptr) {
		box3 B = pc.box();
		clipped_view_ptr->set_scene_extent(B);
	}
	if (!pc.has_normals() != surfel_style.illumination_mode == cgv::render::IM_OFF) {
		if (pc.has_normals())
			surfel_style.illumination_mode = cgv::render::IM_ONE_SIDED;
		else
			surfel_style.illumination_mode = cgv::render::IM_OFF;
		update_member(&surfel_style.illumination_mode);
	}
	if (((pcc_event & PCC_POINTS_MASK) == PCC_POINTS_RESIZE) || ((pcc_event & PCC_POINTS_MASK) == PCC_NEW_POINT_CLOUD)) {
		tree_ds_out_of_date = true;
		if (tree_ds) {
			delete tree_ds;
			tree_ds = 0;
		}
		ng.clear();
		show_point_end = pc.get_nr_points();
		show_point_begin = 0;

		point_selection.resize(pc.get_nr_points());

		update_member(&show_point_begin);
		update_member(&show_point_end);

		show_point_count = pc.get_nr_points();
		update_member(&show_point_count);

		interact_point_step = std::max((unsigned)(show_point_count / 1000000), 1u);
		nr_draw_calls = std::max((unsigned)(show_point_count / 1000000), 1u);
		update_member(&interact_point_step);
		update_member(&nr_draw_calls);

		configure_subsample_controls();

	}

	if ( (pcc_event & PCC_COMPONENTS_MASK) != 0) {
		component_selection.resize(pc.get_nr_components());
		component_selection_colors.resize(pc.get_nr_components());
	}
	for (auto t : tools)
		t->on_point_cloud_change_callback(pcc_event);

	post_redraw();
}

void point_cloud_viewer::on_selection_change_callback(SelectType type)
{
	if (type == ST_POINT)
		color_mode_overwrite = CMO_POINT_SELECTION;
	else {
		if (pc.has_components()) {
			color_mode_overwrite = CMO_COMPONENT_SELECTION;
			for (Cnt ci = 0; ci < component_selection_colors.size(); ++ci)
				component_selection_colors[ci] = point_selection_colors[component_selection[ci]];
		}
	}
	on_set(&color_mode_overwrite);
}


void point_cloud_viewer::configure_subsample_controls()
{
	if (find_control(show_point_begin)) {
		find_control(show_point_begin)->set("max", show_point_end);
		find_control(show_point_end)->set("min", show_point_begin);
		find_control(show_point_end)->set("max", pc.get_nr_points());
		find_control(show_point_count)->set("max", pc.get_nr_points());
		find_control(show_point_start)->set("max", pc.get_nr_points() - show_point_count);
	}
}


bool point_cloud_viewer::open_and_append(const std::string& _file_name)
{
	std::string fn = _file_name;
	if (!append(fn, pc.get_nr_points() > 0, &data_path)) {
		cgv::gui::message(last_error);
		return false;
	}
	on_point_cloud_change_callback(PointCloudChangeEvent(PCC_POINTS_RESIZE + PCC_COMPONENTS_RESIZE));
	return true;
}

std::string point_cloud_viewer::get_type_name() const
{
	return "point_cloud_viewer";
}

bool point_cloud_viewer::self_reflect(cgv::reflect::reflection_handler& srh)
{
	for (auto t : tools)
		if (!t->self_reflect(srh))
			return false;
	if (srh.reflect_member("do_append", do_append) &&
		srh.reflect_member("data_path", data_path) &&
		srh.reflect_member("file_name", file_name) &&
		srh.reflect_member("directory_name", directory_name) &&
		srh.reflect_member("transformation_file_name", transformation_file_name) &&
		srh.reflect_member("interact_point_step", interact_point_step) &&
		srh.reflect_member("surfel_style", surfel_style) &&
		srh.reflect_member("normal_style", normal_style) &&
		srh.reflect_member("box_style", box_style) &&
		srh.reflect_member("box_wire_style", box_wire_style) &&
		srh.reflect_member("show_points", show_points) &&
		srh.reflect_member("show_nmls", show_nmls) &&
		srh.reflect_member("show_boxes", show_boxes) &&
		srh.reflect_member("show_box", show_box) &&
		srh.reflect_member("sort_points", sort_points) &&
		srh.reflect_member("show_neighbor_graph", show_neighbor_graph) &&
		srh.reflect_member("k", k) &&
		srh.reflect_member("do_symmetrize", do_symmetrize) &&
		srh.reflect_member("reorient_normals", reorient_normals) &&
		srh.reflect_member("master_path", master_path))
		return true;
	return false;
}

void point_cloud_viewer::save_to_directory()
{
	std::string dir = cgv::gui::directory_save_dialog("save to directory");
	if (dir.empty())
		return;
	int answer = cgv::gui::question("Choose file extension:", "bpc,ply,obj");
	if (answer == -1)
		return;
	static const char* extensions[] = { "bpc", "ply", "obj" };
	save_directory(dir, extensions[answer]);
}

bool point_cloud_viewer::save_directory(const std::string& dn, const std::string& extension)
{
	for (unsigned ci = 0; ci < pc.get_nr_components(); ++ci) {
		point_cloud tmp_pc;
		const component_info& C = pc.component_point_range(ci);
		size_t ib = C.index_of_first_point;
		size_t ie = ib + C.nr_points;
		tmp_pc.resize(C.nr_points);
		std::copy(&pc.pnt(ib), &pc.pnt(ie), &tmp_pc.pnt(0));
		if (pc.has_colors()) {
			tmp_pc.create_colors();
			std::copy(&pc.clr(ib), &pc.clr(ie), &tmp_pc.clr(0));
		}
		if (pc.has_normals()) {
			tmp_pc.create_normals();
			std::copy(&pc.nml(ib), &pc.nml(ie), &tmp_pc.nml(0));
		}
		if (pc.has_texture_coordinates()) {
			tmp_pc.create_texture_coordinates();
			std::copy(&pc.texcrd(ib), &pc.texcrd(ie), &tmp_pc.texcrd(0));
		}
		std::string file_name = dn + "/" + pc.component_name(ci) + "." + extension;
		if (!tmp_pc.write(file_name))
			return false;
	}
	return true;
}


bool point_cloud_viewer::open_directory(const std::string& dn)
{
	std::vector<std::string> file_names;
	void* handle = cgv::utils::file::find_first(directory_name + "/*.*");
	while (handle) {
		if (!cgv::utils::file::find_directory(handle))
			file_names.push_back(cgv::utils::file::find_name(handle));
		handle = cgv::utils::file::find_next(handle);
	}
	if (file_names.empty()) {
		std::cerr << "did not find files in directory <" << dn << ">" << std::endl;
		return false;
	}
	unsigned i;
	for (i = 0; i < file_names.size(); ++i) {
		std::cout << i << "(" << file_names.size() << "): " << file_names[i];
		std::cout.flush();
		if (!do_append && i == 0)
			open(directory_name + "/" + file_names[i]);
		else
			open_and_append(directory_name + "/" + file_names[i]);
		std::cout << std::endl;
	}
	pc.component_name(Idx(pc.get_nr_components() - file_names.size())) = 
		cgv::utils::file::drop_extension(cgv::utils::file::get_file_name(file_names.front()));
	for (i = 0; i < pc.get_nr_components(); ++i) {
		pc.component_color(i) = cgv::media::color<float, cgv::media::HLS, cgv::media::OPACITY>(float(i) / float(pc.get_nr_components() - 1), 0.5f, 1.0f, 1.0f);
	}
	use_component_colors = true;
	update_member(&use_component_colors);
	return true;
}
void point_cloud_viewer::on_set(void* member_ptr)
{
	if (member_ptr == &color_mode_overwrite) {
		switch (color_mode_overwrite) {
		case CMO_NONE:
			use_these_point_colors = 0;
			use_these_component_colors = 0;
			use_these_point_color_indices = 0;
			use_these_point_palette = 0;
			break;
		case CMO_POINT_SELECTION:
			use_these_point_color_indices = &point_selection;
			use_these_point_palette = &point_selection_colors;
			use_these_component_colors = 0;
			break;
		case CMO_COMPONENT_SELECTION:
			use_these_component_colors = &component_selection_colors;
			use_these_point_color_indices = 0;
			use_these_point_palette = 0;
			use_these_point_colors = 0;
			break;
		}
		post_redraw();
	}
	if (member_ptr == &ne.localization_scale || member_ptr == &ne.normal_sigma || member_ptr == &ne.bw_type || member_ptr == &ne.plane_distance_scale) {
		on_point_cloud_change_callback(PCC_WEIGHTS);
	}
	if (member_ptr == &selected_tool) {
		on_tool_change_callback(last_tool_index);
		last_tool_index = selected_tool;
	}
	if (member_ptr == &file_name) {
		std::cout << "file_name set to " << file_name << std::endl;
		if (!do_append)
			open(file_name);
		else
			open_and_append(file_name);
	}
	if (member_ptr == &directory_name) {
		open_directory(directory_name);
	}
	if (member_ptr == &transformation_file_name) {
		if (ref_tree_node_visible_flag(transformation_file_name)) {
			pc.write_component_transformations(transformation_file_name, as_matrix);
		}
		else {
			if (pc.read_component_transformations(transformation_file_name))
				on_point_cloud_change_callback(PointCloudChangeEvent(PCC_COMPONENT_TRANSFORMATION_CHANGE));
		}
	}
	if (member_ptr == &show_point_start) {
		show_point_begin = show_point_start;
		show_point_end = show_point_start + show_point_count;
		update_member(&show_point_begin);
		update_member(&show_point_end);
		configure_subsample_controls();
	}
	if (member_ptr == &show_point_count) {
		if (show_point_start + show_point_count > pc.get_nr_points()) {
			show_point_start = pc.get_nr_points() - show_point_count;
			show_point_begin = show_point_start;
			update_member(&show_point_begin);
			update_member(&show_point_start);
		}
		show_point_end = show_point_start + show_point_count;
		update_member(&show_point_end);
		configure_subsample_controls();
	}
	if (member_ptr == &show_point_begin) {
		show_point_start = show_point_begin;
		show_point_count = show_point_end - show_point_begin;
		update_member(&show_point_start);
		update_member(&show_point_count);
		configure_subsample_controls();
	}
	if (member_ptr == &show_point_end) {
		show_point_count = show_point_end - show_point_begin;
		update_member(&show_point_count);
		configure_subsample_controls();
	}
	if (member_ptr == &interact_delay) {
		interact_trigger.stop();
		interact_trigger.schedule_recuring(interact_delay);
	}
	if (member_ptr == &use_component_colors) {
		surfel_style.use_group_color = use_component_colors;
		update_member(&surfel_style.use_group_color);
		box_style.use_group_color = use_component_colors;
		update_member(&box_style.use_group_color);
		box_wire_style.use_group_color = use_component_colors;
		update_member(&box_wire_style.use_group_color);
	}
	if (member_ptr == &use_component_transformations) {
		surfel_style.use_group_transformation = use_component_transformations;
		update_member(&surfel_style.use_group_transformation);
		box_style.use_group_transformation = use_component_transformations;
		update_member(&box_style.use_group_transformation);
		box_wire_style.use_group_transformation = use_component_transformations;
		update_member(&box_wire_style.use_group_transformation);
		on_point_cloud_change_callback(PCC_COMPONENT_TRANSFORMATION_CHANGE);
	}
	if (pc.has_components() && pc.has_component_transformations()) {
		if ((member_ptr >= &pc.component_translation(0) && member_ptr < &pc.component_translation(0) + pc.get_nr_components()) ||
			(member_ptr >= &pc.component_rotation(0) && member_ptr < &pc.component_rotation(0) + pc.get_nr_components()))
			on_point_cloud_change_callback(PCC_COMPONENT_TRANSFORMATION_CHANGE);
	}
	update_member(member_ptr);
	post_redraw();
}

#include <cgv/gui/animate.h>

void point_cloud_viewer::auto_set_view()
{
	if (pc.get_nr_points() == 0)
		return;

	std::vector<cgv::render::view*> view_ptrs;
	cgv::base::find_interface<cgv::render::view>(get_node(), view_ptrs);
	if (view_ptrs.empty()) {
		cgv::gui::message("could not find a view to adjust!!");
		return;
	}
//	view_ptrs[0]->set_view_dir(0,0,-1);
//	view_ptrs[0]->set_focus(pc.box().get_center());
//	view_ptrs[0]->set_y_extent_at_focus(1.5f*pc.box().get_extent()(1));
	cgv::gui::animate_with_rotation(view_ptrs[0]->ref_view_up_dir(), dvec3(0,1,0), 0.5)->set_base_ptr(this);
	cgv::gui::animate_with_geometric_blend(view_ptrs[0]->ref_y_extent_at_focus(), 1.5*pc.box().get_extent()(1), 0.5)->set_base_ptr(this);
	cgv::gui::animate_with_linear_blend(view_ptrs[0]->ref_focus(), dvec3(pc.box().get_center()), 0.5)->set_base_ptr(this);
	post_redraw();
}

bool point_cloud_viewer::open_or_append(cgv::gui::event& e, const std::string& file_name)
{
	cgv::utils::tokenizer T(file_name);
	T.set_ws("\n").set_sep("");
	std::vector<cgv::utils::token> toks;
	T.bite_all(toks);
	bool res = false;
	for (unsigned i=0; i<toks.size(); ++i) {
		std::string file_path = cgv::base::find_data_file(to_string(toks[i]), "CM", "", master_path);
		if (e.get_modifiers() == cgv::gui::EM_ALT || i > 0)
			res |= open_and_append(file_path);
		else
			res |= open(file_path);
	}
	return res;
}


bool point_cloud_viewer::handle(cgv::gui::event& e)
{
	if (selected_tool != -1)
		if (tools[selected_tool]->handle(e))
			return true;

	if (e.get_kind() == cgv::gui::EID_KEY) {
		cgv::gui::key_event& ke = static_cast<cgv::gui::key_event&>(e);
		if (ke.get_action() == cgv::gui::KA_RELEASE) {
			if (ke.get_key() == last_modifier_press) {
				switch (ke.get_key()) {
				case  cgv::gui::KEY_Left_Alt:
				case cgv::gui::KEY_Right_Alt:
					if (color_mode_overwrite == CMO_POINT_SELECTION)
						color_mode_overwrite = CMO_NONE;
					else
						color_mode_overwrite = CMO_POINT_SELECTION;
					on_set(&color_mode_overwrite);
					break;
				case cgv::gui::KEY_Left_Ctrl:
				case cgv::gui::KEY_Right_Ctrl:
					if (pc.has_components()) {
						if (color_mode_overwrite == CMO_COMPONENT_SELECTION)
							color_mode_overwrite = CMO_NONE;
						else
							color_mode_overwrite = CMO_COMPONENT_SELECTION;
						on_set(&color_mode_overwrite);
					}
					break;
				}
			}
		}
		last_modifier_press = 0;
		if (ke.get_action() == cgv::gui::KA_PRESS || ke.get_action() == cgv::gui::KA_REPEAT) {
			// first check for pressed modifiers
			switch (ke.get_key()) {
			case cgv::gui::KEY_Left_Alt:
			case cgv::gui::KEY_Right_Alt:
			case cgv::gui::KEY_Left_Ctrl:
			case cgv::gui::KEY_Right_Ctrl:
				last_modifier_press = ke.get_key();
				break;
			default :
				last_modifier_press = 0;
			}
			// first check for tool shortcuts
			for (unsigned ti = 0; ti < tool_shortcuts.size(); ++ti) {
				if (tool_shortcuts[ti].get_key() == ke.get_key() && tool_shortcuts[ti].get_modifiers() == ke.get_modifiers()) {
					selected_tool = ti;
					on_set(&selected_tool);
					return true;
				}
			}
			switch (ke.get_key()) {
			case '0' :
			case '1' :
			case '2' :
			case '3' :
			case '4' :
			case '5' :
			case '6' :
			case '7' :
			case '8' :
			case '9' :
				open_or_append(ke, get_file_name(file_base, (int)(ke.get_key()-'0') + ((ke.get_modifiers()&cgv::gui::EM_CTRL)?10:0)));
				return true;
			case 'Q' :
				if (file_base == FB_NORMAL_ESTIMATION)
					file_base = (FileBase)(FB_LAST-1);
				else
					--(int&)file_base;
				update_member(&file_base);
//				open_or_append(ke, "hahn_00_01_02_03_04_11_12_13_14_15.bpc");
				return true;
			case 'W' :
				++(int&)file_base;
				if (file_base == FB_LAST)
					file_base = FB_NORMAL_ESTIMATION;
				update_member(&file_base);
//				open_or_append(ke, "hahn_05_06_07_08_09_10.bpc");
				return true;
			case '=' :
			case '+' :
				surfel_style.point_size += 1;
				on_set(&surfel_style.point_size); 
				return true;
			case '-' :
				if (surfel_style.point_size > 0) {
					surfel_style.point_size -= 1;
					if (surfel_style.point_size < 1)
						surfel_style.point_size = 1;
					on_set(&surfel_style.point_size); 
				}
				return true;
			case 'P' :
				show_points = !show_points;
				on_set(&show_points); 
				return true;
			case 'C' :
				if (ke.get_modifiers() == 0) {
					if (surfel_style.map_color_to_material == cgv::render::CM_COLOR)
						surfel_style.map_color_to_material = cgv::render::CM_NONE;
					else
						++(int&)surfel_style.map_color_to_material;
					on_set(&surfel_style.map_color_to_material);
				}
				else if (ke.get_modifiers() == cgv::gui::EM_SHIFT) {
					if (surfel_style.culling_mode == cgv::render::CM_FRONTFACE)
						surfel_style.culling_mode = cgv::render::CM_OFF;
					else
						++(int&)surfel_style.culling_mode;
					on_set(&surfel_style.culling_mode);
				}
				return true;
			case 'B' :
				if (ke.get_modifiers() == 0) {
					surfel_style.blend_points = !surfel_style.blend_points;
					on_set(&surfel_style.blend_points);
				}
				else if (ke.get_modifiers() == cgv::gui::EM_SHIFT) {
					show_boxes = !show_boxes;
					on_set(&show_boxes);
				}
				return true;
			case 'N' :
				if (ke.get_modifiers() == 0) {
					show_nmls = !show_nmls;
					on_set(&show_nmls);
				}
				else if(ke.get_modifiers() == cgv::gui::EM_CTRL+ cgv::gui::EM_ALT) {
					if (!pc.has_normals()) {
						pc.create_normals();
						compute_normals();
					}
					else 
						recompute_normals();

					on_point_cloud_change_callback(PCC_NORMALS);
					post_redraw();
				}
				return true;
			case 'I' :
				if (surfel_style.illumination_mode == cgv::render::IM_TWO_SIDED)
					surfel_style.illumination_mode = cgv::render::IM_OFF;
				else
					++(int&)surfel_style.illumination_mode;
				on_set(&surfel_style.illumination_mode);
				return true;
			case 'G' :
				show_neighbor_graph = !show_neighbor_graph;
				on_set(&show_neighbor_graph); 
				return true;
			case 'O' :
				if (ke.get_modifiers() == 0) {
					surfel_style.orient_splats = !surfel_style.orient_splats;
					on_set(&surfel_style.orient_splats);
				}
				else if (ke.get_modifiers() == cgv::gui::EM_SHIFT) {
					orient_normals();
					on_point_cloud_change_callback(PCC_NORMALS);
					post_redraw();
					return true;
				}
				else if (ke.get_modifiers() == cgv::gui::EM_ALT) {
					orient_normals_to_view_point();
					on_point_cloud_change_callback(PCC_NORMALS);
					post_redraw();
					return true;
				}
				else if (ke.get_modifiers() == cgv::gui::EM_CTRL) {
					std::string fn = cgv::gui::file_open_dialog(FILE_OPEN_TITLE, FILE_OPEN_FILTER);
					if (!fn.empty())
						open(fn);
					return true;
				}
				else if (ke.get_modifiers() == cgv::gui::EM_CTRL + cgv::gui::EM_SHIFT) {
					std::string fn = cgv::gui::file_open_dialog("Open Component Transformations", "Text Files (txt):*.txt|All Files:*.*");
					if (!fn.empty()) {
						if (!pc.read_component_transformations(fn)) {
							std::cerr << "error reading component transformation file " << fn << std::endl;
						}
						post_redraw();
					}
					return true;
				}
				return false;
			case 'A' :
				if (ke.get_modifiers() == cgv::gui::EM_CTRL) {
					std::string fn = cgv::gui::file_open_dialog(FILE_APPEND_TITLE, FILE_OPEN_FILTER);
					if (!fn.empty())
						open_and_append(fn);
					return true;
				}
				return false;
			case 'S' :
				if (ke.get_modifiers() == cgv::gui::EM_CTRL + cgv::gui::EM_SHIFT) {
					std::string fn = cgv::gui::file_save_dialog(FILE_SAVE_TITLE, FILE_OPEN_FILTER);
					if (!fn.empty())
						save(fn);
					return true;
				}
				else if (ke.get_modifiers() == cgv::gui::EM_ALT) {
					sort_points = !sort_points;
					on_set(&sort_points);
					return true;
				}
				return false;
			case cgv::gui::KEY_Space :
				if (pc.get_nr_points() == 0)
					return false;
				auto_set_view();
				return true;
			}
		}
	}
	else {
		last_modifier_press = 0;
		if (e.get_kind() == cgv::gui::EID_MOUSE) {
			if (e.get_flags() == cgv::gui::EF_DND) {
				cgv::gui::mouse_event& me = static_cast<cgv::gui::mouse_event&>(e);
				if (me.get_action() == cgv::gui::MA_RELEASE) {
					open_or_append(e, me.get_dnd_text());
				}
				return true;
			}
		}
	}
	return false;
}

void point_cloud_viewer::handle_args(std::vector<std::string>& args)
{
	for (unsigned ai = 0; ai < args.size(); ++ai) {
		if (cgv::utils::file::exists(args[ai])) {
			if (open_and_append(args[ai])) {
				args.erase(args.begin() + ai);
				--ai;
			}
		}
	}
}


void point_cloud_viewer::stream_help(std::ostream& os)
{
	os << "PCV: open (Ctrl-O), append (Ctrl-A), toggle <p>oints, <n>ormals, <b>ox, <g>graph, <i>llum" << std::endl;
	if (selected_tool != -1) {
		tools[selected_tool]->stream_help(os);
	}
}

void point_cloud_viewer::stream_stats(std::ostream& os)
{
	os << "PCV: #P=" << pc.get_nr_points() 
		<< ", #N=" << (pc.has_normals()?pc.get_nr_points():0) 
		<< ", #C=" << (pc.has_colors() ? pc.get_nr_points() : 0)
		<< ", B=" << pc.box().get_center() << "<" << pc.box().get_extent() << ">" << std::endl;
}

void point_cloud_viewer::scale_to_target_extent()
{
	float max_extent = pc.box().get_extent()(pc.box().get_max_extent_coord_index());
	float scale = target_max_extent / max_extent;
	point_cloud::AMat M;
	M.zeros();
	M(0, 0) = M(1, 1) = M(2, 2) = scale;
	pc.transform(M);
	if (pc.has_component_transformations()) {
		for (unsigned ci = 0; ci < pc.get_nr_components(); ++ci)
			pc.component_translation(ci) *= scale;
	}
	on_point_cloud_change_callback(PointCloudChangeEvent(PCC_COMPONENT_TRANSFORMATION_CHANGE + PCC_POINTS));
}

void point_cloud_viewer::create_gui()
{
	add_decorator("Point Cloud Viewer", "heading", "level=2");
	add_member_control(this, "target_max_extent", target_max_extent, "value_slider", "min=0.1;max=1000;log=true;ticks=true'");
	connect_copy(add_button("scale to target extent")->click, cgv::signal::rebind(this, &point_cloud_viewer::scale_to_target_extent));
	add_member_control(this, "color_mode_overwrite", color_mode_overwrite, "dropdown", "enums='none,point selection,component selection'");
	if (begin_tree_node("IO", file_name, true, "level=3")) {
		align("\a");
		add_control("file_base", file_base, "dropdown", "enums='normal_estimation,lego,hahn'");
		add_gui("file_name", file_name, "file_name", "title='" FILE_OPEN_TITLE "';filter='" FILE_OPEN_FILTER "'");
		add_member_control(this, "do_append", do_append, "toggle");
		connect_copy(add_button("save to directory")->click, cgv::signal::rebind(this, &point_cloud_viewer::save_to_directory));
		add_gui("transformation_file_name", transformation_file_name, "file_name",
			"w=150;open=true;save=true;title='" TRANSFORMATION_FILE_OPEN_TITLE "';filter='" TRANSFORMATION_FILE_OPEN_FILTER "'");
		add_member_control(this, "as_matrix", as_matrix, "toggle");
		align("\b");
		end_tree_node(file_name);
	}

	std::string enum_def = "none=-1";
	for (auto t : tools)
		enum_def += std::string(",") + t->get_name();
	add_member_control(this, "selected_tool", (cgv::type::DummyEnum&)selected_tool, "dropdown", std::string("enums='")+enum_def+"'");
	if (selected_tool != -1) {
		if (begin_tree_node(tools[selected_tool]->get_name(), *tools[selected_tool], true, "level=3")) {
			align("\a");
			inline_object_gui(tools[selected_tool]);
			align("\b");
			end_tree_node(*tools[selected_tool]);
		}
	}
	bool show = begin_tree_node("points", show_points, false, "level=3;w=100;align=' '");
	add_member_control(this, "show", show_points, "toggle", "w=50");
	if (show) {
		align("\a");
		add_member_control(this, "accelerate_picking", accelerate_picking, "check");
		if (begin_tree_node("subsample", show_point_step, false, "level=3")) {
			align("\a");
			add_member_control(this, "nr_draw_calls", nr_draw_calls, "value_slider", "min=1;max=100;log=true;ticks=true");
			add_member_control(this, "interact step", interact_point_step, "value_slider", "min=1;max=100;log=true;ticks=true");
			add_member_control(this, "interact delay", interact_delay, "value_slider", "min=0.01;max=1;log=true;ticks=true");
			add_member_control(this, "show step", show_point_step, "value_slider", "min=1;max=20;log=true;ticks=true");
			add_decorator("range control", "heading", "level=3");
			add_member_control(this, "begin", show_point_begin, "value_slider", "min=0;max=10;ticks=true");
			add_member_control(this, "end", show_point_end, "value_slider", "min=0;max=10;ticks=true");
			add_decorator("window control", "heading", "level=3");
			add_member_control(this, "start", show_point_start, "value_slider", "min=0;max=10;ticks=true");
			add_member_control(this, "width", show_point_count, "value_slider", "min=0;max=10;ticks=true");
			configure_subsample_controls();
			align("\b");
			end_tree_node(show_point_step);
		}
		align("\b");
		add_member_control(this, "sort_points", sort_points, "check");
		add_gui("surfel_style", surfel_style);
		end_tree_node(show_points);
	}
	show = begin_tree_node("components", pc.components, false, "level=3;w=100;align=' '");
	add_member_control(this, "show", surfel_style.use_group_color, "toggle", "w=50");
	if (show) {
		align("\a");
		if (begin_tree_node("component colors", pc.component_colors, false)) {
			align("\a");
			for (unsigned i = 0; i < pc.component_colors.size(); ++i) {
				add_member_control(this,
					pc.component_name(i).empty() ? std::string("C") + cgv::utils::to_string(i) : pc.component_name(i),
					pc.component_colors[i]);
			}
			align("\b");
			end_tree_node(pc.component_colors);
		}
		if (begin_tree_node("group transformations", pc.component_translations, false)) {
			align("\a");
			for (unsigned i = 0; i < pc.component_translations.size(); ++i) {
				add_decorator(pc.component_name(i).empty() ? std::string("C") + cgv::utils::to_string(i) : pc.component_name(i), "heading", "level=3");
				add_decorator("translation", "heading", "level=4");
				add_gui(std::string("T") + cgv::utils::to_string(i), pc.component_translations[i]);
				add_decorator("rotation", "heading", "level=4");
				add_gui(std::string("Q") + cgv::utils::to_string(i), (HVec&)pc.component_rotations[i], "direction");
			}
			align("\b");
			end_tree_node(pc.component_translations);
		}
		align("\b");
		end_tree_node(pc.components);
	}
	show = begin_tree_node("neighbor graph", show_neighbor_graph, false, "level=3;w=150;align=' '");
	add_member_control(this, "show", show_neighbor_graph, "toggle", "w=50");
	if (show) {
		add_member_control(this, "k", k, "value_slider", "min=3;max=50;log=true;ticks=true");
		add_member_control(this, "symmetrize", do_symmetrize, "toggle");
		cgv::signal::connect_copy(add_button("build")->click, cgv::signal::rebind(this, &point_cloud_viewer::build_neighbor_graph));
		end_tree_node(show_neighbor_graph);
	}

	show = begin_tree_node("normals", show_nmls, false, "level=3;w=100;align=' '");
	add_member_control(this, "show", show_nmls, "toggle", "w=50");
	if (show) {
		cgv::signal::connect_copy(add_button("toggle orientation")->click, cgv::signal::rebind(this, &point_cloud_viewer::toggle_normal_orientations));
		add_member_control(this, "bilateral weight approach", ne.bw_type, "dropdown", "enums='normal,plane'");
		add_member_control(this, "localization_scale", ne.localization_scale, "value_slider", "min=0.1;max=2;log=true;ticks=true");
		add_member_control(this, "normal_sigma", ne.normal_sigma, "value_slider", "min=0.01;max=2;log=true;ticks=true");
		add_member_control(this, "plane_distance_scale", ne.plane_distance_scale, "value_slider", "min=0.01;max=2;log=true;ticks=true");
		add_member_control(this, "reorient", reorient_normals, "toggle");
		cgv::signal::connect_copy(add_button("compute")->click, cgv::signal::rebind       (this, &point_cloud_viewer::compute_normals));
		cgv::signal::connect_copy(add_button("recompute")->click, cgv::signal::rebind     (this, &point_cloud_viewer::recompute_normals));
		cgv::signal::connect_copy(add_button("orient")->click, cgv::signal::rebind        (this, &point_cloud_viewer::orient_normals));
		cgv::signal::connect_copy(add_button("orient to view")->click, cgv::signal::rebind(this, &point_cloud_viewer::orient_normals_to_view_point));
		add_gui("normal_style", normal_style);
		end_tree_node(show_nmls);
	}

/*	show = begin_tree_node("surfrec", show_surfrec, false, "level=3;w=100;align=' '");
	add_member_control(this, "show", show_surfrec, "toggle", "w=50");
	if (show) {
		add_member_control(this, "debug_mode", SR.debug_mode, "enums='none,symmetrize,make_consistent,cycle_filter'");
		end_tree_node(show_surfrec);
	}
	*/
	show = begin_tree_node("box", show_box, false, "level=3;w=100;align=' '");
	add_member_control(this, "show", show_box, "toggle", "w=50");
	if (show) {
		add_member_control(this, "show", show_boxes, "toggle", "w=50");
		add_gui("color", box_color);
		add_gui("box_style", box_style);
		add_gui("box_wire_style", box_wire_style);
		end_tree_node(show_box);
	}
}

#include <cgv/base/register.h>

/// register a newly created cube with the name "cube1" as constructor argument
extern cgv::base::object_registration<point_cloud_viewer> point_cloud_viewer_reg("");

#ifdef CGV_FORCE_STATIC
extern cgv::base::registration_order_definition dro("stereo_view_interactor;point_cloud_viewer");
#endif


