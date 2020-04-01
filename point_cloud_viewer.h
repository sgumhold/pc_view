#pragma once

#include <cgv/base/base.h>
#include <cgv/base/group.h>
#include <libs/point_cloud/ann_tree.h>
#include <cgv/gui/event_handler.h>
#include <cgv/gui/trigger.h>
#include <cgv/gui/key_event.h>
#include <cgv/gui/provider.h>
#include <cgv/base/register.h>
#include <cgv/media/image/image.h>
#include <libs/point_cloud/gl_point_cloud_drawable.h>
#include <libs/point_cloud/neighbor_graph.h>
#include <libs/point_cloud/normal_estimator.h>

#include "lib_begin.h"

class CGV_API point_cloud_tool;

typedef cgv::data::ref_ptr<point_cloud_tool, true> point_cloud_tool_ptr;

enum SelectType {
	ST_POINT,
	ST_COMPONENT
};

enum PointCloudChangeEvent
{
	PCC_NEW_POINT_CLOUD        = 0x0001,
	PCC_POINTS                 = 0x0002,
	PCC_POINTS_RESIZE          = 0x0003,
	PCC_POINTS_MASK            = 0x0003,
							   
	PCC_COMPONENTS_CREATE      = 0x0004,
	PCC_COMPONENTS_RESIZE      = 0x0008,
	PCC_COMPONENTS_DESTRUCT    = 0x000C,
	PCC_COMPONENTS_MASK        = 0x000C,
							   
	PCC_NORMALS_CREATE         = 0x0010,
	PCC_NORMALS                = 0x0020,
	PCC_NORMALS_DESTRUCT       = 0x0030,
	PCC_NORMALS_MASK           = 0x0030,
							   
	PCC_COLORS_CREATE          = 0x0040,
	PCC_COLORS                 = 0x0080,
	PCC_COLORS_DESTRUCT        = 0x00C0,
	PCC_COLORS_MASK            = 0x00C0,
							   
	PCC_TEXCOORDS_CREATE       = 0x0100,
	PCC_TEXCOORDS              = 0x0200,
	PCC_TEXCOORDS_DESTRUCT     = 0x0300,
	PCC_TEXCOORDS_MASK         = 0x0300,
							   
	PCC_PIXCOORDS_CREATE       = 0x0400,
	PCC_PIXCOORDS              = 0x0800,
	PCC_PIXCOORDS_DESTRUCT     = 0x0C00,
	PCC_PIXCOORDS_MASK         = 0x0C00,

	PCC_NEIGHBORGRAPH_CREATE = 0x1000,
	PCC_NEIGHBORGRAPH = 0x2000,
	PCC_NEIGHBORGRAPH_DESTRUCT = 0x3000,
	PCC_NEIGHBORGRAPH_MASK = 0x3000,

	PCC_WEIGHTS                = 0x4000,
	PCC_COMPONENT_TRANSFORMATION_CHANGE = 0x8000
};


class CGV_API point_cloud_viewer :
	public cgv::base::group,
	public cgv::gui::event_handler,
	public cgv::gui::provider,
	public cgv::base::argument_handler,
	public gl_point_cloud_drawable
{
protected:
	float target_max_extent;
	void scale_to_target_extent();
	std::vector<point_cloud_tool_ptr> tools;
	std::vector<cgv::gui::shortcut> tool_shortcuts;
	std::vector<cgv::media::image::image*> tool_icons;
	Idx selected_tool, last_tool_index;
	void add_tool(point_cloud_tool_ptr t, const cgv::gui::shortcut& sc);

	std::string master_path;
	std::string directory_name;
	std::string file_name;
	std::string transformation_file_name;
	std::string data_path;
	bool do_append;

	float relative_distance_threshold;

	std::size_t show_point_start;
	std::size_t show_point_count;
	unsigned interact_point_step;
	double interact_delay;
	cgv::gui::trigger interact_trigger;

	enum InteractionState {
		IS_INTERMEDIATE_FRAME,
		IS_WAIT_INTERACTION_TO_STOP,
		IS_DRAW_FULL_FRAME,
		IS_FULL_FRAME
	} interact_state;


	// processing stuff
	ann_tree* tree_ds;
	neighbor_graph ng;
	normal_estimator ne;

	bool accelerate_picking;
	bool tree_ds_out_of_date;
	bool show_neighbor_graph;
	unsigned k;
	bool do_symmetrize;

	bool reorient_normals;

	void ensure_tree_ds();
	void build_neighbor_graph();
	void clear();
	void draw_edge_color(unsigned int vi, unsigned int j, bool is_symm, bool is_start) const;
	void draw_graph(cgv::render::context& ctx);
	void draw_gui(cgv::render::context& ctx);

	void toggle_normal_orientations();
	void compute_normals();
	void recompute_normals();
	void orient_normals();
	void orient_normals_to_view_point();

	bool get_picked_point(int x, int y, unsigned& index);

	void interact_callback(double t, double dt);

	void configure_subsample_controls();
	bool save(const std::string& fn);
	bool open(const std::string& fn);
	bool open_directory(const std::string& dn);
	bool open_and_append(const std::string& fn);
	bool open_or_append(cgv::gui::event& e, const std::string& file_name);
	void save_to_directory();
	bool save_directory(const std::string& dn, const std::string& extension);
	void auto_set_view();

	
	// selection
	std::vector<cgv::type::uint8_type> point_selection;
	std::vector<cgv::type::uint8_type> component_selection;
	std::vector<RGBA> point_selection_colors;
	std::vector<RGBA> component_selection_colors;
	enum ColorModeOverwrite {
		CMO_NONE,
		CMO_POINT_SELECTION,
		CMO_COMPONENT_SELECTION
	} color_mode_overwrite;
	int last_modifier_press;
	point_cloud& ref_point_cloud() { return pc; }
	neighbor_graph& ref_neighbor_graph() { return ng; }
	normal_estimator& ref_normal_estimator() { return ne; }
	bool am_i_active(point_cloud_tool_ptr tool_ptr) const { return selected_tool == -1 ? false : (tools[selected_tool] == tool_ptr); }
	friend class point_cloud_tool;
public:
	point_cloud_viewer();
	void on_point_cloud_change_callback(PointCloudChangeEvent pcc_event);
	void on_selection_change_callback(SelectType type);
	void on_tool_change_callback(unsigned prev_tool_index);
	std::string get_type_name() const;
	bool self_reflect(cgv::reflect::reflection_handler& srh);
	void stream_stats(std::ostream&);
	bool init(cgv::render::context& ctx);
	void init_frame(cgv::render::context& ctx);
	void draw(cgv::render::context& ctx);
	bool handle(cgv::gui::event& e);
	void handle_args(std::vector<std::string>& args);
	void stream_help(std::ostream& os);
	void on_set(void* member_ptr);
	void create_gui();
};

typedef cgv::data::ref_ptr<point_cloud_viewer, true> point_cloud_viewer_ptr;

#include <cgv/config/lib_end.h>