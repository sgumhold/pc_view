#pragma once

#include <cgv/base/group.h>
#include <cgv/gui/event_handler.h>
#include <cgv/gui/trigger.h>
#include <cgv/gui/key_event.h>
#include <cgv/gui/provider.h>
#include <cgv/base/register.h>
#include <libs/point_cloud/gl_point_cloud_drawable.h>
#include <libs/point_cloud/neighbor_graph.h>
#include <libs/point_cloud/normal_estimator.h>

#include "lib_begin.h"

class CGV_API point_cloud_tool;

typedef cgv::data::ref_ptr<point_cloud_tool,true> point_cloud_tool_ptr;

enum SelectType {
	ST_POINT,
	ST_COMPONENT
};

class CGV_API point_cloud_viewer :
	public cgv::base::group,
	public cgv::gui::event_handler,
	public cgv::gui::provider,
	public cgv::base::argument_handler,
	public gl_point_cloud_drawable
{
protected:
	std::vector<point_cloud_tool_ptr> tools;
	std::vector<cgv::gui::shortcut> tool_shortcuts;
	Idx selected_tool;
	void add_tool(point_cloud_tool_ptr t, const cgv::gui::shortcut& sc);

	std::string master_path;
	std::string directory_name;
	std::string file_name;
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
	neighbor_graph ng;
	normal_estimator ne;

	bool show_neighbor_graph;
	unsigned k;
	bool do_symmetrize;

	bool reorient_normals;

	void build_neighbor_graph();
	void clear();
	void draw_edge_color(unsigned int vi, unsigned int j, bool is_symm, bool is_start) const;
	void draw_graph(cgv::render::context& ctx);
	
	void toggle_normal_orientations();
	void compute_normals();
	void recompute_normals();
	void orient_normals();
	void orient_normals_to_view_point();

	void interact_callback(double t, double dt);

	void configure_subsample_controls();
	bool save(const std::string& fn);
	bool open(const std::string& fn);
	bool open_directory(const std::string& dn);
	bool open_and_append(const std::string& fn);
	bool open_or_append(cgv::gui::event& e, const std::string& file_name);

	void auto_set_view();

	
	// selection
	std::vector<cgv::type::uint8_type> point_selection;
	std::vector<cgv::type::uint8_type> component_selection;
	std::vector<Clr> point_selection_colors;
	std::vector<Rgba> component_selection_colors;
	enum ColorModeOverwrite {
		CMO_NONE,
		CMO_POINT_SELECTION,
		CMO_COMPONENT_SELECTION
	} color_mode_overwrite;
	int last_modifier_press;
	point_cloud& ref_point_cloud() { return pc; }
	friend class point_cloud_tool;
public:
	point_cloud_viewer();
	void on_point_cloud_change_callback();
	void on_selection_change_callback(SelectType type);
	void on_nr_points_change_callback();
	void on_nr_components_change_callback();
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