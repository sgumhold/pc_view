#pragma once

#include <cgv/math/fvec.h>
#include <cgv/media/axis_aligned_box.h>
#include <numeric>

static const uint32_t child_mask_to_nr_children[16] = {
	0, 1, 1, 2, 1, 2, 2, 3,
	1, 2, 2, 3, 2, 3, 3, 4
};

struct octree_base
{
	typedef uint32_t node_index_type;
	typedef uint32_t point_index_type;
	typedef void* node_handle;
	static const node_index_type invalid_node_index = node_index_type(-1);
	cgv::box3 domain;
	node_index_type root_node_index = 0;
	std::vector<uint32_t> point_indices;
	void compute_domain(const cgv::vec3* points_ptr, point_index_type nr_points, bool ensure_isotropic = true) {
		domain.invalidate();
		for (point_index_type pi=0; pi < nr_points; ++pi)
			domain.add_point(points_ptr[pi]);
		if (ensure_isotropic) {
			unsigned i = domain.get_max_extent_coord_index(), j = (i + 1) % 3, k = (i + 2) % 3;
			float a = domain.get_extent()[i];
			domain.ref_max_pnt()[j] = domain.ref_min_pnt()[j] + a;
			domain.ref_max_pnt()[k] = domain.ref_min_pnt()[k] + a;
		}
	}

	void distribute_points_on_children(std::vector<uint32_t>(&child_point_indices)[8],
		const cgv::box3& B, std::vector<uint32_t>::iterator begin, std::vector<uint32_t>::iterator end,
		const cgv::vec3* points_ptr)
	{
		cgv::vec3 center = B.get_center();
		for (auto iter = begin; iter != end; ++iter) {
			const cgv::vec3& point = points_ptr[*iter];
			uint8_t child_index =
				(point[0] > center[0] ? 1 : 0) +
				(point[1] > center[1] ? 2 : 0) +
				(point[2] > center[2] ? 4 : 0);
			child_point_indices[child_index].emplace_back(*iter);
		}
	}
	void reorder_indices_to_children(std::vector<uint32_t>::iterator(&child_begin)[9],
		const std::vector<uint32_t>(&child_point_indices)[8], std::vector<uint32_t>::iterator begin, std::vector<uint32_t>::iterator end)
	{
		for (unsigned ci = 0; ci < 8; ++ci) {
			child_begin[ci] = begin;
			std::copy(child_point_indices[ci].begin(), child_point_indices[ci].end(), begin);
			begin += child_point_indices[ci].size();
		}
		child_begin[8] = end;
	}
	cgv::box3 child_box(const cgv::box3& B, unsigned ci) const
	{
		cgv::vec3 center = B.get_center();
		return cgv::box3(
			cgv::vec3(
				(ci & 1) == 0 ? B.get_min_pnt()[0] : center[0],
				(ci & 2) == 0 ? B.get_min_pnt()[1] : center[1],
				(ci & 4) == 0 ? B.get_min_pnt()[2] : center[2]
			),
			cgv::vec3(
				(ci & 1) == 0 ? center[0] : B.get_max_pnt()[0],
				(ci & 2) == 0 ? center[1] : B.get_max_pnt()[1],
				(ci & 4) == 0 ? center[2] : B.get_max_pnt()[2]
			));
	}

	virtual void construct(const cgv::vec3* points_ptr, point_index_type nr_points, uint32_t max_nr_points_per_leaf, bool ensure_isotropic) = 0;
	virtual node_index_type get_nr_nodes() const = 0;
	virtual node_handle create_root_node_handle() const = 0;
	virtual node_handle copy_node_handle(node_handle nh) const = 0;
	virtual void release_node_handle(node_handle nh) const = 0;
	virtual bool node_is_leaf(node_handle nh) const = 0;
	virtual bool node_decent(node_handle nh, unsigned ci) const = 0;
	virtual point_index_type node_nr_points(node_handle nh) const = 0;
	virtual node_index_type node_index(node_handle nh) const = 0;
	virtual point_index_type node_first_point(node_handle nh) const = 0;
};

struct simple_point_octree : public octree_base
{
	struct node
	{
		bool is_leaf;
		point_index_type first_point;
		point_index_type nr_points;
		node_index_type children[8];
	};
	struct node_iterator
	{
		const simple_point_octree* octree_ptr;
		node_index_type node_index;
		cgv::box3 B;
		const node& get_node() const { return octree_ptr->nodes[node_index]; }
		bool is_leaf() const { return get_node().is_leaf; }
		bool decent(unsigned ci) {
			node_index_type child_index = get_node().children[ci];
			if (child_index == invalid_node_index)
				return false;
			node_index = child_index;
			B = octree_ptr->child_box(B, ci);
			return true;
		}
	};
	std::vector<node> nodes;
	void construct_node(node_index_type ni, uint32_t max_nr_points_per_leaf, const cgv::box3& B, std::vector<uint32_t>::iterator begin, std::vector<uint32_t>::iterator end, const cgv::vec3* points_ptr)
	{
		point_index_type nr_points = point_index_type(end - begin);
		auto& N = nodes[ni];
		N.first_point = point_index_type(begin - point_indices.begin());
		N.nr_points = nr_points;
		// check for leaf node construction
		if (N.is_leaf = (nr_points <= max_nr_points_per_leaf))
			return;

		std::vector<uint32_t> child_point_indices[8];
		distribute_points_on_children(child_point_indices, B, begin, end, points_ptr);
		std::vector<uint32_t>::iterator child_begin[9];
		reorder_indices_to_children(child_begin, child_point_indices, begin, end);
		for (unsigned ci = 0; ci < 8; ++ci) {
			if (child_begin[ci] == child_begin[ci+1])
				nodes[ni].children[ci] = invalid_node_index;
			else {
				nodes[ni].children[ci] = node_index_type(nodes.size());
				nodes.emplace_back(node());
				construct_node(nodes[ni].children[ci], max_nr_points_per_leaf, child_box(B, ci), child_begin[ci], child_begin[ci + 1], points_ptr);
			}
		}
	}
	void construct(const cgv::vec3* points_ptr, point_index_type nr_points, uint32_t max_nr_points_per_leaf, bool ensure_isotropic)
	{
		point_indices.resize(nr_points);
		std::iota(point_indices.begin(), point_indices.end(), 0);
		nodes.clear();
		nodes.push_back(node());
		root_node_index = 0;
		compute_domain(points_ptr, nr_points, ensure_isotropic);
		construct_node(root_node_index, max_nr_points_per_leaf, domain, point_indices.begin(), point_indices.end(), points_ptr);
	}
	node_iterator get_root() const { return { this, root_node_index, domain }; }
	node_index_type get_nr_nodes() const { return node_index_type(nodes.size()); }
	node_handle create_root_node_handle() const { return new node_iterator(get_root()); }
	void release_node_handle(node_handle nh) const { delete reinterpret_cast<node_iterator*>(nh); }
	node_handle copy_node_handle(node_handle nh) const { return new node_iterator(*reinterpret_cast<node_iterator*>(nh)); }
	bool node_is_leaf(node_handle nh) const { return reinterpret_cast<node_iterator*>(nh)->is_leaf(); }
	bool node_decent(node_handle nh, unsigned ci) const { return reinterpret_cast<node_iterator*>(nh)->decent(ci); }
	node_index_type node_index(node_handle nh) const { return reinterpret_cast<node_iterator*>(nh)->node_index; }
	point_index_type node_nr_points(node_handle nh) const { return reinterpret_cast<node_iterator*>(nh)->get_node().nr_points; }
	point_index_type node_first_point(node_handle nh) const { return reinterpret_cast<node_iterator*>(nh)->get_node().first_point; }
};
/*
struct point_octree
{
	struct node {
		struct bitfield_type {
			uint8_t  child_mask : 8;
			uint8_t  lod : 6;
		} bitfield;
		struct child_iterator {
			uint8_t child_mask;
			uint8_t child_index;
			uint32_t node_index;
			child_iterator(uint8_t _child_mask, uint8_t _child_index, uint32_t _node_index)
				: child_mask(_child_mask), child_index(_child_index), node_index(_node_index)
			{
				skip_empty_children();
			}
			void skip_empty_children()
			{
				while (child_index < 8 && (child_mask & 1) == 0) {
					child_mask >>= 1;
					++child_index;
					++node_index;
				}
			}
			bool operator == (const child_iterator& ci) const { return child_index == ci.child_index; }
			bool operator < (const child_iterator& ci) const { return child_index < ci.child_index; }
			bool operator != (const child_iterator& ci) const { return child_index != ci.child_index; }
			child_iterator& operator ++() {
				while (child_index < 8) {
					child_mask >>= 1;
					++child_index;
					++node_index;
					if ((child_mask & 1) == 1)
						break;
				}
				return *this;
			}
		};
		uint32_t first_index;
		uint32_t nr_points;
		node(uint8_t lod, uint32_t _first_index, uint32_t _nr_points) : first_index(_first_index), nr_points(_nr_points) {
			bitfield.child_mask = 0;
			bitfield.lod = lod;
		}
		bool is_leaf() const { return bitfield.child_mask == 0; }
		child_iterator begin() const { return child_iterator(bitfield.child_mask, 0, first_index); }
		child_iterator end() const { return child_iterator(0, 8, first_index + 8); }
		uint32_t get_nr_children() const { return child_mask_to_nr_children[bitfield.child_mask & 15] + child_mask_to_nr_children[bitfield.child_mask / 16]; }
		uint32_t get_first_point_index() const { return first_index; }
		uint32_t get_nr_points() const { return nr_points; }
	};
	std::vector<uint32_t> point_indices;
	std::vector<uint_fast64_t> morton_indices;
	std::vector<node> nodes;
	void extract_morton_indices(const std::vector<cgv::vec3>& points, uint32_t max_depth = 12) {
		assert(max_depth < 21);
		uint32_t nr_cells = pow(2u, max_depth);
		cgv::vec3 scale = cgv::vec3(float(nr_cells)) / domain.get_extent();
		morton_indices.clear();
		for (const auto& point : points) {
			cgv::uvec3 xyz(scale * (point - domain.get_min_pnt()));
			morton_indices.emplace_back(morton3D_64_encode(xyz[0], xyz[1], xyz[2]));
		}
	}

	void construct_node(node& N, uint32_t max_nr_points_per_leaf, const cgv::box3& B, std::vector<uint32_t>::iterator begin, std::vector<uint32_t>::iterator end, const std::vector<cgv::vec3>& points)
	{
		// check for leaf node construction
		if (N.get_nr_points() <= max_nr_points_per_leaf)
			return;
		// distribute points on children
		cgv::vec3 center = B.get_center();
		std::vector<uint32_t> child_point_indices[8];
		for (auto iter = begin; iter != end; ++iter) {
			const cgv::vec3& point = points[*iter];
			uint8_t child_index =
				(point[0] > center[0] ? 1 : 0) +
				(point[1] > center[1] ? 2 : 0) +
				(point[2] > center[2] ? 4 : 0);
			child_point_indices[child_index].emplace_back(*iter);
		}
		// determine child mask, allocate nodes and rearrange point indices along child node order
		uint8_t child_mask_bit = 1;
		uint32_t first_point_index = N.get_first_point_index();
		N.first_index = nodes.size();
		auto iter = begin;
		uint32_t ci;
		for (ci = 0; ci < 8; ++ci) {
			uint32_t nr_points = child_point_indices[ci].size();
			if (nr_points > 0) {
				N.bitfield.child_mask |= child_mask_bit;
				nodes.emplace_back(node(N.bitfield.lod + 1, first_point_index, nr_points));
				for (uint32_t i = 0; i < nr_points; ++i, ++iter)
					*iter = child_point_indices[ci][i];
				first_point_index += nr_points;
			}
			child_mask_bit <<= 1;
		}
		// construct child nodes recursively
		uint32_t ni = N.first_index;
		for (ci = 0; ci < 8; ++ci) {
			construct_node(nodes[ni], max_nr_points_per_leaf, child_box[i], , , points);
		}
	}
	void construct(const std::vector<cgv::vec3>& points, uint32_t max_nr_points_per_leaf, bool ensure_isotropic)
	{
		point_indices.resize(points.size());
		std::iota(point_indices.begin(), point_indices.end(), 0);
		nodes.clear();
		nodes.push_back(node(0, 0, point_indices.size()));
		root_node_index = 0;
		construct_node(nodes.front(), max_nr_points_per_leaf, domain, point_indices.begin(), point_indices.end(), points);
	}
};
*/