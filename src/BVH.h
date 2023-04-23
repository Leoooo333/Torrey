#pragma once
#include "aabb.h"
#include "3rdparty\pcg.h"


class BVH
{
public:
	BVH(){};
    BVH(const std::vector<std::shared_ptr<Shape>>& src_shapes,
        size_t start, size_t end, double time0, double time1, pcg32_state& rng);
    BVH(const std::vector<std::shared_ptr<Shape>>& src_shapes, double time0, double time1, pcg32_state& rng)
        : BVH(src_shapes, 0, src_shapes.size(), time0, time1, rng){};
    BVH(std::shared_ptr<BVH> left, std::shared_ptr<BVH> right);
    BVH(const std::vector<std::shared_ptr<Shape>>& src_shapes, double time0, double time1, pcg32_state& rng, const int parallel_counts);
    AxisAlignedBoundingBox GetAabbOfShapes(std::vector<std::shared_ptr<Shape>>& src_shapes, double time0, double time1);
    std::shared_ptr<BVH> build_bvh_by_list(std::vector<std::shared_ptr<BVH>>& list, pcg32_state& rng);

    bool isHit(Ray& ray, double t_min, double t_max);
public:
    std::shared_ptr<BVH> left;
    std::shared_ptr<BVH> right;
    std::shared_ptr<Shape> left_shape;
    std::shared_ptr<Shape> right_shape;
    AxisAlignedBoundingBox box;
};

inline bool box_compare(const std::shared_ptr<Shape> a, const std::shared_ptr<Shape> b, int axis) {
    AxisAlignedBoundingBox box_a = GetAabbByShape(*a);
    AxisAlignedBoundingBox box_b = GetAabbByShape(*b);

    return box_a.min()[axis] < box_b.min()[axis];
}


inline bool box_x_compare(const std::shared_ptr<Shape> a, const std::shared_ptr<Shape> b) {
    return box_compare(a, b, 0);
}

inline bool box_y_compare(const std::shared_ptr<Shape> a, const std::shared_ptr<Shape> b) {
    return box_compare(a, b, 1);
}

inline bool box_z_compare(const std::shared_ptr<Shape> a, const std::shared_ptr<Shape> b) {
    return box_compare(a, b, 2);
}
