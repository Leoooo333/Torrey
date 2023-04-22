#include "BVH.h"

BVH::BVH(const std::vector<std::shared_ptr<Shape>>& src_shapes, size_t start, size_t end, double time0, double time1, pcg32_state& rng)
{
    auto shapes = src_shapes; // Create a modifiable array of the source scene objects
    Real r = next_pcg32_real<Real>(rng) * 2;
	auto comparator = (r < 2./3.) ? box_x_compare
        : (2./3. <= r && r >= 4./3.) ? box_y_compare
        : box_z_compare;

    size_t object_span = end - start;
    AxisAlignedBoundingBox box_left, box_right;

    if (object_span == 1) {
        left_shape = shapes[start];
        right_shape = shapes[start];
        box_left = GetAabbByShape(*left_shape);
        box_right = GetAabbByShape(*right_shape);
    }
    else if (object_span == 2) {
        if (comparator(shapes[start], shapes[start + 1])) {
            left_shape = shapes[start];
            right_shape = shapes[start + 1];
        }
        else {
            left_shape = shapes[start + 1];
            right_shape = shapes[start];
        }
        box_left = GetAabbByShape(*left_shape);
        box_right = GetAabbByShape(*right_shape);
    }
    else {
        std::sort(shapes.begin() + start, shapes.begin() + end, comparator);

        auto mid = start + object_span / 2;
        left = std::make_shared<BVH>(shapes, start, mid, time0, time1, rng);
        right = std::make_shared<BVH>(shapes, mid, end, time0, time1, rng);

        box_left = left->box;
        box_right = right->box;
    }

    box = surrounding_box(box_left, box_right);
}
bool BVH::isHit(Ray& ray, double t_min, double t_max)
{
    if (!box.isHit(ray, t_min, t_max))
        return false;
    if (left_shape) // not nullptr
    {
        bool hit_left = FindIntersection(ray, *left_shape, t_min, t_max);
        bool hit_right;
        //if (ray.Objects == nullptr)
        hit_right = FindIntersection(ray, *right_shape, t_min, t_max);
        //else
        //    hit_right = false;
        return hit_left || hit_right;
    }
    else
    {
        bool hit_left = left->isHit(ray, t_min, t_max);
        bool hit_right = right->isHit(ray, t_min, hit_left ? ray.t_nearst : t_max);
        return hit_left || hit_right;
    }
}

//AxisAlignedBoundingBox BVH::GetAabbOfShapes(std::vector<std::shared_ptr<Shape>>&src_shapes, double time0, double time1)
//{
//    AxisAlignedBoundingBox temp_box, output_box;
//    bool first_box = true;
//
//    for (const auto& shape : src_shapes) {
//        temp_box = GetAabbByShape(*shape);
//        output_box = first_box ? temp_box : surrounding_box(output_box, temp_box);
//        first_box = false;
//    }   
//
//    return output_box;
//}
