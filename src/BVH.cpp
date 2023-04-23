#include "BVH.h"

#include <stack>

#include "parallel.h"

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
BVH::BVH(const std::vector<std::shared_ptr<Shape>>& src_shapes, double time0, double time1, pcg32_state& rng, const int parallel_counts)
{
    std::vector<std::shared_ptr<BVH>> bvh_list;
    std::vector<std::vector<std::shared_ptr<Shape>>> shapes_tiles_list;
	const int tile_size = 1;
    int num_tiles = (parallel_counts + tile_size - 1) / tile_size;
    int shapes_size = src_shapes.size();

    for(int i = 0; i < num_tiles; i++)
    {
        int begin = i * (shapes_size / num_tiles + 1);
        int end = min((i + 1) * (shapes_size / num_tiles + 1) - 1, shapes_size - 1);
        std::vector <std::shared_ptr<Shape>> shape_in_tile;
        for(int j  = begin; j <= end; j++)
        {
            shape_in_tile.push_back(src_shapes[j]);
        }
        shapes_tiles_list.push_back(shape_in_tile);
    }

    parallel_for([&](int64_t counts)
    {
        int i0 = counts * tile_size;
        int i1 = min(i0 + tile_size, parallel_counts);
        for(int i = i0; i < i1; i++)
        {
            BVH bvh = BVH(shapes_tiles_list[i], time0, time1, rng);
            bvh_list.push_back(std::make_shared<BVH>(bvh));
        }

    }, num_tiles);

 //   for(int i = 0; i < (int)log2(parallel_counts); i++)
 //   {
 //       ;
 //   }
	for (int i = 0; i < num_tiles; i++)
    {
        bvh_list.push_back(std::make_shared<BVH>(BVH(shapes_tiles_list[i], 0, 0, rng)));
    }
 //   std::vector<std::shared_ptr<BVH>> bvh_list_new;

 //   for (int i = 0; i < 8; i++)
 //   {
 //       bvh_list_new.push_back(std::make_shared<BVH>(BVH(bvh_list[i], bvh_list[16 - 1 - i])));
 //   }
 //   bvh_list.clear();
 //   for(int i = 0; i < 4; i++)
 //   {
 //       bvh_list.push_back(std::make_shared<BVH>(BVH(bvh_list_new[i], bvh_list_new[8 - 1 - i])));
 //   }
 //   bvh_list_new.clear();
 //   for (int i = 0; i < 2; i++)
 //   {
 //       bvh_list_new.push_back(std::make_shared<BVH>(BVH(bvh_list[i], bvh_list[4 - 1 - i])));
 //   }
 //   bvh_list.clear();
    BVH root = *build_bvh_by_list(bvh_list, rng);
    left = root.left;
    right = root.right;
    box = root.box;
}
BVH::BVH(std::shared_ptr<BVH> left, std::shared_ptr<BVH> right)
{
    this->left = left;
    this->right = right;
    box = surrounding_box(left->box, right->box);
}
bool BVH::isHit(Ray& ray, double t_min, double t_max)
{
    if (!box.isHit(ray, t_min, t_max))
        return false;
    if (left_shape) // not nullptr
    {
        bool hit_left = ray.FindIntersection(*left_shape, t_min, t_max);
        bool hit_right;
        //if (ray.Objects == nullptr)
        hit_right = ray.FindIntersection(*right_shape, t_min, t_max);
        //else
        //    hit_right = false;
        return hit_left || hit_right;
    }
    else
    {
        bool hit_left = left->isHit(ray, t_min, t_max);
        bool hit_right = right->isHit(ray, t_min, hit_left ? ray.Distances[0] : t_max);
        return hit_left || hit_right;
    }
}


std::shared_ptr<BVH> BVH::build_bvh_by_list(std::vector<std::shared_ptr<BVH>>& list, pcg32_state& rng)
{
    std::shared_ptr<BVH> left, right;
    std::vector<std::shared_ptr<BVH>> list_half;
    if (list.size() <= 2)
        return std::make_shared<BVH>(BVH(list[0], list[1]));
    for(int i  = 0; i < list.size() / 2; i++)
    {
        list_half.push_back(std::make_shared<BVH>(BVH(list[i], list[list.size() - 1 - i])));
    }
    return build_bvh_by_list(list_half, rng);
};


//std::shared_ptr<BVH> build_bvh_by_stack(std::stack<std::vector<std::shared_ptr<Shape>>>& stack, pcg32_state& rng)
//{
//    std::shared_ptr<BVH> left, right;
//    //std::vector<std::shared_ptr<Shape>> objs = &z
//    std::vector<std::shared_ptr<Shape>> objs = stack.top();
//    stack.pop();
//    if (stack.empty())
//        return std::make_shared<BVH>(BVH(objs, 0, 0, rng));
//    left = build_bvh_by_stack(stack, rng);
//    right = build_bvh_by_stack(stack, rng);
//
//    return 
//
//};
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
