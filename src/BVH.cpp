#include "BVH.h"

#include <stack>

#include "parallel.h"

BVH::BVH(const std::vector<std::shared_ptr<Shape>>& src_shapes, size_t start, size_t end, double time0, double time1, pcg32_state& rng)
{
    auto shapes = src_shapes; // Create a modifiable array of the source scene objects
    Real r = next_pcg32_real<Real>(rng) * 2;

	//auto comparator = (r < 2./3.) ? box_x_compare
 //       : (2./3. <= r && r < 4./3.) ? box_y_compare
 //       : box_z_compare;
    AxisAlignedBoundingBox box_total = GetAabbOfShapes(src_shapes, time0, time1);
    Vector3 length_3d = box_total.max() - box_total.min();
    Real extent = 0.;
    int max_extent_axis = 0;
    for(int i = 0; i < 3; i++)
    {
        if (length_3d[i] > extent)
        {
	        extent = length_3d[i];
            max_extent_axis = i;
        }
    }
    auto comparator = (max_extent_axis == 0) ? box_x_compare
        : (max_extent_axis == 1) ? box_y_compare
        : box_z_compare;

    size_t object_span = end - start;
    AxisAlignedBoundingBox box_left, box_right;

	if (object_span == 1) {
        left_shape = shapes[start];
        right_shape = shapes[start];
        box_left = GetAabbByShape(*left_shape, time1 - time0);
        box_right = box_left;
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
        box_left = GetAabbByShape(*left_shape, time1 - time0);
        box_right = GetAabbByShape(*right_shape, time1 - time0);

    }
    else {
        //std::sort(shapes.begin() + start, shapes.begin() + end, comparator);
        auto mid = start + object_span / 2;
        std::nth_element(shapes.begin() + start, shapes.begin() + mid, shapes.begin() + end, comparator);

        std::vector<std::shared_ptr<Shape>> shapes_left;
        for (int i = start; i < mid; i++)
            shapes_left.push_back(shapes[i]);
        left = std::make_shared<BVH>(shapes_left, start, shapes_left.size(), time0, time1, rng);

        std::vector<std::shared_ptr<Shape>> shapes_right;
        for (int i = mid; i < end; i++)
            shapes_right.push_back(shapes[i]);
        //right = std::make_shared<BVH>(shapes, mid, end, time0, time1, rng);
        right = std::make_shared<BVH>(shapes_right, start, shapes_right.size(), time0, time1, rng);


        box_left = left->box;
        box_right = right->box;
    }
    box = surrounding_box(box_left, box_right);
}
BVH::BVH(const std::vector<std::shared_ptr<Shape>>& src_shapes, double time0, double time1, pcg32_state& rng, const int parallel_counts, const int tile_size_bvh)
{
    std::vector<std::shared_ptr<BVH>> bvh_list;
    std::vector<std::vector<std::shared_ptr<Shape>>> shapes_tiles_list;
	const int tile_size = tile_size_bvh;
    int num_tiles = (parallel_counts + tile_size - 1) / tile_size;
    int shapes_size = src_shapes.size();

    auto shapes = src_shapes;
    shapes_tiles_list.push_back(shapes);

    for(int i = 0; i < log2(num_tiles); i++)
    {
        std::vector<std::vector<std::shared_ptr<Shape>>> shapes_tiles_list_cache;
        for(int j = 0; j < shapes_tiles_list.size(); j++)
        {
            Real r = next_pcg32_real<Real>(rng) * 2;
            auto comparator = (r < 2. / 3.) ? box_x_compare
                : (2. / 3. <= r && r < 4. / 3.) ? box_y_compare
                : box_z_compare;
            std::vector <std::shared_ptr<Shape>>& shape_in_tile = shapes_tiles_list[j];
            auto mid = 0 + shape_in_tile.size() / 2;
            std::nth_element(shape_in_tile.begin(), shape_in_tile.begin() + mid, shape_in_tile.begin() + shape_in_tile.size(), comparator);

            std::vector <std::shared_ptr<Shape>> left_sub, right_sub;
            for(int k = 0; k < mid; k++)
                left_sub.push_back(shape_in_tile[k]);
            for (int k = mid; k < shape_in_tile.size(); k++)
                right_sub.push_back(shape_in_tile[k]);

            shapes_tiles_list_cache.push_back(left_sub);
            shapes_tiles_list_cache.push_back(right_sub);
        }
        shapes_tiles_list.clear();
        shapes_tiles_list = shapes_tiles_list_cache;
    }

    parallel_for([&](int64_t counts)
    {
        int i0 = counts * tile_size;
        int i1 = min(i0 + tile_size, parallel_counts);
        for(int i = i0; i < i1; i++)
        {
            BVH bvh(shapes_tiles_list[i], time0, time1, rng);
            bvh_list.push_back(std::make_shared<BVH>(bvh));
        }

    }, num_tiles);

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
        if(hit_left)
        {
            t_max = ray.distance;
        }
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
        bool hit_right = right->isHit(ray, t_min, hit_left ? ray.distance: t_max);
        return hit_left || hit_right;
    }
    //return true;
}


std::shared_ptr<BVH> BVH::build_bvh_by_list(std::vector<std::shared_ptr<BVH>>& list, pcg32_state& rng)
{
    std::vector<std::shared_ptr<BVH>> list_half;
    if (list.size() <= 2)
        return std::make_shared<BVH>(BVH(list[0], list[1]));
    for(int i  = 0; i < list.size()-1; i+=2)
    {
        list_half.push_back(std::make_shared<BVH>(BVH(list[i], list[i + 1])));
    }
    return build_bvh_by_list(list_half, rng);
};




AxisAlignedBoundingBox BVH::GetAabbOfShapes(const std::vector<std::shared_ptr<Shape>>& src_shapes, double time0, double time1)
{
    AxisAlignedBoundingBox temp_box, output_box;
    bool first_box = true;

    for (const auto& shape : src_shapes) {
        temp_box = GetAabbByShape(*shape);
        output_box = first_box ? temp_box : surrounding_box(output_box, temp_box);
        first_box = false;
    }

    return output_box;
}