[TOC]



# My design 2_6

### Models from Poly Haven
Here I use `Jacaranda Tree` , `rubber duck toy`, `stone fire pit` and `horse` from [Poly Haven](https://polyhaven.com/models).

### Layout using Blender

Layout these objects in Blender and export them as `.obj`.

### Adapt the scene within .XML

We then using a `.xml`file to modify the scene as we want.

### Run this command and find it as hw_2_6.exr

`
  -hw 2_6 ../scenes/design/design_hw2.xml
`

That's a image describe a horse standing on a ball under a tree. There is also a transparent cute duck, who is watching at the tree. You can see the reflect lake.

# Bonus 2_7
### Method 1: parallel BVH construction
In the construction function, firstly seperate our shapes (objects) into `parallel_counts` of segments. And then use `parallel_for` to parallely construct the sub BVH and push back them in `bvh_list`.

First, split the shapes list into `parallel_counts` pieces by random choose a axis.

```cpp
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
```

Then parallel each leaf nodes BVH's construction with in `parallel_for`.

```cpp
BVH::BVH(const std::vector<std::shared_ptr<Shape>>& src_shapes, double time0, double time1, pcg32_state& rng, const int parallel_counts, const int tile_size_bvh)
{
    ...
    ...
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
```

 After all the sub nodes were finished, built the binary tree based on the nodes recursively. There I bind every 2 nodes in the list and output a merged list(with half the length).

```cpp
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
```

### Method 2: nth_element

Since we will sort the objects recursively, the exactly order within the sub node doesn't matters. ` std::nth_element` here could fix the middle object, and separate the smaller half to the left, the bigger half to the right.

```    cpp
	//std::sort(shapes.begin() + start, shapes.begin() + end, comparator);
    auto mid = start + object_span / 2;
    std::nth_element(shapes.begin() + start, shapes.begin() + mid, shapes.begin() + end, comparator);
```

### Comparison (w/o threads limit)

Based on the ''party'', I did some experiments. See the table below: Here we get **5.04x** speedup (72.6 s / 14.4 s)

| *party*         | Parallel BVH construction                  | w/o parallel         |
| --------------- | ------------------------------------------ | -------------------- |
| **nth_element** | 5.5 (construct) + 18.2 (render) = 23.7 s 🚀 | 15.3 + 21.5 = 36.8 s |
| **sort**        | 7.6 (construct) + 17.7 (render) = 25.3 s   | 52.6 + 20 = 72.6 s 🐢 |

And when I choose `largest extent axis` here, I get the best performance within **14.4 s**  🚀 🚀 🚀.

```cpp
 Scene parsing done. Took 1.10206 seconds.
Build BVH Time: 6.19123
 100.00 Percent Done (1200 / 1200)Total Time:   14.4212
```

### Comparison (2 threads only)

Based on the ''party''(2 threads only), See the table below: Here we get **3.68x** speedup (130.3 s / 35.4 s)

| *party*         | Parallel BVH construction                  | w/o parallel          |
| --------------- | ------------------------------------------ | --------------------- |
| **nth_element** | 9.8 (construct) + 46.1 (render) = 55.9 s 🚀 | 14.5 + 65.7 = 80.2 s  |
| **sort**        | 19.2 (construct) + 56.2 (render) = 75.4 s  | 51.0 + 79.3= 130.3 s🐢 |

And when I choose `largest extent axis` here, I get the best performance within **35.4 s**  🚀 🚀 🚀.

```cpp
Scene parsing done. Took 1.0286 seconds.
Build BVH Time: 10.5056
 100.00 Percent Done (1200 / 1200)Total Time:  35.4294
```



### Run

Run this command and render the lovely bunny as hw_2_7.exr with in only a couple of seconds. 
`
  -hw 2_7 ../scenes/party/party.xml
`

Make `    vars.parallel_counts_bvh = 1;` in `hw2_7() hw2.cpp` to run without parallel. My default is `vars.parallel_counts_bvh = 16`.



# Bonus 2_8: Motion blur

### Motion object
In  structure `Ray`, add a attribute named`time` , which indicate the current time of ray to intersect with that's time's object. 

Assume all the objects are moving in the same direction with same speed, we can sample get the sphere ray intersection as below, where I simply move the center of the sphere. And we can transform the vertex of a triangle to implement the motion process as well.
```cpp
	bool FindIntersection(ParsedSphere& sphere, Real t_min, Real t_max)
	{
		Vector3 center = { sphere.position.x, sphere.position.y, sphere.position.z };
		if (time != 0) // no motion
			center += time * Vector3{ 0., 0.05, 0. };
		return FindIntersection(sphere, translate(center), t_min, t_max);
	}
```

```cpp
    bool FindIntersection(Triangle& triangle, Matrix4x4 transform, Real t_min, Real t_max)
    {
        Vector3 orig = Origin;
        if (time != 0) // no motion
            orig += -time * Vector3{ 0., 0.05, 0. };
```

### Motion box

In construction of BVH, I also make it move so that we can get more accurate result.

```cpp

    if (object_span == 1) {
        left_shape = shapes[start];
        right_shape = shapes[start];
        box_left = GetAabbByShape(*left_shape, time1 - time0);
        box_right = GetAabbByShape(*right_shape, time1 - time0);
    }

```

A motion box is just the surrounding box of both box in time_0 and time_1:

```cpp
AxisAlignedBoundingBox::AxisAlignedBoundingBox(const Vector3& min, const Vector3& max, const Real time)
    : AxisAlignedBoundingBox(min, max)
{
    Vector3 motion = { 0., 0.05, 0. };
    motion *= time;
    AxisAlignedBoundingBox box_after = AxisAlignedBoundingBox(min + motion, max + motion);
    AxisAlignedBoundingBox result = surrounding_box(box_after, *this);
    minimum = result.minimum;
    maximum = result.maximum;
}
```



### Run

Here I use my design for hw_1, (which is scene_9 for hw1_scenes), which is my signature of group of glass ball. Run this command to get the motion blur in y-axis image as hw_2_8.exr
`
  -hw 2_8 9 -spp 64 
`





# Bonus 2_9: Object Instancing 

### Method

Actually I have implement the transform for my sphere and triangle originally in my code. Here we take all transform(translate, rotation, scale) as a matrix4x4.

First of all, we need to take a inverse transform on our ray vector, and put the transformed ray into the equations to get the hit point and distance. After that, we need to multiply the result with transform matrix to get the hit point in world coordinates.

```cpp
bool FindIntersection(ParsedSphere& sphere, Matrix4x4 transform, Real t_min, Real t_max)
{
	Matrix4x4 inverseTransform = inverse(transform);

	Vector4 origin_4 = Vector4(inverseTransform * Vector4(Origin, 1.));
	Vector3 origin = Vector3(origin_4.x, origin_4.y, origin_4.z);
	Vector4 direction_4 = Vector4(inverseTransform * Vector4(Direction, 0.));
	Vector3 direction = Vector3(direction_4.x, direction_4.y, direction_4.z);

	Real c = dot(origin, origin) - sphere.radius * sphere.radius;
	Real b = 2. * dot(origin, direction);
	Real a = dot(direction, direction);

	Real delta = b * b - 4 * a * c;
	if (delta < 0)
	{
		return false;
	}
	else
	{
		delta = sqrt(delta);
		Real t1 = (-b + delta) / (2 * a);
		Real t2 = (-b - delta) / (2 * a);
		Real nearest_t = min(t1, t2);
		if (c / a < 0) // c / a == t1 * t2 < 0 , ray is inside the SphereObj
			nearest_t = t1;
		if (nearest_t >= t_min && nearest_t < t_max)
		{
			Objects.push_back(std::make_shared<Shape>(Shape{ sphere }));
			Distances.push_back(nearest_t);
			return true;
		}
		else
			return false;
	}
}
```

And we also need to multiple the transpose inverse transform matrix with normals.

```cpp
Vector4 normal_original = GetNormalByHitPoint(hit_point_original, sphere);
Vector4 normal_transformed_4 = Vector4(transpose(inverse(
		translate(Vector3(sphere.position.x, sphere.position.y, 		sphere.position.z)))) * normal_original);
normal_transformed = normalize(Vector3(normal_transformed_4.x, normal_transformed_4.y, normal_transformed_4.z));
```
