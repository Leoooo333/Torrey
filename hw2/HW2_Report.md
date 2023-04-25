[TOC]



# My design 2_6

### Glass
There I made scene_5 ~ scene_8 for glass version for testing refraction.

I put my designs in `hw1_scenes.h`

## Leo signature
My name is Leo, I design a signature using 3 different spirals with glass materials in:
> hw1_scenes.h --> hw1_scene_9

The content is made by Python code `Leo_generator.ipynb`

#### Run this command and find it as hw_1_9.exr
`
  -hw 1_9 9
`



# Bonus 2_7
### Method 1: parallel BVH construction
In the construction function, firstly seperate our shapes (objects) into `parallel_counts` of segments. And then use `parallel_for` to parallely construct the sub BVH and push back them in `bvh_list`.

```cpp
BVH::BVH(const std::vector<std::shared_ptr<Shape>>& src_shapes, double time0, double time1, pcg32_state& rng, const int parallel_counts, const int tile_size_bvh)
{
    std::vector<std::shared_ptr<BVH>> bvh_list;
    std::vector<std::vector<std::shared_ptr<Shape>>> shapes_tiles_list;
	const int tile_size = tile_size_bvh;
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
            BVH bvh(shapes_tiles_list[i], time0, time1, rng);
            bvh_list.push_back(std::make_shared<BVH>(bvh));
        }

    }, num_tiles);

	for (int i = 0; i < num_tiles; i++)
    {
        bvh_list.push_back(std::make_shared<BVH>(BVH(shapes_tiles_list[i], 0, 0, rng)));
    }
    BVH root = *build_bvh_by_list(bvh_list, rng);
    left = root.left;
    right = root.right;
    box = root.box;
}
```

 After all the sub nodes were finished, built the binary tree based on the nodes recursively.

```cpp
std::shared_ptr<BVH>BVH::build_bvh_by_list(std::vector<std::shared_ptr<BVH>>& list, pcg32_state& rng)
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
```

### Method 2: nth_element

Since we will sort the objects recursively, the exactly order within the sub node doesn't matters. ` std::nth_element` here could fix the middle object, and separate the smaller half to the left, the bigger half to the right.

### Comparison (2 threads only, -spp = 1)

Based on the teapot object, I did some experiments. See the table below: Here we get **2.35x** speedup (6.7 s / 2.85 s)

| *teapot*        | Parallel BVH construction                | w/o parallel      |
| --------------- | ---------------------------------------- | ----------------- |
| **nth_element** | 1.6 (construct) + 1.25 (render) = 2.85 s | 4.6 + 1.1 = 5.7 s |
| **sort**        | 1.6 (construct) + 1.74 (render) = 3.34 s | 5.5 + 1.2 = 6.7 s |

See the table below for bunny: Here we get **15x** speedup (20.2 s / 5*60 s)


| *bunny*         | Parallel BVH construction                | w/o parallel      |
| --------------- | ---------------------------------------- | ----------------- |
| **nth_element** | 13.8 (construct) + 6.4 (render) = 20.2 s | > 5 mins, unknown |
| **sort**        | 16.6 (construct) + 7.5 (render) = 24.1 s | > 5 mins, unknown |

#### Best performance

Let's make `parallel_counts = 512` and no threads limits, based on bunny. And we will get the best performance in 10s, which is almost **30x** faster than original one. 

```
Scene parsing done. Took 0.22027 seconds.
Build BVH Time: 3.74021
 100.00 Percent Done (80 / 80)Total Time:       9.93671
```

And the best performance for ***party*** is 1 min.

### Run

Run this command and render the lovely bunny as hw_2_7.exr with in only a couple of seconds. 
`
  -hw 2_7 ../scenes/bunny/bunny.xml
`



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
