# My design
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

## Code structure
Here I have created some of new .h and .cpp for convenience. I have include them in `cmakelists.txt`

And also I add or modify some methods in `vector.h` and `matrix.h` for convenience.

1. **Renderer.h**
   
   Renderer is a class that contain the tracing process. It is just like a main class of my ray tracer. But what different is we can have lots of renderer instances, and each of them will hold their individual cameras, parameters and so on.

   And for convenience, I put different shading methods into `Renderer.h`. 

   Like

    ``` cpp
    Vector3 Illumination_hw_1_1(Ray& vis_ray, bool isPrimary_ray, Vector3 HitPoint,
	    Vector3 Normal, ParsedShape& NearstObj, ParsedScene& scene, Variables& vars);
    ```

2. **Camera.h**
   
   Camera is a abstract class. There we can implement different kind of ray sampling methods for different type of camera. Here I implement `PersectiveCamera`, and `EnvironmentCamera` which is kind of 360 camera.

3. **Shape.h**

    Shape contains diffrent method for different ray-object intersection.

# Bonus 1_10
### Method
In `hw1_scenes.h` there are structure **Material** and enum **MaterialType**

```cpp
enum class MaterialType {
    Diffuse,
    Mirror,
    Plastic
};
```

where plastic refer to glass material. It is consistent with the `parse_scene.h`.

##### Refract function in `vector.h`

```cpp
template <typename T>
inline TVector3<T> refract(const TVector3<T>& incident, const TVector3<T>& normal, Real etai_over_etat) 
{
    auto k = 1.0 - etai_over_etat * etai_over_etat * (1.0 - dot(incident, normal) * dot(incident, normal));
    return etai_over_etat * incident - (etai_over_etat * dot(incident, normal) + sqrt(k)) * normal;
}
```

#### Dealing with total internal reflection in `Renderer.cpp`
In method  `HitNearst`
```cpp
else if (material.index() == 2) // plastic
		{
			ParsedPlastic material_plastic = std::get<ParsedPlastic>(material);
			Real index_of_refraction = material_plastic.eta;

			if (!isFrontFace(ray, normal_transformed))
			{
				index_of_refraction = 1. / material_plastic.eta;
				normal_transformed = -normal_transformed;
			}

			double cos_theta = min(dot(-ray.Direction, normal_transformed), 1.0);
			double sin_theta = sqrt(1.0 - cos_theta * cos_theta);

			if (index_of_refraction * sin_theta > 1.0) {	
				// Must Reflect
				ray.Direction = reflect(ray.Direction, normal_transformed);
			}
```

#### Transmittance and Reflectance
In method  `HitNearst`

```cpp
if (index_of_refraction * sin_theta > 1.0) {	
    // Must Reflect
    ray.Direction = reflect(ray.Direction, normal_transformed);
}
else {
    Real kr = reflectance(cos_theta, index_of_refraction);
    Real kt = 1 - kr;

    Ray refracted_ray = { ray.Origin, refract(ray.Direction, normal_transformed, index_of_refraction) };
    // Can Refract
    ray.Direction = reflect(ray.Direction, normal_transformed);

    if (0 == max_depth--)
        return miss(ray, scene, m_Vars, max_depth);
    else
    {
        return  color + surface_color * (kt * TraceRay(refracted_ray, scene, true, max_depth, miss, illumination, rng) 
            + kr * TraceRay(ray, scene, true, max_depth, miss, illumination, rng));
```

### Run
Run this command and find scene_1 with glass materials as hw_1_10.exr
`
  -hw 1_10 6
`



# Bonus 1_11

### Method
In `Camera.h` there is a attribute called `CameraParameters`, where maintain parameters of a camera. 

And for defocus blur, I add `aperture` as a new parameter for each kind of camera. 

I think defocus blur is kind of noize, where we randomly choose a direction on each pixel's light field function. So for different pixel, it maybe lighted in different view direction, which causes that kind of blur. There the aperture controll the range of this random effect.
```cpp
	if (m_CameraParameters.aperture > 0)
	{
		Real random_theta = 2 * c_PI * next_pcg32_real<double>(rng);
		Real random_radius = (m_CameraParameters.aperture / 2) * next_pcg32_real<double>(rng);
		Vector2 random_direction = {sin(random_theta), cos(random_theta)};

		//Vector3 a = target;
		//a += random_radius * random_direction.x * u +  random_radius * random_direction.y * v;
		//rayDirection = normalize(a - rayOrigin);
		rayOrigin += random_radius * random_direction.x * u +  random_radius * random_direction.y * v;
		
		rayDirection = normalize(target - rayOrigin);
	}
	else
		rayDirection = normalize(target);
```

### Run
Run this command and find scene_1 with glass materials as hw_1_11.exr
`
  -hw 1_11 7
`
