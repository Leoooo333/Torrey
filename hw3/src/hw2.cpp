#include "hw2.h"

#include "print_scene.h"
#include "Renderer.h"
#include "timer.h"
#include "Variables.h"
#include "hw1_scenes.h"


Image3 hw_2_1(const std::vector<std::string>& params) {
    // Homework 2.1: render a single triangle and outputs
    // its barycentric coordinates.
    // We will use the following camera parameter
    // lookfrom = (0, 0,  0)
    // lookat   = (0, 0, -1)
    // up       = (0, 1,  0)
    // vfov     = 45
    // and we will parse the triangle vertices from params
    // The three vertices are stored in v0, v1, and v2 below.

    std::vector<float> tri_params;
    int spp = 16;
    for (int i = 0; i < (int)params.size(); i++) {
        if (params[i] == "-spp") {
            spp = std::stoi(params[++i]);
        }
        else {
            tri_params.push_back(std::stof(params[i]));
        }
    }

    if (tri_params.size() < 9) {
        // Not enough parameters to parse the triangle vertices.
        return Image3(0, 0);
    }

    Vector3 p0{ tri_params[0], tri_params[1], tri_params[2] };
    Vector3 p1{ tri_params[3], tri_params[4], tri_params[5] };
    Vector3 p2{ tri_params[6], tri_params[7], tri_params[8] };

    UNUSED(spp); // avoid unused warning
    // Your scene is hw1_scenes[scene_id]

    std::shared_ptr<Image3> img;
    Variables vars;

    vars.max_depth = 0; // tracing depth

    Renderer render;
    ParsedScene parsed_scene;
    ParsedTriangleMesh mesh;
    mesh.positions.push_back(p0);
    mesh.positions.push_back(p1);
    mesh.positions.push_back(p2);
    mesh.indices.push_back(Vector3i(0, 1, 2));
    mesh.material_id = 0;

    ParsedMaterial parsed_m;
    ParsedDiffuse diffuse_m = { Vector3(0., 0., 0.) };
    parsed_m = { diffuse_m };

    ParsedShape shape{ mesh };
    parsed_scene.shapes.push_back(shape);
    parsed_scene.materials.push_back(parsed_m);
    vars.cameraParameters.width = 640;
    vars.cameraParameters.height = 480;
    vars.cameraParameters.fovy = 45.;
    vars.cameraParameters.samples_per_pixel = spp;
    Scene scene = ParsedSceneToScene(parsed_scene);
    CameraUnion cam = GenerateCameraByType(vars.cameraParameters, PERSPECTIVE_CAM);
    render.Render(cam, vars, scene, &Renderer::Miss_hw_2_1, &Renderer::Illumination_hw_2_1);
    img = render.GetImage();

    return *img;
}

Image3 hw_2_2(const std::vector<std::string>& params) {
    // Homework 2.2: render a triangle mesh.
    // We will use the same camera parameter:
    // lookfrom = (0, 0,  0)
    // lookat   = (0, 0, -1)
    // up       = (0, 1,  0)
    // vfov     = 45
    // and we will use a fixed triangle mesh: a tetrahedron!
    int spp = 16;
    for (int i = 0; i < (int)params.size(); i++) {
        if (params[i] == "-spp") {
            spp = std::stoi(params[++i]);
        }
    }

    std::vector<Vector3> positions = {
        Vector3{ 0.0,  0.5, -2.0},
        Vector3{ 0.0, -0.3, -1.0},
        Vector3{ 1.0, -0.5, -3.0},
        Vector3{-1.0, -0.5, -3.0}
    };
    std::vector<Vector3i> indices = {
        Vector3i{0, 1, 2},
        Vector3i{0, 3, 1},
        Vector3i{0, 2, 3},
        Vector3i{1, 2, 3}
    };

    UNUSED(spp); // avoid unused warning
    // Your scene is hw1_scenes[scene_id]

    std::shared_ptr<Image3> img;
    Variables vars;

    vars.max_depth = 0; // tracing depth

    Renderer render;
    ParsedScene parsed_scene;
    ParsedTriangleMesh triangle;
    triangle.positions = positions;
    triangle.indices = indices;
    triangle.material_id = 0;

    ParsedMaterial parsed_m;
    ParsedDiffuse diffuse_m = { Vector3(0., 0., 0.) };
    parsed_m = { diffuse_m };

    ParsedShape shape{ triangle };
    parsed_scene.shapes.push_back(shape);
    parsed_scene.materials.push_back(parsed_m);
    vars.cameraParameters.width = 640;
    vars.cameraParameters.height = 480;
    vars.cameraParameters.fovy = 45.;
    vars.cameraParameters.samples_per_pixel = spp;

    CameraUnion cam = GenerateCameraByType(vars.cameraParameters, PERSPECTIVE_CAM);
    Scene s = ParsedSceneToScene(parsed_scene);
    render.Render(cam, vars, s, &Renderer::Miss_hw_2_2, &Renderer::Illumination_hw_2_2);
    img = render.GetImage();

    return *img;
}

Image3 hw_2_3(const std::vector<std::string>& params) {
    // Homework 2.3: render a scene file provided by our parser.
    if (params.size() < 1) {
        return Image3(0, 0);
    }
    Timer timer;
    tick(timer);
    ParsedScene scene = parse_scene(params[0]);
    std::cout << "Scene parsing done. Took " << tick(timer) << " seconds." << std::endl;
    std::cout << scene << std::endl;

    std::shared_ptr<Image3> img;
    Variables vars;

    vars.max_depth = 10; // tracing depth

    Renderer render;

    vars.cameraParameters.width = scene.camera.width;
    vars.cameraParameters.height = scene.camera.height;
    vars.cameraParameters.eye = scene.camera.lookfrom;
    vars.cameraParameters.center = scene.camera.lookat;
    vars.cameraParameters.upvec = scene.camera.up;
    vars.cameraParameters.fovy = scene.camera.vfov;
    vars.cameraParameters.samples_per_pixel = scene.samples_per_pixel;
    CameraUnion cam = GenerateCameraByType(vars.cameraParameters, PERSPECTIVE_CAM);
    Scene s = ParsedSceneToScene(scene);
    render.Render(cam, vars, s, &Renderer::Miss_hw_2_3, &Renderer::Illumination_hw_2_3);
    img = render.GetImage();

    return *img;
}

Image3 hw_2_4(const std::vector<std::string>& params) {
    // Homework 2.4: render the AABBs of the scene.
    if (params.size() < 1) {
        return Image3(0, 0);
    }
    Timer timer;
    tick(timer);
    ParsedScene scene = parse_scene(params[0]);
    std::cout << "Scene parsing done. Took " << tick(timer) << " seconds." << std::endl;
    UNUSED(scene);
    std::shared_ptr<Image3> img;
    Variables vars;

    vars.max_depth = 0; // tracing depth

    Renderer render;

    vars.cameraParameters.width = scene.camera.width;
    vars.cameraParameters.height = scene.camera.height;
    vars.cameraParameters.eye = scene.camera.lookfrom;
    vars.cameraParameters.center = scene.camera.lookat;
    vars.cameraParameters.upvec = scene.camera.up;
    vars.cameraParameters.fovy = scene.camera.vfov;
    vars.cameraParameters.samples_per_pixel = scene.samples_per_pixel;

    CameraUnion cam = GenerateCameraByType(vars.cameraParameters, PERSPECTIVE_CAM);
    Scene s = ParsedSceneToScene(scene);
    render.Render_AABB(cam, vars, s, &Renderer::Miss_hw_2_4, &Renderer::Illumination_hw_2_4);
    img = render.GetImage();

    return *img;
}

Image3 hw_2_5(const std::vector<std::string>& params) {
    // Homework 2.5: rendering with BVHs
    if (params.size() < 1) {
        return Image3(0, 0);
    }
    int spp = 16;
    Timer timer;
    tick(timer);
    ParsedScene scene = parse_scene(params[0]);
    std::cout << "Scene parsing done. Took " << tick(timer) << " seconds." << std::endl;
    UNUSED(scene);

    UNUSED(spp);
    std::shared_ptr<Image3> img;
    Variables vars;

    vars.max_depth = 5; // tracing depth

    Renderer render;

    vars.cameraParameters.width = scene.camera.width;
    vars.cameraParameters.height = scene.camera.height;
    vars.cameraParameters.eye = scene.camera.lookfrom;
    vars.cameraParameters.center = scene.camera.lookat;
    vars.cameraParameters.upvec = scene.camera.up;
    vars.cameraParameters.fovy = scene.camera.vfov;
    vars.cameraParameters.samples_per_pixel = scene.samples_per_pixel;
    //vars.cameraParameters.samples_per_pixel = 1;

    //larger the file, larger the parallel_counts for bvh
    vars.parallel_counts_bvh = 1;
    CameraUnion cam = GenerateCameraByType(vars.cameraParameters, PERSPECTIVE_CAM);
    Scene s = ParsedSceneToScene(scene);
    render.Render_BVH(cam, vars, s, &Renderer::Miss_hw_2_5, &Renderer::Illumination_hw_2_5);
    img = render.GetImage();

    return *img;
}

Image3 hw_2_7(const std::vector<std::string>& params) {
    // Homework 2.5: rendering with BVHs
    if (params.size() < 1) {
        return Image3(0, 0);
    }
    int spp = 16;
    Timer timer;
    tick(timer);
    ParsedScene scene = parse_scene(params[0]);
    std::cout << "Scene parsing done. Took " << tick(timer) << " seconds." << std::endl;
    UNUSED(scene);

    UNUSED(spp);
    std::shared_ptr<Image3> img;
    Variables vars;

    vars.max_depth = 5; // tracing depth

    Renderer render;

    vars.cameraParameters.width = scene.camera.width;
    vars.cameraParameters.height = scene.camera.height;
    vars.cameraParameters.eye = scene.camera.lookfrom;
    vars.cameraParameters.center = scene.camera.lookat;
    vars.cameraParameters.upvec = scene.camera.up;
    vars.cameraParameters.fovy = scene.camera.vfov;
    vars.cameraParameters.samples_per_pixel = scene.samples_per_pixel;

    //vars.cameraParameters.samples_per_pixel = 64;
    //larger the file, larger the parallel_counts for bvh
    vars.parallel_counts_bvh = 16;
    CameraUnion cam = GenerateCameraByType(vars.cameraParameters, PERSPECTIVE_CAM);
    Scene s = ParsedSceneToScene(scene);
    render.Render_BVH(cam, vars, s, &Renderer::Miss_hw_2_5, &Renderer::Illumination_hw_2_5);
    img = render.GetImage();

    return *img;
}

Image3 hw_2_6(const std::vector<std::string>& params) {
    // Homework 2.5: rendering with BVHs
    if (params.size() < 1) {
        return Image3(0, 0);
    }
    int spp = 16;
    Timer timer;
    tick(timer);
    ParsedScene scene = parse_scene(params[0]);
    std::cout << "Scene parsing done. Took " << tick(timer) << " seconds." << std::endl;
    UNUSED(scene);

    UNUSED(spp);
    std::shared_ptr<Image3> img;
    Variables vars;

    vars.max_depth = 5; // tracing depth

    Renderer render;

    vars.cameraParameters.width = scene.camera.width;
    vars.cameraParameters.height = scene.camera.height;
    vars.cameraParameters.eye = scene.camera.lookfrom;
    vars.cameraParameters.center = scene.camera.lookat;
    vars.cameraParameters.upvec = scene.camera.up;
    vars.cameraParameters.fovy = scene.camera.vfov;
    vars.cameraParameters.samples_per_pixel = scene.samples_per_pixel;
    //vars.cameraParameters.samples_per_pixel = 1;

    //larger the file, larger the parallel_counts for bvh
    vars.parallel_counts_bvh = 16;
    CameraUnion cam = GenerateCameraByType(vars.cameraParameters, PERSPECTIVE_CAM);
    Scene s = ParsedSceneToScene(scene);
    render.Render_BVH(cam, vars, s, &Renderer::Miss_hw_1_9, &Renderer::Illumination_hw_2_5);
    img = render.GetImage();

    return *img;
}

Image3 hw_2_8(const std::vector<std::string>& params) {
    // Homework 2.8: motion blur
    if (params.size() < 1) {
        return Image3(0, 0);
    }
    int spp = 16;
    Timer timer;
    tick(timer);
    //ParsedScene scene = parse_scene(params[0]);
    //std::cout << "Scene parsing done. Took " << tick(timer) << " seconds." << std::endl;
    //UNUSED(scene);
    hw1::HW1_Scene& scene_9 = hw1::hw1_scenes[9];
    UNUSED(spp);
    std::shared_ptr<Image3> img;
    Variables vars;
    ParsedScene scene = SceneToParsedScene(scene_9);

    vars.max_depth = 5; // tracing depth

    Renderer render;

    vars.cameraParameters.width = 1280;
    vars.cameraParameters.height = 960;
    vars.cameraParameters.eye = scene.camera.lookfrom;
    vars.cameraParameters.center = scene.camera.lookat;
    vars.cameraParameters.upvec = scene.camera.up;
    vars.cameraParameters.fovy = scene.camera.vfov;
    vars.cameraParameters.samples_per_pixel = scene.samples_per_pixel;

    //larger the file, larger the parallel_counts for bvh
    vars.parallel_counts_bvh = 16;
    vars.motion_blur_samples = 8;
    //vars.cameraParameters.samples_per_pixel = 1;
    CameraUnion cam = GenerateCameraByType(vars.cameraParameters, PERSPECTIVE_CAM);
    Scene s = ParsedSceneToScene(scene);
    render.Render_BVH(cam, vars, s, &Renderer::Miss_hw_1_9, &Renderer::Illumination_hw_2_5);
    img = render.GetImage();

    return *img;
}

Image3 hw_2_9(const std::vector<std::string>& params) {
    // Homework 2.5: rendering with BVHs
    if (params.size() < 1) {
        return Image3(0, 0);
    }
    int spp = 16;
    Timer timer;
    tick(timer);
    ParsedScene scene = parse_scene(params[0]);
    std::cout << "Scene parsing done. Took " << tick(timer) << " seconds." << std::endl;
    UNUSED(scene);

    UNUSED(spp);
    std::shared_ptr<Image3> img;
    Variables vars;

    vars.max_depth = 5; // tracing depth

    Renderer render;

    vars.cameraParameters.width = scene.camera.width;
    vars.cameraParameters.height = scene.camera.height;
    vars.cameraParameters.eye = scene.camera.lookfrom;
    vars.cameraParameters.center = scene.camera.lookat;
    vars.cameraParameters.upvec = scene.camera.up;
    vars.cameraParameters.fovy = scene.camera.vfov;
    vars.cameraParameters.samples_per_pixel = scene.samples_per_pixel;
    //vars.cameraParameters.samples_per_pixel = 1;

    //larger the file, larger the parallel_counts for bvh
    vars.parallel_counts_bvh = 16;
    CameraUnion cam = GenerateCameraByType(vars.cameraParameters, PERSPECTIVE_CAM);
    Scene s = ParsedSceneToScene(scene);
    render.Render_BVH(cam, vars, s, &Renderer::Miss_hw_2_5, &Renderer::Illumination_hw_2_5);
    img = render.GetImage();

    return *img;
}
