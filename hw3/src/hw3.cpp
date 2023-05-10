#include "hw3.h"
#include "parse_scene.h"
#include "Renderer.h"
#include "timer.h"
#include "Variables.h"

Image3 hw_3_1(const std::vector<std::string> &params) {
    // Homework 3.1: image textures
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
    render.Render_BVH(cam, vars, s, &Renderer::Miss_hw_3_1, &Renderer::Illumination_hw_3_1);
    img = render.GetImage();

    return *img;
}

Image3 hw_3_2(const std::vector<std::string> &params) {
    // Homework 3.2: shading normals
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
    // enable shading_normal
    vars.shading_normal = true;
    CameraUnion cam = GenerateCameraByType(vars.cameraParameters, PERSPECTIVE_CAM);
    Scene s = ParsedSceneToScene(scene);
    render.Render_BVH(cam, vars, s, &Renderer::Miss_hw_3_1, &Renderer::Illumination_hw_3_1);
    img = render.GetImage();

    return *img;
}

Image3 hw_3_3(const std::vector<std::string> &params) {
    // Homework 3.3: Fresnel
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
    // enable shading_normal
    vars.shading_normal = true;
    CameraUnion cam = GenerateCameraByType(vars.cameraParameters, PERSPECTIVE_CAM);
    Scene s = ParsedSceneToScene(scene);
    render.Render_BVH(cam, vars, s, &Renderer::Miss_hw_3_3, &Renderer::Illumination_hw_3_3);
    img = render.GetImage();

    return *img;
}

Image3 hw_3_4(const std::vector<std::string> &params) {
    // Homework 3.4: area lights
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
    // enable shading_normal
    vars.shading_normal = false;
    CameraUnion cam = GenerateCameraByType(vars.cameraParameters, PERSPECTIVE_CAM);
    Scene s = ParsedSceneToScene(scene);
    render.Render_BVH(cam, vars, s, &Renderer::Miss_hw_3_4, &Renderer::Illumination_hw_3_4);
    img = render.GetImage();    

    return *img;
}

Image3 hw_3_7(const std::vector<std::string>& params) {
    // Homework 3.7: volume
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

    vars.max_depth = 100; // tracing depth

    Renderer render;

    vars.cameraParameters.width = scene.camera.width;
    vars.cameraParameters.height = scene.camera.height;
    vars.cameraParameters.eye = scene.camera.lookfrom;
    vars.cameraParameters.center = scene.camera.lookat;
    vars.cameraParameters.upvec = scene.camera.up;
    vars.cameraParameters.fovy = scene.camera.vfov;
    vars.cameraParameters.samples_per_pixel = scene.samples_per_pixel;
    //vars.cameraParameters.samples_per_pixel = 8;

    //larger the file, larger the parallel_counts for bvh
    vars.parallel_counts_bvh = 16;
    // enable shading_normal
    vars.shading_normal = false;
    CameraUnion cam = GenerateCameraByType(vars.cameraParameters, PERSPECTIVE_CAM);
    Scene s = ParsedSceneToScene(scene);
    render.Render_BVH(cam, vars, s, &Renderer::Miss_hw_3_7, &Renderer::Illumination_hw_3_7);
    img = render.GetImage();

    return *img;
}

Image3 hw_3_9(const std::vector<std::string>& params) {
    // Homework 3.9: Stratification
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

    vars.max_depth = 10; // tracing depth

    Renderer render;

    vars.cameraParameters.width = scene.camera.width;
    vars.cameraParameters.height = scene.camera.height;
    vars.cameraParameters.eye = scene.camera.lookfrom;
    vars.cameraParameters.center = scene.camera.lookat;
    vars.cameraParameters.upvec = scene.camera.up;
    vars.cameraParameters.fovy = scene.camera.vfov;
    //vars.cameraParameters.samples_per_pixel = scene.samples_per_pixel;
    vars.cameraParameters.samples_per_pixel = 1;

    //larger the file, larger the parallel_counts for bvh
    vars.parallel_counts_bvh = 16;
    // enable shading_normal
    vars.shading_normal = false;
    // enable multiple sampling on area lights, must be square
    vars.area_light_samples = 4*4;

    CameraUnion cam = GenerateCameraByType(vars.cameraParameters, PERSPECTIVE_CAM);
    Scene s = ParsedSceneToScene(scene);
    render.Render_BVH(cam, vars, s, &Renderer::Miss_hw_3_9, &Renderer::Illumination_hw_3_9);
    img = render.GetImage();

    return *img;
}

Image3 hw_3_11(const std::vector<std::string>& params) {
    // Homework 3.11: cone sampling
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

    vars.max_depth = 10; // tracing depth

    Renderer render;

    vars.cameraParameters.width = scene.camera.width;
    vars.cameraParameters.height = scene.camera.height;
    vars.cameraParameters.eye = scene.camera.lookfrom;
    vars.cameraParameters.center = scene.camera.lookat;
    vars.cameraParameters.upvec = scene.camera.up;
    vars.cameraParameters.fovy = scene.camera.vfov;
    vars.cameraParameters.samples_per_pixel = scene.samples_per_pixel;
    //larger the file, larger the parallel_counts for bvh
    vars.parallel_counts_bvh = 16;
    // enable shading_normal
    vars.shading_normal = false;
    // enable multiple sampling on area lights, must be square
    //vars.area_light_samples = 4 * 4;

    CameraUnion cam = GenerateCameraByType(vars.cameraParameters, PERSPECTIVE_CAM);
    Scene s = ParsedSceneToScene(scene);
    render.Render_BVH(cam, vars, s, &Renderer::Miss_hw_3_11, &Renderer::Illumination_hw_3_11);
    img = render.GetImage();

    return *img;
}

Image3 hw_3_5(const std::vector<std::string>& params) {
    // Homework 3.5: design
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

    vars.max_depth = 10; // tracing depth

    Renderer render;

    vars.cameraParameters.width = scene.camera.width;
    vars.cameraParameters.height = scene.camera.height;
    vars.cameraParameters.eye = scene.camera.lookfrom;
    vars.cameraParameters.center = scene.camera.lookat;
    vars.cameraParameters.upvec = scene.camera.up;
    vars.cameraParameters.fovy = scene.camera.vfov;
    vars.cameraParameters.samples_per_pixel = scene.samples_per_pixel;
    //vars.cameraParameters.samples_per_pixel = 2;

    //larger the file, larger the parallel_counts for bvh
    vars.parallel_counts_bvh = 16;
    // enable shading_normal
    vars.shading_normal = true;
    // enable multiple sampling on area lights, must be square  
    //vars.area_light_samples = 4 * 4;

    CameraUnion cam = GenerateCameraByType(vars.cameraParameters, PERSPECTIVE_CAM);
    Scene s = ParsedSceneToScene(scene);
    render.Render_BVH(cam, vars, s, &Renderer::Miss_hw_3_11, &Renderer::Illumination_hw_3_11);
    img = render.GetImage();

    return *img;
}