#include "viewer.h"

#include <filesystem/resolver.h>

void render(Scene* scene, ImageBlock* result, std::string outputName, bool* done)
{
    if(!scene)
        return;

    clock_t t = clock();

    const Camera *camera = scene->camera();
    const Integrator* integrator = scene->integrator();
    integrator->preprocess(scene);

    float tanfovy2 = tan(camera->fovY()*0.5);
    //Vector3f camX = camera->right() * 0.5 * camera->vpWidth();
    //Vector3f camY = camera->up() * 0.5 * camera->vpHeight();

    Vector3f camX = camera->right() * tanfovy2 * camera->nearDist() * camera->vpWidth()/float(camera->vpHeight());
    Vector3f camY = - camera->up() * tanfovy2 * camera->nearDist();

    Vector3f camF = camera->direction() * camera->nearDist();

    /// TODO:
    ///  1. iterate over the image pixels
    ///  2. generate a primary ray
    ///  3. call the integartor to compute the color along this ray
    ///  4. write this color in the result image

    /// Part 1 :
    Vector3f direction;
    for (size_t i = 0; i < camera->vpWidth(); i++) {
      for (size_t j = 0; j < camera->vpHeight(); j++) {
        /* code */
        float coefI = 2*(i+0.5f)/camera->vpWidth()-1;
        float coefJ = 2*(j+0.5f)/camera->vpHeight()-1;
        direction = camF + coefI*camX + coefJ*camY;
        direction.normalize();

        Ray *r = new Ray(camera->position(),direction);
        //Hit *h = new Hit();
        //scene->intersect(*r,*h);
        Color3f color = integrator->Li(scene,*r);
        result->put(Vector2f(i,j), color);
      }
    }

    t = clock() - t;
    std::cout << "Raytracing time : " << float(t)/CLOCKS_PER_SEC << "s"<<std::endl;

    Bitmap* img = result->toBitmap();
    img->save(outputName);
    delete img;
    *done = true;
}

int main(int argc, char *argv[])
{
    getFileResolver()->prepend(DATA_DIR);

    try {
        nanogui::init();
        Viewer *screen = new Viewer();;

        if (argc == 2) {
            /* load file from the command line */
            filesystem::path path(argv[1]);

            if(path.extension() == "scn") { // load scene file
                screen->loadScene(argv[1]);
            }else if(path.extension() == "exr") { // load OpenEXR image
                screen->loadImage(argv[1]);
            }
        }

        /* Enter the application main loop */
        nanogui::mainloop();

        delete screen;
        nanogui::shutdown();
    } catch (const std::exception &e) {
        cerr << "Fatal error: " << e.what() << endl;
        return -1;
    }
    return 0;
}
