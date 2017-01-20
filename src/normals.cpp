#include "integrator.h"
#include "scene.h"

class NormalsIntegrator : public Integrator {
public:
    NormalsIntegrator(const PropertyList &props) {
        /* No parameters this time */
    }

    Color3f Li(const Scene *scene, const Ray &ray) const {
        /** TODO : Find the surface that is visible in the requested direction
                   Return its ambiant color */
        Hit *hit = new Hit();
        scene->intersect(ray,*hit);
        if (hit->foundIntersection()) {
          Normal3f n = hit->normal();
          printf("%f\n", n.x());
          Color3f *c = new Color3f(abs(n.x()),abs(n.y()),abs(n.z()));

          return *c;
        }
        return scene->backgroundColor();
    }

    std::string toString() const {
        return "NormalsIntegrator[]";
    }
};

REGISTER_CLASS(NormalsIntegrator, "normals");
