#include "integrator.h"
#include "scene.h"

class DirectIntegrator : public Integrator {
public:
    DirectIntegrator(const PropertyList &props) {
        /* No parameters this time */
    }

    Color3f Li(const Scene *scene, const Ray &ray) const {
        /** TODO : Find the surface that is visible in the requested direction
                   Return its ambiant color */
        Hit *hit = new Hit();
        scene->intersect(ray,*hit);
        if (hit->foundIntersection()) {
          Normal3f n = hit->normal();

          Color3f c = Color3f(0.0f);
          Vector3f lightDir;
          Point3f x = ray.origin + hit->t() * ray.direction;

          for (size_t i=0; i<scene->lightList().size(); ++i) {
            //const Vector3f& viewDir, const Vector3f& lightDir, const Normal3f& normal, const Vector2f& uv
            Color3f ro = hit->shape()->material()->brdf(-ray.direction,scene->lightList()[i]->direction(x),n,NULL);
            Color3f ii = std::max(scene->lightList()[i]->direction(x).dot(n),0.0f)*scene->lightList()[i]->intensity(x);
            c += ro*ii;
          }

          return c;
        }
        return scene->backgroundColor();
    }

    std::string toString() const {
        return "NormalsIntegrator[]";
    }
};

REGISTER_CLASS(DirectIntegrator, "direct");
