#include "integrator.h"
#include "scene.h"

class WhittedIntegrator : public Integrator {
public:
    WhittedIntegrator(const PropertyList &props) {
      maxRecursion = props.getInteger("maxRecursion",0);
    }

    Color3f Li(const Scene *scene, const Ray &ray) const {
        /** TODO : Find the surface that is visible in the requested direction
                   Return its ambiant color */

        if (ray.recursionLevel == 0) {
          return Color3f(0.0f);
        }
        Hit *hit = new Hit();
        scene->intersect(ray,*hit);
        if (hit->foundIntersection()) {
          Normal3f n = hit->normal();

          Color3f c = Color3f(0.0f);
          Vector3f lightDir;
          Point3f x = ray.at(hit->t());
          Point3f xBis = x + 0.0001f * n;
          //Pour chacune des lights
          for (size_t i=0; i<scene->lightList().size(); ++i) {
            //First, check if the light is visible
            //On commence le test pour savoir si la lumière touche l'objet
            Point3f xBis = x + 0.0001f * n;
            Ray *lightRay = new Ray(xBis, scene->lightList()[i]->direction(xBis));
            Hit *lightHit = new Hit();
            scene->intersect(*lightRay,*lightHit);
            //Si ça touche alors on vérifie que ce n'est pas derrière la lumière :
            // object1 --> Lumiere --> object2
            if (lightHit->foundIntersection()) {
              Point3f lightHitPoint = lightRay->at(lightHit->t());
              //On compare les deux directions, si le scalaire = -1 alors elles sont inverse (donc pas blocant)
              //On utilise round pour limiter les problemes de float
              if (round(scene->lightList()[i]->direction(xBis).dot(scene->lightList()[i]->direction(lightHitPoint))) == -1) {
                Color3f ro = hit->shape()->material()->brdf(-ray.direction,scene->lightList()[i]->direction(x),n,NULL);
                Color3f ii = std::max(scene->lightList()[i]->direction(x).dot(n),0.0f)*scene->lightList()[i]->intensity(x);
                c += ro*ii;
              }
            } else { //Rien ne bloque la lumière donc on calcule
              Color3f ro = hit->shape()->material()->brdf(-ray.direction,scene->lightList()[i]->direction(x),n,NULL);
              Color3f ii = std::max(scene->lightList()[i]->direction(x).dot(n),0.0f)*scene->lightList()[i]->intensity(x);
              c += ro*ii;
            }
          }
          Vector3f dRefl = 2*n.dot(-ray.direction)*n+ray.direction;
          //dRefl.normalize();
          Ray *r = new Ray(xBis,dRefl);
          //If ray.recursionLevel == -1 : we need to first set the maxRecursion value
          if (ray.recursionLevel == -1) {
            r->recursionLevel = maxRecursion - 1;
          } else {
            r->recursionLevel = ray.recursionLevel - 1;
          }

          //Max
          float max = 0.0f;
          if(r->direction.dot(hit->normal()) > 0) {
            max = r->direction.dot(hit->normal());
          }

          Color3f cr = Li(scene, *r);
          return c + cr*hit->shape()->material()->reflectivity()*max;
        }
        return scene->backgroundColor();
    }

    std::string toString() const {
        return "WhittedIntegrator[]";
    }
private:
  int maxRecursion;
};

REGISTER_CLASS(WhittedIntegrator, "whitted");
