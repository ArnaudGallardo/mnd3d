#include "sphere.h"
#include <iostream>

Sphere::Sphere(float radius)
    : m_radius(radius)
{
}

Sphere::Sphere(const PropertyList &propList)
{
    m_radius = propList.getFloat("radius",1.f);
    m_center = propList.getPoint("center",Point3f(0,0,0));
}

Sphere::~Sphere()
{
}

bool Sphere::intersect(const Ray& ray, Hit& hit) const
{
    /// TODO: compute ray-sphere intersection

    //throw RTException("Sphere::intersect not implemented yet.");

    Point3f o = ray.origin;
    Point3f center = m_center;
    Vector3f d = ray.direction;
    float r = m_radius;

    float a = d.squaredNorm();

    float b = 2*d.dot(o-center);

    float c = (o-center).dot(o-center) - r*r;

    float delta = b*b-4*a*c;

    if (delta>0) {
      hit.setShape(this);
      float res1 = (-b+sqrt(delta))/(2*a);
      float res2 = (-b-sqrt(delta))/(2*a);
      if (res1 > res2 && res2 > 0) {
        hit.setT(res2);

        //Calcul Point
        Point3f h = o+res2*d;
        Normal3f n = h-center;
        n.normalize();
        hit.setNormal(n);

      }
      else {
        if (res1 > 0) {
          hit.setT(res1);

          //Calcul Point
          Point3f h = o+res2*d;
          Normal3f n = h-center;

          n.normalize();
          hit.setNormal(n);
        }
      }
      return true;
    } else if (delta==0) {
      if (-b/(2*a)>0) {
        hit.setShape(this);
        //Calcul Point
        Point3f h = o+(-b/(2*a))*d;
        Normal3f n = h-center;

        n.normalize();
        hit.setNormal(n);
        hit.setT(-b/(2*a));
        return true;
      }
    }

    return false;
}

REGISTER_CLASS(Sphere, "sphere")
