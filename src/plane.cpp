#include "plane.h"

Plane::Plane()
{
}

Plane::Plane(const PropertyList &propList)
{
    m_position = propList.getPoint("position",Point3f(0,0,0));
    m_normal = propList.getVector("direction",Point3f(0,0,1));
}

Plane::~Plane()
{
}

bool Plane::intersect(const Ray& ray, Hit& hit) const
{
    /// TODO
    Point3f o = ray.origin;
    Point3f a = m_position;
    Vector3f d = ray.direction;
    Vector3f n = m_normal;

    float t = (n.dot(a)-n.dot(o))/(n.dot(d));

    if (std::isinf(t) || t < 0) {
      return false;
    }

    hit.setShape(this);
    hit.setT(t);
    hit.setNormal(n.norm());
    return true;
}

REGISTER_CLASS(Plane, "plane")
