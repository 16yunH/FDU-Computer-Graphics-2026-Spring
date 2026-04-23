#include "Object3D.h"
#include "VecUtils.h"

bool Sphere::intersect(const Ray &r, float tmin, Hit &h) const
{
    // BEGIN STARTER

    // We provide sphere intersection code for you.
    // You should model other intersection implementations after this one.

    // Locate intersection point ( 2 pts )
    const Vector3f &rayOrigin = r.getOrigin(); //Ray origin in the world coordinate
    const Vector3f &dir = r.getDirection();

    Vector3f origin = rayOrigin - _center;      //Ray origin in the sphere coordinate

    float a = dir.absSquared();
    float b = 2 * Vector3f::dot(dir, origin);
    float c = origin.absSquared() - _radius * _radius;

    // no intersection
    if (b * b - 4 * a * c < 0) {
        return false;
    }

    float d = sqrt(b * b - 4 * a * c);

    float tplus = (-b + d) / (2.0f*a);
    float tminus = (-b - d) / (2.0f*a);

    // the two intersections are at the camera back
    if ((tplus < tmin) && (tminus < tmin)) {
        return false;
    }

    float t = 10000;
    // the two intersections are at the camera front
    if (tminus > tmin) {
        t = tminus;
    }

    // one intersection at the front. one at the back 
    if ((tplus > tmin) && (tminus < tmin)) {
        t = tplus;
    }

    if (t < h.getT()) {
        Vector3f normal = r.pointAtParameter(t) - _center;
        normal = normal.normalized();
        h.set(t, this->material, normal);
        return true;
    }
    // END STARTER
    return false;
}

// Add object to group
void Group::addObject(Object3D *obj) {
    m_members.push_back(obj);
}

// Return number of objects in group
int Group::getGroupSize() const {
    return (int)m_members.size();
}

bool Group::intersect(const Ray &r, float tmin, Hit &h) const
{
    // BEGIN STARTER
    // we implemented this for you
    bool hit = false;
    for (Object3D* o : m_members) {
        if (o->intersect(r, tmin, h)) {
            hit = true;
        }
    }
    return hit;
    // END STARTER
}


Plane::Plane(const Vector3f &normal, float d, Material *m) : Object3D(m), _normal(normal.normalized()), _d(d) {
}
bool Plane::intersect(const Ray &r, float tmin, Hit &h) const
{
    float denom = Vector3f::dot(_normal, r.getDirection());
    if (fabs(denom) < 1e-6) return false;
    float t = (_d - Vector3f::dot(_normal, r.getOrigin())) / denom;
    if (t < tmin || t >= h.getT()) return false;
    Vector3f normal = _normal;
    if (denom > 0) normal = -normal;
    h.set(t, material, normal);
    return true;
}
bool Triangle::intersect(const Ray &r, float tmin, Hit &h) const 
{
    Vector3f E1 = _v[1] - _v[0];
    Vector3f E2 = _v[2] - _v[0];
    Vector3f S = r.getOrigin() - _v[0];
    Vector3f D = r.getDirection();

    Vector3f SxE1 = Vector3f::cross(S, E1);
    Vector3f DxE2 = Vector3f::cross(D, E2);
    float denom = Vector3f::dot(DxE2, E1);
    if (fabs(denom) < 1e-8) return false;

    float invDenom = 1.0f / denom;
    float t = Vector3f::dot(SxE1, E2) * invDenom;
    float b1 = Vector3f::dot(DxE2, S) * invDenom;
    float b2 = Vector3f::dot(SxE1, D) * invDenom;
    float b0 = 1.0f - b1 - b2;

    if (b0 < 0 || b1 < 0 || b2 < 0) return false;
    if (t < tmin || t >= h.getT()) return false;

    Vector3f normal = (b0 * _normals[0] + b1 * _normals[1] + b2 * _normals[2]).normalized();
    h.set(t, material, normal);
    return true;
}


Transform::Transform(const Matrix4f &m,
    Object3D *obj) : _object(obj), _matrix(m), _inverseMatrix(m.inverse()), _inverseTransposeMatrix(m.inverse().transposed()) {
}
bool Transform::intersect(const Ray &r, float tmin, Hit &h) const
{
    Vector3f origin = VecUtils::transformPoint(_inverseMatrix, r.getOrigin());
    Vector3f dir = VecUtils::transformDirection(_inverseMatrix, r.getDirection());
    Ray localRay(origin, dir);

    Hit localHit;
    if (!_object->intersect(localRay, tmin, localHit)) return false;

    float t = localHit.getT();
    Vector3f worldNormal = VecUtils::transformDirection(_inverseTransposeMatrix, localHit.getNormal()).normalized();

    if (t >= h.getT()) return false;
    h.set(t, localHit.getMaterial(), worldNormal);
    return true;
}