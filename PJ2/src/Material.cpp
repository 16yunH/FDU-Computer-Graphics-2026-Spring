#include "Material.h"
#include <algorithm>
Vector3f Material::shade(const Ray &ray,
    const Hit &hit,
    const Vector3f &dirToLight,
    const Vector3f &lightIntensity)
{
    Vector3f N = hit.getNormal().normalized();
    Vector3f L = dirToLight.normalized();

    float NdL = Vector3f::dot(N, L);
    float diff = std::max(NdL, 0.0f);
    Vector3f diffuse = diff * _diffuseColor * lightIntensity;

    Vector3f R = (2.0f * NdL * N - L).normalized();
    Vector3f V = (-ray.getDirection()).normalized();
    float RdV = std::max(Vector3f::dot(R, V), 0.0f);
    Vector3f specular = pow(RdV, _shininess) * _specularColor * lightIntensity;

    return diffuse + specular;
}
