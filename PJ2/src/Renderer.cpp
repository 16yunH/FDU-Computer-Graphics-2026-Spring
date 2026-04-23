#include "Renderer.h"

#include "ArgParser.h"
#include "Camera.h"
#include "Image.h"
#include "Ray.h"
#include "VecUtils.h"

#include <limits>
#include <cstdlib>
#include <cmath>


Renderer::Renderer(const ArgParser &args) :
    _args(args),
    _scene(args.input_file)
{
}

void
Renderer::Render()
{
    int w = _args.width;
    int h = _args.height;

    if (_args.filter) {
        int scale = 3;
        int sw = w * scale;
        int sh = h * scale;
        Image simage(sw, sh);
        Image snimage(sw, sh);
        Image sdimage(sw, sh);

        Camera* cam = _scene.getCamera();
        for (int y = 0; y < sh; ++y) {
            float ndcy = 2 * (y / (sh - 1.0f)) - 1.0f;
            for (int x = 0; x < sw; ++x) {
                float ndcx = 2 * (x / (sw - 1.0f)) - 1.0f;

                Vector3f color(0, 0, 0);
                Vector3f normalAcc(0, 0, 0);
                float depthAcc = 0;
                int samples = 16;

                for (int s = 0; s < samples; ++s) {
                    float jx = _args.jitter ? ((float)rand() / RAND_MAX - 0.5f) / sw : 0;
                    float jy = _args.jitter ? ((float)rand() / RAND_MAX - 0.5f) / sh : 0;

                    Ray r = cam->generateRay(Vector2f(ndcx + jx, ndcy + jy));
                    Hit hit;
                    color += traceRay(r, cam->getTMin(), _args.bounces, hit);
                    normalAcc += (hit.getNormal() + 1.0f) / 2.0f;
                    float range = (_args.depth_max - _args.depth_min);
                    if (range) {
                        float depth = (hit.t < std::numeric_limits<float>::max()) ?
                            (hit.t - _args.depth_min) / range : 0.0f;
                        depthAcc += depth;
                    }
                }

                simage.setPixel(x, y, color / samples);
                snimage.setPixel(x, y, normalAcc / samples);
                float range = (_args.depth_max - _args.depth_min);
                if (range) {
                    sdimage.setPixel(x, y, Vector3f(depthAcc / samples));
                }
            }
        }

        Image image(w, h);
        Image nimage(w, h);
        Image dimage(w, h);

        float kernel[3][3] = {
            {1.0f/16, 2.0f/16, 1.0f/16},
            {2.0f/16, 4.0f/16, 2.0f/16},
            {1.0f/16, 2.0f/16, 1.0f/16}
        };

        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                Vector3f color(0, 0, 0);
                Vector3f normal(0, 0, 0);
                Vector3f depth(0, 0, 0);
                for (int ky = 0; ky < scale; ++ky) {
                    for (int kx = 0; kx < scale; ++kx) {
                        int sx = x * scale + kx;
                        int sy = y * scale + ky;
                        color += kernel[ky][kx] * simage.getPixel(sx, sy);
                        normal += kernel[ky][kx] * snimage.getPixel(sx, sy);
                        depth += kernel[ky][kx] * sdimage.getPixel(sx, sy);
                    }
                }
                image.setPixel(x, y, color);
                nimage.setPixel(x, y, normal);
                dimage.setPixel(x, y, depth);
            }
        }

        if (_args.output_file.size()) {
            image.savePNG(_args.output_file);
        }
        if (_args.depth_file.size()) {
            dimage.savePNG(_args.depth_file);
        }
        if (_args.normals_file.size()) {
            nimage.savePNG(_args.normals_file);
        }
        return;
    }

    Image image(w, h);
    Image nimage(w, h);
    Image dimage(w, h);

    Camera* cam = _scene.getCamera();
    for (int y = 0; y < h; ++y) {
        float ndcy = 2 * (y / (h - 1.0f)) - 1.0f;
        for (int x = 0; x < w; ++x) {
            float ndcx = 2 * (x / (w - 1.0f)) - 1.0f;

            if (_args.jitter) {
                Vector3f color(0, 0, 0);
                Vector3f normalAcc(0, 0, 0);
                float depthAcc = 0;
                int samples = 16;
                for (int s = 0; s < samples; ++s) {
                    float jx = ((float)rand() / RAND_MAX - 0.5f) / w;
                    float jy = ((float)rand() / RAND_MAX - 0.5f) / h;
                    Ray r = cam->generateRay(Vector2f(ndcx + jx, ndcy + jy));
                    Hit hit;
                    color += traceRay(r, cam->getTMin(), _args.bounces, hit);
                    normalAcc += (hit.getNormal() + 1.0f) / 2.0f;
                    float range = (_args.depth_max - _args.depth_min);
                    if (range) {
                        float depth = (hit.t < std::numeric_limits<float>::max()) ?
                            (hit.t - _args.depth_min) / range : 0.0f;
                        depthAcc += depth;
                    }
                }
                image.setPixel(x, y, color / samples);
                nimage.setPixel(x, y, normalAcc / samples);
                float range = (_args.depth_max - _args.depth_min);
                if (range) {
                    dimage.setPixel(x, y, Vector3f(depthAcc / samples));
                }
            } else {
                Ray r = cam->generateRay(Vector2f(ndcx, ndcy));
                Hit hit;
                Vector3f color = traceRay(r, cam->getTMin(), _args.bounces, hit);

                image.setPixel(x, y, color);
                nimage.setPixel(x, y, (hit.getNormal() + 1.0f) / 2.0f);
                float range = (_args.depth_max - _args.depth_min);
                if (range) {
                    float depth = (hit.t < std::numeric_limits<float>::max()) ?
                        (hit.t - _args.depth_min) / range : 0.0f;
                    dimage.setPixel(x, y, Vector3f(depth));
                }
            }
        }
    }

    if (_args.output_file.size()) {
        image.savePNG(_args.output_file);
    }
    if (_args.depth_file.size()) {
        dimage.savePNG(_args.depth_file);
    }
    if (_args.normals_file.size()) {
        nimage.savePNG(_args.normals_file);
    }
}



Vector3f
Renderer::traceRay(const Ray &r,
    float tmin,
    int bounces,
    Hit &h) const
{
    if (_scene.getGroup()->intersect(r, tmin, h)) {
        Vector3f color = _scene.getAmbientLight() * h.getMaterial()->getDiffuseColor();

        for (int i = 0; i < _scene.getNumLights(); ++i) {
            Vector3f tolight, intensity;
            float distToLight;
            _scene.getLight(i)->getIllumination(r.pointAtParameter(h.getT()), tolight, intensity, distToLight);

            if (_args.shadows) {
                Ray shadowRay(r.pointAtParameter(h.getT()), tolight);
                Hit shadowHit;
                if (_scene.getGroup()->intersect(shadowRay, 1e-4, shadowHit)) {
                    if (shadowHit.getT() < distToLight) continue;
                }
            }

            color += h.getMaterial()->shade(r, h, tolight, intensity);
        }

        if (bounces > 0) {
            Vector3f N = h.getNormal().normalized();
            Vector3f V = r.getDirection().normalized();
            Vector3f R = V - 2.0f * Vector3f::dot(V, N) * N;
            Ray reflectRay(r.pointAtParameter(h.getT()), R);
            Hit reflectHit;
            Vector3f reflectColor = traceRay(reflectRay, 1e-4, bounces - 1, reflectHit);
            color += h.getMaterial()->getSpecularColor() * reflectColor;
        }

        return color;
    } else {
        return _scene.getBackgroundColor(r.getDirection());
    }
}

