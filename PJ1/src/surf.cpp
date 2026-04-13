#include "surf.h"
#include "vertexrecorder.h"
#include <algorithm>
#include <cmath>
using namespace std;

namespace
{
    const float c_pi = 3.14159265358979323846f;

    inline bool isNearZero(const Vector3f &v)
    {
        const float eps = 1e-12f;
        return v.absSquared() < eps;
    }

    inline float clampSigned(float x)
    {
        return std::max(-1.0f, std::min(1.0f, x));
    }

    static bool approxPoint(const Vector3f &a, const Vector3f &b, float eps = 1e-6f)
    {
        return (a - b).absSquared() <= eps * eps;
    }

    static Vector3f rotateAroundAxis(const Vector3f &v, const Vector3f &axisUnit, float angle)
    {
        const float c = cosf(angle);
        const float s = sinf(angle);
        return v * c + Vector3f::cross(axisUnit, v) * s + axisUnit * (Vector3f::dot(axisUnit, v) * (1.0f - c));
    }

    static void appendTriangleStripFaces(
        unsigned ringCount,
        unsigned ringSize,
        vector<Tup3u> &faces)
    {
        if (ringCount < 2 || ringSize < 2)
            return;

        faces.reserve((ringCount - 1) * (ringSize - 1) * 2);

        for (unsigned r = 0; r + 1 < ringCount; ++r)
        {
            const unsigned base0 = r * ringSize;
            const unsigned base1 = (r + 1) * ringSize;
            for (unsigned i = 0; i + 1 < ringSize; ++i)
            {
                const unsigned A = base0 + i;
                const unsigned B = base1 + i;
                const unsigned C = base0 + i + 1;
                const unsigned D = base1 + i + 1;

                faces.push_back(Tup3u(A, C, B));
                faces.push_back(Tup3u(C, D, B));
            }
        }
    }

    // We're only implenting swept surfaces where the profile curve is
    // flat on the xy-plane.  This is a check function.
    static bool checkFlat(const Curve &profile)
    {
        for (unsigned i = 0; i < profile.size(); i++)
            if (profile[i].V[2] != 0.0 ||
                profile[i].T[2] != 0.0 ||
                profile[i].N[2] != 0.0)
                return false;

        return true;
    }
}

// DEBUG HELPER
Surface quad()
{
    Surface ret;
    ret.VV.push_back(Vector3f(-1, -1, 0));
    ret.VV.push_back(Vector3f(+1, -1, 0));
    ret.VV.push_back(Vector3f(+1, +1, 0));
    ret.VV.push_back(Vector3f(-1, +1, 0));

    ret.VN.push_back(Vector3f(0, 0, 1));
    ret.VN.push_back(Vector3f(0, 0, 1));
    ret.VN.push_back(Vector3f(0, 0, 1));
    ret.VN.push_back(Vector3f(0, 0, 1));

    ret.VF.push_back(Tup3u(0, 1, 2));
    ret.VF.push_back(Tup3u(0, 2, 3));
    return ret;
}

Surface makeSurfRev(const Curve &profile, unsigned steps)
{
    Surface surface;
    if (profile.size() < 2 || steps == 0)
        return surface;

    if (!checkFlat(profile))
    {
        cerr << "surfRev profile curve must be flat on xy plane." << endl;
        exit(0);
    }

    const unsigned ringSize = (unsigned)profile.size();
    const unsigned ringCount = steps + 1;

    surface.VV.reserve(ringSize * ringCount);
    surface.VN.reserve(ringSize * ringCount);

    for (unsigned s = 0; s <= steps; ++s)
    {
        const float theta = 2.0f * c_pi * float(s) / float(steps);
        const Matrix4f R = Matrix4f::rotateY(theta);
        const Matrix3f R3 = R.getSubmatrix3x3(0, 0);

        for (unsigned i = 0; i < ringSize; ++i)
        {
            const Vector4f vp(profile[i].V, 1.0f);
            const Vector3f pos = (R * vp).xyz();

            Vector3f normal = R3 * (-profile[i].N);
            if (isNearZero(normal))
                normal = Vector3f(1, 0, 0);
            normal = normal.normalized();

            surface.VV.push_back(pos);
            surface.VN.push_back(normal);
        }
    }

    appendTriangleStripFaces(ringCount, ringSize, surface.VF);

    return surface;
}

Surface makeGenCyl(const Curve &profile, const Curve &sweep)
{
    Surface surface;
    if (profile.size() < 2 || sweep.size() < 2)
        return surface;

    if (!checkFlat(profile))
    {
        cerr << "genCyl profile curve must be flat on xy plane." << endl;
        exit(0);
    }

    const unsigned ringSize = (unsigned)profile.size();
    const unsigned ringCount = (unsigned)sweep.size();

    vector<Vector3f> frameN(ringCount);
    vector<Vector3f> frameB(ringCount);
    vector<Vector3f> frameT(ringCount);
    vector<Vector3f> frameV(ringCount);

    for (unsigned j = 0; j < ringCount; ++j)
    {
        frameN[j] = sweep[j].N.normalized();
        frameB[j] = sweep[j].B.normalized();
        frameT[j] = sweep[j].T.normalized();
        frameV[j] = sweep[j].V;
    }

    const bool closedSweep =
        approxPoint(frameV.front(), frameV.back()) &&
        Vector3f::dot(frameT.front(), frameT.back()) > 0.999f;

    if (closedSweep && ringCount > 2)
    {
        const Vector3f n0 = frameN.front();
        const Vector3f n1 = frameN.back();
        const Vector3f axis = frameT.front();

        float alpha = acosf(clampSigned(Vector3f::dot(n1, n0)));
        const float sign = (Vector3f::dot(Vector3f::cross(n1, n0), axis) >= 0.0f) ? 1.0f : -1.0f;
        alpha *= sign;

        for (unsigned j = 0; j < ringCount; ++j)
        {
            const float t = float(j) / float(ringCount - 1);
            const float theta = alpha * t;

            const Vector3f axisJ = frameT[j];
            frameN[j] = rotateAroundAxis(frameN[j], axisJ, theta).normalized();
            frameB[j] = Vector3f::cross(axisJ, frameN[j]).normalized();
        }
    }

    surface.VV.reserve(ringSize * ringCount);
    surface.VN.reserve(ringSize * ringCount);

    for (unsigned j = 0; j < ringCount; ++j)
    {
        const Vector3f N = frameN[j];
        const Vector3f B = frameB[j];
        const Vector3f T = frameT[j];
        const Vector3f V = frameV[j];

        for (unsigned i = 0; i < ringSize; ++i)
        {
            const Vector3f p = profile[i].V;
            const Vector3f pWorld = V + p[0] * N + p[1] * B + p[2] * T;

            Vector3f n = profile[i].N;
            Vector3f nWorld = n[0] * N + n[1] * B + n[2] * T;
            if (isNearZero(nWorld))
                nWorld = N;

            surface.VV.push_back(pWorld);
            surface.VN.push_back(nWorld.normalized());
        }
    }

    appendTriangleStripFaces(ringCount, ringSize, surface.VF);

    return surface;
}

void recordSurface(const Surface &surface, VertexRecorder *recorder)
{
    const Vector3f WIRECOLOR(0.4f, 0.4f, 0.4f);
    for (int i = 0; i < (int)surface.VF.size(); i++)
    {
        recorder->record(surface.VV[surface.VF[i][0]], surface.VN[surface.VF[i][0]], WIRECOLOR);
        recorder->record(surface.VV[surface.VF[i][1]], surface.VN[surface.VF[i][1]], WIRECOLOR);
        recorder->record(surface.VV[surface.VF[i][2]], surface.VN[surface.VF[i][2]], WIRECOLOR);
    }
}

void recordNormals(const Surface &surface, VertexRecorder *recorder, float len)
{
    const Vector3f NORMALCOLOR(0, 1, 1);
    for (int i = 0; i < (int)surface.VV.size(); i++)
    {
        recorder->record_poscolor(surface.VV[i], NORMALCOLOR);
        recorder->record_poscolor(surface.VV[i] + surface.VN[i] * len, NORMALCOLOR);
    }
}

void outputObjFile(ostream &out, const Surface &surface)
{

    for (int i = 0; i < (int)surface.VV.size(); i++)
        out << "v  "
            << surface.VV[i][0] << " "
            << surface.VV[i][1] << " "
            << surface.VV[i][2] << endl;

    for (int i = 0; i < (int)surface.VN.size(); i++)
        out << "vn "
            << surface.VN[i][0] << " "
            << surface.VN[i][1] << " "
            << surface.VN[i][2] << endl;

    out << "vt  0 0 0" << endl;

    for (int i = 0; i < (int)surface.VF.size(); i++)
    {
        out << "f  ";
        for (unsigned j = 0; j < 3; j++)
        {
            unsigned a = surface.VF[i][j] + 1;
            out << a << "/" << "1" << "/" << a << " ";
        }
        out << endl;
    }
}
