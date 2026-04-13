#include "curve.h"
#include "vertexrecorder.h"
#include <algorithm>
#include <cmath>
using namespace std;

const float c_pi = 3.14159265358979323846f;

namespace
{
	// Approximately equal to.  We don't want to use == because of
	// precision issues with floating point.
	inline bool approx(const Vector3f &lhs, const Vector3f &rhs)
	{
		const float eps = 1e-8f;
		return (lhs - rhs).absSquared() < eps;
	}

	inline float clamp01(float x)
	{
		return std::max(0.0f, std::min(1.0f, x));
	}

	inline bool isNearZero(const Vector3f &v)
	{
		const float eps = 1e-12f;
		return v.absSquared() < eps;
	}

	void buildCurveFrame(Curve &curve, bool planarXY)
	{
		if (curve.empty())
			return;

		for (size_t i = 0; i < curve.size(); ++i)
		{
			if (isNearZero(curve[i].T))
			{
				if (i > 0)
					curve[i].T = curve[i - 1].T;
				else
					curve[i].T = Vector3f(1, 0, 0);
			}
			curve[i].T = curve[i].T.normalized();
		}

		if (planarXY)
		{
			const Vector3f Bup(0, 0, 1);
			for (size_t i = 0; i < curve.size(); ++i)
			{
				curve[i].B = Bup;
				Vector3f n = Vector3f::cross(Bup, curve[i].T);
				if (isNearZero(n))
				{
					n = (i > 0) ? curve[i - 1].N : Vector3f(1, 0, 0);
				}
				curve[i].N = n.normalized();
			}
			return;
		}

		Vector3f t0 = curve[0].T;
		Vector3f ref(0, 0, 1);
		if (isNearZero(Vector3f::cross(ref, t0)))
			ref = Vector3f(1, 0, 0);

		curve[0].B = Vector3f::cross(ref, t0).normalized();
		curve[0].N = Vector3f::cross(curve[0].B, t0).normalized();
		curve[0].B = Vector3f::cross(t0, curve[0].N).normalized();

		for (size_t i = 1; i < curve.size(); ++i)
		{
			Vector3f n = Vector3f::cross(curve[i - 1].B, curve[i].T);
			if (isNearZero(n))
				n = curve[i - 1].N;
			curve[i].N = n.normalized();

			Vector3f b = Vector3f::cross(curve[i].T, curve[i].N);
			if (isNearZero(b))
				b = curve[i - 1].B;
			curve[i].B = b.normalized();
		}
	}

	void evalBezierPiece(const Vector3f &p0,
						 const Vector3f &p1,
						 const Vector3f &p2,
						 const Vector3f &p3,
						 unsigned steps,
						 bool skipFirst,
						 Curve &out)
	{
		const unsigned start = skipFirst ? 1u : 0u;
		for (unsigned k = start; k <= steps; ++k)
		{
			const float t = (steps == 0) ? 0.0f : clamp01(float(k) / float(steps));
			const float omt = 1.0f - t;

			const Vector3f v =
				(omt * omt * omt) * p0 +
				(3.0f * omt * omt * t) * p1 +
				(3.0f * omt * t * t) * p2 +
				(t * t * t) * p3;

			const Vector3f d1 =
				(3.0f * omt * omt) * (p1 - p0) +
				(6.0f * omt * t) * (p2 - p1) +
				(3.0f * t * t) * (p3 - p2);

			CurvePoint cp;
			cp.V = v;
			cp.T = d1;
			cp.N = Vector3f(0, 0, 0);
			cp.B = Vector3f(0, 0, 0);
			out.push_back(cp);
		}
	}

}

Curve evalBezier(const vector<Vector3f> &P, unsigned steps)
{
	// Check
	if (P.size() < 4 || P.size() % 3 != 1)
	{
		cerr << "evalBezier must be called with 3n+1 control points." << endl;
		exit(0);
	}

	const unsigned pieces = (unsigned(P.size()) - 1u) / 3u;
	Curve result;
	result.reserve(pieces * std::max(steps, 1u) + 1u);

	for (unsigned seg = 0; seg < pieces; ++seg)
	{
		const unsigned i = 3u * seg;
		evalBezierPiece(P[i + 0], P[i + 1], P[i + 2], P[i + 3], steps, seg > 0, result);
	}

	bool planarXY = true;
	for (size_t i = 0; i < P.size(); ++i)
	{
		if (fabs(P[i][2]) > 1e-6f)
		{
			planarXY = false;
			break;
		}
	}

	buildCurveFrame(result, planarXY);
	return result;
}

Curve evalBspline(const vector<Vector3f> &P, unsigned steps)
{
	// Check
	if (P.size() < 4)
	{
		cerr << "evalBspline must be called with 4 or more control points." << endl;
		exit(0);
	}

	vector<Vector3f> bezierCP;
	const unsigned segCount = unsigned(P.size()) - 3u;
	bezierCP.reserve(3u * segCount + 1u);

	for (unsigned i = 0; i < segCount; ++i)
	{
		const Vector3f &p0 = P[i + 0];
		const Vector3f &p1 = P[i + 1];
		const Vector3f &p2 = P[i + 2];
		const Vector3f &p3 = P[i + 3];

		const Vector3f b0 = (p0 + 4.0f * p1 + p2) / 6.0f;
		const Vector3f b1 = (4.0f * p1 + 2.0f * p2) / 6.0f;
		const Vector3f b2 = (2.0f * p1 + 4.0f * p2) / 6.0f;
		const Vector3f b3 = (p1 + 4.0f * p2 + p3) / 6.0f;

		if (i == 0)
		{
			bezierCP.push_back(b0);
			bezierCP.push_back(b1);
			bezierCP.push_back(b2);
			bezierCP.push_back(b3);
		}
		else
		{
			bezierCP.push_back(b1);
			bezierCP.push_back(b2);
			bezierCP.push_back(b3);
		}
	}

	return evalBezier(bezierCP, steps);
}

Curve evalCircle(float radius, unsigned steps)
{
	// This is a sample function on how to properly initialize a Curve
	// (which is a vector< CurvePoint >).

	// Preallocate a curve with steps+1 CurvePoints
	Curve R(steps + 1);

	// Fill it in counterclockwise
	for (unsigned i = 0; i <= steps; ++i)
	{
		// step from 0 to 2pi
		float t = 2.0f * c_pi * float(i) / steps;

		// Initialize position
		// We're pivoting counterclockwise around the y-axis
		R[i].V = radius * Vector3f(cos(t), sin(t), 0);

		// Tangent vector is first derivative
		R[i].T = Vector3f(-sin(t), cos(t), 0);

		// Normal vector is second derivative
		R[i].N = Vector3f(-cos(t), -sin(t), 0);

		// Finally, binormal is facing up.
		R[i].B = Vector3f(0, 0, 1);
	}

	return R;
}

void recordCurve(const Curve &curve, VertexRecorder *recorder)
{
	const Vector3f WHITE(1, 1, 1);
	for (int i = 0; i < (int)curve.size() - 1; ++i)
	{
		recorder->record_poscolor(curve[i].V, WHITE);
		recorder->record_poscolor(curve[i + 1].V, WHITE);
	}
}
void recordCurveFrames(const Curve &curve, VertexRecorder *recorder, float framesize)
{
	Matrix4f T;
	const Vector3f RED(1, 0, 0);
	const Vector3f GREEN(0, 1, 0);
	const Vector3f BLUE(0, 0, 1);

	const Vector4f ORGN(0, 0, 0, 1);
	const Vector4f AXISX(framesize, 0, 0, 1);
	const Vector4f AXISY(0, framesize, 0, 1);
	const Vector4f AXISZ(0, 0, framesize, 1);

	for (int i = 0; i < (int)curve.size(); ++i)
	{
		T.setCol(0, Vector4f(curve[i].N, 0));
		T.setCol(1, Vector4f(curve[i].B, 0));
		T.setCol(2, Vector4f(curve[i].T, 0));
		T.setCol(3, Vector4f(curve[i].V, 1));

		// Transform orthogonal frames into model space
		Vector4f MORGN = T * ORGN;
		Vector4f MAXISX = T * AXISX;
		Vector4f MAXISY = T * AXISY;
		Vector4f MAXISZ = T * AXISZ;

		// Record in model space
		recorder->record_poscolor(MORGN.xyz(), RED);
		recorder->record_poscolor(MAXISX.xyz(), RED);

		recorder->record_poscolor(MORGN.xyz(), GREEN);
		recorder->record_poscolor(MAXISY.xyz(), GREEN);

		recorder->record_poscolor(MORGN.xyz(), BLUE);
		recorder->record_poscolor(MAXISZ.xyz(), BLUE);
	}
}
