#include <algorithm>
#include "Base.h"
#include "math.h"
#include "poly.h"

namespace SharpPhysics {
	const Vec2d Vec2d::Zero{ 0.0, 0.0 };
	const Vec2d Vec2d::NaN{ SharpPhysics::NaN, SharpPhysics::NaN };

	spf LineSegsDistanceSquared(const LineSeg &l1, const LineSeg &l2)
	{
		if (LineSegsIntersect(l1, l2)) { return 0; }
		return std::min(
			std::min(
			PointLineSegDistanceSquared(l1.a, l2),
			PointLineSegDistanceSquared(l1.b, l2)),
			std::min(
			PointLineSegDistanceSquared(l2.a, l1),
			PointLineSegDistanceSquared(l2.b, l1)));
	}

	spf PointLineSegDistanceSquared(const Point2d &p, const LineSeg &l)
	{
		const Vec2d ld = l.GetDelta();
		if (ld.x == 0 && ld.y == 0)
		{
			// Line is a point, return point distance.
			return (l.a - p).SqrMagnitude();
		}
		spf t = ((p.x - l.a.x) * ld.x + (p.y - l.a.y) * ld.y) / (ld.x * ld.x + ld.y * ld.y);
		if (t < 0)
		{
			// Point is closest to l.a.
			return (l.a - p).SqrMagnitude();
		}
		if (t > 1)
		{
			// Point is closest to l.b.
			return (l.b - p).SqrMagnitude();
		}
		Vec2d near{ l.a.x + t * ld.x, l.a.y + t * ld.y };
		return (near - p).SqrMagnitude();
	}

	bool LineSegsIntersect(const LineSeg &l1, const LineSeg &l2)
	{
		Vec2d l1d = l1.GetDelta();
		Vec2d l2d = l2.GetDelta();
		spf delta = l2d.x * l1d.y - l2d.y * l1d.x;
		if (delta == 0) { return false; }  // parallel
		spf s = (l1d.x * (l2.a.y - l1.a.y) + l1d.y * (l1.a.x - l1.a.x)) / delta;
		spf t = (l2d.x * (l1.a.y - l2.a.y) + l2d.y * (l2.a.x - l2.a.x)) / (-delta);
		return (0 <= s && s <= 1) && (0 <= t && t <= 1);
	}

	spf SolveQuartic(spf a, spf b, spf c, spf d, spf e, bool only_inward)
	{
		double root[4];
		int n;
		if (a == 0) {
			n = Poly::SolveP3(root, c / b, d / b, e / b);
		}
		else {
			n = Poly::SolveP4(root, b / a, c / a, d / a, e / a);
		}
		if (n == 0) return NaN;
		auto InvalidateBadRoot = [=](double t) {
			if (t <= 0) return NaN;  // We don't care about collisions backwards in time!
			if (only_inward) {
				double grade = a * 4 * std::pow(t, 3) + b * 3 * std::pow(t, 2) + c * 2 * t + d;
				if (grade > 0) return NaN;  // They're moving apart, don't collide.
			}
			return t;
		};
		double best = std::min(InvalidateBadRoot(root[0]), InvalidateBadRoot(root[1]));
		if (n == 4) best = std::min(best, std::min(InvalidateBadRoot(root[2]), InvalidateBadRoot(root[3])));
		return best;
	}

	spf SolveQuadratic(spf a, spf b, spf c, bool only_inward) {
		spf t1 = (-b + std::sqrt(b*b - 4 * a*c)) / (4 * a*a);
		spf t2 = (-b - std::sqrt(b*b - 4 * a*c)) / (4 * a*a);
		auto InvalidateBadRoot = [=](spf t) {
			if (t <= 0) return NaN; // No collisions backwards in time.
			if (only_inward) {
				double grade = a * 2 * t + b;
				if (grade > 0) return NaN;  // They're moving apart, don't collide.
			}
			return t;
		};
		return std::min(InvalidateBadRoot(t1), InvalidateBadRoot(t2));
	}
}
