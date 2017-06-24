#ifndef __SHARPPHYSICS_MATH_H_
#define __SHARPPHYSICS_MATH_H_

#include "Base.h"

namespace SharpPhysics {
	// Returns the square of the shortest distance between two line segments.
	// Returns zero if they intersect.
	spf LineSegsDistanceSquared(const LineSeg &l1, const LineSeg &l2);

	// Returns the square of the distance between a point and a line segment.
	spf PointLineSegDistanceSquared(const Point2d &p, const LineSeg &l);

	// Returns true if two line segments intersect.
	bool LineSegsIntersect(const LineSeg &l1, const LineSeg &l2);

	// Solves a quartic equation as used to determine the time of
	// collision between two moving, accelerating objects (or more
	// specifically, the time at which distance == radius+other_radius
	// between two moving, accelerating points). Solving the equation
	// produces multiple roots; any roots where time t is negative are
	// discarded, and if only_inward is true, any roots where the delta
	// is positive (which means the objects are actually moving apart)
	// are discarded. The smallest surviving root is returned, or, if
	// no roots remain, NaN is returned.
	spf SolveQuartic(spf a, spf b, spf c, spf d, spf e, bool only_inward);

	// Same as SolveQuartic but simpler.
	spf SolveQuadratic(spf a, spf b, spf c, bool only_inward);
}
#endif // __SHARPPHYSICS_MATH_H_
