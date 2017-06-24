#ifndef __SHARPPHYSICS_BASE_H_
#define __SHARPPHYSICS_BASE_H_
#include <cmath>
#include <limits>
namespace SharpPhysics {
	typedef double spf;
	// Durations are in seconds.
	typedef spf Duration;
	// Timestamps are in seconds since the physics engine was started.
	typedef spf Timestamp;
	const spf NaN = std::numeric_limits<spf>::quiet_NaN();
	const spf Infinity = std::numeric_limits<spf>::infinity();
	const spf PI = std::acos(-1);
	struct Vec2d {
		spf x, y;
		static spf Dot(Vec2d a, Vec2d b) { 
			return a.x * b.x + a.y * b.y; 
		}
		spf SqrMagnitude() const { 
			return x*x + y*y; 
		}
		spf Magnitude() const { 
			return std::sqrt(SqrMagnitude()); 
		}
		Vec2d Normalized() const { 
			return (x == 0 && y == 0) ? Vec2d::Zero : *this * (1.0 / Magnitude()); 
		}
		Vec2d &operator+= (Vec2d a) { 
			x += a.x; y += a.y; return *this; 
		}
		Vec2d operator* (spf n) const {
			return Vec2d{ x * n, y*n }; 
		}
		Vec2d operator+ (Vec2d b) const { 
			return Vec2d{ x + b.x, y + b.y }; 
		}
		Vec2d operator- (Vec2d b) const { 
			return Vec2d{ x - b.x, y - b.y }; 
		}
		Vec2d operator-() const { 
			return Vec2d{ -x, -y }; 
		}
		static const Vec2d Zero;
		static const Vec2d NaN;
		bool IsNaN() const {
			return std::isnan(x);
		}
	};
	typedef Vec2d Point2d;
struct LineSeg {
	Point2d a, b;
	Vec2d GetDelta() const {
		return b - a; 
	}
};

}
#endif //__SHARPPHYSICS_BASE_H_
