#include <algorithm>
#include "Body.h"
#include "Math.h"

namespace SharpPhysics {
	BodyType Circle::Type = "Circle";
	BodyType Line::Type = "Line";

	std::unique_ptr<Body> Circle::CopyAfterDuration(Duration d) const {
		std::unique_ptr<Body> c(new Circle(*this));
		c->SetPosition(PositionAfterDuration(d));
		c->SetVelocity(VelocityAfterDuration(d));
		return c;
	}
	std::unique_ptr<Body> Line::CopyAfterDuration(Duration t) const {
		std::unique_ptr<Body> l(new Line(*this));
		return l;
	}

	Duration Circle::TimeUntilCollide(const Circle &other, Duration maxtime) const {
		Vec2d pos_maxt = PositionAfterDuration(maxtime);
		Vec2d other_pos_maxt = other.PositionAfterDuration(maxtime);
		spf distSquared = LineSegsDistanceSquared(LineSeg{ Position(), pos_maxt }, LineSeg{ other.Position(), other_pos_maxt });
		spf combined_radius_squared = std::pow(Radius() + other.Radius(), 2);
		if (distSquared > combined_radius_squared)
		{  // discs don't even cross paths in the given time range, so cheap no collision.
			return NaN;
		}
		Vec2d relvel = other.Velocity() - Velocity();
		Vec2d relpos = other.Position() - Position();
		Vec2d relaccel = other.Acceleration() - Acceleration();
		// DistAtTime = relpos + relvel * t + relaccel * t^2 / 2;
		// xt = rpx + rvx*t + rax/2*t^2;
		// yt = rpy + rvy*t + ray/2*t^2;
		// collision when xt^2 + yt^2 = (combined_radius)^2
		// xt^2 = rpx^2 + rpx*rvx*2*t + rpx*rax*t^2 + rvx^2*t^2 + rvx*rax*t^3 + rax^2/4*t^4;
		// (rax^2/4)t^4 + (rvx*rax)t^3 + (rvx^2 + rpx*rax)t^2 + (rpx*rvx*2)t + rpx^2
		// Collision when at^4 + bt^3 + ct^2 + dt + e = 0
		spf a = (std::pow(relaccel.x,2) + std::pow(relaccel.y,2)) / 4;
		spf b = (relvel.x * relaccel.x + relvel.y * relaccel.y);
		spf c = std::pow(relvel.x,2) + std::pow(relvel.y,2) + (relpos.x * relaccel.x + relpos.y * relaccel.y);
		spf d = (relpos.x * relvel.x + relpos.y * relvel.y) * 2;
		spf e = std::pow(relpos.x,2) + std::pow(relpos.y,2) - combined_radius_squared;
		spf t = SolveQuartic(a, b, c, d, e, other.IsTangible());
		if (t > maxtime) return NaN;
		return t;
	}

	Duration Circle::TimeUntilCollide(const Point2d &point, Duration maxtime) const {
		//TODO: collision with a point.
		return NaN;
	}

	Duration Circle::TimeUntilCollide(const Line &other, Duration maxtime) const {
		Vec2d pos_maxt = PositionAfterDuration(maxtime); 
		spf distSquared = LineSegsDistanceSquared(LineSeg{ Position(), pos_maxt }, other.LinePos());
		spf radius_squared = std::pow(Radius(), 2);
		if (distSquared > radius_squared) {
			return NaN;  // no contact in the given time range.
		}
		Vec2d other_normal = other.Normal();
		LineSeg other_line = other.LinePos();
		Vec2d other_dir = other_line.b - other_line.a;
		// The vector between a point on the line and a point off the line, projected into the normal, equals the distance from line to point.
		spf normalDist = Vec2d::Dot(Position() - other_line.a, other_normal);
		spf sign = signbit(normalDist) ? -1.0 : 1.0;
		if (std::abs(normalDist) > radius) {
			// We're not already overlapping the infiniline, so we should consider the main line collision first.
			spf normalVel = Vec2d::Dot(Velocity(), other_normal);
			spf normalAccel = Vec2d::Dot(Acceleration(), other_normal);
			spf collisionDist = normalDist + radius * sign;
			// 1/2a t^2 + v t + collisionDist = 0
			spf t = SolveQuadratic(normalAccel / 2, normalVel, collisionDist, other.IsTangible());
			Vec2d pos_collisiont = PositionAfterDuration(t);
			spf d1 = Vec2d::Dot(pos_collisiont - other_line.a, other_dir);
			spf d2 = Vec2d::Dot(pos_collisiont - other_line.b, other_dir);
			if (signbit(d1) != signbit(d2)) {
				// We're between the ends when we touch the infiniline - that means we also touched the lineseg, and we're done here.
				return t;
			}
		}
		return std::min(TimeUntilCollide(other_line.a, maxtime), TimeUntilCollide(other_line.b, maxtime));
	}

	Duration Circle::TimeUntilCollide(const Body &other, Duration maxtime) const {
		if (other.GetType() == Circle::Type) {
			return TimeUntilCollide(static_cast<const Circle &>(other), maxtime);
		}
		else if (other.GetType() == Line::Type) {
			return TimeUntilCollide(static_cast<const Line &>(other), maxtime);
		}
		throw "Unexpected collision type";
	}

	void Circle::ApplyCollision(Body *other) {
		Vec2d collision_dir = CollisionDir(*other);
		spf avi = Vec2d::Dot(Velocity(), collision_dir);
		if (std::isnan(other->Mass())) {
			// Other object is intangible.
			// TODO: Trigger things!
			return;
		}
		else if (std::isinf(other->Mass())) {
			velocity = velocity - collision_dir * (avi * 2);
		}
		else {
			spf bvi = Vec2d::Dot(other->Velocity(), collision_dir);
			spf massdiff = Mass() - other->Mass();
			spf combined_mass = Mass() + other->Mass();
			spf avo = (avi*massdiff + 2 * other->Mass() * bvi) / combined_mass;
			spf bvo = (-bvi*massdiff + 2 * Mass() * avi) / combined_mass;
			AddVelocity(collision_dir * (avo - avi));
			other->AddVelocity(collision_dir * (bvo - bvi));
		}
		// TODO: Trigger things.
	}
	bool Circle::IsTouchingPointAt(Duration t, Point2d p) const {
		Point2d c = PositionAfterDuration(t);
		return (p - c).SqrMagnitude() < std::pow(radius, 2);
	}
	Vec2d Circle::CollisionDir(const Body &other) const {
		if (other.GetType() == Circle::Type) {
			return CollisionDir(static_cast<const Circle &>(other));
		}
		else if (other.GetType() == Line::Type) {
			return CollisionDir(static_cast<const Line &>(other));
		}
		throw "Unexpected collision type";
	}
	Vec2d Circle::CollisionDir(const Circle &other) const {
		return (other.Position() - Position()).Normalized();
	}
	Vec2d Circle::CollisionDir(const Line &other) const {
		// TODO: point collision if it's the end of the line.
		return other.Normal();
	}
}
