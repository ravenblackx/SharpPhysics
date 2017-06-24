#ifndef __SHARPPHYSICS_BODY_H_
#define __SHARPPHYSICS_BODY_H_
#include "Base.h"
#include <memory>

namespace SharpPhysics {
	typedef const char *const BodyType;
	typedef int BodyID;
	// ExtraData can be subclassed for bodies that want more data attached.
	class ExtraData {};

	// A Body represents an object with position, velocity, friction and mass.
	// It's only useful for subclassing.
	class Body {
	public:
		Body(const Body &src) = default;
		Body(BodyID id, std::shared_ptr<ExtraData> e, Point2d pos, spf fric, spf mas) : ID(id), extra(e), position(pos), friction(fric), mass(mas), velocity(Vec2d::Zero), stopped(true) {}

		// Override GetType with a function that returns a static const string;
		// this makes dynamic type comparison easy. See class Line below for example.
		virtual BodyType GetType() const = 0;

		// CopyAfterDuration returns a new Body * that's the same as this body, but with
		// its position and velocity updated as if its current velocity and
		// acceleration had been applied for duration t.
		virtual std::unique_ptr<Body> CopyAfterDuration(Duration d) const = 0;

		// TimeUntilCollide returns the time at which this body and 'other' will
		// collide, assuming neither experiences any additional impulses. Setting
		// a maxtime can allow for early exit if the objects' trajectories only
		// go near each other after more than maxtime.
		// Returns NaN if the objects would not collide within maxtime.
		virtual Duration TimeUntilCollide(const Body &other, Duration maxtime = Infinity) const = 0;

		// ApplyCollision changes the velocities of two colliding objects,
		// according to conservation of momentum.
		virtual void ApplyCollision(Body *other) = 0;

		// IsTouchingPointAt returns true if the body would overlap with point
		// p after t, assuming no additional impulses.
		virtual bool IsTouchingPointAt(Duration t, Point2d p) const = 0;

		void Stop() { stopped = true; velocity = Vec2d::Zero; }
		bool IsStopped() const { return stopped; }
		bool IsTangible() const { return !std::isnan(mass); }
		const Point2d &Position() const { return position; }
		void SetPosition(const Point2d &pos) { position = pos; }
		const Vec2d &Velocity() const { return velocity; }
		void SetVelocity(const Vec2d &v) { velocity = v; }
		Point2d PositionAfterDuration(Duration t) const { return Position() + Velocity() * t + Acceleration() * (t * t / 2); }
		Vec2d VelocityAfterDuration(Duration t) const { return Velocity() + Acceleration() * t; }
		Vec2d Acceleration() const { return velocity.Normalized() * -Friction(); }
		const spf &Friction() const { return friction; }
		const spf &Mass() const { return mass; }
		Duration TimeUntilStop() const { return Velocity().Magnitude() / Friction(); }
		void AddVelocity(Vec2d add) { velocity += add; stopped = false; }
		ExtraData *Extra() const { return extra.get(); }
		BodyID ID;
	protected:
		std::shared_ptr<ExtraData> extra;
		bool stopped;
		Point2d position;
		Vec2d velocity;
		spf friction, mass;  // friction is 1/f, so zero is infinite friction.
	};

	// A Line is a static Body with infinite mass.
	class Line : public Body {
	public:
		Line(const Line &src) = default;
		Line(BodyID id, std::shared_ptr<ExtraData> e, const Point2d &start, const Point2d &end) : Body(id, e, start, 0.0, Infinity), b(end) {}
		BodyType GetType() const override { return Type; }
		static BodyType Type;
		std::unique_ptr<Body> CopyAfterDuration(Duration t) const override;
		Duration TimeUntilCollide(const Body &other, Duration maxtime = Infinity) const override { return NaN; };
		void ApplyCollision(Body *other) override {}
		bool IsTouchingPointAt(Duration t, Point2d p) const override { return false; }
		Vec2d Normal() const { return Vec2d{ position.y - b.y, b.x - position.x }.Normalized(); }
		LineSeg LinePos() const { return LineSeg{ position, b }; }
	protected:
		Point2d b;
	};

	// A Circle is the main dynamic body type.
	class Circle : public Body {
	public:
		Circle(const Circle &src, Duration d);
		Circle(BodyID id, std::shared_ptr<ExtraData> e, const Point2d &pos, spf r, spf fric = 0.0, spf mas = Infinity) : Body(id, e, pos, fric, mas), radius(r) {}
		BodyType GetType() const override { return Type; }
		static BodyType Type;
		std::unique_ptr<Body> CopyAfterDuration(Duration t) const override;
		Duration TimeUntilCollide(const Body &other, Duration maxtime = Infinity) const override;
		void ApplyCollision(Body *other) override;
		bool IsTouchingPointAt(Duration t, Point2d p) const override;

		const spf &Radius() const { return radius; }
		Duration TimeUntilCollide(const Circle &other, Duration maxtime = Infinity) const;
		Duration TimeUntilCollide(const Line &other, Duration maxtime = Infinity) const;
		Duration TimeUntilCollide(const Point2d &point, Duration maxtime = Infinity) const;
		Vec2d CollisionDir(const Body &other) const;
		Vec2d CollisionDir(const Circle &other) const;
		Vec2d CollisionDir(const Line &other) const;
	protected:
		spf radius;
	};
}
#endif // __SHARPPHYSICS_BODY_H_
