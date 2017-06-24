#ifndef __SHARPPHYSICS_SYSTEM_H_
#define __SHARPPHYSICS_SYSTEM_H_

#include <functional>
#include <map>
#include <set>
#include <memory>
#include <vector>
#include "Body.h"

namespace SharpPhysics {
	typedef spf Timestamp;
	typedef bool Applied;
	typedef std::function<void(Body *body)> BodyFunc;
	typedef std::function<void(Duration t, Body *body)> DurationBodyFunc;

	// A Snapshot captures a momentary state of the system.
	class Snapshot {
	public:
		std::map<BodyID, std::unique_ptr<Body>> bodies;
		void ForEach(BodyFunc func) const;

		// Careful! GetBody doesn't validate that a body with the given id exists.
		// You'll just crash if you try to operate on a body that doesn't exist in
		// the snapshot.
		Body *GetBody(BodyID id) const { auto f = bodies.find(id); return f->second.get(); }

		// When creating a new snapshot, populate it from the previous snapshot
		// and a duration. Typically, you do this at the moment of a collision
		// or external impulse, then for the bodies involved in the collision
		// or impulse, update their velocity appropriately in this new snapshot.
		void FillFromPrevious(const Snapshot &prev, Duration t);
	};

	// An Action is typically a lambda that operates on a snapshot, eg.
	// one possible Action might be [body_id](Snapshot *ss) {
	//   ss->GetBody(body_id)->Stop();
	// }
	// This action would cause the body identified by body_id to stop,
	// in the snapshot on which it is called (usually a newly created
	// snapshot).
	typedef std::function<void(Snapshot *ss)> Action;

	// A System contains a series of snapshots which enables replaying of
	// the simulation from any point. There is also a single Snapshot,
	// 'fixtures', which contains static Bodies that never move or change
	// properties.
	class System {
	public:
		// snapshots 
		std::map<Timestamp, std::unique_ptr<Snapshot>> snapshots;
		Snapshot fixtures;
		std::map<Timestamp, std::vector<Action>> input_queue;

		// Duration is the time between the last snapshot and the transition.
		std::pair<Duration, std::vector<Action>> next_transition;

		// Must call Calculate after initialization, and after inserting to input_queue.
		// Calculate figures out what next_transition is.
		void Calculate();

		// CalculateToTime checks if time t is beyond next_transition; if it
		// is, a new snapshot is created at the moment of next_transition,
		// next_transition's Actions are applied, and CalculateToTime is
		// executed again.
		void CalculateToTime(Timestamp t);

		// RewindToTime removes snapshots after time t. To insert a backdated
		// input, for example, one would RewindToTime(new_input_time), add the
		// input to input_queue, then CalculateToTime(current_time), and the
		// world will be updated as if the input had occurred at time t.
		void RewindToTime(Timestamp t);

		// Call a function for every body at timestamp t. A duration is provided
		// to the target function so that, eg. one could check for bodies with
		// x > 5 at time q with something like
		// system->ForEachAt(q, [](Duration d, Body *b) {
		//   if (b->PositionAfterTime(d).x > 5.0) {
		//     [...]
		//   }
		// });
		void ForEachAt(Timestamp t, DurationBodyFunc func, bool include_fixtures = DontIncludeFixtures);

		// Convenience wrapper around AddInputEvent; adds an InputEvent
		// that updates a single body's velocity by the vector [line].
		void AddImpulseEvent(Timestamp t, BodyID id, const Vec2d &line);

		// Add an Action to occur on a new snapshot at time t.
		void AddInputEvent(Timestamp t, Action action);

		// Return the snapshot that covers time t, and the duration after that
		// snapshot that time t would be at.
		std::pair<Duration, Snapshot*> At(Timestamp t);

		static const bool IncludeFixtures = true;
		static const bool DontIncludeFixtures = false;
	private:
		bool ShouldAddTransition(Duration t);
	};

}

#endif //__SHARPPHYSICS_SYSTEM_H_
