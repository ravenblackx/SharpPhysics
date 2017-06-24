#include <Logger/logger.h>
#include "System.h"

namespace SharpPhysics {

	void Snapshot::FillFromPrevious(const Snapshot &prev, Duration t) {
		for (const auto &b : prev.bodies) {
			bodies[b.first] = std::unique_ptr<Body>(b.second->CopyAfterDuration(t));
		}
	}

	void Snapshot::ForEach(BodyFunc func) const {
		for (const auto &b : bodies) {
			func(b.second.get());
		}
	}

	std::pair<Duration, Snapshot*> System::At(Timestamp ts) {
		auto it = std::prev(snapshots.upper_bound(ts));
		return std::make_pair(ts - it->first, it->second.get());
	}
	void System::ForEachAt(Timestamp ts, DurationBodyFunc func, bool include_fixtures) {
		auto it = At(ts);
		Duration d = it.first;
		auto f = [func, d](Body *b) { func(d, b); };
		if (include_fixtures) {
			fixtures.ForEach(f);
		}
		it.second->ForEach(f);
	}

	void System::RewindToTime(Timestamp ts) {
		auto cutoff = snapshots.lower_bound(ts);
		snapshots.erase(cutoff, snapshots.end());
		next_transition.first = NaN;
		next_transition.second.clear();
		Calculate();
	}

	bool System::ShouldAddTransition(Duration t) {
		if (std::isnan(t) || t > next_transition.first) return false;
		if (!(next_transition.first == t)) next_transition.second.clear();
		next_transition.first = t;
		return true;
	}

	void System::AddImpulseEvent(Timestamp ts, BodyID id, const Vec2d &line) {
		AddInputEvent(ts, [id, line](Snapshot *ss) { ss->GetBody(id)->AddVelocity(line); });
	}

	void System::AddInputEvent(Timestamp ts, Action action) {
		input_queue[ts].push_back(action);
		RewindToTime(ts);
	}

	void System::Calculate() {
		auto it = std::prev(snapshots.cend());
		Timestamp ts = it->first;
		const Snapshot &ss = *it->second;
		auto next_input = input_queue.upper_bound(ts);
		next_transition.second.clear();
		if (next_input != input_queue.end()) {
			auto end_input = input_queue.upper_bound(next_input->first);
			next_transition.first = next_input->first - ts;
			for (auto it = next_input; it != end_input; it++) {
				next_transition.second.insert(next_transition.second.end(), it->second.begin(), it->second.end());
			}
		}
		else {
			next_transition.first = NaN;
			next_transition.second.clear();
		}
		// Check for friction stops.
		for (auto it = ss.bodies.begin(); it != ss.bodies.end(); it++) {
			if (it->second->IsStopped()) continue;
			BodyID id = it->first;
			Duration stops_at = it->second->TimeUntilStop();
			if (ShouldAddTransition(stops_at)) {
				next_transition.second.emplace_back([id](Snapshot *ss) {ss->GetBody(id)->Stop(); });
			}
		}
		// Check for collisions.
		for (auto it = ss.bodies.begin(); it != ss.bodies.end(); it++) {
			if (it->second->IsStopped()) continue;
			BodyID id = it->first;
			// Check for stopped bodies earlier than this one.
			for (auto other = ss.bodies.begin(); other != it; other++) {
				if (!other->second->IsStopped()) continue;
				BodyID other_id = other->first;
				Duration ctime = it->second->TimeUntilCollide(*other->second, next_transition.first);
				if (ShouldAddTransition(ctime)) {
					next_transition.second.emplace_back([id, other_id](Snapshot *ss) {
						ss->GetBody(id)->ApplyCollision(ss->GetBody(other_id));
					});
				}
			}
			// Check against all bodies later than this one.
			for (auto other = std::next(it); other != ss.bodies.end(); other++) {
				BodyID other_id = other->first;
				Duration ctime = it->second->TimeUntilCollide(*other->second, next_transition.first);
				if (ShouldAddTransition(ctime)) {
					next_transition.second.emplace_back([this, id, other_id](Snapshot *ss) {
						ss->GetBody(id)->ApplyCollision(ss->GetBody(other_id));
					});
				}
			}
			// Check against fixtures.
			for (auto other = fixtures.bodies.begin(); other != fixtures.bodies.end(); other++) {
				Body *other_body = other->second.get();
				Duration ctime = it->second->TimeUntilCollide(*other_body, next_transition.first);
				if (ShouldAddTransition(ctime)) {
					next_transition.second.emplace_back([this, id, other_body](Snapshot *ss) {
						ss->GetBody(id)->ApplyCollision(other_body);
					});
				}
			}
		}
	}

	void System::CalculateToTime(Timestamp t) {
		auto end = std::prev(snapshots.cend());
		if (!(t > end->first + next_transition.first)) return;
		auto &ss = snapshots[end->first + next_transition.first];
		ss->FillFromPrevious(*end->second, next_transition.first);
		for (const auto &action : next_transition.second) {
			action(ss.get());
		}
		Calculate();
		CalculateToTime(t);
	}
}
