# SharpPhysics

SharpPhysics is a 2D physics engine intended for precision and reproducibility.

It only allows circular objects to move (because those are all its target game required).

It also supports static lines, either tangible with infinite mass, or intangible to use
as triggers.

It is intended for small numbers of live objects (like a game of pool or something like
skee-ball) - as such there is no kind of tree based optimization to minimize collision
detections - every moving object is checked against every other object.

Where it excels is in sparse simulations with few external changes (again, like a pool
table) - since collision times are calculated using polynomial equations, once the time
of the next collision has been detected, no further collision detection is required until
after that collision has occurred (compared to most physics engines which check for
collisions every frame). Which is to say, this engine is more expensive for resolving
a collision, but very cheap the rest of the time.

The other cost of this unusual model is that it will be very unfamiliar to use!

## How does it work?

The engine consists of a `System` object, which contains a number of `Snapshot` objects.
During initialization, a base zero-time Snapshot should be created in `System.snapshots`,
and populated with all the objects that will be moving during the simulation, and 
`System.fixtures` should be populated with all the objects that will be persistent and
non-moving for the duration of the simulation, then `System.Calculate()` should be called.

To move an object (eg. if the player applies a force), one must introduce an action with
a timestamp into `System.input_queue`, then call `System.Calculate()`.

To advance the simulation, call `System.CalculateToTime(timestamp)` - this will create
new snapshots at any transition points (collisions, objects stopping due to friction,
input events) between the current snapshot and the given timestamp.

To get the positions of bodies, eg. for rendering, call `System.ForEachAt(timestamp, ...)`

This iterates over all bodies, calling a provided lambda on each of them, with a `Duration`
and a `Body*` which indicates a body from within the Snapshot closest to timestamp. You
can get the position and velocity of this body by calling `body->PositionAfterTime(duration)`
and `body->VelocityAfterTime(duration)` - this is necessary because the *stored* Body
contains only its position and velocity at the time of the snapshot. It's only a minor
newtonian calculation to get the adjusted position and velocity, but since for some
purposes you may not need them, the updated values aren't calculated unless requested.

`ExtraData` on a body is a convenient place to store rendering functions and other
per-object data. Note that any data that mutates over time can be tricky here, as one
BodyID shares the same instance of ExtraData across multiple Body instances, one for
each snapshot.
