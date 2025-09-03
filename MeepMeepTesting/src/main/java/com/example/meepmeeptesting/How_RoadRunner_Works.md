# Understanding RoadRunner Coordinates and Trajectory Commands

FTC/RoadRunner uses a fixed field coordinate system.  
The origin (0,0) is at the center of the field, the X‑axis runs parallel to the Red Alliance wall (increasing to the right), and the Y‑axis runs perpendicular to that wall (increasing away from the Red Alliance side).  
A robot’s pose is given by a `Pose2d(x, y, heading)`, where *x* and *y* are the field coordinates and *heading* is its orientation in radians (measured counter-clockwise from the +X direction).  
For example, `new Pose2d(0, -58, 0)` means the robot starts 58 units toward the Red side (negative Y) at X=0, facing 0° (along the +X axis).  Calling `.turn(angle)` adds that many radians to the heading (positive is CCW).

## Trajectory Primitives: `lineToX(x)`, `lineToY(y)`, and Alternatives

In RoadRunner’s `TrajectoryBuilder` (or `actionBuilder`) interface, commands like `lineToX(x)` and `lineToY(y)` use the robot’s current heading to generate a straight-line path to the target coordinate. 

Concretely:
* `lineToX(xTarget)`: drives the robot in a straight line in the direction of its current heading until its *global X* coordinate reaches `xTarget`.  (If the robot is not already facing exactly along the X direction, it will move diagonally.)
* `lineToY(yTarget)`: similarly drives straight along the current heading until its *global Y* equals `yTarget`.

Both methods assume the heading remains fixed (they do not rotate the robot while moving).  In fact, the RoadRunner docs warn that the robot’s heading should *not* be orthogonal (i.e. 90° off) from the line direction when using `lineToX/Y`.  
If the robot is facing exactly perpendicular to the desired axis, those methods will fail or error.  In practice, you typically use a `.turn(…)` or `.setTangent(…)` first to orient the robot so that `lineToX` or `lineToY` can proceed.

* In your code, for example, the bot starts facing 0° (along +X), then `.turn(16.5°)`.  Now `lineToX(58)` drives in that 16.5° direction until the *x*-coordinate reaches 58.  
* After further turns, each subsequent `lineToX(…)` or `lineToY(…)` moves along whatever heading was set by the last turn.

Key points: `lineToX` and `lineToY` do not forcibly move the robot purely horizontally or vertically; they respect the robot’s orientation.  
RoadRunner documentation puts it like: *“Robot moves to the specified X coordinate in the direction of the robot heading (straight line).”*.  Likewise for Y.  
This is why these commands “are associated with angles” – they shoot a straight path in the robot’s current facing direction.

* `strafeTo(new Vector2d(x,y))`: by contrast, will move the robot straight to the point (x,y) while keeping the heading fixed, regardless of orientation.  This is equivalent to moving purely parallel/perpendicular to axes if desired.  
* In fact, RoadRunner’s guides recommend using `strafeTo` for pure axis-aligned moves.  For example, if you want to move directly to (58, -7) without changing heading, you could use `.strafeTo(new Vector2d(58, -7))` rather than mixing turns and `lineToX/Y`.

In summary, coordinate moves in RoadRunner depend on the robot’s heading.  The methods `lineToX(x)` and `lineToY(y)` mean “go straight in the current direction until the X (or Y) value equals the target”.  
If you assumed they move purely along field axes regardless of orientation, you’ll see diagonal motions instead.  Always remember to set your heading appropriately (or use `strafeTo`) when you want simple axis-aligned moves.

Sources: FTC/RoadRunner docs and tutorials on the coordinate system and trajectory functions.
