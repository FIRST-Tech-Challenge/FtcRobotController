# Modified Pure Pursuit

Get to the target point. Don't stop at every waypoint along the way.

* intersect a circle (r=lookahead) with the path, finding the point furthest along the path

### Required targets
Either:
*   automatically adjust hard waypoints by projecting the next waypoint forward
    in the direction relative to the robot
*   or shrink the lookahead radius until the waypoint is reached (caveat:
    introduces discontinuities, meaning there will be obvious hard switches
    in the middle of the motion)
*   or adjust the waypoints by hand